/**
 * @file RealSenseProvider.cpp
 *
 * This file implements a module that grabs images from the RealSense sensor.
 *
 * @author Thomas Röfer
 */

#include "RealSenseProvider.h"
#include "Math/Range.h"
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include "Platform/Thread.h"
#include "Platform/Time.h"
#ifdef TARGET_BOOSTER
#include <librealsense2/h/rs_pipeline.h>
#include <librealsense2/h/rs_frame.h>
#endif

MAKE_MODULE(RealSenseProvider);

thread_local RealSenseProvider* RealSenseProvider::theInstance = nullptr;

const Rangei RealSenseProvider::settingLimits[RealSenseProvider::numOfSettings]
{
  {0, 1},
  {-64, 64},
  {0, 100},
  {1, 10000},
  {0, 128},
  {100, 500},
  {-180, 180},
  {0, 100},
  {0, 100},
  {2800, 6500},
  {0, 1},
  {0, 1},
  {0, 32},
  {0, 3},
  {0, 1}
};

const std::unordered_map<RealSenseProvider::Setting, RealSenseProvider::Setting> RealSenseProvider::skipIfEnabled =
{
  {RealSenseProvider::exposure, RealSenseProvider::autoExposure},
  {RealSenseProvider::gain, RealSenseProvider::autoExposure},
  {RealSenseProvider::whiteBalance, RealSenseProvider::autoWhiteBalance}
};

#ifdef TARGET_BOOSTER
const rs2_option RealSenseProvider::options[RealSenseProvider::numOfSettings] =
{
  RS2_OPTION_BACKLIGHT_COMPENSATION,
  RS2_OPTION_BRIGHTNESS,
  RS2_OPTION_CONTRAST,
  RS2_OPTION_EXPOSURE,
  RS2_OPTION_GAIN,
  RS2_OPTION_GAMMA,
  RS2_OPTION_HUE,
  RS2_OPTION_SATURATION,
  RS2_OPTION_SHARPNESS,
  RS2_OPTION_WHITE_BALANCE,
  RS2_OPTION_ENABLE_AUTO_EXPOSURE,
  RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE,
  RS2_OPTION_FRAMES_QUEUE_SIZE,
  RS2_OPTION_POWER_LINE_FREQUENCY,
  RS2_OPTION_AUTO_EXPOSURE_PRIORITY
};
#endif

void RealSenseProvider::Settings::read(In& stream)
{
  FOREACH_ENUM(Setting, setting)
    switch(setting)
    {
      case backlightCompensation:
      case autoExposure:
      case autoWhiteBalance:
      case autoExposurePriority:
      {
        bool value = static_cast<bool>((*this)[setting]);
        Streaming::streamIt(stream, TypeRegistry::getEnumName(setting), value);
        (*this)[setting] = static_cast<int>(value);
        break;
      }
      case powerLineFrequency:
      {
        PowerLineFrequency value = static_cast<PowerLineFrequency>((*this)[setting]);
        Streaming::streamIt(stream, TypeRegistry::getEnumName(setting), value);
        (*this)[setting] = static_cast<int>(value);
        break;
      }
      default:
        Streaming::streamIt(stream, TypeRegistry::getEnumName(setting), (*this)[setting]);
    }
}

void RealSenseProvider::Settings::write(Out& stream) const
{
  FOREACH_ENUM(Setting, setting)
    switch(setting)
    {
      case backlightCompensation:
      case autoExposure:
      case autoWhiteBalance:
      case autoExposurePriority:
      {
        Streaming::streamIt(stream, TypeRegistry::getEnumName(setting), static_cast<bool>((*this)[setting]));
        break;
      }
      case powerLineFrequency:
      {
        Streaming::streamIt(stream, TypeRegistry::getEnumName(setting), static_cast<PowerLineFrequency>((*this)[setting]));
        break;
      }
      default:
        Streaming::streamIt(stream, TypeRegistry::getEnumName(setting), (*this)[setting]);
    }
}

void RealSenseProvider::Settings::reg()
{
  PUBLISH(reg);
  REG_CLASS(Settings);
  FOREACH_ENUM(Setting, setting)
    switch(setting)
    {
      case backlightCompensation:
      case autoExposure:
      case autoWhiteBalance:
      case autoExposurePriority:
        TypeRegistry::addAttribute(_type, typeid(bool).name(), TypeRegistry::getEnumName(setting));
        break;
      case powerLineFrequency:
        TypeRegistry::addAttribute(_type, typeid(PowerLineFrequency).name(), TypeRegistry::getEnumName(setting));
        break;
      default:
        TypeRegistry::addAttribute(_type, typeid(int).name(), TypeRegistry::getEnumName(setting));
    }
}

RealSenseProvider::RealSenseProvider()
  : cameraInfo(CameraInfo::upper)
{
  theInstance = this;
  VERIFY(readCameraIntrinsics());
  VERIFY(readCameraResolution());

  // Initialize applied settings with a value illegal for all settings.
  std::fill(appliedSettings.begin(), appliedSettings.end(), 20000);

#ifdef TARGET_BOOSTER
  // Create a context object. This object owns the handles to all connected RealSense devices.
  context = rs2_create_context(RS2_API_VERSION, &e);
  VERIFY(ok());

  // Create a pipeline to configure, start and stop camera streaming
  pipeline = rs2_create_pipeline(context, &e);
  VERIFY(ok());

  // Create a config instance, used to specify hardware configuration
  config = rs2_create_config(&e);
  VERIFY(ok());
#endif

  setupCamera();
}

RealSenseProvider::~RealSenseProvider()
{
  stopStream();

#ifdef TARGET_BOOSTER
  // Release RealSense resources.
  rs2_delete_config(config);
  rs2_delete_pipeline(pipeline);
  rs2_delete_context(context);
#endif

  theInstance = nullptr;
}

void RealSenseProvider::update(CameraImage& theCameraImage)
{
#ifdef TARGET_BOOSTER
  if(yuvFrameData)
  {
    const unsigned timestamp = static_cast<long long>(frameMetadataTimeOfArrival) > static_cast<long long>(Time::getSystemTimeBase())
                               ? static_cast<unsigned>(frameMetadataTimeOfArrival - Time::getSystemTimeBase()) : 100000;
    theCameraImage.setReference(cameraInfo.width / 2, cameraInfo.height, const_cast<unsigned char*>(yuvFrameData), std::max(lastImageTimestamp + 1, timestamp));
    lastImageTimestamp = theCameraImage.timestamp;
  }
  else
#endif
  {
    theCameraImage.setResolution(cameraInfo.width / 2, cameraInfo.height);
    theCameraImage.timestamp = Time::getCurrentSystemTime();
  }
}

void RealSenseProvider::update(JPEGImage& theJPEGImage)
{
  theJPEGImage.fromCameraImage(theCameraImage, jpegQuality);
}

bool RealSenseProvider::readCameraIntrinsics()
{
  InMapFile stream("cameraIntrinsics.cfg");
  bool exist = stream.exists();
  if(exist)
    stream >> cameraIntrinsics;
  return exist;
}

bool RealSenseProvider::readCameraResolution()
{
  InMapFile stream("cameraResolution.cfg");
  bool exist = stream.exists();
  if(exist)
    stream >> cameraResolutionRequest;
  return exist;
}

bool RealSenseProvider::processResolutionRequest()
{
  const CameraResolutionRequest::Resolutions requestedResolution = theCameraResolutionRequest.resolutions[CameraInfo::upper];
  CameraResolutionRequest::Resolutions& currentResolution = cameraResolutionRequest.resolutions[CameraInfo::upper];
  if(SystemCall::getMode() == SystemCall::Mode::physicalRobot && requestedResolution != lastResolutionRequest)
  {
    lastResolutionRequest = requestedResolution;
    switch(requestedResolution)
    {
      case CameraResolutionRequest::noRequest:
        return false;
      case CameraResolutionRequest::defaultRes:
        if(!readCameraResolution())
          currentResolution = CameraResolutionRequest::w640h480;
        return true;
      case CameraResolutionRequest::w424h240:
      case CameraResolutionRequest::w480x270:
      case CameraResolutionRequest::w640h360:
      case CameraResolutionRequest::w640h480:
      case CameraResolutionRequest::w848h480:
      case CameraResolutionRequest::w1280h720:
      case CameraResolutionRequest::w1280h800:
        currentResolution = requestedResolution;
        return true;
      default:
        FAIL("Unknown resolution.");
        return false;
    }
  }
  else
    return false;
}

void RealSenseProvider::applySettings()
{
  FOREACH_ENUM(Setting, setting)
  {
    const auto skipCheck = skipIfEnabled.find(setting);
    if(settings[setting] != appliedSettings[setting]
       && (skipCheck == skipIfEnabled.end() || appliedSettings[skipCheck->second] != 1))
    {
      const int limitedSetting = settingLimits[setting].limit(settings[setting]);
      if(limitedSetting != settings[setting])
        OUTPUT_WARNING(TypeRegistry::getEnumName(setting) << " should be inside [" << settingLimits[setting].min
                       << ", " << settingLimits[setting].max << "], but is " << settings[setting]);
#ifdef TARGET_BOOSTER
      rs2_set_option(reinterpret_cast<rs2_options*>(sensor), options[setting], static_cast<float>(limitedSetting), &e);
      VERIFY(ok());
#endif
      appliedSettings[setting] = settings[setting];
    }
  }
}

void RealSenseProvider::setupCamera()
{
  // set resolution
  cameraResolutionRequest.apply(CameraInfo::upper, cameraInfo);

  // set opening angle
  cameraInfo.openingAngleWidth = cameraIntrinsics.cameras[CameraInfo::upper].openingAngleWidth;
  cameraInfo.openingAngleHeight = cameraIntrinsics.cameras[CameraInfo::upper].openingAngleHeight;

  // set optical center
  cameraInfo.opticalCenter.x() = cameraIntrinsics.cameras[CameraInfo::upper].opticalCenter.x() * cameraInfo.width;
  cameraInfo.opticalCenter.y() = cameraIntrinsics.cameras[CameraInfo::upper].opticalCenter.y() * cameraInfo.height;

  // update focal length
  cameraInfo.updateFocalLength();

  startStream();
}

void RealSenseProvider::startStream()
{
#ifdef TARGET_BOOSTER
  // Request a specific configuration
  rs2_config_enable_stream(config, RS2_STREAM_COLOR, 0, cameraInfo.width, cameraInfo.height, RS2_FORMAT_YUYV, 30, &e);
  VERIFY(ok());

  // Depth image is required to avoid constant restart of RealSense
  rs2_config_enable_stream(config, RS2_STREAM_DEPTH, 0, 424, 240, RS2_FORMAT_Z16, 5, &e);
  VERIFY(ok());

  Thread::getCurrentThread()->setPriority(11); // One more than Upper when waiting for an image

  // Start the pipeline streaming
  pipelineProfile = rs2_pipeline_start_with_config(pipeline, config, &e);
  VERIFY(ok());

  Thread::getCurrentThread()->setPriority(0);

  // Get sensor to apply settings
  rs2_device* device = rs2_pipeline_profile_get_device(pipelineProfile, &e);
  VERIFY(ok());

  // Find the sensor that represents the color camera
  rs2_sensor_list* sensors = rs2_query_sensors(device, &e);
  VERIFY(ok());

  const int numOfSensors = rs2_get_sensors_count(sensors, &e);
  VERIFY(ok());

  for(int i = 0; i < numOfSensors; ++i)
  {
    sensor = rs2_create_sensor(sensors, i, &e);
    VERIFY(ok());

    const std::string name = rs2_get_sensor_info(sensor, RS2_CAMERA_INFO_NAME, &e);
    VERIFY(ok());

    if(name == "Stereo Module") // First sensor
    {
      rs2_set_option(reinterpret_cast<rs2_options*>(sensor), RS2_OPTION_EMITTER_ENABLED, 0.f, &e);
      VERIFY(ok());
    }
    else if(name == "RGB Camera") // Second sensor
      break;

    rs2_delete_sensor(sensor);
    sensor = nullptr;
  }

  ASSERT(numOfSensors && sensor);

  // Free objects not needed anymore
  rs2_delete_sensor_list(sensors);
  rs2_delete_device(device);
#endif
}

void RealSenseProvider::stopStream()
{
#ifdef TARGET_BOOSTER
  // Free image frame
  if(yuvFrameData)
  {
    rs2_release_frame(frame);
    rs2_release_frame(frames);
    yuvFrameData = nullptr;
  }

  // Free sensor
  rs2_delete_sensor(sensor);

  // Stop the pipeline streaming
  rs2_pipeline_stop(pipeline, &e);
  rs2_delete_pipeline_profile(pipelineProfile);
#endif
}

void RealSenseProvider::waitForFrameData2()
{
#ifdef TARGET_BOOSTER
  const bool hadFrame = yuvFrameData != nullptr;
  if(yuvFrameData)
  {
    rs2_release_frame(frame);
    rs2_release_frame(frames);
    yuvFrameData = nullptr;
  }

  applySettings();

  if(processResolutionRequest())
  {
    stopStream();
    setupCamera();
  }

  // This call waits until a new composite_frame is available
  // composite_frame holds a set of frames. It is used to prevent frame drops
  // The returned object should be released with rs2_release_frame(...)
  while(!yuvFrameData)
  {
    frames = rs2_pipeline_wait_for_frames(pipeline, yuvFrameData ? minWaitForImage : maxWaitForImage, &e);
    if(ok(false))
    {
      // Returns the number of frames embedded within the composite frame.
      int numOfFrames = rs2_embedded_frames_count(frames, &e);
      VERIFY(ok());

      for(int i = numOfFrames - 1; i >= 0 && !yuvFrameData; --i)
      {
        frame = rs2_extract_frame(frames, i, &e);
        VERIFY(ok());

        const rs2_stream_profile* streamProfile = rs2_get_frame_stream_profile(frame, &e);
        VERIFY(ok());

        rs2_stream stream;
        rs2_format format;
        int index, frameRate, uniqueId;
        rs2_get_stream_profile_data(streamProfile, &stream, &format, &index, &frameRate, &uniqueId, &e);
        VERIFY(ok());

        if(stream == RS2_STREAM_COLOR)
        {
          yuvFrameData = (const uint8_t*)(rs2_get_frame_data(frame, &e));
          VERIFY(ok());

          frameMetadataTimeOfArrival = rs2_get_frame_metadata(frame, RS2_FRAME_METADATA_TIME_OF_ARRIVAL, &e);
          VERIFY(ok());

          if(!hadFrame)
            SystemCall::say("Camera ready");
        }
      }
    }
  }
#endif
}

bool RealSenseProvider::ok([[maybe_unused]] const bool print)
{
#ifdef TARGET_BOOSTER
  if(e)
  {
    if(print)
      OUTPUT_ERROR("RealSenseProvider: " << rs2_get_error_message(e));
    rs2_free_error(e);
    e = nullptr;
    return false;
  }
  else
#endif
    return true;
}

bool RealSenseProvider::isFrameDataComplete()
{
#ifdef TARGET_BOOSTER
  if(theInstance)
    return theInstance->yuvFrameData != nullptr;
  else
#endif
    return true;
}

void RealSenseProvider::waitForFrameData()
{
#ifdef TARGET_ROBOT
  if(theInstance)
    theInstance->waitForFrameData2();
#endif
}
