/**
 * @file OrbbecProvider.cpp
 *
 * This file implements a module that grabs images from the Orbbec camera sensor.
 *
 * @author Thomas Röfer
 */

#include "OrbbecProvider.h"
#include "Math/Range.h"
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include "Platform/Thread.h"
#include "Platform/Time.h"

MAKE_MODULE(OrbbecProvider);

thread_local OrbbecProvider* OrbbecProvider::theInstance = nullptr;

const Rangei OrbbecProvider::settingLimits[OrbbecProvider::numOfSettings]
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

const std::unordered_map<OrbbecProvider::Setting, OrbbecProvider::Setting> OrbbecProvider::skipIfEnabled =
{
  {OrbbecProvider::exposure, OrbbecProvider::autoExposure},
  {OrbbecProvider::gain, OrbbecProvider::autoExposure},
  {OrbbecProvider::whiteBalance, OrbbecProvider::autoWhiteBalance}
};

void OrbbecProvider::Settings::read(In& stream)
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

void OrbbecProvider::Settings::write(Out& stream) const
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

void OrbbecProvider::Settings::reg()
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

OrbbecProvider::OrbbecProvider()
  : cameraInfo(CameraInfo::upper)
{
  theInstance = this;
  VERIFY(readCameraIntrinsics());
  VERIFY(readCameraResolution());

  // Initialize applied settings with a value illegal for all settings.
  std::fill(appliedSettings.begin(), appliedSettings.end(), 20000);

  setupCamera();
}

OrbbecProvider::~OrbbecProvider()
{
  stopStream();
  theInstance = nullptr;
}

void OrbbecProvider::update(CameraImage& theCameraImage)
{
  {
    theCameraImage.setResolution(cameraInfo.width / 2, cameraInfo.height);
    theCameraImage.timestamp = Time::getCurrentSystemTime();
  }
}

void OrbbecProvider::update(JPEGImage& theJPEGImage)
{
  theJPEGImage.fromCameraImage(theCameraImage, jpegQuality);
}

bool OrbbecProvider::readCameraIntrinsics()
{
  InMapFile stream("cameraIntrinsics.cfg");
  bool exist = stream.exists();
  if(exist)
    stream >> cameraIntrinsics;
  return exist;
}

bool OrbbecProvider::readCameraResolution()
{
  InMapFile stream("cameraResolution.cfg");
  bool exist = stream.exists();
  if(exist)
    stream >> cameraResolutionRequest;
  return exist;
}

bool OrbbecProvider::processResolutionRequest()
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

void OrbbecProvider::applySettings()
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
      appliedSettings[setting] = settings[setting];
    }
  }
}

void OrbbecProvider::setupCamera()
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

void OrbbecProvider::startStream()
{
}

void OrbbecProvider::stopStream()
{
}

void OrbbecProvider::waitForFrameData2()
{
  Thread::sleep(33);
}

bool OrbbecProvider::isFrameDataComplete()
{
  return true;
}

void OrbbecProvider::waitForFrameData()
{
  if(theInstance)
    theInstance->waitForFrameData2();
}
