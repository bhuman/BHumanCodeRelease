/**
 * @file RealSenseProvider.h
 *
 * This file declares a module that grabs images from the RealSense sensor.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/Configuration/CameraIntrinsics.h"
#include "Representations/Configuration/CameraResolutionRequest.h"
#include "Representations/Infrastructure/CameraImage.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/CameraStatus.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JPEGImage.h"
#include "Framework/Module.h"

#if defined TARGET_ROBOT && (defined __arm64__ || defined __aarch64__)
#define TARGET_BOOSTER
#include <librealsense2/rs.h>
#endif

MODULE(RealSenseProvider,
{,
  REQUIRES(CameraResolutionRequest),
  REQUIRES(CameraImage),
  PROVIDES_WITHOUT_MODIFY(CameraImage),
  PROVIDES(FrameInfo),
  PROVIDES(CameraInfo),
  PROVIDES(CameraIntrinsics),
  PROVIDES(CameraStatus),
  PROVIDES_WITHOUT_MODIFY(JPEGImage),
  LOADS_PARAMETERS(
  {
    ENUM(Setting,
    {,
      backlightCompensation, /**< Enable/disable color backlight compensation. */
      brightness, /**< Color image brightness [-64 .. 64]. */
      contrast, /**< Color image contrast [0 .. 100]. */
      exposure, /**< Controls exposure time of color camera [1 .. 10000]. Setting any value will disable auto exposure. */
      gain, /**< Color image gain [0 .. 128]. */
      gamma, /**< Color image gamma setting [100 .. 500]. */
      hue, /**< Color image hue [-180 .. 180]. */
      saturation, /**< Color image saturation setting [0 .. 100]. */
      sharpness, /**< Color image sharpness setting [0 .. 100]. */
      whiteBalance, /**< Controls white balance of color image in steps of 10 [2800 .. 6500]. Setting any value will disable auto white balance. */
      autoExposure,/**< Enable/disable auto-exposure. */
      autoWhiteBalance, /**< Enable/disable color image auto-white-balance. */
      framesQueueSize, /**< Number of frames the user is allowed to keep per stream [0 .. 32]. Trying to hold-on to more frames will cause frame-drops. */
      powerLineFrequency, /**< Power Line Frequency control for anti-flickering hzOff/hz50/hz60/hzAuto. */
      autoExposurePriority, /**< Allows sensor to dynamically adjust the frame rate depending on lighting conditions. */
    });

    ENUM(PowerLineFrequency,
    {,
      hzOff,
      hz50,
      hz60,
      hzAuto,
    });

    class Settings : public std::array<int COMMA numOfSettings> COMMA public Streamable
    {
    protected:
      void read(In& stream) override;
      void write(Out& stream) const override;

    private:
      static void reg();
    },

    (int) maxWaitForImage, /**< Timeout in ms for waiting for new images after the camera was just set up. */
    (int) minWaitForImage, /**< Timeout in ms for waiting for new images when the camera was previously working. */
    (int) jpegQuality, /**< The quality of the JPEG compressing (0 = bad ... 100 = very good). */
    (Settings) settings, /**< The settings of the color camera. */
  }),
});

class RealSenseProvider : public RealSenseProviderBase
{
  static thread_local RealSenseProvider* theInstance; /**< The only instance of this module. */
  unsigned time = 0; /**< The time the current frame began (in ms). */
  CameraInfo cameraInfo; /**< The intrinsic calibration of the color camera. It is \c upper. */
  CameraIntrinsics cameraIntrinsics; /**< The intrinsic calibration of the cameras. The color camera is \c upper. */
  CameraResolutionRequest cameraResolutionRequest; /**< The resolution request for the cameras. The color camera is \c upper. */
  CameraResolutionRequest::Resolutions lastResolutionRequest = CameraResolutionRequest::defaultRes; /**< The last resolution requested. */
  unsigned lastImageTimestamp = 0; /**< The timestamp of the last image received. */
  Settings appliedSettings; /**< The settings that were already applied. */
  static const std::unordered_map<Setting, Setting> skipIfEnabled; /**< Skip setting an option if another option is currently enabled. */
  static const Rangei settingLimits[numOfSettings]; /**< The limits for the values of the settings. */

#ifdef TARGET_BOOSTER
  static const rs2_option options[numOfSettings]; /**< The mapping from settings to RealSense options. */
  rs2_error* e = nullptr; /**< This pointer will become non-zero if an error occurs. */
  rs2_context* context; /**< The RealSense context. */
  rs2_sensor* sensor; /**< The color camera. */
  rs2_pipeline* pipeline; /**< The pipeline to configure, start, and stop camera streaming. */
  rs2_config* config; /**< The  config instance, used to specify hardware configuration. */
  rs2_pipeline_profile* pipelineProfile; /**< The profile of the current stream. */
  rs2_frame* frames; /**< The frames received. */
  rs2_frame* frame; /**< The latest frame received. */
  const uint8_t* yuvFrameData = nullptr; /**< The image data in the latest frame received. */
  rs2_metadata_type frameMetadataTimeOfArrival = 0; /**< The time the last frame arrived (in ms). */
#endif

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theCameraImage The representation updated.
   */
  void update(CameraImage& theCameraImage) override;

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theCameraInfo The representation updated.
   */
  void update(CameraInfo& theCameraInfo) override {theCameraInfo = cameraInfo;}

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theCameraIntrinsics The representation updated.
   */
  void update(CameraIntrinsics& theCameraIntrinsics) override {theCameraIntrinsics = cameraIntrinsics;}

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theCameraStatus The representation updated.
   */
  void update(CameraStatus& theCameraStatus) override {theCameraStatus.ok = true;}

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theFrameInfo The representation updated.
   */
  void update(FrameInfo& theFrameInfo) override {theFrameInfo.time = theCameraImage.timestamp;}

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theJPEGImage The representation updated.
   */
  void update(JPEGImage& theJPEGImage) override;

  /** Read the camera intrinsics from a configuration file. */
  bool readCameraIntrinsics();

  /** Read the initial camera resolutions from a configuration file. */
  bool readCameraResolution();

  /**
   * Check whether the resolution should be changed.
   * @return Should it be changed?
   */
  bool processResolutionRequest();

  /** Applies the current settings and thereby modifies \c appliedSettings. */
  void applySettings();

  /** Setup the camera information and the actual camera. */
  void setupCamera();

  /** Starts the camera stream. */
  void startStream();

  /** Stops the camera stream. */
  void stopStream();

  /** The method waits for a new image (called by static method). */
  void waitForFrameData2();

  /**
   * Checks whether an error occurred and frees the error object.
   * @param print Print error message?
   * @return Everything ok?
   */
  bool ok(const bool print = true);

public:
  RealSenseProvider();
  ~RealSenseProvider();

  /**
   * The method returns whether a new image is available.
   * @return Is an new image available?
   */
  static bool isFrameDataComplete();

  /** The method waits for a new image. */
  static void waitForFrameData();
};
