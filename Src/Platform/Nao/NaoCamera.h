/**
 * @file Platform/Nao/NaoCamera.h
 * Interface to a camera of the NAO.
 * @author Colin Graf
 */

#pragma once

#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/CameraSettings.h"

/**
 * @class NaoCamera
 * Interface to a camera of the NAO
 */
class NaoCamera
{
public:
  unsigned int timeWaitedForLastImage = 0; /**< The time (in _milliseconds_) passed while waiting for a new image. */

private:
  CameraInfo::Camera camera; /**< The camera accessed by this driver. */
  CameraSettings::CameraSettingCollection settings; /**< The camera control settings. */
  CameraSettings::CameraSettingCollection appliedSettings; /**< The camera settings that are known to be applied. */

  static constexpr unsigned frameBufferCount = 3; /**< Amount of available frame buffers. */

  unsigned WIDTH; /**< The width of the yuv 422 image */
  unsigned HEIGHT; /**< The height of the yuv 422 image */
#ifndef NDEBUG
  unsigned SIZE; /**< The size of an image in bytes */
#endif
  int fd; /**< The file descriptor for the video device. */
  void* mem[frameBufferCount]; /**< Frame buffer addresses. */
  int memLength[frameBufferCount]; /**< The length of each frame buffer. */
  struct v4l2_buffer* buf = nullptr; /**< Reusable parameter struct for some ioctl calls. */
  struct v4l2_buffer* currentBuf = nullptr; /**< The last dequeued frame buffer. */
  bool first = true; /**< First image grabbed? */
  unsigned long long timeStamp = 0; /**< Timestamp of the last captured image in microseconds. */

public:
  /**
   * Constructor.
   * @param device The name of the camera device (e.g. /dev/video0).
   * @param camera Whether this is the lower or upper camera.
   * @param width The width of the camera image in pixels. V4L only allows certain values (e.g. 320 or 640)
   * @param height The height of the camera image in pixels. V4L only allows certain values (e.g. 240 or 320)
   * @param flip Whether the image should be flipped
   */
  NaoCamera(const char* device, CameraInfo::Camera camera, int width, int height, bool flip);

  virtual ~NaoCamera();

  /**
   * The method blocks till a new image arrives.
   * @return true (except a not manageable exception occurs)
   */
  bool captureNew();

  /**
   * The method blocks till one of the given cameras returns an image.
   * @param cam1 The first camera
   * @param cam2 The second camera
   * @param timeout The maximum waiting time
   * @param errorCam1 Whether an error occured on cam1
   * @param errorCam2 Whether an error occured on cam2
   */
  static bool captureNew(NaoCamera& cam1, NaoCamera& cam2, int timeout, bool& errorCam1, bool& errorCam2);

  /**
   * Releases an image that has been captured. That way the buffer can be used to capture another image
   */
  void releaseImage();

  /**
   * The last captured image.
   * @return The image data buffer.
   */
  const unsigned char* getImage() const;

  /**
   * Whether an image has been captured.
   * @return true if there is one
   */
  virtual bool hasImage();

  /**
   * Timestamp of the last captured image in µs.
   * @return The timestamp.
   */
  unsigned long long getTimeStamp() const;

  /**
   * Returns the frame rate used by the camera
   * @return The frame rate in frames per second
   */
  float getFrameRate() const;

  /**
   * Set frame rate in frames per 10 seconds.
   */
  void setFrameRate(unsigned numerator = 1, unsigned denominator = 30);

  /**
   * Requests the current camera control settings of the camera.
   * @return The settings.
   */
  const CameraSettings::CameraSettingCollection& getSettings() const { return appliedSettings; };

  /**
   * Sets the camera control settings to the camera.
   * @param settings The settings.
   */
  void setSettings(const CameraSettings::CameraSettingCollection& settings);

  /**
   * Asserts that the actual camera settings are correct.
   */
  void assertCameraSettings();

  /**
   * Unconditional write of the camera settings
   */
  void writeCameraSettings();

  void readCameraSettings();

  void doAutoWhiteBalance();

private:
  /**
   * Requests the value of a camera control setting from camera.
   * @param id The setting id.
   * @return The value.
   */
  bool getControlSetting(unsigned int id, int& value);

  /**
   * Sets the value of a camera control setting to camera.
   * @param id The setting id.
   * @param value The value.
   * @return True on success.
   */
  bool setControlSetting(unsigned int id, int value);

  bool assertCameraSetting(CameraSettings::CameraSetting setting);

  void initOpenVideoDevice(const char* device);
  void initSetImageFormat();
  void initRequestAndMapBuffers();
  void initQueueAllBuffers();
  void initDefaultControlSettings(bool flip);
  void startCapturing();
};
