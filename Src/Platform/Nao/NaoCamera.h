/**
 * @file Platform/Nao/NaoCamera.h
 * Interface to a camera of the NAO.
 * @author Colin Graf
 */

#pragma once

#include "Representations/Configuration/AutoExposureWeightTable.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/CameraSettings.h"

/**
 * @class NaoCamera
 * Interface to a camera of the NAO
 */
class NaoCamera
{
public:
  /**
   * Constructor.
   * @param device The name of the camera device (e.g. /dev/video0).
   * @param camera Whether this is the lower or upper camera.
   * @param width The width of the camera image in pixels. V4L only allows certain values (e.g. 320 or 640).
   * @param height The height of the camera image in pixels. V4L only allows certain values (e.g. 240 or 480).
   * @param flip Whether the image should be flipped, i.e. rotated by 180°.
   * @param settings The initial camera settings.
   * @param autoExposureWeightTable The initial auto exposure weight table. The table contains 5x5 values in
   *                                the range [0 .. 100] that weight the influence the corresponding area of
   *                                the image (rows top to bottom, columns left to right) on the auto exposure
   *                                computation. If the table does only contains zeros, the image will be black.
   */
  NaoCamera(const char* device, CameraInfo::Camera camera, int width, int height, bool flip,
            const CameraSettings::CameraSettingsCollection& settings, const Matrix5uc& autoExposureWeightTable);

  ~NaoCamera();

  void changeResolution(int width, int height);

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
  static bool captureNew(NaoCamera& cam1, NaoCamera& cam2, int timeout);

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
  bool hasImage();

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

  CameraSettings::CameraSettingsCollection getCameraSettingsCollection() const;
  Matrix5uc getAutoExposureWeightTable() const;

  void setSettings(const CameraSettings::CameraSettingsCollection& settings, const Matrix5uc& autoExposureWeightTable);

  /**
   * Unconditional write of the camera settings
   */
  void writeCameraSettings();

  void readCameraSettings();

  void doAutoWhiteBalance();

private:
  class V4L2Setting
  {
  public:
    int command = 0;
    int value = 0;

    CameraSettings::CameraSetting notChangableWhile = CameraSettings::numOfCameraSettings;

    V4L2Setting() = default;
    V4L2Setting(int command, int value, int min, int max, CameraSettings::CameraSetting notChangableWhile = CameraSettings::numOfCameraSettings);

    bool operator==(const V4L2Setting& other) const;
    bool operator!=(const V4L2Setting& other) const;

    void enforceBounds();
    void setCameraBounds(int camMin, int camMax);

  private:
    int min = std::numeric_limits<int>::min();
    int max = std::numeric_limits<int>::max();
  };

  struct CameraSettingsCollection
  {
    std::array<V4L2Setting, CameraSettings::numOfCameraSettings> settings;
    static const constexpr size_t sizeOfAutoExposureWeightTable = 25;
    std::array<V4L2Setting, sizeOfAutoExposureWeightTable> autoExposureWeightTable;

    CameraSettingsCollection();

    bool operator==(const CameraSettingsCollection& other) const;
    bool operator!=(const CameraSettingsCollection& other) const;
  };

  struct CameraSettingsSpecial
  {
    V4L2Setting verticalFlip;
    V4L2Setting horizontalFlip;
    V4L2Setting doAutoWhiteBallance;

    CameraSettingsSpecial();
  };

  CameraInfo::Camera camera; /**< The camera accessed by this driver. */
  CameraSettingsCollection settings; /**< The camera control settings. */
  CameraSettingsCollection appliedSettings; /**< The camera settings that are known to be applied. */
  CameraSettingsSpecial specialSettings; /**< Special settings that are only set */

  static const constexpr unsigned frameBufferCount = 3; /**< Amount of available frame buffers. */

  unsigned WIDTH; /**< The width of the yuv 422 image */
  unsigned HEIGHT; /**< The height of the yuv 422 image */
  int fd; /**< The file descriptor for the video device. */
  void* mem[frameBufferCount]; /**< Frame buffer addresses. */
  int memLength[frameBufferCount]; /**< The length of each frame buffer. */
  struct v4l2_buffer* buf = nullptr; /**< Reusable parameter struct for some ioctl calls. */
  struct v4l2_buffer* currentBuf = nullptr; /**< The last dequeued frame buffer. */
  bool first = true; /**< First image grabbed? */
  unsigned long long timeStamp = 0; /**< Timestamp of the last captured image in microseconds. */

  void checkSettingsAvailability();

  void checkV4L2Setting(V4L2Setting& setting) const;

  /**
   * Requests the value of a camera control setting from camera.
   * @param id The setting id.
   * @return The value.
   */
  bool getControlSetting(V4L2Setting& setting);

  /**
   * Sets the value of a camera control setting to camera.
   * @param id The setting id.
   * @param value The value.
   * @return True on success.
   */
  bool setControlSetting(V4L2Setting& setting);

  bool assertCameraSetting(CameraSettings::CameraSetting setting);
  bool assertAutoExposureWeightTableEntry(size_t entry);

  void setImageFormat();

  void mapBuffers();
  void unmapBuffers();
  void queueBuffers();

  void startCapturing();
  void stopCapturing();
};
