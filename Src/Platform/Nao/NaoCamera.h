/**
 * @file Platform/Nao/NaoCamera.h
 * Interface to a camera of the NAO.
 * @author Colin Graf
 */

#pragma once

#include "Representations/Configuration/AutoExposureWeightTable.h"
#include "Representations/Configuration/CameraSettings.h"
#include "Representations/Infrastructure/CameraInfo.h"

#include <linux/i2c.h>
#include <linux/i2c-dev.h>

/**
 * @class NaoCamera
 * Interface to a camera of the NAO
 */
class NaoCamera
{
public:
  bool resetRequired = false;

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
            const CameraSettings::Collection& settings,
            const AutoExposureWeightTable::Table& autoExposureWeightTable);

  ~NaoCamera();

  void changeResolution(int width, int height);

  /**
   * The method blocks till a new image arrives.
   * @param timeout The maximum waiting time
   * @return true (except a not manageable exception occurs)
   */
  bool captureNew(int timeout);

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
  unsigned long long getTimestamp() const;

  /**
   * Returns the frame rate used by the camera
   * @return The frame rate in frames per second
   */
  float getFrameRate() const;

  /**
   * Set frame rate in frames per 10 seconds.
   */
  bool setFrameRate(unsigned numerator = 1, unsigned denominator = 30);

  CameraSettings::Collection getCameraSettingsCollection() const;
  AutoExposureWeightTable::Table getAutoExposureWeightTable() const;

  void setSettings(const CameraSettings::Collection& settings, const AutoExposureWeightTable::Table& autoExposureWeightTable);

  /**
   * Unconditional write of the camera settings
   */
  void writeCameraSettings();

  void readCameraSettings();

  void doAutoWhiteBalance();

  /**
   * Set a camera register.
   * @param address The address of the register.
   * @param value The value to set. Although the command could set two bytes,
   *              actual tests showed that only a single byte is set.
   * @return Was the call successful?
   */
  bool setRegister(unsigned short address, unsigned short value) const;

  /**
   * Get the current value of a register.
   * @param address The address of the register.
   * @param value The value returned. Although the command could return two bytes,
   *              actual tests showed that only a single byte is read.
   * @return Could the register be read?
   */
  bool getRegister(unsigned short address, unsigned short& value) const;

  static void resetCamera();
  static bool getI2CDeviceNumber(int& i2cDeviceNumber);
  static bool openI2CDevice(int i2cDeviceNumber, int& fileDescriptor);
  static bool i2cReadWriteAccess(int fileDescriptor, unsigned char readWrite, unsigned char command, unsigned char size, i2c_smbus_data& data);
  static bool i2cWriteBlockData(int fileDescriptor, unsigned char deviceAddress, unsigned char dataAddress, std::vector<unsigned char> data);
  static bool i2cReadByteData(int fileDescriptor, unsigned char deviceAddress, unsigned char dataAddress, unsigned char& res);

private:
  class V4L2Setting
  {
  public:
    int command = 0;
    int value = 0;

    CameraSettings::Collection::CameraSetting notChangableWhile = CameraSettings::Collection::numOfCameraSettings;
    int invert = true;

    V4L2Setting() = default;
    V4L2Setting(int command, int value, int min, int max, CameraSettings::Collection::CameraSetting notChangableWhile = CameraSettings::Collection::numOfCameraSettings, int invert = 0);

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
    std::array<V4L2Setting, CameraSettings::Collection::numOfCameraSettings> settings;
    static constexpr size_t sizeOfAutoExposureWeightTable = AutoExposureWeightTable::width * AutoExposureWeightTable::height;
    std::array<V4L2Setting, sizeOfAutoExposureWeightTable> autoExposureWeightTable;

    CameraSettingsCollection();

    bool operator==(const CameraSettingsCollection& other) const;
    bool operator!=(const CameraSettingsCollection& other) const;
  };

  struct CameraSettingsSpecial
  {
    V4L2Setting verticalFlip;
    V4L2Setting horizontalFlip;

    CameraSettingsSpecial();
  };

  CameraInfo::Camera camera; /**< The camera accessed by this driver. */
  CameraSettingsCollection settings; /**< The camera control settings. */
  CameraSettingsCollection appliedSettings; /**< The camera settings that are known to be applied. */
  CameraSettingsSpecial specialSettings; /**< Special settings that are only set */

  static constexpr unsigned frameBufferCount = 3; /**< Amount of available frame buffers. */

  unsigned WIDTH; /**< The width of the yuv 422 image */
  unsigned HEIGHT; /**< The height of the yuv 422 image */
  int fd; /**< The file descriptor for the video device. */
  void* mem[frameBufferCount]; /**< Frame buffer addresses. */
  int memLength[frameBufferCount]; /**< The length of each frame buffer. */
  struct v4l2_buffer* buf = nullptr; /**< Reusable parameter struct for some ioctl calls. */
  struct v4l2_buffer* currentBuf = nullptr; /**< The last dequeued frame buffer. */
  bool first = true; /**< First image grabbed? */
  unsigned long long timestamp = 0; /**< Timestamp of the last captured image in microseconds. */

  bool checkSettingsAvailability();

  bool checkV4L2Setting(V4L2Setting& setting) const;

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

  bool assertCameraSetting(CameraSettings::Collection::CameraSetting setting);

  bool setImageFormat();

  bool mapBuffers();
  void unmapBuffers();
  bool queueBuffers();

  bool startCapturing();
  bool stopCapturing();

  /**
   * Query a UVC control.
   * @param set Set value (instead of reading it)?
   * @param control The number of the control.
   * @param value Address of the value to set or get.
   * @param size Size of the value to set or get in bytes.
   * @return Did the call succeed?
   */
  bool queryXU(bool set, unsigned char control, void* value, unsigned short size) const;

  /**
   * Set a UVC control.
   * @param control The number of the control to set.
   * @param value The value to set.
   * @return Did the call succeed?
   */
  template<typename T> bool setXU(unsigned char control, const T& value) const
  {
    return queryXU(true, control, const_cast<T*>(&value), static_cast<unsigned short>(sizeof(T)));
  }

  /**
   * Get a UVC control.
   * @param control The number of the control to get.
   * @param value The value that is filled with the result.
   * @return Did the call succeed?
   */
  template<typename T> bool getXU(unsigned char control, const T& value) const
  {
    return queryXU(false, control, const_cast<T*>(&value), static_cast<unsigned short>(sizeof(T)));
  }
};
