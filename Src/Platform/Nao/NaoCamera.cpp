/**
 * @file Platform/Nao/NaoCamera.cpp
 * Interface to a camera of the NAO.
 * @author Colin Graf
 * @author Thomas Röfer
 */

#include <cstring>
#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <cerrno>
#include <poll.h>
#include <linux/videodev2.h>
#include <linux/uvcvideo.h>
#include <linux/usb/video.h>

#include "NaoCamera.h"
#include "Platform/BHAssert.h"
#include "Platform/Time.h"
#include "Tools/Debugging/Debugging.h"

NaoCamera::NaoCamera(const char* device, CameraInfo::Camera camera, int width, int height, bool flip,
                     const CameraSettings::Collection& settings,
                     const AutoExposureWeightTable::Table& autoExposureWeightTable) :
  camera(camera),
  WIDTH(width),
  HEIGHT(height)
{
  resetRequired = (fd = open(device, O_RDWR | O_NONBLOCK)) == -1;
  usleep(30000); // Experimental: Add delay between opening and using camera device
  resetRequired = resetRequired || !setImageFormat() || !setFrameRate(1, 30)  || !mapBuffers() || !queueBuffers() || !checkSettingsAvailability();
  if(!resetRequired)
  {
    specialSettings.horizontalFlip.value = flip ? 1 : 0;
    setControlSetting(specialSettings.horizontalFlip);
    specialSettings.verticalFlip.value = flip ? 1 : 0;
    setControlSetting(specialSettings.verticalFlip);
    setSettings(settings, autoExposureWeightTable);
    writeCameraSettings();
    readCameraSettings();
    resetRequired = !startCapturing();
  }
  if(resetRequired)
    OUTPUT_ERROR("Setting up NaoCamera failed!");
}

NaoCamera::~NaoCamera()
{
  stopCapturing();
  unmapBuffers();

  close(fd);
}

bool NaoCamera::captureNew(NaoCamera& cam1, NaoCamera& cam2, int timeout)
{
  NaoCamera* cams[2] = {&cam1, &cam2};

  ASSERT(cam1.currentBuf == nullptr);
  ASSERT(cam2.currentBuf == nullptr);

  pollfd pollfds[2] =
  {
    {cams[0]->fd, POLLIN | POLLPRI, 0},
    {cams[1]->fd, POLLIN | POLLPRI, 0},
  };
  int polled = poll(pollfds, 2, timeout);
  if(polled < 0)
  {
    OUTPUT_ERROR("Cannot poll for camera images. Reason: " << strerror(errno));
    FAIL("Cannot poll for camera images. Reason: " << strerror(errno) << ".");
    return false;
  }
  else if(polled == 0)
  {
    OUTPUT_ERROR("Reading images from the cameras timed out after " << timeout << " ms. Terminating.");
    return false;
  }

  for(int i = 0; i < 2; ++i)
  {
    if(pollfds[i].revents & POLLIN)
    {
      v4l2_buffer lastBuf;
      bool first = true;
      while(ioctl(cams[i]->fd, VIDIOC_DQBUF, cams[i]->buf) == 0)
      {
        if(first)
          first = false;
        else if(ioctl(cams[i]->fd, VIDIOC_QBUF, &lastBuf) != 0)
          return false;
        lastBuf = *cams[i]->buf;
      }
      if(errno != EAGAIN)
      {
        OUTPUT_ERROR("VIDIOC_DQBUF failed: " << strerror(errno));
        return false;
      }
      else
      {
        cams[i]->currentBuf = cams[i]->buf;
        cams[i]->timestamp = static_cast<unsigned long long>(cams[i]->currentBuf->timestamp.tv_sec) * 1000000ll + cams[i]->currentBuf->timestamp.tv_usec;

        if(cams[i]->first)
        {
          cams[i]->first = false;
          printf("%s camera is working\n", TypeRegistry::getEnumName(cams[i]->camera));
        }
      }
    }
    else if(pollfds[i].revents)
    {
      OUTPUT_ERROR("strange poll results: " << pollfds[i].revents);
      return false;
    }
  }

  return true;
}

bool NaoCamera::captureNew(int timeout)
{
  // requeue the buffer of the last captured image which is obsolete now
  if(currentBuf)
  {
    BH_TRACE;
    if(ioctl(fd, VIDIOC_QBUF, currentBuf) == -1)
      return false;
  }
  BH_TRACE;

  pollfd pollfd = {fd, POLLIN | POLLPRI, 0};
  int polled = poll(&pollfd, 1, timeout);
  if(polled < 0)
  {
    OUTPUT_ERROR(TypeRegistry::getEnumName(camera) << "camera : Cannot poll. Reason: " << strerror(errno));
    FAIL(TypeRegistry::getEnumName(camera) << "camera : Cannot poll. Reason: " << strerror(errno) << ".");
  }
  else if(polled == 0)
  {
    OUTPUT_ERROR(TypeRegistry::getEnumName(camera) << "camera : " << timeout << " ms passed and there's still no image to read from the camera. Terminating.");
    return false;
  }
  else if(pollfd.revents & (POLLERR | POLLNVAL))
  {
    OUTPUT_ERROR(TypeRegistry::getEnumName(camera) << "camera : Polling failed.");
    return false;
  }

  // dequeue a frame buffer (this call blocks when there is no new image available) */
  v4l2_buffer lastBuf;
  bool firstAttempt = true;
  while(ioctl(fd, VIDIOC_DQBUF, buf) == 0)
  {
    if(firstAttempt)
      firstAttempt = false;
    else if(ioctl(fd, VIDIOC_QBUF, &lastBuf) != 0)
      return false;
    lastBuf = *buf;
  }

  if(errno != EAGAIN)
  {
    OUTPUT_ERROR("VIDIOC_DQBUF failed: " << strerror(errno));
    return false;
  }
  else
  {
    BH_TRACE;
    currentBuf = buf;
    timestamp = static_cast<unsigned long long>(currentBuf->timestamp.tv_sec) * 1000000ll + currentBuf->timestamp.tv_usec;

    if(first)
    {
      first = false;
      printf("%s camera is working\n", TypeRegistry::getEnumName(camera));
    }

    return true;
  }
}

void NaoCamera::releaseImage()
{
  if(currentBuf)
  {
    if(ioctl(fd, VIDIOC_QBUF, currentBuf) == -1)
    {
      OUTPUT_ERROR("Releasing image failed!");
      resetRequired = true;
    }
    currentBuf = nullptr;
  }
}

const unsigned char* NaoCamera::getImage() const
{
  return currentBuf ? static_cast<unsigned char*>(mem[currentBuf->index]) : nullptr;
}

bool NaoCamera::hasImage()
{
  return !!currentBuf;
}

unsigned long long NaoCamera::getTimestamp() const
{
  if(!currentBuf)
    return 0;
  ASSERT(currentBuf);
  return timestamp;
}

float NaoCamera::getFrameRate() const
{
  return 1.f / 30.f;
}

bool NaoCamera::setFrameRate(unsigned numerator, unsigned denominator)
{
  // set frame rate
  v4l2_streamparm fps;
  memset(&fps, 0, sizeof(v4l2_streamparm));
  fps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if(ioctl(fd, VIDIOC_G_PARM, &fps))
    return false;
  fps.parm.capture.timeperframe.numerator = numerator;
  fps.parm.capture.timeperframe.denominator = denominator;
  return ioctl(fd, VIDIOC_S_PARM, &fps) != -1;
}

void NaoCamera::resetCamera()
{
  BH_TRACE_MSG("reset camera");
  usleep(100000);
  int i2cDeviceNumber = -1, fileDescriptor = -1;
  if(getI2CDeviceNumber(i2cDeviceNumber) && openI2CDevice(i2cDeviceNumber, fileDescriptor))
  {
    unsigned char config;
    if(i2cReadByteData(fileDescriptor, 0x41, 0x3, config) && (config & 0xc) != 0)
    {
      i2cWriteBlockData(fileDescriptor, 0x41, 0x1, {0x0});
      i2cWriteBlockData(fileDescriptor, 0x41, 0x3, {0xf3});
    }
    i2cWriteBlockData(fileDescriptor, 0x41, 0x1, {0x0});
    i2cWriteBlockData(fileDescriptor, 0x41, 0x1, {0xc});
  }
  if(fileDescriptor != -1) close(fileDescriptor);
  sleep(2);
}

bool NaoCamera::getI2CDeviceNumber(int& i2cDeviceNumber)
{
  char result[4096];
  long length = ::readlink("/dev/i2c-head", result, sizeof(result)-1);
  if(length >= 0)
  {
    result[length] = '\0';
    unsigned long pos = std::string(result).find_last_of('-');
    i2cDeviceNumber = std::atoi(std::string(result).substr(pos+1).c_str());
    return true;
  }
  return false;
}

bool NaoCamera::openI2CDevice(int i2cDeviceNumber, int& fileDescriptor)
{
  return (fileDescriptor = open(("/dev/i2c-" + std::to_string(i2cDeviceNumber)).c_str(), O_RDWR | O_NONBLOCK)) >= 0;
}

bool NaoCamera::i2cReadWriteAccess(int fileDescriptor, unsigned char readWrite, unsigned char command, unsigned char size, i2c_smbus_data& data)
{
  struct i2c_smbus_ioctl_data ioctlData;
  ioctlData.read_write = readWrite;
  ioctlData.command = command;
  ioctlData.size = size;
  ioctlData.data = &data;
  return ioctl(fileDescriptor, I2C_SMBUS, &ioctlData) >= 0;
}

bool NaoCamera::i2cWriteBlockData(int fileDescriptor, unsigned char deviceAddress, unsigned char dataAddress, std::vector<unsigned char> data)
{
  if(ioctl(fileDescriptor, I2C_SLAVE, deviceAddress) < 0)
    return false;

  i2c_smbus_data writeData;
  writeData.block[0] = static_cast<unsigned char>(std::min(32, static_cast<int>(data.size())));
  for(int i = 1; i <= writeData.block[0]; ++i)
    writeData.block[i] = data[i-1];
  return i2cReadWriteAccess(fileDescriptor, I2C_SMBUS_WRITE, dataAddress, I2C_SMBUS_I2C_BLOCK_DATA, writeData);;
}

bool NaoCamera::i2cReadByteData(int fileDescriptor, unsigned char deviceAddress, unsigned char dataAddress, unsigned char& res)
{
  if(ioctl(fileDescriptor, I2C_SLAVE, deviceAddress) < 0)
    return false;

  i2c_smbus_data readData;
  if(!i2cReadWriteAccess(fileDescriptor, I2C_SMBUS_READ, dataAddress, I2C_SMBUS_BYTE_DATA, readData))
    return false;
  res = readData.block[0];
  return true;
}

CameraSettings::Collection NaoCamera::getCameraSettingsCollection() const
{
  CameraSettings::Collection collection;
  FOREACH_ENUM(CameraSettings::Collection::CameraSetting, setting)
  {
    collection.settings[setting] = appliedSettings.settings[setting].value;
  }
  return collection;
}

AutoExposureWeightTable::Table NaoCamera::getAutoExposureWeightTable() const
{
  AutoExposureWeightTable::Table table;
  int i = 0;
  for(int y = 0; y < table.rows(); ++y)
    for(int x = 0; x < table.cols(); ++x)
      table(y, x) = static_cast<uint8_t>(appliedSettings.autoExposureWeightTable[i++].value);
  return table;
}

void NaoCamera::setSettings(const CameraSettings::Collection& cameraSettingCollection,
                            const AutoExposureWeightTable::Table& autoExposureWeightTable)
{
  FOREACH_ENUM(CameraSettings::Collection::CameraSetting, setting)
  {
    settings.settings[setting].value = cameraSettingCollection.settings[setting];
    settings.settings[setting].enforceBounds();
  }

  int i = 0;
  for(int y = 0; y < autoExposureWeightTable.rows(); ++y)
    for(int x = 0; x < autoExposureWeightTable.cols(); ++x)
    {
      settings.autoExposureWeightTable[i].value = autoExposureWeightTable(y, x);
      settings.autoExposureWeightTable[i++].enforceBounds();
    }
}

void NaoCamera::writeCameraSettings()
{
  const auto oldSettings = appliedSettings.settings;
  FOREACH_ENUM(CameraSettings::Collection::CameraSetting, settingName)
  {
    V4L2Setting& currentSetting = settings.settings[settingName];
    V4L2Setting& appliedSetting = appliedSettings.settings[settingName];

    if(timestamp == 0)
    {
      if(currentSetting.notChangableWhile != CameraSettings::Collection::numOfCameraSettings &&
         settings.settings[currentSetting.notChangableWhile].value ^ currentSetting.invert)
        continue;
    }
    else
    {
      if(currentSetting.notChangableWhile != CameraSettings::Collection::numOfCameraSettings)
      {
        const bool nowActive = settings.settings[currentSetting.notChangableWhile].value ^ currentSetting.invert;
        const bool oldActive = oldSettings[currentSetting.notChangableWhile].value ^ currentSetting.invert;
        if(nowActive || (!oldActive && currentSetting.value == appliedSetting.value))
          continue;
      }
      else if(currentSetting.value == appliedSetting.value)
        continue;
    }

    if(!setControlSetting(currentSetting))
      OUTPUT_WARNING("NaoCamera: Setting camera control " << TypeRegistry::getEnumName(settingName) << " failed for value: " << currentSetting.value);
    else
    {
      appliedSetting.value = currentSetting.value;
#ifdef _DEBUG
      assertCameraSetting(settingName);
#endif
    }
  }

  for(size_t i = 0; i < CameraSettingsCollection::sizeOfAutoExposureWeightTable; ++i)
  {
    V4L2Setting& currentTableEntry = settings.autoExposureWeightTable[i];
    V4L2Setting& appliedTableEntry = appliedSettings.autoExposureWeightTable[i];
    if(timestamp != 0 && currentTableEntry.value == appliedTableEntry.value)
      continue;

    // Set all weights at once
    unsigned char value[17] =
    {
      1, 0, 0, 0, 0,
      static_cast<unsigned char>(WIDTH >> 8), static_cast<unsigned char>(WIDTH & 8),
      static_cast<unsigned char>(HEIGHT >> 8), static_cast<unsigned char>(HEIGHT & 8)
    };

    // Set weights in value to set
    for(size_t i = 0; i < CameraSettingsCollection::sizeOfAutoExposureWeightTable; ++i)
    {
      V4L2Setting& currentTableEntry = settings.autoExposureWeightTable[i];
      currentTableEntry.enforceBounds();
      ASSERT(9 + i / 2 < sizeof(value));
      value[9 + i / 2] |= currentTableEntry.value << (i & 1) * 4;
    }

    // Use extension unit to set exposure weight table
    if(setXU(9, value))
      appliedSettings.autoExposureWeightTable = settings.autoExposureWeightTable;
    else
      OUTPUT_ERROR("NaoCamera: setting auto exposure weight table failed");

    break; // All weights were set, no need to search for more differences
  }
}

void NaoCamera::readCameraSettings()
{
  for(V4L2Setting& setting : appliedSettings.settings)
    getControlSetting(setting);
  unsigned char value[17];
  if(getXU(9, value))
    for(size_t i = 0; i < CameraSettingsCollection::sizeOfAutoExposureWeightTable; ++i)
      settings.autoExposureWeightTable[i].value = (value[9 + i / 2] >> (i & 1) * 4) & 0x0f;
  else
    OUTPUT_ERROR("NaoCamera: reading auto exposure weight table failed");
}

void NaoCamera::doAutoWhiteBalance()
{
  appliedSettings.settings[CameraSettings::Collection::autoWhiteBalance].value = 1;
  setControlSetting(appliedSettings.settings[CameraSettings::Collection::autoWhiteBalance]);
  usleep(100);
  unsigned short hiRedGain, loRedGain, hiGreenGain, loGreenGain, hiBlueGain, loBlueGain;
  if(getRegister(0x3400, hiRedGain) && getRegister(0x3401, loRedGain)
     && getRegister(0x3402, hiGreenGain) && getRegister(0x3403, loGreenGain)
     && getRegister(0x3404, hiBlueGain) && getRegister(0x3405, loBlueGain))
  {
    appliedSettings.settings[CameraSettings::Collection::redGain].value
      = settings.settings[CameraSettings::Collection::redGain].value = hiRedGain << 8 | loRedGain;
    appliedSettings.settings[CameraSettings::Collection::greenGain].value
      = settings.settings[CameraSettings::Collection::greenGain].value = hiGreenGain << 8 | loGreenGain;
    appliedSettings.settings[CameraSettings::Collection::blueGain].value
      = settings.settings[CameraSettings::Collection::blueGain].value = hiBlueGain << 8 | loBlueGain;
    OUTPUT_TEXT("New white balance is RGB = (" << settings.settings[CameraSettings::Collection::redGain].value << ", "
                                               << settings.settings[CameraSettings::Collection::greenGain].value << ", "
                                               << settings.settings[CameraSettings::Collection::blueGain].value << ")");
    appliedSettings.settings[CameraSettings::Collection::autoWhiteBalance].value = 0;
    setControlSetting(appliedSettings.settings[CameraSettings::Collection::autoWhiteBalance]);
  }
}

bool NaoCamera::checkSettingsAvailability()
{
  bool status = true;
  for(V4L2Setting& setting : appliedSettings.settings)
    status = status && checkV4L2Setting(setting);
  return status;
}

bool NaoCamera::checkV4L2Setting(V4L2Setting& setting) const
{
  if(setting.command < 0)
    return true;

  v4l2_queryctrl queryctrl;
  queryctrl.id = setting.command;
  if(ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl) < 0)
  {
    OUTPUT_ERROR("ioctl to query setting failed for camera setting " << setting.command);
    return false;
  }
  if(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
  {
    FAIL("Camera setting " << setting.command << " is disabled");
  }
  if(queryctrl.type != V4L2_CTRL_TYPE_BOOLEAN && queryctrl.type != V4L2_CTRL_TYPE_INTEGER && queryctrl.type != V4L2_CTRL_TYPE_MENU)
  {
    FAIL("Camera setting " << setting.command <<  "is unsupported ");
  }
  setting.setCameraBounds(queryctrl.minimum, queryctrl.maximum);
  return true;
}

bool NaoCamera::getControlSetting(V4L2Setting& setting)
{
  if(setting.command >= 0)
  {
    v4l2_control control_s;
    control_s.id = setting.command;
    if(ioctl(fd, VIDIOC_G_CTRL, &control_s) < 0)
    {
      OUTPUT_ERROR("NaoCamera: Retrieving camera setting " << setting.command << " failed");
      return false;
    }
    setting.value = control_s.value;
  }
  return true;
}

bool NaoCamera::setControlSetting(V4L2Setting& setting)
{
  setting.enforceBounds();
  if(setting.command < 0) // command for extension unit?
  {
    if((-setting.command) >> 16 == 14) // set register
    {
      unsigned short address = static_cast<unsigned short>(-setting.command & 0xffff);
      if(!setRegister(address, static_cast<unsigned char>(setting.value >> 8)))
      {
        OUTPUT_ERROR("NaoCamera: Setting register " << address << " failed.");
        return false;
      }
      if(!setRegister(address + 1, static_cast<unsigned char>(setting.value & 0xff)))
      {
        OUTPUT_ERROR("NaoCamera: Setting register " << address << " failed.");
        return false;
      }
    }
    else if(!setXU(static_cast<unsigned char>((-setting.command) >> 16), static_cast<unsigned short>(setting.value)))
    {
      OUTPUT_ERROR("NaoCamera: Extension unit selector " << ((-setting.command) >> 16) << " failed.");
      return false;
    }
  }
  else
  {
    v4l2_control control_s;
    control_s.id = setting.command;
    control_s.value = setting.value;

    const int ret = ioctl(fd, VIDIOC_S_CTRL, &control_s);
    if(ret < 0)
    {
      OUTPUT_ERROR("NaoCamera: Setting value ID: " << setting.command << " failed. VIDIOC_S_CTRL return value is " << ret);
      return false;
    }
  }
  return true;
}

bool NaoCamera::assertCameraSetting(CameraSettings::Collection::CameraSetting setting)
{
  appliedSettings.settings[setting] = settings.settings[setting];
  if(settings.settings[setting].command < 0)
    return true;

  const int oldValue = appliedSettings.settings[setting].value;
  if(getControlSetting(appliedSettings.settings[setting]))
  {
    if(appliedSettings.settings[setting].value == oldValue)
      return true;
    else
    {
      OUTPUT_WARNING("Value for command " << appliedSettings.settings[setting].command << " (" << TypeRegistry::getEnumName(setting) << ") is "
                   << appliedSettings.settings[setting].value << " but should be " << oldValue << ".");
    }
  }
  return false;
}

void NaoCamera::changeResolution(int width, int height)
{
  BH_TRACE_MSG("change resolution");

  resetRequired = resetRequired || !stopCapturing();
  unmapBuffers();

  WIDTH = width;
  HEIGHT = height;

  resetRequired = resetRequired || !setImageFormat() || !mapBuffers() || !queueBuffers() || !startCapturing();
  if(resetRequired)
    OUTPUT_ERROR("Changing camera resolution failed!");
}

bool NaoCamera::setImageFormat()
{
  // set format
  v4l2_format fmt;
  memset(&fmt, 0, sizeof(v4l2_format));
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = WIDTH;
  fmt.fmt.pix.height = HEIGHT;
  fmt.fmt.pix.sizeimage = WIDTH * HEIGHT * 2;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  if(ioctl(fd, VIDIOC_S_FMT, &fmt))
    return false;

  ASSERT(fmt.fmt.pix.sizeimage == WIDTH * HEIGHT * 2);
  return true;
}

bool NaoCamera::mapBuffers()
{
  // request buffers
  v4l2_requestbuffers rb;
  memset(&rb, 0, sizeof(v4l2_requestbuffers));
  rb.count = frameBufferCount;
  rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  rb.memory = V4L2_MEMORY_MMAP;
  if(ioctl(fd, VIDIOC_REQBUFS, &rb) == -1)
    return false;
  ASSERT(rb.count == frameBufferCount);

  // map or prepare the buffers
  ASSERT(!buf);
  buf = static_cast<v4l2_buffer*>(calloc(1, sizeof(v4l2_buffer)));
  buf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf->memory = V4L2_MEMORY_MMAP;
  for(unsigned i = 0; i < frameBufferCount; ++i)
  {
    buf->index = i;
    if(ioctl(fd, VIDIOC_QUERYBUF, buf) == -1)
      return false;
    memLength[i] = buf->length;
    mem[i] = mmap(0, buf->length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf->m.offset);
    ASSERT(mem[i] != MAP_FAILED);
  }
  return true;
}

void NaoCamera::unmapBuffers()
{
  // unmap buffers
  for(unsigned i = 0; i < frameBufferCount; ++i)
  {
    munmap(mem[i], memLength[i]);
    mem[i] = nullptr;
    memLength[i] = 0;
  }

  free(buf);
  currentBuf = buf = nullptr;
}

bool NaoCamera::queueBuffers()
{
  for(unsigned i = 0; i < frameBufferCount; ++i)
  {
    buf->index = i;
    if(ioctl(fd, VIDIOC_QBUF, buf) == -1)
      return false;
  }
  return true;
}

bool NaoCamera::startCapturing()
{
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  return ioctl(fd, VIDIOC_STREAMON, &type) != -1;
}

bool NaoCamera::stopCapturing()
{
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  return ioctl(fd, VIDIOC_STREAMOFF, &type) != -1;
}

bool NaoCamera::queryXU(bool set, unsigned char control, void* value, unsigned short size) const
{
  uvc_xu_control_query xu;
  xu.unit = 3;
  xu.selector = control;
  xu.query = set ? UVC_SET_CUR : UVC_GET_CUR;
  xu.size = size;
  xu.data = static_cast<__u8*>(value);
  return !ioctl(fd, UVCIOC_CTRL_QUERY, &xu);
}

bool NaoCamera::setRegister(unsigned short address, unsigned short value) const
{
  unsigned char bytes[5];
  bytes[0] = 1;
  bytes[1] = static_cast<unsigned char>(address >> 8);
  bytes[2] = static_cast<unsigned char>(address & 0xff);
  bytes[3] = static_cast<unsigned char>(value >> 8);
  bytes[4] = static_cast<unsigned char>(value & 0xff);
  return setXU(14, bytes);
}

bool NaoCamera::getRegister(unsigned short address, unsigned short& value) const
{
  unsigned char bytes[5] = {0, static_cast<unsigned char>(address >> 8),
                            static_cast<unsigned char>(address & 0xff)};
  if(setXU(14, bytes))
  {
    usleep(100000);
    if(getXU(14, bytes))
    {
      value = static_cast<unsigned short>(bytes[3] << 8 | bytes[4]);
      return true;
    }
  }
  return false;
}

NaoCamera::V4L2Setting::V4L2Setting(int command, int value, int min, int max, CameraSettings::Collection::CameraSetting notChangableWhile, int invert) :
  command(command), value(value), notChangableWhile(notChangableWhile), invert(invert), min(min), max(max)
{
  ASSERT(min <= max);
}

bool NaoCamera::V4L2Setting::operator==(const V4L2Setting& other) const
{
  return command == other.command && value == other.value;
}

bool NaoCamera::V4L2Setting::operator!=(const V4L2Setting& other) const
{
  return !(*this == other);
}

void NaoCamera::V4L2Setting::enforceBounds()
{
  if(value < min)
    value = min;
  else if(value > max)
    value = max;
}

void NaoCamera::V4L2Setting::setCameraBounds(int camMin, int camMax)
{
  min = std::max(camMin, min);
  max = std::min(camMax, max);
}

NaoCamera::CameraSettingsCollection::CameraSettingsCollection()
{
  settings[CameraSettings::Collection::autoExposure] = V4L2Setting(V4L2_CID_EXPOSURE_AUTO, -1000, 0, 3);
  settings[CameraSettings::Collection::autoExposureBrightness] = V4L2Setting(V4L2_CID_BRIGHTNESS, -1000, -255, 255);
  settings[CameraSettings::Collection::exposure] = V4L2Setting(V4L2_CID_EXPOSURE_ABSOLUTE, -1000, 0, 1048575, CameraSettings::Collection::autoExposure, 1);
  settings[CameraSettings::Collection::gain] = V4L2Setting(V4L2_CID_GAIN, -1000, 0, 1023,
      CameraSettings::Collection::autoExposure, 1);
  settings[CameraSettings::Collection::autoWhiteBalance] = V4L2Setting(V4L2_CID_AUTO_WHITE_BALANCE, -1000, 0, 1);
  settings[CameraSettings::Collection::autoFocus] = V4L2Setting(V4L2_CID_FOCUS_AUTO, -1000, 0, 1);
  settings[CameraSettings::Collection::focus] = V4L2Setting(V4L2_CID_FOCUS_ABSOLUTE, -1000, 0, 250, CameraSettings::Collection::autoFocus);
  settings[CameraSettings::Collection::autoHue] = V4L2Setting(V4L2_CID_HUE_AUTO, -1000, 0, 1);
  settings[CameraSettings::Collection::hue] = V4L2Setting(V4L2_CID_HUE, -1000, -180, 180, CameraSettings::Collection::autoHue);
  settings[CameraSettings::Collection::saturation] = V4L2Setting(V4L2_CID_SATURATION, -1000, 0, 255);
  settings[CameraSettings::Collection::contrast] = V4L2Setting(V4L2_CID_CONTRAST, -1000, 0, 255);
  settings[CameraSettings::Collection::sharpness] = V4L2Setting(V4L2_CID_SHARPNESS, -1000, 0, 9);
  settings[CameraSettings::Collection::redGain] = V4L2Setting(-0xe3400, -1000, 0, 4095, CameraSettings::Collection::autoWhiteBalance);
  settings[CameraSettings::Collection::greenGain] = V4L2Setting(-0xe3402, -1000, 0, 4095, CameraSettings::Collection::autoWhiteBalance);
  settings[CameraSettings::Collection::blueGain] = V4L2Setting(-0xe3404, -1000, 0, 4095, CameraSettings::Collection::autoWhiteBalance);
  for(size_t i = 0; i < sizeOfAutoExposureWeightTable; ++i)
    autoExposureWeightTable[i] = V4L2Setting(0, -1000, 0, AutoExposureWeightTable::maxWeight);
}

bool NaoCamera::CameraSettingsCollection::operator==(const CameraSettingsCollection& other) const
{
  FOREACH_ENUM(CameraSettings::Collection::CameraSetting, setting)
  {
    if(settings[setting] != other.settings[setting])
      return false;
  }
  for(size_t i = 0; i < autoExposureWeightTable.size(); ++i)
  {
    if(autoExposureWeightTable[i] != other.autoExposureWeightTable[i])
      return false;
  }
  return true;
}

bool NaoCamera::CameraSettingsCollection::operator!=(const CameraSettingsCollection& other) const
{
  return !(*this == other);
}

NaoCamera::CameraSettingsSpecial::CameraSettingsSpecial() :
  verticalFlip(-0xc0000, 0, 0, 1),
  horizontalFlip(-0xd0000, 0, 0, 1)
{}
