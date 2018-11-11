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
#include <iostream>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <cerrno>
#include <poll.h>
#include <linux/videodev2.h>

#include "NaoCamera.h"
#include "Platform/BHAssert.h"
#include "Platform/Time.h"
#include "Tools/Debugging/Debugging.h"

NaoCamera::NaoCamera(const char* device, CameraInfo::Camera camera, int width, int height, bool flip,
                     const CameraSettings::CameraSettingsCollection& settings, const Matrix5uc& autoExposureWeightTable) :
  camera(camera),
  WIDTH(width),
  HEIGHT(height)
{
  VERIFY((fd = open(device, O_RDWR | O_NONBLOCK)) != -1);

  mapBuffers();
  queueBuffers();

  setImageFormat();
  setFrameRate(1, 30);

  checkSettingsAvailability();

  specialSettings.horizontalFlip.value = flip ? 1 : 0;
  setControlSetting(specialSettings.horizontalFlip);
  specialSettings.verticalFlip.value = flip ? 1 : 0;
  setControlSetting(specialSettings.verticalFlip);
  setSettings(settings, autoExposureWeightTable);
  writeCameraSettings();
  readCameraSettings();

  startCapturing();
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
        else
          VERIFY(ioctl(cams[i]->fd, VIDIOC_QBUF, &lastBuf) == 0);
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
        cams[i]->timeStamp = static_cast<unsigned long long>(cams[i]->currentBuf->timestamp.tv_sec) * 1000000ll + cams[i]->currentBuf->timestamp.tv_usec;

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

bool NaoCamera::captureNew()
{
  // requeue the buffer of the last captured image which is obsolete now
  if(currentBuf)
  {
    BH_TRACE;
    VERIFY(ioctl(fd, VIDIOC_QBUF, currentBuf) != -1);
  }
  BH_TRACE;

  pollfd pollfd = {fd, POLLIN | POLLPRI, 0};
  int polled = poll(&pollfd, 1, 200); // Fail after missing 6 frames (200ms)
  if(polled < 0)
  {
    OUTPUT_ERROR(TypeRegistry::getEnumName(camera) << "camera : Cannot poll. Reason: " << strerror(errno));
    FAIL(TypeRegistry::getEnumName(camera) << "camera : Cannot poll. Reason: " << strerror(errno) << ".");
  }
  else if(polled == 0)
  {
    OUTPUT_ERROR(TypeRegistry::getEnumName(camera) << "camera : 200 ms passed and there's still no image to read from the camera. Terminating.");
    return false;
  }
  else if(pollfd.revents & (POLLERR | POLLNVAL))
  {
    OUTPUT_ERROR(TypeRegistry::getEnumName(camera) << "camera : Polling failed.");
    return false;
  }
  // dequeue a frame buffer (this call blocks when there is no new image available) */
  VERIFY(ioctl(fd, VIDIOC_DQBUF, buf) != -1);
  BH_TRACE;
  currentBuf = buf;
  timeStamp = static_cast<unsigned long long>(currentBuf->timestamp.tv_sec) * 1000000ll + currentBuf->timestamp.tv_usec;

  if(first)
  {
    first = false;
    printf("%s camera is working\n", TypeRegistry::getEnumName(camera));
  }

  return true;
}

void NaoCamera::releaseImage()
{
  if(currentBuf)
  {
    VERIFY(ioctl(fd, VIDIOC_QBUF, currentBuf) != -1);
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

unsigned long long NaoCamera::getTimeStamp() const
{
  if(!currentBuf)
    return 0;
  ASSERT(currentBuf);
  return timeStamp;
}

float NaoCamera::getFrameRate() const
{
  return 1.f / 30.f;
}

void NaoCamera::setFrameRate(unsigned numerator, unsigned denominator)
{
  // set frame rate
  v4l2_streamparm fps;
  memset(&fps, 0, sizeof(v4l2_streamparm));
  fps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  VERIFY(!ioctl(fd, VIDIOC_G_PARM, &fps));
  fps.parm.capture.timeperframe.numerator = numerator;
  fps.parm.capture.timeperframe.denominator = denominator;
  VERIFY(ioctl(fd, VIDIOC_S_PARM, &fps) != -1);
}

CameraSettings::CameraSettingsCollection NaoCamera::getCameraSettingsCollection() const
{
  CameraSettings::CameraSettingsCollection collection;
  FOREACH_ENUM(CameraSettings::CameraSetting, setting)
  {
    collection.settings[setting] = appliedSettings.settings[setting].value;
  }
  return collection;
}

Matrix5uc NaoCamera::getAutoExposureWeightTable() const
{
  Matrix5uc table;
  for(int y = 0; y < 5; ++y)
  {
    for(int x = 0; x < 5; ++x)
    {
      table(y, x) = static_cast<uint8_t>(appliedSettings.autoExposureWeightTable[5 * y + x].value);
    }
  }
  return table;
}

void NaoCamera::setSettings(const CameraSettings::CameraSettingsCollection& cameraSettingCollection, const Matrix5uc& autoExposureWeightTable)
{
  FOREACH_ENUM(CameraSettings::CameraSetting, setting)
  {
    settings.settings[setting].value = cameraSettingCollection.settings[setting];
    settings.settings[setting].enforceBounds();
  }

  for(int y = 0; y < 5; ++y)
  {
    for(int x = 0; x < 5; ++x)
    {
      settings.autoExposureWeightTable[5 * y + x].value = autoExposureWeightTable(y, x);
      settings.autoExposureWeightTable[5 * y + x].enforceBounds();
    }
  }
}

void NaoCamera::writeCameraSettings()
{
  const auto oldSettings = appliedSettings.settings;
  FOREACH_ENUM(CameraSettings::CameraSetting, settingName)
  {
    V4L2Setting& currentSetting = settings.settings[settingName];
    V4L2Setting& appliedSetting = appliedSettings.settings[settingName];

    if(timeStamp == 0)
    {
      if(currentSetting.notChangableWhile != CameraSettings::numOfCameraSettings &&
         settings.settings[currentSetting.notChangableWhile].value)
        continue;
    }
    else
    {
      if(currentSetting.notChangableWhile != CameraSettings::numOfCameraSettings)
      {
        const bool nowActive = settings.settings[currentSetting.notChangableWhile].value;
        const bool oldActive = oldSettings[currentSetting.notChangableWhile].value;
        if(nowActive || (!oldActive && currentSetting.value == appliedSetting.value))
          continue;
      }
      else if(currentSetting.value == appliedSetting.value)
        continue;
    }

    if(!setControlSetting(currentSetting))
    {
      std::cerr << "NaoCamera: Setting camera control " << TypeRegistry::getEnumName(settingName) << " failed for value: " << currentSetting.value;
    }
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
    if(timeStamp != 0 && currentTableEntry.value == appliedTableEntry.value)
      continue;

    if(!setControlSetting(currentTableEntry))
    {
      std::cerr << "NaoCamera: Setting autoExposureWeightTableEntry " << i << " failed for value: " << currentTableEntry.value;
    }
    else
    {
      appliedTableEntry.value = currentTableEntry.value;
#ifdef _DEBUG
      assertAutoExposureWeightTableEntry(i);
#endif
    }
  }
}

void NaoCamera::readCameraSettings()
{
  for(V4L2Setting& setting : appliedSettings.settings)
    getControlSetting(setting);
  for(V4L2Setting& setting : appliedSettings.autoExposureWeightTable)
    getControlSetting(setting);
}

void NaoCamera::doAutoWhiteBalance()
{
  setControlSetting(specialSettings.doAutoWhiteBallance);
  if(getControlSetting(settings.settings[CameraSettings::whiteBalanceTemperature]))
  {
    OUTPUT_TEXT("New white balance is " << settings.settings[CameraSettings::whiteBalanceTemperature].value);
    appliedSettings.settings[CameraSettings::whiteBalanceTemperature] = settings.settings[CameraSettings::whiteBalanceTemperature];
  }
}

void NaoCamera::checkSettingsAvailability()
{
  for(V4L2Setting& setting : appliedSettings.settings)
    checkV4L2Setting(setting);
  for(V4L2Setting& setting : appliedSettings.autoExposureWeightTable)
    checkV4L2Setting(setting);
  checkV4L2Setting(specialSettings.doAutoWhiteBallance);
  checkV4L2Setting(specialSettings.verticalFlip);
  checkV4L2Setting(specialSettings.horizontalFlip);
}

void NaoCamera::checkV4L2Setting(V4L2Setting& setting) const
{
  v4l2_queryctrl queryctrl;
  queryctrl.id = setting.command;
  if(ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl) < 0)
  {
    FAIL("ioctl to query setting failed for camera setting " << setting.command);
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
}

bool NaoCamera::getControlSetting(V4L2Setting& setting)
{
  v4l2_control control_s;
  control_s.id = setting.command;
  if(ioctl(fd, VIDIOC_G_CTRL, &control_s) < 0)
  {
    OUTPUT_ERROR("NaoCamera: Retrieving camera setting " << setting.command << " failed");
    return false;
  }
  setting.value = control_s.value;
  return true;
}

bool NaoCamera::setControlSetting(V4L2Setting& setting)
{
  setting.enforceBounds();
  v4l2_control control_s;
  control_s.id = setting.command;
  control_s.value = setting.value;

  const int ret = ioctl(fd, VIDIOC_S_CTRL, &control_s);
  if(ret < 0)
  {
    OUTPUT_ERROR("NaoCamera: Setting value ID: " << setting.command << " failed. VIDIOC_S_CTRL return value is " << ret);
    return false;
  }
  return true;
}

bool NaoCamera::assertCameraSetting(CameraSettings::CameraSetting setting)
{
  appliedSettings.settings[setting] = settings.settings[setting];
  const int oldValue = appliedSettings.settings[setting].value;
  if(getControlSetting(appliedSettings.settings[setting]))
  {
    if(appliedSettings.settings[setting].value == oldValue)
      return true;
    else
    {
      OUTPUT_ERROR("Value for command " << appliedSettings.settings[setting].command << " (" << TypeRegistry::getEnumName(setting) << ") is "
                   << appliedSettings.settings[setting].value << " but should be " << oldValue << ".");
    }
  }
  return false;
}

bool NaoCamera::assertAutoExposureWeightTableEntry(size_t entry)
{
  if(entry < settings.autoExposureWeightTable.size())
  {
    appliedSettings.autoExposureWeightTable[entry] = settings.autoExposureWeightTable[entry];
    const int oldValue = appliedSettings.autoExposureWeightTable[entry].value;
    if(getControlSetting(appliedSettings.autoExposureWeightTable[entry]))
    {
      if(appliedSettings.autoExposureWeightTable[entry].value == oldValue)
        return true;
      else
      {
        OUTPUT_ERROR("Value for command " << appliedSettings.autoExposureWeightTable[entry].command << " (autoExposureWeightTableEntry " << entry << ") is "
                     << appliedSettings.autoExposureWeightTable[entry].value << " but should be " << oldValue << ".");
      }
    }
  }
  return false;
}

void NaoCamera::changeResolution(int width, int height)
{
  BH_TRACE_MSG("change resolution");

  stopCapturing();
  unmapBuffers();

  WIDTH = width;
  HEIGHT = height;

  setImageFormat();
  mapBuffers();
  queueBuffers();
  startCapturing();
}

void NaoCamera::setImageFormat()
{
  // set format
  v4l2_format fmt;
  memset(&fmt, 0, sizeof(v4l2_format));
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = WIDTH;
  fmt.fmt.pix.height = HEIGHT;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  VERIFY(!ioctl(fd, VIDIOC_S_FMT, &fmt));

  ASSERT(fmt.fmt.pix.sizeimage == WIDTH * HEIGHT * 2);
}

void NaoCamera::mapBuffers()
{
  // request buffers
  v4l2_requestbuffers rb;
  memset(&rb, 0, sizeof(v4l2_requestbuffers));
  rb.count = frameBufferCount;
  rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  rb.memory = V4L2_MEMORY_MMAP;
  VERIFY(ioctl(fd, VIDIOC_REQBUFS, &rb) != -1);
  ASSERT(rb.count == frameBufferCount);

  // map or prepare the buffers
  ASSERT(!buf);
  buf = static_cast<v4l2_buffer*>(calloc(1, sizeof(v4l2_buffer)));
  buf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf->memory = V4L2_MEMORY_MMAP;
  for(unsigned i = 0; i < frameBufferCount; ++i)
  {
    buf->index = i;
    VERIFY(ioctl(fd, VIDIOC_QUERYBUF, buf) != -1);
    memLength[i] = buf->length;
    mem[i] = mmap(0, buf->length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf->m.offset);
    ASSERT(mem[i] != MAP_FAILED);
  }
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

void NaoCamera::queueBuffers()
{
  for(unsigned i = 0; i < frameBufferCount; ++i)
  {
    buf->index = i;
    VERIFY(ioctl(fd, VIDIOC_QBUF, buf) != -1);
  }
}

void NaoCamera::startCapturing()
{
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  VERIFY(ioctl(fd, VIDIOC_STREAMON, &type) != -1);
}

void NaoCamera::stopCapturing()
{
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  VERIFY(ioctl(fd, VIDIOC_STREAMOFF, &type) != -1);
}

NaoCamera::V4L2Setting::V4L2Setting(int command, int value, int min, int max, CameraSettings::CameraSetting notChangableWhile) :
  command(command), value(value), notChangableWhile(notChangableWhile), min(min), max(max)
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

#define V4L2_MT9M114_FADE_TO_BLACK V4L2_CID_PRIVATE_BASE
#define V4L2_MT9M114_BRIGHTNESS_DARK (V4L2_CID_PRIVATE_BASE+1)
#define V4L2_MT9M114_AE_TARGET_GAIN (V4L2_CID_PRIVATE_BASE+2)
#define V4L2_MT9M114_AE_MIN_VIRT_AGAIN (V4L2_CID_PRIVATE_BASE+3)
#define V4L2_MT9M114_AE_MAX_VIRT_AGAIN (V4L2_CID_PRIVATE_BASE+4)
#define V4L2_MT9M114_AE_MIN_VIRT_DGAIN (V4L2_CID_PRIVATE_BASE+5)
#define V4L2_MT9M114_AE_MAX_VIRT_DGAIN (V4L2_CID_PRIVATE_BASE+6)
#define V4L2_MT9M114_AE_WEIGHT_TABLE_0_0 (V4L2_CID_PRIVATE_BASE+7)

NaoCamera::CameraSettingsCollection::CameraSettingsCollection()
{
  settings[CameraSettings::autoExposure] = V4L2Setting(V4L2_CID_EXPOSURE_AUTO, -1000, 0, 1);
  settings[CameraSettings::autoExposureAlgorithm] = V4L2Setting(V4L2_CID_EXPOSURE_ALGORITHM, -1000, 0, 3);
  settings[CameraSettings::autoExposureBrightness] = V4L2Setting(V4L2_CID_BRIGHTNESS, -1000, 0, 255);
  //settings[CameraSettings::autoExposureBrightnessDark] = V4L2Setting(V4L2_MT9M114_BRIGHTNESS_DARK, -1000, 0, 255);
  settings[CameraSettings::autoExposureMinVirtAnalogGain] = V4L2Setting(V4L2_MT9M114_AE_MIN_VIRT_AGAIN, -1000, 0, 32767);
  settings[CameraSettings::autoExposureMaxVirtAnalogGain] = V4L2Setting(V4L2_MT9M114_AE_MAX_VIRT_AGAIN, -1000, 0, 32767);
  settings[CameraSettings::autoExposureMinVirtDigitalGain] = V4L2Setting(V4L2_MT9M114_AE_MIN_VIRT_DGAIN, -1000, 0, 32767);
  settings[CameraSettings::autoExposureMaxVirtDigitalGain] = V4L2Setting(V4L2_MT9M114_AE_MAX_VIRT_DGAIN, -1000, 0, 32767);
  settings[CameraSettings::autoExposureTargetGain] = V4L2Setting(V4L2_MT9M114_AE_TARGET_GAIN, -1000, 0, 32767);
  settings[CameraSettings::autoWhiteBalance] = V4L2Setting(V4L2_CID_AUTO_WHITE_BALANCE, 0, 0, 1);
  settings[CameraSettings::contrast] = V4L2Setting(V4L2_CID_CONTRAST, -1000, 16, 64);
  settings[CameraSettings::exposure] = V4L2Setting(V4L2_CID_EXPOSURE, -1000, 0, 333, CameraSettings::autoExposure);
  settings[CameraSettings::fadeToBlack] = V4L2Setting(V4L2_MT9M114_FADE_TO_BLACK, -1000, 0, 1);
  settings[CameraSettings::gain] = V4L2Setting(V4L2_CID_GAIN, -1000, 32, 255, CameraSettings::autoExposure);
  settings[CameraSettings::hue] = V4L2Setting(V4L2_CID_HUE, -1000, -22, 22);
  settings[CameraSettings::powerLineFrequency] = V4L2Setting(V4L2_CID_POWER_LINE_FREQUENCY, -1000, 1, 2);
  settings[CameraSettings::saturation] = V4L2Setting(V4L2_CID_SATURATION, -1000, 0, 255);
  settings[CameraSettings::sharpness] = V4L2Setting(V4L2_CID_SHARPNESS, -1000, -7, 7);
  settings[CameraSettings::whiteBalanceTemperature] = V4L2Setting(V4L2_CID_WHITE_BALANCE_TEMPERATURE, -1000, 2700, 6500, CameraSettings::autoWhiteBalance);

  for(size_t i = 0; i < sizeOfAutoExposureWeightTable; ++i)
  {
    autoExposureWeightTable[i] = V4L2Setting(V4L2_MT9M114_AE_WEIGHT_TABLE_0_0 + i, -1000, 0, 100);
  }
}

bool NaoCamera::CameraSettingsCollection::operator==(const CameraSettingsCollection& other) const
{
  FOREACH_ENUM(CameraSettings::CameraSetting, setting)
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
  verticalFlip(V4L2_CID_VFLIP, 0, 0, 1),
  horizontalFlip(V4L2_CID_HFLIP, 0, 0, 1),
  doAutoWhiteBallance(V4L2_CID_DO_WHITE_BALANCE, 1, 1, 1)
{}
