/**
 * @file Platform/Linux/NaoCamera.cpp
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

#include "NaoCamera.h"
#include "BHAssert.h"
#include "SystemCall.h"
#include "Tools/Debugging/Debugging.h"

NaoCamera::NaoCamera(const char* device, CameraInfo::Camera camera, int width, int height, bool flip) :
  camera(camera),
  settings(camera),
  appliedSettings(camera),
  WIDTH(width * 2),
  HEIGHT(height * 2)
#ifndef NDEBUG
  ,
  SIZE(WIDTH * HEIGHT * 2)
#endif
{
  initOpenVideoDevice(device);

  initRequestAndMapBuffers();
  initQueueAllBuffers();

  initSetImageFormat();
  setFrameRate(1, 30);
  initDefaultControlSettings(flip);

  startCapturing();
}

NaoCamera::~NaoCamera()
{
  // disable streaming
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  VERIFY(ioctl(fd, VIDIOC_STREAMOFF, &type) != -1);

  // unmap buffers
  for(int i = 0; i < frameBufferCount; ++i)
    munmap(mem[i], memLength[i]);

  // close the device
  close(fd);
  free(buf);
}

bool NaoCamera::captureNew(NaoCamera& cam1, NaoCamera& cam2, int timeout, bool& errorCam1, bool& errorCam2)
{
  NaoCamera* cams[2] = {&cam1, &cam2};

  ASSERT(cam1.currentBuf == nullptr);
  ASSERT(cam2.currentBuf == nullptr);

  errorCam1 = errorCam2 = false;

  struct pollfd pollfds[2] =
  {
    {cams[0]->fd, POLLIN | POLLPRI, 0},
    {cams[1]->fd, POLLIN | POLLPRI, 0},
  };
  int polled = poll(pollfds, 2, timeout);
  if(polled < 0)
  {
    OUTPUT_ERROR("Cannot poll for camera images. Reason: " << strerror(errno));
    ASSERT(false);
    return false;
  }
  else if(polled == 0)
  {
    OUTPUT_ERROR("One second passed and there's still no image to read from any camera. Terminating.");
    return false;
  }

  for(int i = 0; i < 2; ++i)
  {
    if(pollfds[i].revents & POLLIN)
    {
      if(ioctl(cams[i]->fd, VIDIOC_DQBUF, cams[i]->buf) == -1)
      {
        OUTPUT_ERROR("VIDIOC_DQBUF failed: " << strerror(errno));
        (i == 0 ? errorCam1 : errorCam2) = true;
      }
      else
      {
        //OUTPUT_ERROR("VIDIOC_DQBUF success revents=" << pollfds[i].revents);
        //ASSERT(buf->bytesused == SIZE);
        cams[i]->currentBuf = cams[i]->buf;
        cams[i]->timeStamp = static_cast<unsigned long long>(cams[i]->currentBuf->timestamp.tv_sec) * 1000000ll + cams[i]->currentBuf->timestamp.tv_usec;

        if(cams[i]->first)
        {
          cams[i]->first = false;
          printf("%s camera is working\n", CameraInfo::getName(cams[i]->camera));
        }
      }
    }
    else if(pollfds[i].revents)
    {
      OUTPUT_ERROR("strane poll results: " << pollfds[i].revents);
      (i == 0 ? errorCam1 : errorCam2) = true;
    }
  }
  /*
  if(!success)
  {
    OUTPUT_ERROR("Polling failed.");
    return false;
  }
  */

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

  const unsigned startPollingTimestamp = SystemCall::getCurrentSystemTime();
  struct pollfd pollfd = {fd, POLLIN | POLLPRI, 0};
  int polled = poll(&pollfd, 1, 200); // Fail after missing 6 frames (200ms)
  if(polled < 0)
  {
    OUTPUT_ERROR(CameraInfo::getName(camera) << "camera : Cannot poll. Reason: " << strerror(errno));
    ASSERT(false);
  }
  else if(polled == 0)
  {
    OUTPUT_ERROR(CameraInfo::getName(camera) << "camera : 200 ms passed and there's still no image to read from the camera. Terminating.");
    return false;
  }
  else if(pollfd.revents & (POLLERR | POLLNVAL))
  {
    OUTPUT_ERROR(CameraInfo::getName(camera) << "camera : Polling failed.");
    return false;
  }
  // dequeue a frame buffer (this call blocks when there is no new image available) */
  VERIFY(ioctl(fd, VIDIOC_DQBUF, buf) != -1);
  BH_TRACE;
  //ASSERT(buf->bytesused == SIZE);
  currentBuf = buf;
  timeStamp = static_cast<unsigned long long>(currentBuf->timestamp.tv_sec) * 1000000ll + currentBuf->timestamp.tv_usec;
  const unsigned endPollingTimestamp = SystemCall::getCurrentSystemTime();
  timeWaitedForLastImage = endPollingTimestamp - startPollingTimestamp;

  if(first)
  {
    first = false;
    printf("%s camera is working\n", CameraInfo::getName(camera));
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
  struct v4l2_streamparm fps;
  memset(&fps, 0, sizeof(struct v4l2_streamparm));
  fps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  VERIFY(!ioctl(fd, VIDIOC_G_PARM, &fps));
  fps.parm.capture.timeperframe.numerator = numerator;
  fps.parm.capture.timeperframe.denominator = denominator;
  VERIFY(ioctl(fd, VIDIOC_S_PARM, &fps) != -1);
}

void NaoCamera::setSettings(const CameraSettings& settings)
{
  ASSERT(settings.camera == camera);
  if(settings.camera != camera)
  {
    OUTPUT_ERROR("Setting camera settings for wrong camera!");
    return;
  }
  this->settings = settings;
}

void NaoCamera::assertCameraSettings()
{
  bool allFine = true;
  // check frame rate
  struct v4l2_streamparm fps;
  memset(&fps, 0, sizeof(fps));
  fps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  VERIFY(!ioctl(fd, VIDIOC_G_PARM, &fps));
  if(fps.parm.capture.timeperframe.numerator != 1)
  {
    OUTPUT_ERROR("fps.parm.capture.timeperframe.numerator is wrong.");
    allFine = false;
  }
  if(fps.parm.capture.timeperframe.denominator != 30)
  {
    OUTPUT_ERROR("fps.parm.capture.timeperframe.denominator is wrong.");
    allFine = false;
  }

  // check camera settings
  for(int i = 0; i < CameraSettings::numOfCameraSettings; ++i)
  {
    if(!assertCameraSetting(static_cast<CameraSettings::CameraSetting>(i)))
      allFine = false;
  }

  if(allFine)
  {
    OUTPUT_TEXT("Camera settings match settings stored in hardware/driver.");
  }
}

void NaoCamera::writeCameraSettings()
{
  for(int i = 0; i < CameraSettings::numOfCameraSettings; ++i)
  {
    CameraSettings::V4L2Setting& currentSetting = settings.settings[i];
    CameraSettings::V4L2Setting& appliedSetting = appliedSettings.settings[i];

    if(currentSetting.value == appliedSetting.value)
      continue;
    else
    {
      CameraSettings::CameraSetting setting = static_cast<CameraSettings::CameraSetting>(i);
      if(!setControlSetting(currentSetting.command, currentSetting.value))
      {
        OUTPUT_WARNING("NaoCamera: Setting camera control " << CameraSettings::getName(setting) << " failed for value: " << currentSetting.value);
      }
      else
      {
        appliedSetting.value = currentSetting.value;
        assertCameraSetting(setting);
        for(auto setting : currentSetting.influencingSettings)
        {
          if(setting != CameraSettings::numOfCameraSettings)
          {
            int value = getControlSetting(settings.settings[setting].command);
            if(settings.settings[setting].value == appliedSettings.settings[setting].value)
              settings.settings[setting].value = appliedSettings.settings[setting].value = value;
          }
        }
      }
    }
  }
}

void NaoCamera::readCameraSettings()
{
  for(CameraSettings::V4L2Setting& setting : appliedSettings.settings)
  {
    int value = getControlSetting(setting.command);
    setting.value = value;
  }
}

void NaoCamera::doAutoWhiteBalance()
{
  setControlSetting(V4L2_CID_DO_WHITE_BALANCE, 1);
  int value = getControlSetting(V4L2_CID_WHITE_BALANCE_TEMPERATURE);
  if(value > 0)
  {
    OUTPUT_TEXT("New white balance is " << value);
    settings.settings[CameraSettings::WhiteBalance].value = value;
    appliedSettings.settings[CameraSettings::WhiteBalance].value = value;
  }
}

int NaoCamera::getControlSetting(unsigned int id)
{
  struct v4l2_queryctrl queryctrl;
  queryctrl.id = id;
  if(ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl) < 0)
  {
    fprintf(stderr, "ioctl to query setting failed for camera setting %d.\n", id);
    return -1;
  }
  if(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
  {
    fprintf(stderr, "Camera setting %d is disabled.\n", id);
    return -1; // not available
  }
  if(queryctrl.type != V4L2_CTRL_TYPE_BOOLEAN && queryctrl.type != V4L2_CTRL_TYPE_INTEGER && queryctrl.type != V4L2_CTRL_TYPE_MENU)
  {
    fprintf(stderr, "Camera setting %d is unsupported.\n", id);
    return -1; // not supported
  }

  struct v4l2_control control_s;
  control_s.id = id;
  if(ioctl(fd, VIDIOC_G_CTRL, &control_s) < 0)
  {
    fprintf(stderr, "ioctl to retrieve camera setting failed for camera setting %d.\n", id);
    return -1;
  }
  return control_s.value;
}

bool NaoCamera::setControlSetting(unsigned int id, int value)
{
  struct v4l2_queryctrl queryctrl;
  queryctrl.id = id;
  if(ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl) < 0)
  {
    OUTPUT_WARNING("NaoCamera:  VIDIOC_QUERYCTRL " << id << " call failed (value: " << value << ")!");
    return false;
  }
  if(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
  {
    OUTPUT_WARNING("NaoCamera:  VIDIOC_QUERYCTRL call failed. Command " << id << " disabled!");
    return false; // not available
  }
  if(queryctrl.type != V4L2_CTRL_TYPE_BOOLEAN && queryctrl.type != V4L2_CTRL_TYPE_INTEGER && queryctrl.type != V4L2_CTRL_TYPE_MENU)
  {
    OUTPUT_WARNING("NaoCamera:  VIDIOC_QUERYCTRL call failed. Command " << id << " not supported!");
    return false; // not supported
  }
  // clip value
  if(value < queryctrl.minimum)
  {
    OUTPUT_WARNING("NaoCamera: Clipping control value. ID: " << id << " to " << queryctrl.minimum);
    value = queryctrl.minimum;
  }
  if(value > queryctrl.maximum)
  {
    value = queryctrl.maximum;
    OUTPUT_WARNING("NaoCamera: Clipping control value. ID: " << id << " to " << queryctrl.maximum);
  }

  struct v4l2_control control_s;
  control_s.id = id;
  control_s.value = value;

  if(ioctl(fd, VIDIOC_S_CTRL, &control_s) < 0)
  {
    OUTPUT_WARNING("NaoCamera: Setting value ID: " << id << " failed. VIDIOC_S_CTRL return value < 0");
    return false;
  }

  return true;
}

bool NaoCamera::assertCameraSetting(CameraSettings::CameraSetting setting)
{
  int value = getControlSetting(settings.settings[setting].command);
  if(value == settings.settings[setting].value)
    return true;
  else
  {
    OUTPUT_ERROR("Value for command " << settings.settings[setting].command << " (" << CameraSettings::getName(setting) << ") is " << value << " but should be " << settings.settings[setting].value << ".");
    appliedSettings.settings[setting].value = value;
    return false;
  }
}

void NaoCamera::initOpenVideoDevice(const char* device)
{
  // open device
  fd = open(device, O_RDWR);
  ASSERT(fd != -1);
}

void NaoCamera::initSetImageFormat()
{
  // set format
  struct v4l2_format fmt;
  memset(&fmt, 0, sizeof(struct v4l2_format));
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = WIDTH;
  fmt.fmt.pix.height = HEIGHT;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  VERIFY(!ioctl(fd, VIDIOC_S_FMT, &fmt));

  ASSERT(fmt.fmt.pix.sizeimage == SIZE);
}

void NaoCamera::initRequestAndMapBuffers()
{
  // request buffers
  struct v4l2_requestbuffers rb;
  memset(&rb, 0, sizeof(struct v4l2_requestbuffers));
  rb.count = frameBufferCount;
  rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  rb.memory = V4L2_MEMORY_MMAP;
  VERIFY(ioctl(fd, VIDIOC_REQBUFS, &rb) != -1);
  ASSERT(rb.count == frameBufferCount);

  // map or prepare the buffers
  buf = static_cast<struct v4l2_buffer*>(calloc(1, sizeof(struct v4l2_buffer)));
  for(int i = 0; i < frameBufferCount; ++i)
  {
    buf->index = i;
    buf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf->memory = V4L2_MEMORY_MMAP;
    VERIFY(ioctl(fd, VIDIOC_QUERYBUF, buf) != -1);
    memLength[i] = buf->length;
    mem[i] = mmap(0, buf->length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf->m.offset);
    ASSERT(mem[i] != MAP_FAILED);
  }
}

void NaoCamera::initQueueAllBuffers()
{
  // queue the buffers
  for(int i = 0; i < frameBufferCount; ++i)
  {
    buf->index = i;
    buf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf->memory = V4L2_MEMORY_MMAP;
    VERIFY(ioctl(fd, VIDIOC_QBUF, buf) != -1);
  }
}

void NaoCamera::initDefaultControlSettings(bool flip)
{
  setControlSetting(V4L2_CID_HFLIP, flip ? 1 : 0);
  setControlSetting(V4L2_CID_VFLIP, flip ? 1 : 0);
}

void NaoCamera::startCapturing()
{
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  VERIFY(ioctl(fd, VIDIOC_STREAMON, &type) != -1);
}
