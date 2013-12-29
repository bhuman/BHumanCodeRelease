/**
* @file Platform/Linux/NaoCamera.cpp
* Interface to a camera of the NAO.
* @author Colin Graf
* @author Thomas Röfer
*/

#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <cerrno>
#include <poll.h>
#include <linux/videodev2.h>
#ifdef USE_USERPTR
#include <malloc.h> // memalign
#endif

#include "NaoCamera.h"
#include "BHAssert.h"
#include "SystemCall.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Streams/InStreams.h"

NaoCamera::NaoCamera(const char* device, CameraInfo::Camera camera, int width, int height, bool flip) :
  timeWaitedForLastImage(0),
  WIDTH(width * 2),
  HEIGHT(height * 2),
#ifndef NDEBUG
  SIZE(WIDTH * HEIGHT * 2),
#endif
  currentBuf(0),
  timeStamp(0), camera(camera), first(true),
  lastCameraSettingTimestamp(0), cameraSettingApplicationRate(16000)
{
  initOpenVideoDevice(device);

  initRequestAndMapBuffers();
  initQueueAllBuffers();

  initSetImageFormat();
  setFrameRate(1, 15);
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
#ifdef USE_USERPTR
    free(mem[i]);
#else
    munmap(mem[i], memLength[i]);
#endif

  // close the device
  close(fd);
  free(buf);
}

void NaoCamera::releaseImage()
{
  if(currentBuf)
  {
    VERIFY(ioctl(fd, VIDIOC_QBUF, currentBuf) != -1);
    currentBuf = 0;
  }
}

bool NaoCamera::captureNew(NaoCamera& cam1, NaoCamera& cam2, int timeout, bool& errorCam1, bool& errorCam2)
{
  NaoCamera* cams[2] = { &cam1, &cam2 };

  ASSERT(cam1.currentBuf == 0);
  ASSERT(cam2.currentBuf == 0);

  errorCam1 = errorCam2 = false;

  struct pollfd pollfds[2] = {
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
      //VERIFY(ioctl(cams[i]->fd, VIDIOC_DQBUF, cams[i]->buf) != -1);
      int error = ioctl(cams[i]->fd, VIDIOC_DQBUF, cams[i]->buf);
      if(error == -1)
      {
        OUTPUT_ERROR("VIDIOC_DQBUF failed: " << strerror(errno));
        (i == 0 ? errorCam1 : errorCam2) = true;
      }
      else
      {
        //OUTPUT_ERROR("VIDIOC_DQBUF success revents=" << pollfds[i].revents);
        //ASSERT(buf->bytesused == SIZE);
        cams[i]->currentBuf = cams[i]->buf;
        cams[i]->timeStamp = (unsigned long long) cams[i]->currentBuf->timestamp.tv_sec * 1000000ll + cams[i]->currentBuf->timestamp.tv_usec;

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
  timeStamp = (unsigned long long) currentBuf->timestamp.tv_sec * 1000000ll + currentBuf->timestamp.tv_usec;
  const unsigned endPollingTimestamp = SystemCall::getCurrentSystemTime();
  timeWaitedForLastImage = endPollingTimestamp - startPollingTimestamp;

  if(first)
  {
    first = false;
    printf("%s camera is working\n", CameraInfo::getName(camera));
  }

  return true;
}

const unsigned char* NaoCamera::getImage() const
{
#ifdef USE_USERPTR
  unsigned char* imageBuffer = currentBuf ? (unsigned char*)currentBuf->m.userptr : 0;
#else
  unsigned char* imageBuffer = currentBuf ? static_cast<unsigned char*>(mem[currentBuf->index]) : 0;
#endif
  return imageBuffer;
}

bool NaoCamera::hasImage() const
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

bool NaoCamera::setControlSettings(std::list<CameraSettings::V4L2Setting> controlsettings,
                                   std::list<CameraSettings::V4L2Setting> appliedControlSettings)
{
  std::list<CameraSettings::V4L2Setting>::iterator ait = appliedControlSettings.begin();
  std::list<CameraSettings::V4L2Setting>::const_iterator it = controlsettings.begin();
  std::list<CameraSettings::V4L2Setting>::const_iterator end = controlsettings.end();
  bool success = true;
  int counter = 0;
  for(; it != end && timeStamp - lastCameraSettingTimestamp >= cameraSettingApplicationRate; ++it, ++ait, counter++)
  {
    if(it->value == ait->value)
    {
      continue; // This setting has successfully been applied, so we don't have to deal with it.
    }

    if(!setControlSetting(it->command, it->value))
    {
      OUTPUT_WARNING("NaoCamera: Setting camera control value failed: " << it->command);
      success = false; // The settings should have been set but wasn't => no success.
    }
    else
    {
      appliedSettings.setSetting(*it);
      lastCameraSettingTimestamp = timeStamp;
    }
  }
  return success;
}

bool NaoCamera::setControlSetting(unsigned int id, int value)
{

  struct v4l2_queryctrl queryctrl;
  queryctrl.id = id;
  if(ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl) < 0)
  {
    OUTPUT_WARNING("NaoCamera:  VIDIOC_QUERYCTRL call failed");
    return false;
  }
  if(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
  {
    OUTPUT_WARNING("NaoCamera:  VIDIOC_QUERYCTRL call failed. Command " << id << " disabled");
    return false; // not available
  }
  if(queryctrl.type != V4L2_CTRL_TYPE_BOOLEAN && queryctrl.type != V4L2_CTRL_TYPE_INTEGER && queryctrl.type != V4L2_CTRL_TYPE_MENU)
  {
    OUTPUT_WARNING("NaoCamera:  VIDIOC_QUERYCTRL call failed. Command " << id << " not supported");
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

void NaoCamera::setSettings(const CameraSettings& newset)
{
  if(settings == newset)
    return;

  // Ignore the changes since the camera provider now calls writeCameraSettings in every frame.
  settings.getChangesAndAssign(newset);
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

void NaoCamera::initRequestAndMapBuffers()
{
  // request buffers
  struct v4l2_requestbuffers rb;
  memset(&rb, 0, sizeof(struct v4l2_requestbuffers));
  rb.count = frameBufferCount;
  rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
#ifdef USE_USERPTR
  rb.memory = V4L2_MEMORY_USERPTR;
#else
  rb.memory = V4L2_MEMORY_MMAP;
#endif
  VERIFY(ioctl(fd, VIDIOC_REQBUFS, &rb) != -1);
  ASSERT(rb.count == frameBufferCount);

  // map or prepare the buffers
  buf = static_cast<struct v4l2_buffer*>(calloc(1, sizeof(struct v4l2_buffer)));
#ifdef USE_USERPTR
  unsigned int bufferSize = SIZE;
  unsigned int pageSize = getpagesize();
  bufferSize = (bufferSize + pageSize - 1) & ~(pageSize - 1);
#endif
  for(int i = 0; i < frameBufferCount; ++i)
  {
#ifdef USE_USERPTR
    memLength[i] = bufferSize;
    mem[i] = memalign(pageSize, bufferSize);
#else
    buf->index = i;
    buf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf->memory = V4L2_MEMORY_MMAP;
    VERIFY(ioctl(fd, VIDIOC_QUERYBUF, buf) != -1);
    memLength[i] = buf->length;
    mem[i] = mmap(0, buf->length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf->m.offset);
    ASSERT(mem[i] != MAP_FAILED);
#endif
  }
}

void NaoCamera::initQueueAllBuffers()
{
  // queue the buffers
  for(int i = 0; i < frameBufferCount; ++i)
  {
    buf->index = i;
    buf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
#ifdef USE_USERPTR
    buf->memory = V4L2_MEMORY_USERPTR;
    buf->m.userptr = (unsigned long)mem[i];
    buf->length  = memLength[i];
#else
    buf->memory = V4L2_MEMORY_MMAP;
#endif
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
    OUTPUT(idText, text, "fps.parm.capture.timeperframe.numerator is wrong.");
    allFine = false;
  }
  if(fps.parm.capture.timeperframe.denominator != 30)
  {
    OUTPUT(idText, text, "fps.parm.capture.timeperframe.denominator is wrong.");
    allFine = false;
  }

  // check camera settings
  std::list<CameraSettings::V4L2Setting> v4l2settings = settings.getSettings();
  std::list<CameraSettings::V4L2Setting>::const_iterator it = v4l2settings.begin();
  std::list<CameraSettings::V4L2Setting>::const_iterator end = v4l2settings.end();
  for(; it != end; it++)
  {
    int value = getControlSetting((*it).command);
    if(value != (*it).value)
    {
      OUTPUT(idText, text, "Value for command " << (*it).command << " is " << value << " but should be " << (*it).value << ".");
      allFine = false;
    }
  }

  if(allFine)
  {
    OUTPUT(idText, text, "Camera settings match settings stored in hardware/driver.");
  }
}

unsigned int NaoCamera::writeCameraSettings()
{
  const unsigned ts = SystemCall::getCurrentSystemTime();
  std::list<CameraSettings::V4L2Setting> v4l2settings = settings.getSettings();
  std::list<CameraSettings::V4L2Setting> appliedv4l2settings = appliedSettings.getSettings();
  VERIFY(setControlSettings(v4l2settings, appliedv4l2settings));
  return SystemCall::getCurrentSystemTime() - ts;
}
