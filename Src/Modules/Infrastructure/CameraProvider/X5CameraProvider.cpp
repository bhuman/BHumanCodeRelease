/**
 * @file X5CameraProvider.cpp
 *
 * This file implements a module that provides camera images on the K1 robot
 * using Rico Tilgner's \c cam_stream service.
 *
 * @author Thomas Röfer
 */

#include "X5CameraProvider.h"
#include "Platform/SystemCall.h"
#include "Platform/Time.h"
#ifdef TARGET_ROBOT
#include <cam_stream_receiver.cpp>
#endif

MAKE_MODULE(X5CameraProvider);

thread_local X5CameraProvider* X5CameraProvider::theInstance = nullptr;

X5CameraProvider::X5CameraProvider()
#ifdef TARGET_ROBOT
  : receiver(ip.c_str(), port)
#endif
{
  theInstance = this;

  VERIFY(readCameraIntrinsics());
  VERIFY(readCameraResolution());
  ASSERT(cameraResolutionRequest.resolution == CameraResolutionRequest::w544h448f60);
  cameraResolutionRequest.apply(cameraInfo);
  updateCameraInfo();

#ifdef TARGET_ROBOT
  receiver.onFrame([this](const cam_stream::CamFrame& frame)
  {
    if(frame.channel == channel)
    {
      SYNC;
      X5CameraProvider::frame = frame;
      lastImageTimestamp = static_cast<unsigned>(frame.timestamp_ns / 1000000 - Time::getSystemTimeBase());
      imageSignal.post();
    }
  });
  receiver.start();
#endif
}

X5CameraProvider::~X5CameraProvider()
{
#ifdef TARGET_ROBOT
  receiver.stop();
#endif
  theInstance = nullptr;
}

bool X5CameraProvider::readCameraIntrinsics()
{
  InMapFile stream("cameraIntrinsics.cfg");
  bool exist = stream.exists();
  if(exist)
    stream >> cameraIntrinsics;
  return exist;
}

bool X5CameraProvider::readCameraResolution()
{
  InMapFile stream("cameraResolution.cfg");
  bool exist = stream.exists();
  if(exist)
    stream >> cameraResolutionRequest;
  return exist;
}

void X5CameraProvider::updateCameraInfo()
{
  // set opening angle
  cameraInfo.openingAngleWidth = cameraIntrinsics.openingAngleWidth;
  cameraInfo.openingAngleHeight = cameraIntrinsics.openingAngleHeight;

  // set optical center
  cameraInfo.opticalCenter = {cameraIntrinsics.opticalCenter.x() * cameraInfo.width,
                              cameraIntrinsics.opticalCenter.y() * cameraInfo.height};

  // update focal length
  cameraInfo.updateFocalLength();
}

void X5CameraProvider::update(CameraImage& theCameraImage)
{
#ifdef TARGET_ROBOT
  SYNC;
  theCameraImage.setResolution(cameraInfo.width / 2, cameraInfo.height);

  // Already received an image?
  if(frame.nv12.empty())
    return;

  if(!intrinsicsUpdated)
  {
    cam_stream::RectifiedIntrinsics rectifiedIntrinsics;
    if(receiver.getRectifiedIntrinsics(rectifiedIntrinsics))
    {
      cameraIntrinsics.openingAngleWidth = 2.f * std::atan(rectifiedIntrinsics.width / (static_cast<float>(rectifiedIntrinsics.fx) * 2.f));
      cameraIntrinsics.openingAngleHeight = 2.f * std::atan(rectifiedIntrinsics.height / (static_cast<float>(rectifiedIntrinsics.fy) * 2.f));
      cameraIntrinsics.opticalCenter = {static_cast<float>(rectifiedIntrinsics.cx / rectifiedIntrinsics.width),
        static_cast<float>(rectifiedIntrinsics.cy / rectifiedIntrinsics.height)};
      this->cameraInfo.width = rectifiedIntrinsics.width;
      this->cameraInfo.height = rectifiedIntrinsics.height;
      updateCameraInfo();
      intrinsicsUpdated = true;
    }
  }

  ASSERT(frame.width == cameraInfo.width && frame.height == cameraInfo.height);
  const int width = cameraInfo.width;
  const unsigned char* yBase = frame.nv12.data();
  const unsigned char* uvBase = yBase + theCameraImage.height * width;
  for(int y = 0; y < cameraInfo.height; ++y)
  {
    const unsigned char* yChannel = yBase + y * width;
    const unsigned char* uvChannel = uvBase + y / 2 * width;
    for(PixelTypes::YUYVPixel* p = theCameraImage[y], * pEnd = p + theCameraImage.width; p < pEnd; ++p)
    {
      p->y0 = *yChannel++;
      p->u = *uvChannel++;
      p->y1 = *yChannel++;
      p->v = *uvChannel++;
    }
  }

  theCameraImage.timestamp = lastImageTimestamp;
  theJPEGImageRequest.startCompression();
#else
  theCameraImage.setResolution(cameraInfo.width / 2, cameraInfo.height);
  theCameraImage.timestamp = Time::getCurrentSystemTime();
#endif
}

bool X5CameraProvider::isFrameDataComplete()
{
#ifdef TARGET_ROBOT
  if(theInstance)
    return theInstance->theCameraImage.timestamp != theInstance->lastImageTimestamp;
  else
#endif
    return true;
}

void X5CameraProvider::waitForFrameData()
{
#ifdef TARGET_ROBOT
  if(theInstance)
  {
    const bool noImageYet = !theInstance->theCameraImage.timestamp;
    const bool success = theInstance->imageSignal.wait(noImageYet ? theInstance->maxWaitForImage : theInstance->minWaitForImage);
    if(success && noImageYet)
      SystemCall::say("Camera ready");
    while(success && theInstance->imageSignal.tryWait());
  }
#endif
}
