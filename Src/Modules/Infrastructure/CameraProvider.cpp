/**
* @file CameraProvider.cpp
* This file declares a module that provides camera images.
* @author Colin Graf
*/

#include <cstdio>

#include "CameraProvider.h"
#include "Representations/Perception/JPEGImage.h"
#include "Platform/SystemCall.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Debugging/Stopwatch.h"

PROCESS_WIDE_STORAGE(CameraProvider) CameraProvider::theInstance = 0;

CameraProvider::CameraProvider() : currentImageCamera(0)
#ifdef CAMERA_INCLUDED
, imageTimeStamp(0), otherImageTimeStamp(0), lastImageTimeStamp(0), lastImageTimeStampLL(0)
#endif
{
  InMapFile upperStream("upperCameraInfo.cfg");
  ASSERT(upperStream.exists());
  upperStream >> upperCameraInfo;
  InMapFile lowerStream("lowerCameraInfo.cfg");
  ASSERT(lowerStream.exists());
  lowerStream >> lowerCameraInfo;

#ifdef CAMERA_INCLUDED
  upperCamera = new NaoCamera("/dev/video0", upperCameraInfo.camera, upperCameraInfo.width, upperCameraInfo.height, true);
  lowerCamera = new NaoCamera("/dev/video1", lowerCameraInfo.camera, lowerCameraInfo.width, lowerCameraInfo.height, false);
  cycleTime = upperCamera->getFrameRate();
  ASSERT(upperCamera->getFrameRate() == lowerCamera->getFrameRate());
#else
  upperCamera = lowerCamera = NULL;
  cycleTime = 1.f / 30.f;
#endif
  theInstance = this;
}

CameraProvider::~CameraProvider()
{
#ifdef CAMERA_INCLUDED
  if(upperCamera)
    delete upperCamera;
  if(lowerCamera)
    delete lowerCamera;
#endif
  theInstance = 0;
}

void CameraProvider::update(Image& image)
{
#ifdef CAMERA_INCLUDED
  ASSERT(!currentImageCamera);
  if(upperCamera->hasImage() && (!lowerCamera->hasImage() || upperCamera->getTimeStamp() < lowerCamera->getTimeStamp()))
  {
    image.setResolution(upperCameraInfo.width, upperCameraInfo.height);
    image.setImage(const_cast<unsigned char*>(upperCamera->getImage()));
    lastImageTimeStampLL = upperCamera->getTimeStamp();
    imageTimeStamp = image.timeStamp = std::max(lastImageTimeStamp + 1, (unsigned) (upperCamera->getTimeStamp() / 1000) - SystemCall::getSystemTimeBase());
    upperCamera->setSettings(theCameraSettings);
    upperCamera->writeCameraSettings();
    currentImageCamera = upperCamera;
  }
  else if(lowerCamera->hasImage())
  {
    image.setResolution(lowerCameraInfo.width, lowerCameraInfo.height);
    image.setImage(const_cast<unsigned char*>(lowerCamera->getImage()));
    lastImageTimeStampLL = lowerCamera->getTimeStamp();
    otherImageTimeStamp = image.timeStamp = std::max(lastImageTimeStamp + 1, (unsigned) (lowerCamera->getTimeStamp() / 1000) - SystemCall::getSystemTimeBase());
    lowerCamera->setSettings(theCameraSettings);
    lowerCamera->writeCameraSettings();
    currentImageCamera = lowerCamera;
  }
  ASSERT(image.timeStamp >= lastImageTimeStamp);
  lastImageTimeStamp = image.timeStamp;
#endif // CAMERA_INCLUDED
  STOP_TIME_ON_REQUEST("compressJPEG",
  {
    DEBUG_RESPONSE("representation:JPEGImage", OUTPUT(idJPEGImage, bin, JPEGImage(image)););
  });
}

void CameraProvider::update(FrameInfo& frameInfo)
{
  frameInfo.time = theImage.timeStamp;
  frameInfo.cycleTime = cycleTime * 0.5f;
}

void CameraProvider::update(CognitionFrameInfo& cognitionFrameInfo)
{
  cognitionFrameInfo.time = theImage.timeStamp;
  cognitionFrameInfo.cycleTime = cycleTime * 0.5f;
}

void CameraProvider::update(CameraInfo& cameraInfo)
{
  if(currentImageCamera == upperCamera)
    cameraInfo = upperCameraInfo;
  else
    cameraInfo = lowerCameraInfo;
}

bool CameraProvider::isFrameDataComplete()
{
#ifdef CAMERA_INCLUDED
  if(theInstance)
    return theInstance->upperCamera->hasImage() || theInstance->lowerCamera->hasImage();
  else
#endif
    return true;
}

void CameraProvider::waitForFrameData2()
{
#ifdef CAMERA_INCLUDED

  const unsigned int timeout = 2000 * 10;

  if(currentImageCamera)
  {
    currentImageCamera->releaseImage();
    currentImageCamera = 0;
  }

  for(;;)
  {
    if(upperCamera->hasImage() || lowerCamera->hasImage())
      return;

    bool resetUpper = false;
    bool resetLower = false;
    if(!NaoCamera::captureNew(*upperCamera, *lowerCamera, timeout, resetUpper, resetLower))
    {
      OUTPUT_WARNING("CameraProvider: Poll failed. Resetting both cameras");
      resetUpper = resetLower = true;
    }
    else
    {
      if(resetUpper)
        OUTPUT_WARNING("CameraProvider: Capturing image failed. Resetting upper camera.");
      if(resetLower)
        OUTPUT_WARNING("CameraProvider: Capturing image failed. Resetting lower camera.");
    }

    unsigned int now = SystemCall::getRealSystemTime();
    if(!resetUpper && imageTimeStamp && now - imageTimeStamp >= timeout)
    {
      OUTPUT_WARNING("CameraProvider: Capturing image timed out. Resetting upper camera.");
      resetUpper = true;
    }
    if(!resetLower && otherImageTimeStamp && now - otherImageTimeStamp >= timeout)
    {
      OUTPUT_WARNING("CameraProvider: Capturing image timed out. Resetting lower camera.");
      resetLower = true;
    }

    if(resetUpper)
    {
      delete upperCamera;
      upperCamera = new NaoCamera("/dev/video0", upperCameraInfo.camera, upperCameraInfo.width, upperCameraInfo.height, true);
      imageTimeStamp = 0;
      SystemCall::playSound("cameraReset.wav");
    }

    if(resetLower)
    {
      delete lowerCamera;
      lowerCamera = new NaoCamera("/dev/video1", lowerCameraInfo.camera, lowerCameraInfo.width, lowerCameraInfo.height, false);
      otherImageTimeStamp = 0;
      SystemCall::playSound("cameraReset.wav");
    }

    if(upperCamera->hasImage() && upperCamera->getTimeStamp() < lastImageTimeStampLL)
      upperCamera->releaseImage();
    if(lowerCamera->hasImage() && lowerCamera->getTimeStamp() < lastImageTimeStampLL)
      lowerCamera->releaseImage();
  }

#endif
}

void CameraProvider::waitForFrameData()
{
#ifdef CAMERA_INCLUDED
  if(theInstance)
    theInstance->waitForFrameData2();
#endif
}

MAKE_MODULE(CameraProvider, Cognition Infrastructure)
