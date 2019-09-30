/**
 * @file CameraProvider.cpp
 *
 * This file implements a module that handles the communication with the two
 * cameras. This implementation starts a separate thread to avoid blocking
 * calls to slow down processing.
 *
 * @author Colin Graf
 * @author Thomas Röfer
 */

#include "CameraProvider.h"
#include "Platform/SystemCall.h"
#include "Platform/Thread.h"
#include "Platform/Time.h"
#include "Tools/Settings.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Debugging/Stopwatch.h"
#include "Tools/Debugging/Annotation.h"
#include <cstdio>

MAKE_MODULE(CameraProvider, infrastructure)

thread_local CameraProvider* CameraProvider::theInstance = nullptr;
#ifdef CAMERA_INCLUDED
Semaphore CameraProvider::performingReset = Semaphore(1);
bool CameraProvider::resetPending = false;
#endif

CameraProvider::CameraProvider()
  : whichCamera(Thread::getCurrentThreadName() == "Upper" ?  CameraInfo::upper : CameraInfo::lower),
    cameraInfo(whichCamera)
{
  VERIFY(readCameraIntrinsics());
  VERIFY(readCameraResolution());

  theInstance = this;
  setupCamera();

#ifdef CAMERA_INCLUDED
  headName = Global::getSettings().headName.c_str();
  thread.start(this, &CameraProvider::takeImages);
  takeNextImage.post();
  imageTaken.wait();
#endif
}

CameraProvider::~CameraProvider()
{
#ifdef CAMERA_INCLUDED
  thread.announceStop();
  takeNextImage.post();
  thread.stop();

  if(camera)
    delete camera;
#endif
  theInstance = nullptr;
}

void CameraProvider::update(CameraImage& theCameraImage)
{
#ifdef CAMERA_INCLUDED
  unsigned timestamp = static_cast<long long>(camera->getTimestamp() / 1000) > static_cast<long long>(Time::getSystemTimeBase())
                       ? static_cast<unsigned>(camera->getTimestamp() / 1000 - Time::getSystemTimeBase()) : 100000;
  if(camera->hasImage())
  {
    theCameraImage.setReference(cameraInfo.width / 2, cameraInfo.height, const_cast<unsigned char*>(camera->getImage()), std::max(lastImageTimestamp + 1, timestamp));

    if(theCameraImage.timestamp - timestampLastRowChange > 3000)
    {
      timestampLastRowChange = theCameraImage.timestamp;
      currentRow = Random::uniformInt(1, cameraInfo.height - 2);
      rowBuffer.clear();
    }
    std::string res = MD5().digestMemory(reinterpret_cast<unsigned char*>(theCameraImage[currentRow]), cameraInfo.width * sizeof(CameraImage::PixelType));
    rowBuffer.push_front(res);

    int appearances = 0;
    for(auto i = rowBuffer.begin(); i != rowBuffer.end(); ++i)
      if(*i == res && ++appearances > 25)
      {
        OUTPUT_ERROR("Probaly encountered a puzzle image!");
        camera->resetRequired = true;
        return;
      }
  }
  else if(theCameraImage.isReference())
  {
    theCameraImage.setResolution(cameraInfo.width / 2, cameraInfo.height);
    theCameraImage.timestamp = std::max(lastImageTimestamp + 1, timestamp);
  }

  if(whichCamera == CameraInfo::upper)
  {
    DEBUG_RESPONSE_ONCE("module:CameraProvider:doWhiteBalanceUpper") camera->doAutoWhiteBalance();
    DEBUG_RESPONSE_ONCE("module:CameraProvider:readCameraSettingsUpper") camera->readCameraSettings();
  }
  else
  {
    DEBUG_RESPONSE_ONCE("module:CameraProvider:doWhiteBalanceLower") camera->doAutoWhiteBalance();
    DEBUG_RESPONSE_ONCE("module:CameraProvider:readCameraSettingsLower") camera->readCameraSettings();
  }
  lastImageTimestampLL = camera->getTimestamp();

  ASSERT(theCameraImage.timestamp >= lastImageTimestamp);
  lastImageTimestamp = theCameraImage.timestamp;
#else
  theCameraImage.setResolution(cameraInfo.width / 2, cameraInfo.height);
  theCameraImage.timestamp = Time::getCurrentSystemTime();
#endif // CAMERA_INCLUDED
}

void CameraProvider::update(JPEGImage& jpegImage)
{
  jpegImage = theCameraImage;
}

void CameraProvider::update(CameraInfo& cameraInfo)
{
  cameraInfo = this->cameraInfo;
}

void CameraProvider::update(CameraStatus& cameraStatus)
{
  if(!cameraOk)
  {
    if(cameraStatus.ok)
    {
      ANNOTATION("CameraProvider", "Could not acquire new image.");
      SystemCall::playSound("sirene.wav");
      SystemCall::say("Camera broken");
    }
#ifdef NDEBUG
    else if(!SystemCall::soundIsPlaying())
    {
      SystemCall::playSound("sirene.wav");
      SystemCall::say("Camera broken");
    }
#endif
  }

  cameraStatus.ok = cameraOk;
}

bool CameraProvider::readCameraIntrinsics()
{
  InMapFile stream("cameraIntrinsics.cfg");
  bool exist = stream.exists();
  if(exist)
    stream >> cameraIntrinsics;
  return exist;
}

bool CameraProvider::readCameraResolution()
{
  InMapFile stream("cameraResolution.cfg");
  bool exist = stream.exists();
  if(exist)
    stream >> cameraResolutionRequest;
  return exist;
}

bool CameraProvider::processResolutionRequest()
{
  if(SystemCall::getMode() == SystemCall::Mode::physicalRobot
     && theCameraResolutionRequest.resolutions[whichCamera] != lastResolutionRequest)
  {
    lastResolutionRequest = theCameraResolutionRequest.resolutions[whichCamera];
    switch(theCameraResolutionRequest.resolutions[whichCamera])
    {
      case CameraResolutionRequest::noRequest:
        return false;
      case CameraResolutionRequest::defaultRes:
        if(!readCameraResolution())
          cameraResolutionRequest.resolutions[whichCamera] = whichCamera == CameraInfo::upper
                                                             ? CameraResolutionRequest::w640h480
                                                             : CameraResolutionRequest::w320h240;
        return true;
      case CameraResolutionRequest::w320h240:
      case CameraResolutionRequest::w640h480:
      case CameraResolutionRequest::w1280h960:
        cameraResolutionRequest.resolutions[whichCamera] = theCameraResolutionRequest.resolutions[whichCamera];
        return true;
      default:
        FAIL("Unknown resolution.");
        return false;
    }
  }
  else
    return false;
}

void CameraProvider::setupCamera()
{
  // set resolution
  switch(cameraResolutionRequest.resolutions[whichCamera])
  {
    case CameraResolutionRequest::w320h240:
      cameraInfo.width = 320;
      cameraInfo.height = 240;
      break;
    case CameraResolutionRequest::w640h480:
      cameraInfo.width = 640;
      cameraInfo.height = 480;
      break;
    case CameraResolutionRequest::w1280h960:
      cameraInfo.width = 1280;
      cameraInfo.height = 960;
      break;
    default:
      FAIL("Unknown resolution.");
      break;
  }

  // set opening angle
  cameraInfo.openingAngleWidth = cameraIntrinsics.cameras[whichCamera].openingAngleWidth;
  cameraInfo.openingAngleHeight = cameraIntrinsics.cameras[whichCamera].openingAngleHeight;

  // set optical center
  cameraInfo.opticalCenter.x() = cameraIntrinsics.cameras[whichCamera].opticalCenter.x() * cameraInfo.width;
  cameraInfo.opticalCenter.y() = cameraIntrinsics.cameras[whichCamera].opticalCenter.y() * cameraInfo.height;

  // update focal length
  cameraInfo.updateFocalLength();

#ifdef CAMERA_INCLUDED
  ASSERT(camera == nullptr);
  camera = new NaoCamera(whichCamera == CameraInfo::upper ?
                         "/dev/video-top" : "/dev/video-bottom",
                         cameraInfo.camera,
                         cameraInfo.width, cameraInfo.height, whichCamera == CameraInfo::upper,
                         theCameraSettings.cameras[whichCamera], theAutoExposureWeightTable.tables[whichCamera]);
#else
  camera = nullptr;
#endif
}

bool CameraProvider::isFrameDataComplete()
{
#ifdef CAMERA_INCLUDED
  if(theInstance)
    return theInstance->camera->hasImage();
  else
#endif
    return true;
}

void CameraProvider::takeImages()
{
#ifdef CAMERA_INCLUDED
  BH_TRACE_INIT(whichCamera == CameraInfo::upper ? "UpperCameraProvider" : "LowerCameraProvider");
  Thread::nameCurrentThread("CameraProvider");
  thread.setPriority(11);
  unsigned imageReceived = Time::getRealSystemTime();
  while(thread.isRunning())
  {
    if(camera->resetRequired)
      resetPending = true;
    if(resetPending)
    {
      delete camera;
      camera = nullptr;
      performingReset.wait();
      if(resetPending)
      {
        NaoCamera::resetCamera();
        SystemCall::say(headName.c_str());
        SystemCall::say("Camera reset");
        resetPending = false;
      }
      setupCamera();
      performingReset.post();
      continue;
    }

    takeNextImage.wait();

    if(camera->hasImage())
      camera->releaseImage();

    // update resolution
    if(processResolutionRequest())
    {
      delete camera;
      camera = nullptr;
      setupCamera();
    }

    while(!camera->hasImage())
    {
      cameraOk = camera->captureNew(maxWaitForImage);

      if(!cameraOk)
      {
        BH_TRACE_MSG("Camera broken");
        break;
      }
    }

    if(camera->hasImage())
      imageReceived = Time::getRealSystemTime();
    else if(Time::getRealTimeSince(imageReceived) > resetDelay)
    {
      delete camera;
      camera = new NaoCamera(whichCamera == CameraInfo::upper ?
                             "/dev/video-top" : "/dev/video-bottom",
                             cameraInfo.camera,
                             cameraInfo.width, cameraInfo.height, whichCamera == CameraInfo::upper,
                             theCameraSettings.cameras[whichCamera], theAutoExposureWeightTable.tables[whichCamera]);
      imageReceived = Time::getRealSystemTime();
    }

    if(camera->hasImage())
      camera->setSettings(theCameraSettings.cameras[whichCamera], theAutoExposureWeightTable.tables[whichCamera]);

    imageTaken.post();

    if(camera->hasImage())
      camera->writeCameraSettings();
  }
#endif
}

void CameraProvider::waitForFrameData()
{
#ifdef CAMERA_INCLUDED
  if(theInstance)
  {
    theInstance->takeNextImage.post();
    theInstance->imageTaken.wait();
  }
#endif
}
