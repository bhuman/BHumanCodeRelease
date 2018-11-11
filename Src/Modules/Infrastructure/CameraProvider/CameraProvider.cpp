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
#include "Platform/Time.h"
#include "Representations/Infrastructure/JPEGImage.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Debugging/Stopwatch.h"
#include "Tools/Debugging/Annotation.h"
#include <cstdio>

MAKE_MODULE(CameraProvider, cognitionInfrastructure)

thread_local CameraProvider* CameraProvider::theInstance = nullptr;

CameraProvider::CameraProvider() :
  upperCameraInfo(CameraInfo::upper), lowerCameraInfo(CameraInfo::lower)
{
  VERIFY(readCameraIntrinsics());
  VERIFY(readCameraResolution());

  theInstance = this;
  setupCameras();

#ifdef CAMERA_INCLUDED
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

  currentImageCamera = nullptr;
  if(upperCamera)
  {
    delete upperCamera;
    upperCamera = nullptr;
  }
  if(lowerCamera)
  {
    delete lowerCamera;
    lowerCamera = nullptr;
  }
#endif
  theInstance = nullptr;
}

void CameraProvider::update(Image& theImage)
{
#ifdef CAMERA_INCLUDED
  ASSERT(!currentImageCamera);
  if(upperCamera->hasImage() && (!lowerCamera->hasImage() || upperCamera->getTimeStamp() < lowerCamera->getTimeStamp()))
    useImage(true, std::max(lastImageTimeStamp + 1, (unsigned)(upperCamera->getTimeStamp() / 1000) - Time::getSystemTimeBase()), upperCameraInfo, theImage, upperCamera);
  else
    useImage(false, std::max(lastImageTimeStamp + 1, (unsigned)(lowerCamera->getTimeStamp() / 1000) - Time::getSystemTimeBase()), lowerCameraInfo, theImage, lowerCamera);
  ASSERT(theImage.timeStamp >= lastImageTimeStamp);
  lastImageTimeStamp = theImage.timeStamp;
  MODIFY("module:CameraProvider:fullSize", theImage.isFullSize);
#else
  theImage.timeStamp = Time::getCurrentSystemTime();
#endif // CAMERA_INCLUDED
  STOPWATCH("compressJPEG")
  {
    DEBUG_RESPONSE("representation:JPEGImage") OUTPUT(idJPEGImage, bin, JPEGImage(theImage));
  }
}

void CameraProvider::update(CameraInfo& cameraInfo)
{
  if(currentImageCamera == upperCamera)
    cameraInfo = upperCameraInfo;
  else
    cameraInfo = lowerCameraInfo;
}

void CameraProvider::update(CameraStatus& cameraStatus)
{
  if(!camerasOk)
  {
    if(cameraStatus.ok)
    {
      ANNOTATION("CameraProvider", "Could not acquire new image.");
      SystemCall::playSound("cameraBroken.wav");
    }
#ifdef NDEBUG
    else if(!SystemCall::soundIsPlaying())
      SystemCall::playSound("cameraBroken.wav");
#endif
  }

  cameraStatus.ok = camerasOk;
}

void CameraProvider::useImage(bool isUpper, unsigned timestamp, CameraInfo& cameraInfo, Image& image, NaoCamera* naoCam)
{
#ifdef CAMERA_INCLUDED
  image.setResolution(cameraInfo.width / 2, cameraInfo.height / 2, true);
  if(naoCam->hasImage())
    image.setImage(const_cast<unsigned char*>(naoCam->getImage()));
  else if(image.isReference)
  {
    image.setImage(new Image::Pixel[Image::maxResolutionWidth * Image::maxResolutionHeight * 2]);
    image.isReference = false;
  }
  image.timeStamp = timestamp;

  if(isUpper)
  {
    DEBUG_RESPONSE_ONCE("module:CameraProvider:doWhiteBalanceUpper") naoCam->doAutoWhiteBalance();
    DEBUG_RESPONSE_ONCE("module:CameraProvider:readCameraSettingsUpper") naoCam->readCameraSettings();
  }
  else
  {
    DEBUG_RESPONSE_ONCE("module:CameraProvider:doWhiteBalanceLower") naoCam->doAutoWhiteBalance();
    DEBUG_RESPONSE_ONCE("module:CameraProvider:readCameraSettingsLower") naoCam->readCameraSettings();
  }
  currentImageCamera = naoCam;
  lastImageTimeStampLL = naoCam->getTimeStamp();
#endif // CAMERA_INCLUDED
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
    stream >> cameraResolution;
  return exist;
}

bool CameraProvider::processResolutionRequest()
{
  if(SystemCall::getMode() != SystemCall::Mode::physicalRobot)
  {
    return false;
  }
  if(theCameraResolutionRequest.timestamp > cameraResolution.timestamp)
  {
    switch(theCameraResolutionRequest.resolutionUpper)
    {
      case CameraResolution::Resolutions::noRequest:
        break;
      case CameraResolution::Resolutions::defaultRes:
      {
        const CameraResolution::Resolutions tmp = cameraResolution.resolutionLower;
        if(!readCameraResolution())
        {
          cameraResolution.resolutionUpper = CameraResolution::Resolutions::w640h480;
          cameraResolution.timestamp = theCameraResolutionRequest.timestamp;
        }
        cameraResolution.resolutionLower = tmp;
        break;
      }
      case CameraResolution::Resolutions::w320h240:
      case CameraResolution::Resolutions::w640h480:
      case CameraResolution::Resolutions::w1280h960:
        cameraResolution.resolutionUpper = theCameraResolutionRequest.resolutionUpper;
        cameraResolution.timestamp = theCameraResolutionRequest.timestamp;
        break;
      default:
        FAIL("Unknown resolution.");
        return false;
    }
    switch(theCameraResolutionRequest.resolutionLower)
    {
      case CameraResolution::Resolutions::noRequest:
        break;
      case CameraResolution::Resolutions::defaultRes:
      {
        const CameraResolution::Resolutions tmp = cameraResolution.resolutionUpper;
        if(!readCameraResolution())
        {
          cameraResolution.resolutionLower = CameraResolution::Resolutions::w320h240;
          cameraResolution.timestamp = theCameraResolutionRequest.timestamp;
        }
        cameraResolution.resolutionUpper = tmp;
        break;
      }
      case CameraResolution::Resolutions::w320h240:
      case CameraResolution::Resolutions::w640h480:
      case CameraResolution::Resolutions::w1280h960:
        cameraResolution.resolutionLower = theCameraResolutionRequest.resolutionLower;
        cameraResolution.timestamp = theCameraResolutionRequest.timestamp;
        break;
      default:
        FAIL("Unknown resolution.");
        return false;
    }
    return true;
  }
  else
    return false;
}

void CameraProvider::setupCameras()
{
  // set resolution
  switch(cameraResolution.resolutionUpper)
  {
    case CameraResolution::Resolutions::w320h240:
      upperCameraInfo.width = 320;
      upperCameraInfo.height = 240;
      break;
    case CameraResolution::Resolutions::w640h480:
      upperCameraInfo.width = 640;
      upperCameraInfo.height = 480;
      break;
    case CameraResolution::Resolutions::w1280h960:
      upperCameraInfo.width = 1280;
      upperCameraInfo.height = 960;
      break;
    default:
      FAIL("Unknown resolution.");
      break;
  }
  switch(cameraResolution.resolutionLower)
  {
    case CameraResolution::Resolutions::w320h240:
      lowerCameraInfo.width = 320;
      lowerCameraInfo.height = 240;
      break;
    case CameraResolution::Resolutions::w640h480:
      lowerCameraInfo.width = 640;
      lowerCameraInfo.height = 480;
      break;
    case CameraResolution::Resolutions::w1280h960:
      lowerCameraInfo.width = 1280;
      lowerCameraInfo.height = 960;
      break;
    default:
      FAIL("Unknown resolution.");
      break;
  }

  // set opening angle
  upperCameraInfo.openingAngleWidth = cameraIntrinsics.upperOpeningAngleWidth;
  upperCameraInfo.openingAngleHeight = cameraIntrinsics.upperOpeningAngleHeight;
  lowerCameraInfo.openingAngleWidth = cameraIntrinsics.lowerOpeningAngleWidth;
  lowerCameraInfo.openingAngleHeight = cameraIntrinsics.lowerOpeningAngleHeight;
  // set optical center
  upperCameraInfo.opticalCenter.x() = cameraIntrinsics.upperOpticalCenter.x() * upperCameraInfo.width;
  upperCameraInfo.opticalCenter.y() = cameraIntrinsics.upperOpticalCenter.y() * upperCameraInfo.height;
  lowerCameraInfo.opticalCenter.x() = cameraIntrinsics.lowerOpticalCenter.x() * lowerCameraInfo.width;
  lowerCameraInfo.opticalCenter.y() = cameraIntrinsics.lowerOpticalCenter.y() * lowerCameraInfo.height;
  // update focal length
  upperCameraInfo.updateFocalLength();
  lowerCameraInfo.updateFocalLength();
#ifdef CAMERA_INCLUDED
  currentImageCamera = nullptr;
  if(upperCamera == nullptr)
    upperCamera = new NaoCamera("/dev/video0", upperCameraInfo.camera, upperCameraInfo.width, upperCameraInfo.height, true,
                                theCameraSettings.upper, theAutoExposureWeightTable.tables[CameraInfo::upper]);
  else
    upperCamera->changeResolution(upperCameraInfo.width, upperCameraInfo.height);
  if(lowerCamera == nullptr)
    lowerCamera = new NaoCamera("/dev/video1", lowerCameraInfo.camera, lowerCameraInfo.width, lowerCameraInfo.height, false,
                                theCameraSettings.lower, theAutoExposureWeightTable.tables[CameraInfo::lower]);
  else
    lowerCamera->changeResolution(lowerCameraInfo.width, lowerCameraInfo.height);

  ASSERT(upperCamera->getFrameRate() == lowerCamera->getFrameRate());
#else
  upperCamera = lowerCamera = nullptr;
#endif
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

void CameraProvider::takeImages()
{
#ifdef CAMERA_INCLUDED
  BH_TRACE_INIT("CameraProvider");
  Thread::nameThread("CameraProvider");
  thread.setPriority(11);
  while(thread.isRunning())
  {
    takeNextImage.wait();

    if(currentImageCamera)
    {
      currentImageCamera->releaseImage();
      currentImageCamera = nullptr;
    }

    // update resolution
    if(processResolutionRequest())
      setupCameras();

    while(!upperCamera->hasImage() && !lowerCamera->hasImage())
    {
      camerasOk = NaoCamera::captureNew(*upperCamera, *lowerCamera, maxWaitForImage);

      if(!camerasOk)
      {
        BH_TRACE_MSG("Camera broken");
        break;
      }

      if(upperCamera->hasImage() && upperCamera->getTimeStamp() < lastImageTimeStampLL)
        upperCamera->releaseImage();
      if(lowerCamera->hasImage() && lowerCamera->getTimeStamp() < lastImageTimeStampLL)
        lowerCamera->releaseImage();
    }

    if(upperCamera->hasImage() && (!lowerCamera->hasImage() || upperCamera->getTimeStamp() < lowerCamera->getTimeStamp()))
      upperCamera->setSettings(theCameraSettings.upper, theAutoExposureWeightTable.tables[CameraInfo::upper]);
    else
      lowerCamera->setSettings(theCameraSettings.lower, theAutoExposureWeightTable.tables[CameraInfo::lower]);

    imageTaken.post();

    if(upperCamera->hasImage() && (!lowerCamera->hasImage() || upperCamera->getTimeStamp() < lowerCamera->getTimeStamp()))
      upperCamera->writeCameraSettings();
    else
      lowerCamera->writeCameraSettings();
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
