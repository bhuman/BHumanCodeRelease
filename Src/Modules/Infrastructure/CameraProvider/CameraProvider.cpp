/**
 * @file CameraProvider.cpp
 * This file declares a module that provides camera images.
 * @author Colin Graf
 */

#include <cstdio>

#include "CameraProvider.h"
#include "Platform/SystemCall.h"
#include "Platform/Time.h"
#include "Representations/Infrastructure/JPEGImage.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Debugging/Stopwatch.h"
#include "Tools/Debugging/Annotation.h"

thread_local CameraProvider* CameraProvider::theInstance = nullptr;

CameraProvider::CameraProvider() :
  upperCameraInfo(CameraInfo::upper), lowerCameraInfo(CameraInfo::lower)
{
  VERIFY(readCameraIntrinsics());
  VERIFY(readCameraResolution());

  setupCameras();
  theInstance = this;
  waitForFrameData2();
}

CameraProvider::~CameraProvider()
{
#ifdef CAMERA_INCLUDED
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

void CameraProvider::update(Image& image)
{
#ifdef CAMERA_INCLUDED
  ASSERT(!currentImageCamera);
  if(upperCamera->hasImage() && (!lowerCamera->hasImage() || upperCamera->getTimeStamp() < lowerCamera->getTimeStamp()))
    useImage(true, std::max(lastImageTimeStamp + 1, (unsigned)(upperCamera->getTimeStamp() / 1000) - Time::getSystemTimeBase()), upperCameraInfo, image, upperCamera,
             const_cast<CameraSettings::CameraSettingsCollection&>(theCameraSettings.upper), const_cast<AutoExposureWeightTable&>(theAutoExposureWeightTable));
  else
    useImage(false, std::max(lastImageTimeStamp + 1, (unsigned)(lowerCamera->getTimeStamp() / 1000) - Time::getSystemTimeBase()), lowerCameraInfo, image, lowerCamera,
             const_cast<CameraSettings::CameraSettingsCollection&>(theCameraSettings.lower), const_cast<AutoExposureWeightTable&>(theAutoExposureWeightTable));
  ASSERT(image.timeStamp >= lastImageTimeStamp);
  lastImageTimeStamp = image.timeStamp;
  MODIFY("module:CameraProvider:fullSize", image.isFullSize);
#endif // CAMERA_INCLUDED
  STOPWATCH("compressJPEG")
  {
    DEBUG_RESPONSE("representation:JPEGImage") OUTPUT(idJPEGImage, bin, JPEGImage(image));
  }
}

void CameraProvider::update(CameraStatus& cameraStatus)
{
#ifdef CAMERA_INCLUDED
  cameraStatus.ok = Time::getRealSystemTime() < timeWhenCamerasWereOk + notOkDelay;
#endif
}

void CameraProvider::useImage(bool isUpper, unsigned timestamp, CameraInfo& cameraInfo, Image& image, NaoCamera* naoCam,
                              CameraSettings::CameraSettingsCollection& settings, AutoExposureWeightTable& autoExposureWeightTable)
{
#ifdef CAMERA_INCLUDED
  image.setResolution(cameraInfo.width / 2, cameraInfo.height / 2, true);
  image.setImage(const_cast<unsigned char*>(naoCam->getImage()));
  image.timeStamp = timestamp;

  naoCam->setSettings(settings, autoExposureWeightTable);
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
  naoCam->writeCameraSettings();
  settings = naoCam->getCameraSettingsCollection();
  autoExposureWeightTable = naoCam->getAutoWhiteBalanceTable();
  currentImageCamera = naoCam;
  lastImageTimeStampLL = naoCam->getTimeStamp();
#endif // CAMERA_INCLUDED
}

void CameraProvider::update(FrameInfo& frameInfo)
{
  frameInfo.time = theImage.timeStamp;
}

void CameraProvider::update(CameraInfo& cameraInfo)
{
  if(currentImageCamera == upperCamera)
    cameraInfo = upperCameraInfo;
  else
    cameraInfo = lowerCameraInfo;
}

void CameraProvider::update(CameraIntrinsics& cameraIntrinsics)
{
  cameraIntrinsics = this->cameraIntrinsics;
}

void CameraProvider::update(CameraResolution& cameraResolution)
{
  cameraResolution = this->cameraResolution;
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
                                theCameraSettings.upper, theAutoExposureWeightTable);
  else
    upperCamera->changeResolution(upperCameraInfo.width, upperCameraInfo.height);
  if(lowerCamera == nullptr)
    lowerCamera = new NaoCamera("/dev/video1", lowerCameraInfo.camera, lowerCameraInfo.width, lowerCameraInfo.height, false,
                                theCameraSettings.lower, theAutoExposureWeightTable);
  else
    lowerCamera->changeResolution(lowerCameraInfo.width, lowerCameraInfo.height);

  upperImageReceived = lowerImageReceived = Time::getRealSystemTime() + maxDelayAfterInit;
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

void CameraProvider::waitForFrameData2()
{
#ifdef CAMERA_INCLUDED
  // update cameraIntrinsics
  if(theCameraIntrinsicsNext.hasNext())
  {
    cameraIntrinsics = const_cast<CameraIntrinsicsNext&>(theCameraIntrinsicsNext).getNext();
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
  }

  if(currentImageCamera)
  {
    currentImageCamera->releaseImage();
    currentImageCamera = nullptr;
  }

  // update resolution
  if(processResolutionRequest())
  {
    setupCameras();
  }

  while(!upperCamera->hasImage() && !lowerCamera->hasImage())
  {
    bool resetUpper = false;
    bool resetLower = false;
    if(!NaoCamera::captureNew(*upperCamera, *lowerCamera, maxWaitForImage, resetUpper, resetLower))
    {
      ANNOTATION("CameraProvider", "polling cameras timed out.");
      OUTPUT_ERROR("CameraProvider: Poll failed. Resetting both cameras");
      resetUpper = resetLower = true;
    }
    else
    {
      if(resetUpper)
      {
        ANNOTATION("CameraProvider", "Capturing image failed. Resetting upper camera.");
        OUTPUT_ERROR("CameraProvider: Capturing image failed. Resetting upper camera.");
      }
      if(resetLower)
      {
        ANNOTATION("CameraProvider", "Capturing image failed. Resetting lower camera.");
        OUTPUT_ERROR("CameraProvider: Capturing image failed. Resetting lower camera.");
      }
    }

    unsigned now = Time::getRealSystemTime();
    if(upperCamera->hasImage())
      upperImageReceived = now;
    if(lowerCamera->hasImage())
      lowerImageReceived = now;

    if(!resetUpper && now >= upperImageReceived + maxWaitForImage)
    {
      ANNOTATION("CameraProvider", "Capturing image timed out. Resetting upper camera.");
      OUTPUT_ERROR("CameraProvider: Capturing image timed out. Resetting upper camera. " << now - upperImageReceived << " ms delay");
      resetUpper = true;
    }
    if(!resetLower && now >= lowerImageReceived + maxWaitForImage)
    {
      ANNOTATION("CameraProvider", "Capturing image timed out. Resetting lower camera.");
      OUTPUT_ERROR("CameraProvider: Capturing image timed out. Resetting lower camera." << now - lowerImageReceived << " ms delay");
      resetLower = true;
    }

    DEBUG_RESPONSE_ONCE("module:CameraProvider:resetUpper") resetUpper = true;
    DEBUG_RESPONSE_ONCE("module:CameraProvider:resetLower") resetLower = true;

    if(resetUpper)
    {
      delete upperCamera;
      upperCamera = new NaoCamera("/dev/video0", upperCameraInfo.camera, upperCameraInfo.width, upperCameraInfo.height, true,
                                  theCameraSettings.upper, theAutoExposureWeightTable);
      upperImageReceived = Time::getRealSystemTime() + maxDelayAfterInit;
      lastCameraReset = Time::getRealSystemTime();
      SystemCall::playSound("cameraReset.wav");
    }

    if(resetLower)
    {
      delete lowerCamera;
      lowerCamera = new NaoCamera("/dev/video1", lowerCameraInfo.camera, lowerCameraInfo.width, lowerCameraInfo.height, false,
                                  theCameraSettings.lower, theAutoExposureWeightTable);
      lowerImageReceived = Time::getRealSystemTime() + maxDelayAfterInit;
      lastCameraReset = Time::getRealSystemTime();
      SystemCall::playSound("cameraReset.wav");
    }

    if(now > lastCameraReset + maxWaitForImage)
      timeWhenCamerasWereOk = now;

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

MAKE_MODULE(CameraProvider, cognitionInfrastructure)
