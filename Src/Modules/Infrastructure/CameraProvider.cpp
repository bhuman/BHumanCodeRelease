/**
* @file CameraProvider.cpp
* This file declares a module that provides camera images.
* @author Colin Graf
*/

#include <cstdio>

#include "CameraProvider.h"
#include "Platform/SystemCall.h"
#include "Representations/Infrastructure/JPEGImage.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Debugging/Stopwatch.h"

PROCESS_LOCAL CameraProvider* CameraProvider::theInstance = 0;

CameraProvider::CameraProvider() :
  upperCameraInfo(CameraInfo::upper), lowerCameraInfo(CameraInfo::lower),
  upperCameraSettings(CameraInfo::upper), lowerCameraSettings(CameraInfo::lower)
#ifdef CAMERA_INCLUDED
  , imageTimeStamp(0), otherImageTimeStamp(0), lastImageTimeStamp(0), lastImageTimeStampLL(0)
#endif
{
  VERIFY(readCameraSettings());
  VERIFY(readCameraIntrinsics());
  VERIFY(readCameraResolution());

  setupCameras();
  theInstance = this;
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
  MODIFY_ONCE("representation:UpperCameraSettings", upperCameraSettings);
  MODIFY_ONCE("representation:LowerCameraSettings", lowerCameraSettings);
  DEBUG_RESPONSE_ONCE("module:CameraProvider:LoadCameraSettings") readCameraSettings();
#ifdef CAMERA_INCLUDED
  ASSERT(!currentImageCamera);
  if(upperCamera->hasImage() && (!lowerCamera->hasImage() || upperCamera->getTimeStamp() < lowerCamera->getTimeStamp()))
  {
    image.setResolution(upperCameraInfo.width, upperCameraInfo.height);
    image.setImage(const_cast<unsigned char*>(upperCamera->getImage()));
    lastImageTimeStampLL = upperCamera->getTimeStamp();
    imageTimeStamp = image.timeStamp = std::max(lastImageTimeStamp + 1, (unsigned)(upperCamera->getTimeStamp() / 1000) - SystemCall::getSystemTimeBase());
    upperCamera->setSettings(upperCameraSettings);
    DEBUG_RESPONSE_ONCE("module:CameraProvider:DoWhiteBalanceUpper") upperCamera->doAutoWhiteBalance();
    DEBUG_RESPONSE_ONCE("module:CameraProvider:ReadCameraSettingsUpper") upperCamera->readCameraSettings();
    upperCamera->writeCameraSettings();
    upperCameraSettings = upperCamera->getSettings();
    currentImageCamera = upperCamera;
  }
  else if(lowerCamera->hasImage())
  {
    image.setResolution(lowerCameraInfo.width, lowerCameraInfo.height);
    image.setImage(const_cast<unsigned char*>(lowerCamera->getImage()));
    lastImageTimeStampLL = lowerCamera->getTimeStamp();
    otherImageTimeStamp = image.timeStamp = std::max(lastImageTimeStamp + 1, (unsigned)(lowerCamera->getTimeStamp() / 1000) - SystemCall::getSystemTimeBase());
    lowerCamera->setSettings(lowerCameraSettings);
    DEBUG_RESPONSE_ONCE("module:CameraProvider:DoWhiteBalanceLower") lowerCamera->doAutoWhiteBalance();
    DEBUG_RESPONSE_ONCE("module:CameraProvider:ReadCameraSettingsLower") lowerCamera->readCameraSettings();
    lowerCamera->writeCameraSettings();
    lowerCameraSettings = lowerCamera->getSettings();
    currentImageCamera = lowerCamera;
  }
  ASSERT(image.timeStamp >= lastImageTimeStamp);
  lastImageTimeStamp = image.timeStamp;
  MODIFY("module:CameraProvider:fullSize", image.isFullSize);
#endif // CAMERA_INCLUDED
  STOPWATCH("compressJPEG")
  {
    DEBUG_RESPONSE("representation:JPEGImage") OUTPUT(idJPEGImage, bin, JPEGImage(image));
  }
}

void CameraProvider::update(FrameInfo& frameInfo)
{
  frameInfo.time = theImage.timeStamp;
  frameInfo.cycleTime = cycleTime * 0.5f;
}

void CameraProvider::update(CameraInfo& cameraInfo)
{
  if(currentImageCamera == upperCamera)
    cameraInfo = upperCameraInfo;
  else
    cameraInfo = lowerCameraInfo;
}

void CameraProvider::update(CameraSettings& cameraSettings)
{
  if(currentImageCamera == upperCamera)
    cameraSettings = upperCameraSettings;
  else
    cameraSettings = lowerCameraSettings;
}

void CameraProvider::update(CameraIntrinsics& cameraIntrinsics)
{
  cameraIntrinsics = this->cameraIntrinsics;
}

void CameraProvider::update(CameraResolution& cameraResolution)
{
  cameraResolution = this->cameraResolution;
}

bool CameraProvider::readCameraSettings()
{
  InMapFile upperStream("upperCameraSettings.cfg");
  InMapFile lowerStream("lowerCameraSettings.cfg");
  bool exist = upperStream.exists() && lowerStream.exists();
  if(exist)
  {
    upperStream >> upperCameraSettings;
    lowerStream >> lowerCameraSettings;
  }
  return exist;
}

bool CameraProvider::readCameraIntrinsics()
{
  InMapFile stream("cameraIntrinsics.cfg");
  bool exist = stream.exists();
  if(exist)
  {
    stream >> cameraIntrinsics;
  }
  return exist;
}

bool CameraProvider::readCameraResolution()
{
  InMapFile stream("cameraResolution.cfg");
  bool exist = stream.exists();
  if(exist)
  {
    stream >> cameraResolution;
  }
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
    switch(theCameraResolutionRequest.resolution)
    {
      case CameraResolution::Resolutions::noRequest:
        break;
      case CameraResolution::Resolutions::defaultRes:
        if(!readCameraResolution())
        {
          cameraResolution.resolution = CameraResolution::Resolutions::upper640;
          cameraResolution.timestamp = theCameraResolutionRequest.timestamp;
        }
        break;
      case CameraResolution::Resolutions::upper640:
      case CameraResolution::Resolutions::lower640:
      case CameraResolution::Resolutions::both320:
        cameraResolution.resolution = theCameraResolutionRequest.resolution;
        cameraResolution.timestamp = theCameraResolutionRequest.timestamp;
        break;
      default:
        ASSERT(false);
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
  switch(cameraResolution.resolution)
  {
    case CameraResolution::Resolutions::upper640:
      upperCameraInfo.width = 640;
      upperCameraInfo.height = 480;
      lowerCameraInfo.width = 320;
      lowerCameraInfo.height = 240;
      break;
    case CameraResolution::Resolutions::lower640:
      upperCameraInfo.width = 320;
      upperCameraInfo.height = 240;
      lowerCameraInfo.width = 640;
      lowerCameraInfo.height = 480;
      break;
    case CameraResolution::Resolutions::both320:
      upperCameraInfo.width = 320;
      upperCameraInfo.height = 240;
      lowerCameraInfo.width = 320;
      lowerCameraInfo.height = 240;
      break;
    case CameraResolution::Resolutions::both640:
      upperCameraInfo.width = 640;
      upperCameraInfo.height = 480;
      lowerCameraInfo.width = 640;
      lowerCameraInfo.height = 480;
      break;
    default:
      ASSERT(false);
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
  if(upperCamera != nullptr)
    delete upperCamera;
  if(lowerCamera != nullptr)
    delete lowerCamera;
  upperCamera = new NaoCamera("/dev/video0", upperCameraInfo.camera, upperCameraInfo.width, upperCameraInfo.height, true);
  lowerCamera = new NaoCamera("/dev/video1", lowerCameraInfo.camera, lowerCameraInfo.width, lowerCameraInfo.height, false);
  cycleTime = upperCamera->getFrameRate();
  ASSERT(upperCamera->getFrameRate() == lowerCamera->getFrameRate());
#else
  upperCamera = lowerCamera = nullptr;
  cycleTime = 1.f / 30.f;
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

  // update resolution
  if(processResolutionRequest())
  {
    setupCameras();
  }
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

MAKE_MODULE(CameraProvider, cognitionInfrastructure)
