/**
 * @file CognitionLogDataProvider.cpp
 * This file implements a module that provides data replayed from a log file.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "CognitionLogDataProvider.h"
#include "Platform/Time.h"
#include "Representations/Infrastructure/Thumbnail.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Debugging/DebugImages.h"
#include <iostream>

thread_local CognitionLogDataProvider* CognitionLogDataProvider::theInstance = nullptr;

MAKE_MODULE(CognitionLogDataProvider, cognitionInfrastructure)

CognitionLogDataProvider::CognitionLogDataProvider() :
  frameDataComplete(false),
  lowFrameRateImage(nullptr),
  thumbnail(nullptr)
{
  theInstance = this;
}

CognitionLogDataProvider::~CognitionLogDataProvider()
{
  if(lowFrameRateImage)
    delete lowFrameRateImage;
  if(thumbnail)
    delete thumbnail;

  theInstance = nullptr;
}

void CognitionLogDataProvider::update(Image& image)
{
  CameraInfo& info = (CameraInfo&) Blackboard::getInstance()["CameraInfo"];

  if(SystemCall::getMode() == SystemCall::logfileReplay)
  {
    if(lowFrameRateImage)
    {
      if(lowFrameRateImage->imageUpdated)
        lastImages[info.camera].fromCameraImage(lowFrameRateImage->image);
      image = lastImages[info.camera];
    }
    else if(thumbnail)
      thumbnail->toImage(image);
    else if(info.width / 2 != image.width || info.height / 2 != image.height)
    {
      image = Image(true, info.width / 2, info.height / 2);
      image.isFullSize = true;
    }
  }

  static const float distance = 300.f;

  DECLARE_DEBUG_DRAWING3D("representation:Image", "camera");
  IMAGE3D("representation:Image", distance, 0, 0, 0, 0, 0,
          distance * theCameraInfo.width / theCameraInfo.focalLength,
          distance * theCameraInfo.height / theCameraInfo.focalLength,
          image);
  DEBUG_RESPONSE("representation:JPEGImage") OUTPUT(idJPEGImage, bin, JPEGImage(image));
}

void CognitionLogDataProvider::update(ECImage& ecImage)
{
  if(SystemCall::getMode() == SystemCall::logfileReplay && thumbnail)
    thumbnail->toECImage(ecImage);
}

void CognitionLogDataProvider::update(FieldColors& fieldColors)
{
  DEBUG_RESPONSE_ONCE("representation:FieldColors:once")
    OUTPUT(idFieldColors, bin, fieldColors);
}

bool CognitionLogDataProvider::handleMessage(InMessage& message)
{
  return theInstance && theInstance->handleMessage2(message);
}

bool CognitionLogDataProvider::isFrameDataComplete()
{
  if(!theInstance)
  {
    return true;
  }
  else if(theInstance->frameDataComplete)
  {
    OUTPUT(idLogResponse, bin, '\0');
    theInstance->frameDataComplete = false;
    return true;
  }
  else
    return false;
}

bool CognitionLogDataProvider::handleMessage2(InMessage& message)
{
  switch(message.getMessageID())
  {
    case idCameraInfo:
      if(handle(message) && Blackboard::getInstance().exists("ImageCoordinateSystem"))
        ((ImageCoordinateSystem&) Blackboard::getInstance()["ImageCoordinateSystem"]).cameraInfo = ((const CameraInfo&) Blackboard::getInstance()["CameraInfo"]);
      return true;

    case idImageCoordinateSystem:
      if(handle(message) && Blackboard::getInstance().exists("CameraInfo"))
        ((ImageCoordinateSystem&) Blackboard::getInstance()["ImageCoordinateSystem"]).cameraInfo = ((const CameraInfo&) Blackboard::getInstance()["CameraInfo"]);
      return true;

    case idFrameInfo:
      if(handle(message) && Blackboard::getInstance().exists("Image"))
        ((Image&) Blackboard::getInstance()["Image"]).timeStamp = ((const FrameInfo&) Blackboard::getInstance()["FrameInfo"]).time;
      return true;

    case idImage:
      if(handle(message) && Blackboard::getInstance().exists("FrameInfo"))
      {
        FrameInfo& frameInfo = (FrameInfo&) Blackboard::getInstance()["FrameInfo"];
        const Image& image = (const Image&) Blackboard::getInstance()["Image"];
        frameInfo.time = image.timeStamp;
      }
      return true;

    case idImagePatches:
      if(Blackboard::getInstance().exists("Image"))
      {
        ImagePatches imagePatches;
        message.bin >> imagePatches;

        CameraImage img;
        if(fillImagePatchesBackground)
        {
          PixelTypes::YUYVPixel color;
          color.color = imagePatchesBackgroundColor;
          imagePatches.toImage(img, color);
        }
        else
          imagePatches.toImage(img);
        ((Image&)Blackboard::getInstance()["Image"]).fromCameraImage(img);
      }
      return true;

    case idThumbnail:
      if(Blackboard::getInstance().exists("Image") || Blackboard::getInstance().exists("ECImage"))
      {
        if(!thumbnail)
          thumbnail = new Thumbnail;
        message.bin >> *thumbnail;
      }
      return true;

    case idProcessFinished:
      frameDataComplete = true;
      return true;

    case idStopwatch:
    {
      DEBUG_RESPONSE_NOT("timing")
      {
        const int size = message.getMessageSize();
        std::vector<unsigned char> data;
        data.resize(size);
        message.bin.read(&data[0], size);
        Global::getDebugOut().bin.write(&data[0], size);
        Global::getDebugOut().finishMessage(idStopwatch);
      }
      return true;
    }

    case idAnnotation:
    {
      const int size = message.getMessageSize();
      std::vector<unsigned char> data;
      data.resize(size);
      message.bin.read(&data[0], size);
      Global::getDebugOut().bin.write(&data[0], size);
      Global::getDebugOut().finishMessage(idAnnotation);
      return true;
    }

    case idJPEGImage:
      if(Blackboard::getInstance().exists("Image"))
      {
        JPEGImage jpegImage;
        message.bin >> jpegImage;
        jpegImage.toImage((Image&) Blackboard::getInstance()["Image"]);
      }
      if(Blackboard::getInstance().exists("FrameInfo"))
        ((FrameInfo&) Blackboard::getInstance()["FrameInfo"]).time = ((Image&) Blackboard::getInstance()["Image"]).timeStamp;
      return true;

    case idLowFrameRateImage:
      if(Blackboard::getInstance().exists("Image"))
      {
        if(!lowFrameRateImage)
          lowFrameRateImage = new LowFrameRateImage;
        message.bin >> *lowFrameRateImage;
      }
      return true;

    default:
      return LogDataProvider::handle(message);
  }
}
