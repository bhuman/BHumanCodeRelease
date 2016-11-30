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
  lowFrameRateImage(nullptr)
{
  theInstance = this;
}

CognitionLogDataProvider::~CognitionLogDataProvider()
{
  if(lowFrameRateImage)
    delete lowFrameRateImage;

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
        lastImages[info.camera] = lowFrameRateImage->image;
      image = lastImages[info.camera];
    }
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

void CognitionLogDataProvider::update(ImageCoordinateSystem& imageCoordinateSystem)
{
  imageCoordinateSystem.setCameraInfo(theCameraInfo);
  DECLARE_DEBUG_DRAWING("loggedHorizon", "drawingOnImage"); // displays the horizon
  ARROW("loggedHorizon",
        imageCoordinateSystem.origin.x(),
        imageCoordinateSystem.origin.y(),
        imageCoordinateSystem.origin.x() + imageCoordinateSystem.rotation(0, 0) * 50,
        imageCoordinateSystem.origin.y() + imageCoordinateSystem.rotation(1, 0) * 50,
        0, Drawings::solidPen, ColorRGBA(255, 0, 0));
  ARROW("loggedHorizon",
        imageCoordinateSystem.origin.x(),
        imageCoordinateSystem.origin.y(),
        imageCoordinateSystem.origin.x() + imageCoordinateSystem.rotation(0, 1) * 50,
        imageCoordinateSystem.origin.y() + imageCoordinateSystem.rotation(1, 1) * 50,
        0, Drawings::solidPen, ColorRGBA(255, 0, 0));
  COMPLEX_IMAGE("corrected")
  {
    if(Blackboard::getInstance().exists("Image"))
    {
      const Image& image = (const Image&) Blackboard::getInstance()["Image"];
      corrected.setResolution(theCameraInfo.width / 2, theCameraInfo.height);
      memset(corrected[0], 0, (theCameraInfo.width / 2) * theCameraInfo.height * sizeof(PixelTypes::YUVPixel));
      int yDest = -imageCoordinateSystem.toCorrectedCenteredNeg(0, 0).y();
      for(int ySrc = 0; ySrc < theCameraInfo.height; ++ySrc)
        for(int yDest2 = -imageCoordinateSystem.toCorrectedCenteredNeg(0, ySrc).y(); yDest <= yDest2; ++yDest)
        {
          int xDest = -imageCoordinateSystem.toCorrectedCenteredNeg(0, ySrc).x() / 2;
          for(int xSrc = 0; xSrc < theCameraInfo.width; xSrc += 2)
          {
            for(int xDest2 = -imageCoordinateSystem.toCorrectedCenteredNeg(xSrc, ySrc).x() / 2; xDest <= xDest2; ++xDest)
            {
              corrected[yDest + int(theCameraInfo.opticalCenter.y() + 0.5f)][xDest + int(theCameraInfo.opticalCenter.x() + 0.5f) / 2].color = (image[ySrc / 2] + image.width * (ySrc & 1))[xSrc / 2].color;
            }
          }
        }
      SEND_DEBUG_IMAGE("corrected", corrected);
    }
  }
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
    case idFrameInfo:
      if(handle(message) && Blackboard::getInstance().exists("Image"))
        ((Image&) Blackboard::getInstance()["Image"]).timeStamp = ((const FrameInfo&) Blackboard::getInstance()["FrameInfo"]).time;
      return true;

    case idImage:
      if(handle(message) && Blackboard::getInstance().exists("FrameInfo"))
      {
        FrameInfo& frameInfo = (FrameInfo&) Blackboard::getInstance()["FrameInfo"];
        const Image& image = (const Image&) Blackboard::getInstance()["Image"];
        frameInfo.cycleTime = (float)(image.timeStamp - frameInfo.time) * 0.001f;
        frameInfo.time = image.timeStamp;
      }
      return true;

    case idImagePatches:
      if(Blackboard::getInstance().exists("Image"))
      {
        ImagePatches imagePatches;
        message.bin >> imagePatches;
        if(fillImagePatchesBackground)
          imagePatches.toImage((Image&) Blackboard::getInstance()["Image"], imagePatchesBackgroundColor);
        else
          imagePatches.toImage((Image&) Blackboard::getInstance()["Image"]);
      }
      return true;

    case idThumbnail:
      if(Blackboard::getInstance().exists("Image"))
      {
        Thumbnail thumbnail;
        message.bin >> thumbnail;
        thumbnail.toImage((Image&) Blackboard::getInstance()["Image"]);
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
