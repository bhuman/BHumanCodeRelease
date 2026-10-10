/**
 * @file ParallelJPEGImageProvider.cpp
 *
 * This file implements a module that compresses images in parallel to the
 * current thread.
 *
 * @author Thomas Röfer
 */

#include "ParallelJPEGImageProvider.h"

MAKE_MODULE(ParallelJPEGImageProvider);

ParallelJPEGImageProvider::ParallelJPEGImageProvider()
{
  start(this, &ParallelJPEGImageProvider::compressImages);
}

ParallelJPEGImageProvider::~ParallelJPEGImageProvider()
{
  stop();
}

void ParallelJPEGImageProvider::compressImages()
{
  while(isRunning())
    while(requestedSignal.wait(100) && isRunning())
    {
      const_cast<JPEGImage&>(theJPEGImage).fromCameraImage(theCameraImage, jpegQuality);
      completedSignal.post();
    }
}

void ParallelJPEGImageProvider::update(JPEGImage& theJPEGImage)
{
  static_cast<void>(&theBallPercept);
  if(jobsRequested > jobResultsUsed && completedSignal.wait())
    ++jobResultsUsed;
  else
    theJPEGImage.fromCameraImage(theCameraImage, jpegQuality);
}

void ParallelJPEGImageProvider::update(JPEGImageRequest& theJPEGImageRequest)
{
  theJPEGImageRequest.startCompression = [this]()
  {
    ++jobsRequested;
    requestedSignal.post();
  };
}
