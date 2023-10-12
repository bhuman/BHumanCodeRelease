/**
 * @file BOPPerceptor.cpp
 *
 * This file implements a module that runs a neural network on a full image
 * to detect balls, obstacles and penalty marks.
 *
 * @author Arne Hasselbring
 */

#include "BOPPerceptor.h"
#include "Platform/BHAssert.h"
#include "Platform/File.h"
#include "Debugging/DebugImages.h"
#include "Streaming/Global.h"
#include "ImageProcessing/Image.h"
#include "Tools/Math/InImageSizeCalculations.h"
#include "Tools/Math/Transformation.h"
#include <algorithm>
#include <cstring>
#include <type_traits>

MAKE_MODULE(BOPPerceptor);

BOPPerceptor::BOPPerceptor() :
  network(&Global::getAsmjitRuntime())
{
  model = std::make_unique<NeuralNetwork::Model>(std::string(File::getBHDir()) + "/Config/NeuralNets/BOP/net.h5");
  model->setInputUInt8(0);
  NeuralNetwork::CompilationSettings settings;
#if defined MACOS && defined __arm64__
  settings.useCoreML = true;
#endif
  network.compile(*model, settings);

  ASSERT(network.valid());

  ASSERT(network.numOfInputs() == 1);
  ASSERT(network.numOfOutputs() == 1);

  ASSERT(network.input(0).rank() == 3);
  inputSize = Vector2i(network.input(0).dims(1), network.input(0).dims(0));
  ASSERT(network.input(0).dims(2) == 2);

  ASSERT(network.output(0).rank() == 3);
  outputSize = Vector2i(network.output(0).dims(1), network.output(0).dims(0));
  ASSERT(network.output(0).dims(2) == numOfChannels);

  ASSERT(inputSize.x() % outputSize.x() == 0);
  ASSERT(inputSize.y() % outputSize.y() == 0);
  scale = Vector2i(inputSize.x() / outputSize.x(), inputSize.y() / outputSize.y());
}

bool BOPPerceptor::apply()
{
  DECLARE_DEBUG_RESPONSE("debug images:ball");
  DECLARE_DEBUG_RESPONSE("debug images:obstacles");
  DECLARE_DEBUG_RESPONSE("debug images:penaltyMark");

  if(lastPrediction == theCameraImage.timestamp)
    return true;

  if(theCameraInfo.width != inputSize.x() || theCameraInfo.height != inputSize.y())
    return false;

  static_assert(std::is_same<CameraImage::PixelType, PixelTypes::YUYVPixel>::value);
  // TODO: CompiledNN should be able to take an external buffer as input (but this is more complicated than one could think).
  // In the meantime, one could directly convert to float in this copy operation using SSE.
  std::memcpy(reinterpret_cast<std::uint8_t*>(network.input(0).data()), theCameraImage[0], inputSize.x() * inputSize.y() * 2);
  STOPWATCH("module:BOPPerceptor:apply")
    network.apply();

  lastPrediction = theCameraImage.timestamp;

  COMPLEX_IMAGE("ball")
  {
    GrayscaledImage ballImage(outputSize.x(), outputSize.y());
    const float* data = network.output(0).data() + ballIndex;
    for(int y = 0; y < outputSize.y(); ++y)
      for(int x = 0; x < outputSize.x(); ++x)
        ballImage[y][x] = static_cast<unsigned char>(std::max(0.f, std::min(data[(y * outputSize.x() + x) * numOfChannels] * 255.f, 255.f)));
    SEND_DEBUG_IMAGE("ball", ballImage);
  }

  COMPLEX_IMAGE("obstacles")
  {
    GrayscaledImage obstaclesImage(outputSize.x(), outputSize.y());
    const float* data = network.output(0).data() + obstaclesIndex;
    for(int y = 0; y < outputSize.y(); ++y)
      for(int x = 0; x < outputSize.x(); ++x)
        obstaclesImage[y][x] = static_cast<unsigned char>(std::max(0.f, std::min(data[(y * outputSize.x() + x) * numOfChannels] * 255.f, 255.f)));
    SEND_DEBUG_IMAGE("obstacles", obstaclesImage);
  }

  COMPLEX_IMAGE("penaltyMark")
  {
    GrayscaledImage penaltyMarkImage(outputSize.x(), outputSize.y());
    const float* data = network.output(0).data() + penaltyMarkIndex;
    for(int y = 0; y < outputSize.y(); ++y)
      for(int x = 0; x < outputSize.x(); ++x)
        penaltyMarkImage[y][x] = static_cast<unsigned char>(std::max(0.f, std::min(data[(y * outputSize.x() + x) * numOfChannels] * 255.f, 255.f)));
    SEND_DEBUG_IMAGE("penaltyMark", penaltyMarkImage);
  }

  return true;
}

void BOPPerceptor::update(BallSpots& ballSpots)
{
  ballSpots.ballSpots.clear();
  ballSpots.firstSpotIsPredicted = false;

  if(!apply())
    return;

  const float* data = network.output(0).data();

  Vector2i maxPos;
  float max = ballThreshold;
  for(int y = 0; y < outputSize.y(); ++y)
    for(int x = 0; x < outputSize.x(); ++x)
    {
      const float f = *data;
      if(f > max)
      {
        max = f;
        maxPos.x() = x;
        maxPos.y() = y;
      }
      data += numOfChannels;
    }
  if(max > ballThreshold)
    ballSpots.addBallSpot(maxPos.x() * scale.x() + scale.x() / 2, maxPos.y() * scale.y() + scale.y() / 2);
}

void BOPPerceptor::update(PenaltyMarkRegions& penaltyMarkRegions)
{
  penaltyMarkRegions.regions.clear();

  if(!apply())
    return;

  const float* data = network.output(0).data() + penaltyMarkIndex;

  Vector2i maxPos;
  float max = penaltyMarkThreshold;
  for(int y = 0; y < outputSize.y(); ++y)
    for(int x = 0; x < outputSize.x(); ++x)
    {
      const float f = *data;
      if(f > max)
      {
        max = f;
        maxPos.x() = x;
        maxPos.y() = y;
      }
      data += numOfChannels;
    }
  if(max > penaltyMarkThreshold)
  {
    const Vector2i center(maxPos.x() * scale.x() + scale.x() / 2, maxPos.y() * scale.y() + scale.y() / 2);
    float expectedWidth;
    float expectedHeight;
    static constexpr float sizeToleranceRatio = 0.5f;
    static constexpr int blockSizeX = 16;
    static constexpr int blockSizeY = 16;
    if(!IISC::calculateImagePenaltyMeasurementsByCenter(center.cast<float>(), expectedWidth, expectedHeight, theCameraInfo, theCameraMatrix, theFieldDimensions))
      return;
    // TODO: This is wrong.
    Boundaryi region(Rangei((center.x() - scale.x()) / blockSizeX * blockSizeX,
                            (center.x() + scale.x()) / blockSizeX * blockSizeX),
                     Rangei((center.y() - scale.y()) / blockSizeY * blockSizeY,
                            (center.y() + scale.y()) / blockSizeY * blockSizeY));
    const int xExtent = (static_cast<int>(expectedWidth * (1.f + sizeToleranceRatio) / 2.f) + blockSizeX - 1) / blockSizeX * blockSizeX;
    const int yExtent = (static_cast<int>(expectedHeight * (1.f + sizeToleranceRatio) / 2.f) + blockSizeY - 1) / blockSizeY * blockSizeY;
    const Boundaryi cnsRegion(Rangei(std::max(0, region.x.min - xExtent),
                                     std::min(theCameraInfo.width, region.x.max + xExtent)),
                              Rangei(std::max(0, region.y.min - yExtent),
                                     std::min(theCameraInfo.height, region.y.max + yExtent)));
    region = Boundaryi(Rangei(cnsRegion.x.min + xExtent, cnsRegion.x.max - xExtent),
                       Rangei(cnsRegion.y.min + yExtent, cnsRegion.y.max - yExtent));
    if(region.x.min >= region.x.max || region.y.min >= region.y.max)
      return;
    penaltyMarkRegions.regions.push_back(region);
  }
}

void BOPPerceptor::update(ObstacleScan& obstacleScan)
{
  obstacleScan.yLowerInImage.clear();
  obstacleScan.pointsOnField.clear();

  if(!apply())
    return;

  obstacleScan.xStepInImage = theCameraInfo.width / static_cast<unsigned int>(outputSize.x());
  obstacleScan.xOffsetInImage = obstacleScan.xStepInImage / 2;

  const int yStep = theCameraInfo.height / outputSize.y();

  const float* data = network.output(0).data() + obstaclesIndex;

  obstacleScan.yLowerInImage.resize(outputSize.x(), -1);
  obstacleScan.pointsOnField.resize(outputSize.x());
  for(int x = 0; x < outputSize.x(); ++x)
  {
    for(int y = outputSize.y() - 1; y >= 0; --y)
    {
      const float f = data[(y * outputSize.x() + x) * numOfChannels];
      if(f > obstaclesThreshold)
      {
        obstacleScan.yLowerInImage[x] = y * yStep + yStep / 2;
        if(!Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(Vector2f(obstacleScan.xOffsetInImage + x * obstacleScan.xStepInImage, obstacleScan.yLowerInImage[x])), theCameraMatrix, theCameraInfo, obstacleScan.pointsOnField[x]))
          obstacleScan.yLowerInImage[x] = -1;
        break;
      }
    }
  }
}

void BOPPerceptor::update(SegmentedObstacleImage& segmentedObstacleImage)
{
  if(!apply())
  {
    segmentedObstacleImage.obstacle.setResolution(0, 0);
    return;
  }

  segmentedObstacleImage.obstacle.setResolution(outputSize.x(), outputSize.y());
  const float* data = network.output(0).data() + obstaclesIndex;
  for(int y = 0; y < outputSize.y(); ++y)
    for(int x = 0; x < outputSize.x(); ++x, data += numOfChannels)
      segmentedObstacleImage.obstacle[y][x] = static_cast<unsigned char>(std::max(0.f, std::min(*data * 255.f, 255.f)));
}
