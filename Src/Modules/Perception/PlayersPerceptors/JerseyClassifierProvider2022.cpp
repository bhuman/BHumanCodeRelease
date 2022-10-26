/**
 * @file JerseyClassifierProvider2022.cpp
 *
 * This file implements a module that uses a neural network  to classify jerseys from the upper camera.
 *
 * @author Florian Scholz
 */

#include "Platform/File.h"
#include "JerseyClassifierProvider2022.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"

MAKE_MODULE(JerseyClassifierProvider2022, perception);

JerseyClassifierProvider2022::JerseyClassifierProvider2022()
{
  loadNeuralNetworks();
}

void JerseyClassifierProvider2022::detectJersey(const ObstaclesImagePercept::Obstacle& obstacleInImage, ObstaclesFieldPercept::Obstacle& obstacleOnField)
{
  auto& network = networkArray[theGameState.ownTeam.color];
  auto& patchSize = patchSizeArray[theGameState.ownTeam.color];

  ASSERT(network.valid());

  // Calculating the borders of the provided obstacle
  const int left = std::max(0, obstacleInImage.left);
  const int right = std::min(obstacleInImage.right, static_cast<int>(theCameraImage.width * 2));
  const int top = std::max(0, obstacleInImage.top);
  const int bottom = std::min(obstacleInImage.bottom, static_cast<int>(theCameraImage.height));

  const int cropWidth = right - left;
  const int cropHeight = bottom - top;

  // Checks if the obstable width and height are large enough
  if(cropWidth < minWidth || cropHeight < minHeight)
    return;

  // Bilinear resizing of the obstacle and forwarding the image to the neural network
  resizeBilinear(theCameraImage, network.input(0).data(), patchSize, Vector2i(cropWidth, cropHeight), Vector2i(left, top));

  network.apply();

  // Results for the three classes (teammate, opponent, unknown)
  const float firstResult = network.output(0)[0];
  const float secondResult = network.output(0)[1];
  const float thirdResult = network.output(0)[2];

  const float firstExp = std::exp(firstResult);
  const float secondExp = std::exp(secondResult);
  const float thirdExp = std::exp(thirdResult);

  // The probability for each class
  const float firstProb = firstExp / (firstExp + secondExp + thirdExp);
  const float secondProb = secondExp / (secondExp + firstExp + thirdExp);
  const float thirdProb = thirdExp / (firstExp + secondExp + thirdExp);

  // The detected jersey belongs to a teammate
  if(firstProb > secondProb && firstProb > thirdProb)
  {
    DRAW_TEXT("module:JerseyClassifierProvider2022:cutouts:teammates", left, top, 20, ColorRGBA::black, std::string("Teammate: ") + std::to_string(firstProb));
    RECTANGLE("module:JerseyClassifierProvider2022:cutouts:teammates", left, top, right, bottom, 4, Drawings::solidPen, ColorRGBA::green);

    obstacleOnField.type = ObstaclesFieldPercept::Type::ownPlayer;
    obstacleOnField.probability = firstProb;
  }
  // The detected jersey belongs to an opponent
  else if(secondProb > firstProb && secondProb > thirdProb)
  {
    DRAW_TEXT("module:JerseyClassifierProvider2022:cutouts:opponent", left, top, 20, ColorRGBA::black, std::string("Opponent: ") + std::to_string(secondProb));
    RECTANGLE("module:JerseyClassifierProvider2022:cutouts:opponent", left, top, right, bottom, 4, Drawings::solidPen, ColorRGBA::red);

    obstacleOnField.type = ObstaclesFieldPercept::Type::opponentPlayer;
    obstacleOnField.probability = secondProb;
  }
  // The detected jeresy neither belongs to a teammate nor an opponent
  else
  {
    DRAW_TEXT("module:JerseyClassifierProvider2022:cutouts:unknown", left, top, 20, ColorRGBA::black, std::string("Unknown: ") + std::to_string(thirdProb));
    RECTANGLE("module:JerseyClassifierProvider2022:cutouts:unknown", left, top, right, bottom, 4, Drawings::solidPen, ColorRGBA::gray);

    obstacleOnField.type = ObstaclesFieldPercept::Type::unknown;
    obstacleOnField.probability = thirdProb;
  }
}

void JerseyClassifierProvider2022::resizeBilinear(const CameraImage& orig, float* output, const Vector2i& outSize, const Vector2i& inSize, const Vector2i& offset)
{
  const float xRatio = (static_cast<float>(inSize.x() - 1)) / (static_cast<float>(outSize.x() - 1));
  const float yRatio = (static_cast<float>(inSize.y() - 1)) / (static_cast<float>(outSize.y() - 1));

  const int xMax = offset.x() + inSize.x() - 1;
  const int yMax = offset.y() + inSize.y() - 1;

  for(int y = 0; y < outSize.y(); y++)
  {
    const float yRatioTimesY = yRatio * y + static_cast<float>(offset.y());

    const int yLower = static_cast<int>(yRatioTimesY);
    const int yHigher = std::min(yLower + 1, yMax);

    const float yWeight1 = yRatioTimesY - yLower;
    const float yWeight0 = 1.f - yWeight1;

    for(int x = 0; x < outSize.x(); x++)
    {
      const float xRatioTimesX = xRatio * x + static_cast<float>(offset.x());

      const int xLower = static_cast<int>(xRatioTimesX);
      const int xHigher = std::min(xLower + 1, xMax);

      const float xWeight1 = xRatioTimesX - xLower;
      const float xWeight0 = 1.f - xWeight1;

      const PixelTypes::YUVPixel topLeft = orig.getYUV(xLower, yLower);
      const PixelTypes::YUVPixel topRight = orig.getYUV(xHigher, yLower);
      const PixelTypes::YUVPixel bottomLeft = orig.getYUV(xLower, yHigher);
      const PixelTypes::YUVPixel bottomRight = orig.getYUV(xHigher, yHigher);

      *output++ = static_cast<float>(topLeft.y) * xWeight0 * yWeight0 +
                  static_cast<float>(topRight.y) * xWeight1 * yWeight0 +
                  static_cast<float>(bottomLeft.y) * xWeight0 * yWeight1 +
                  static_cast<float>(bottomRight.y) * xWeight1 * yWeight1;

      *output++ = static_cast<float>(topLeft.u) * xWeight0 * yWeight0 +
                  static_cast<float>(topRight.u) * xWeight1 * yWeight0 +
                  static_cast<float>(bottomLeft.u) * xWeight0 * yWeight1 +
                  static_cast<float>(bottomRight.u) * xWeight1 * yWeight1;

      *output++ = static_cast<float>(topLeft.v) * xWeight0 * yWeight0 +
                  static_cast<float>(topRight.v) * xWeight1 * yWeight0 +
                  static_cast<float>(bottomLeft.v) * xWeight0 * yWeight1 +
                  static_cast<float>(bottomRight.v) * xWeight1 * yWeight1;
    }
  }
}

/**
 * Checks for the team color and loads the models.
 */
void JerseyClassifierProvider2022::loadNeuralNetworks()
{
  FOREACH_ENUM(Settings::TeamColor, teamColor)
  {
    const std::string path = std::string(File::getBHDir()) + "/Config/NeuralNets/JerseyClassifier2022/" + TypeRegistry::getEnumName(teamColor) + ".h5";

    const File file(path, "rb", false);

    if(!file.exists())
      continue;

    auto& network = networkArray[teamColor];
    network.compile(path);

    ASSERT(network.numOfInputs() == 1);
    ASSERT(network.input(0).rank() == 3);
    ASSERT(network.input(0).dims(2) == 3);
    ASSERT(network.numOfOutputs() == 1);
    ASSERT(network.output(0).rank() == 1);
    ASSERT(network.output(0).dims(0) == 3);
    ASSERT(network.valid());

    patchSizeArray[teamColor] = Vector2i(network.input(0).dims(1), network.input(0).dims(0));
  }
}

void JerseyClassifierProvider2022::update(JerseyClassifier& theJerseyClassifier)
{
  DECLARE_DEBUG_DRAWING("module:JerseyClassifierProvider2022:cutouts:teammates", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:JerseyClassifierProvider2022:cutouts:opponent", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:JerseyClassifierProvider2022:cutouts:unknown", "drawingOnImage");

  // Invokes the color based jeresy classifier if no suitable net was found or if the image comes from the lower camera
  if(theCameraInfo.camera == CameraInfo::lower
      || !networkArray[theGameState.ownTeam.color].valid())
  {
    theJerseyClassifier.detectJersey = theJerseyClassifierOld.detectJersey;
    return;
  }

  theJerseyClassifier.detectJersey = [this](const ObstaclesImagePercept::Obstacle& obstacleInImage, ObstaclesFieldPercept::Obstacle& obstacleOnField)
  {
    return detectJersey(obstacleInImage, obstacleOnField);
  };
}
