/**
 * @file BallPerceptor.cpp
 *
 * This file implements a module that detects balls in images with a neural network.
 *
 * @author Bernd Poppinga
 * @author Felix Thielke
 * @author Gerrit Felsch
 */

#include "BallPerceptor.h"
#include "Platform/File.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/Modify.h"
#include "Tools/Debugging/Stopwatch.h"
#include "Tools/ImageProcessing/InImageSizeCalculations.h"
#include "Tools/Math/Transformation.h"
#include "Tools/NeuralNetwork/SimpleNN.h"

MAKE_MODULE(BallPerceptor, perception)

BallPerceptor::BallPerceptor() { compile(); }

void BallPerceptor::update(BallPercept& theBallPercept)
{
  MODIFY("module:BallPerceptor:stats", stats);

  DECLARE_DEBUG_DRAWING("module:BallPerceptor:spots", "drawingOnImage");
  theBallPercept.status = BallPercept::notSeen;

  if(theLabelImage.valid && theLabelImage.hasLabel(LabelImage::Annotation::Ball))
    stats.total++;

  DEBUG_RESPONSE_ONCE("module:BallPerceptor:compile")
    compile();

  if(!encoder.valid() || !classifier.valid() || !corrector.valid())
    return;

  // AnyPlaceDemo handling
  const std::vector<Vector2i>& ballSpots = theDemoConfirmedBallSpots.positionsInImage.empty() ? theBallSpots.ballSpots
                                                                                              : theDemoConfirmedBallSpots.positionsInImage;
  if(ballSpots.empty())
    return;

  probs.resize(ballSpots.size());
  int maxIdx;
  float max = 0.f;

  std::vector<Vector2f> ballPosition;
  std::vector<float> radius;
  ballPosition.resize(ballSpots.size());
  radius.resize(ballSpots.size());

  for(unsigned int i =  0; i < ballSpots.size(); i++)
  {
    float prob = apply(ballSpots[i], i, Vector2f(0, 0), ballPosition, radius);
    probs[i] = prob;

    std::stringstream ss;
    ss << i << ": " << static_cast<int>(probs[i] * 100) << "\n";
    DRAWTEXT("module:BallPerceptor:spots", ballSpots[i].x(), ballSpots[i].y(), 15, ColorRGBA::red, ss.str());

#ifdef TARGET_ROBOT
    if(probs[i] >= ensureThreshold)
    {
      theBallPercept.positionInImage = ballPosition[i];
      theBallPercept.radiusInImage = radius[i];
      if(!Transformation::imageToRobotHorizontalPlane(theImageCoordinateSystem.toCorrected(ballPosition[i]), theBallSpecification.radius, theCameraMatrix, theCameraInfo, theBallPercept.positionOnField))
        continue;
      theBallPercept.status = BallPercept::seen;
      return;
    }
#endif
  }

  max = probs.maxCoeff(&maxIdx);

  if(max > guessedThreshold)
  {
    //float estimatedRadius = IISC::getImageBallRadiusByCenter(ballPosition[maxIdx], theCameraInfo, theCameraMatrix, theBallSpecification);

    theBallPercept.positionInImage = ballPosition[maxIdx];
    theBallPercept.radiusInImage = radius[maxIdx];
    if(!Transformation::imageToRobotHorizontalPlane(theImageCoordinateSystem.toCorrected(ballPosition[maxIdx]), theBallSpecification.radius, theCameraMatrix, theCameraInfo, theBallPercept.positionOnField))
      return;
    theBallPercept.status = max >= acceptThreshold ? BallPercept::seen : BallPercept::guessed;
  }
}

// No ball is detected if this function is optimized
#ifdef WINDOWS
#pragma optimize("", off)
#endif

float BallPerceptor::apply(const Vector2i& ballSpot, int i, const Vector2f& offset, std::vector<Vector2f>& ballPosition, std::vector<float>& predRadius)
{
  Vector2f relativePoint;
  if(!Transformation::imageToRobotHorizontalPlane(ballSpot.cast<float>(), theBallSpecification.radius, theCameraMatrix, theCameraInfo, relativePoint))
    return 0.f;

  float radius = IISC::getImageBallRadiusByCenter(ballSpot.cast<float>(), theCameraInfo, theCameraMatrix, theBallSpecification);
  int ballArea = static_cast<int>(radius * ballAreaFactor);
  ballArea += 4 - (ballArea % 4);

  Vector2i ballSpot_ = ballSpot - (offset.cast<float>().array() * radius).matrix().cast<int>();
  STOPWATCH("module:BallPerceptor:getImageSection")
    if(useFloat)
    {
      PatchUtilities::extractPatch(ballSpot_, Vector2i(ballArea, ballArea), Vector2i(patchSize, patchSize), theECImage.grayscaled, encoder.input(0).data(), extractionMode);
      if(useContrastNormalization)
        PatchUtilities::normalizeContrast(encoder.input(0).data(), Vector2i(patchSize, patchSize), contrastNormalizationPercent);
    }
    else
    {
      PatchUtilities::extractPatch(ballSpot_, Vector2i(ballArea, ballArea), Vector2i(patchSize, patchSize), theECImage.grayscaled, reinterpret_cast<unsigned char*>(encoder.input(0).data()), extractionMode);
      if(useContrastNormalization)
        PatchUtilities::normalizeContrast(reinterpret_cast<unsigned char*>(encoder.input(0).data()), Vector2i(patchSize, patchSize), contrastNormalizationPercent);
    }
  float stepSize = static_cast<float>(ballArea) / static_cast<float>(patchSize);

  // encode patch
  encoder.apply();
  corrector.input(0) = encoder.output(0);
  classifier.input(0) = encoder.output(0);

  // classify
  classifier.apply();
  float pred = classifier.output(0)[0];

  // predict ball position if poss for ball is high enough
  if(pred > guessedThreshold)
  {
    corrector.apply();
    ballPosition[i][0] = (corrector.output(0)[0] - patchSize / 2) * stepSize + ballSpot[0];
    ballPosition[i][1] = (corrector.output(0)[1] - patchSize / 2) * stepSize + ballSpot[1];
    predRadius[i] = corrector.output(0)[2] * stepSize;
  }

  return pred;
}

#ifdef WINDOWS
#pragma optimize("", on)
#endif

void BallPerceptor::compile()
{
  encModel = std::make_unique<NeuralNetwork::Model>("NeuralNets/BallPerceptor/" + encoderName);
  clModel = std::make_unique<NeuralNetwork::Model>("NeuralNets/BallPerceptor/" + classifierName);
  corModel = std::make_unique<NeuralNetwork::Model>("NeuralNets/BallPerceptor/" + correctorName);

  if(!useFloat)
    encModel->setInputUInt8(0);

  encoder.compile(*encModel);
  classifier.compile(*clModel);
  corrector.compile(*corModel);

  ASSERT(encoder.numOfInputs() == 1);
  ASSERT(classifier.numOfInputs() == 1);
  ASSERT(corrector.numOfInputs() == 1);

  ASSERT(classifier.numOfOutputs() == 1);
  ASSERT(corrector.numOfOutputs() == 1);
  ASSERT(encoder.numOfOutputs() == 1);

  ASSERT(encoder.input(0).rank() == 3);
  ASSERT(encoder.input(0).dims(0) == encoder.input(0).dims(1));
  ASSERT(encoder.input(0).dims(2) == 1);

  ASSERT(classifier.output(0).rank() == 1);
  ASSERT(classifier.output(0).dims(0) == 1 && corrector.output(0).dims(0) == 3);
  patchSize = encoder.input(0).dims(0);
}
