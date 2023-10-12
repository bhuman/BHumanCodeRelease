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
#include "Platform/SystemCall.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/Stopwatch.h"
#include "Streaming/Global.h"
#include "Tools/Math/Projection.h"
#include "Tools/Math/Transformation.h"

MAKE_MODULE(BallPerceptor);

BallPerceptor::BallPerceptor() :
  encoder(&Global::getAsmjitRuntime()),
  classifier(&Global::getAsmjitRuntime()),
  corrector(&Global::getAsmjitRuntime())
{
  compile();
}

void BallPerceptor::update(BallPercept& theBallPercept)
{
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:spots", "drawingOnImage");

  DEBUG_RESPONSE_ONCE("module:BallPerceptor:compile")
    compile();

  theBallPercept.status = BallPercept::notSeen;

  if(!encoder.valid() || !classifier.valid() || !corrector.valid())
    return;

  const std::vector<Vector2i>& ballSpots = theBallSpots.ballSpots;
  if(ballSpots.empty())
    return;

  float prob, bestProb = guessedThreshold;
  Vector2f ballPosition, bestBallPosition;
  float radius, bestRadius;
  for(std::size_t i = 0; i < ballSpots.size(); ++i)
  {
    prob = apply(ballSpots[i], ballPosition, radius);

    COMPLEX_DRAWING("module:BallPerceptor:spots")
    {
      std::stringstream ss;
      ss << i << ": " << static_cast<int>(prob * 100);
      DRAW_TEXT("module:BallPerceptor:spots", ballSpots[i].x(), ballSpots[i].y(), 15, ColorRGBA::red, ss.str());
    }

    if(prob > bestProb)
    {
      bestProb = prob;
      bestBallPosition = ballPosition;
      bestRadius = radius;
      if(SystemCall::getMode() == SystemCall::physicalRobot && prob >= ensureThreshold)
        break;
    }
  }

  if(bestProb > guessedThreshold)
  {
    theBallPercept.positionInImage = bestBallPosition;
    theBallPercept.radiusInImage = bestRadius;

    // TL: Function is called with hardcoded numbers here, as this parameter is only for backward compatibility to previous approach and should vanish hopefully soon.
    if(theMeasurementCovariance.transformWithCovLegacy(theBallPercept.positionInImage, theBallSpecification.radius, Vector2f(0.04f, 0.06f),
                                                       theBallPercept.positionOnField, theBallPercept.covarianceOnField))
    {
      theBallPercept.status = bestProb >= acceptThreshold ? BallPercept::seen : BallPercept::guessed;
      return;
    }
  }

  // Special ball handling for penalty goal keeper
  if((theGameState.state == GameState::opponentPenaltyShot || theGameState.state == GameState::opponentPenaltyKick) &&
     theMotionInfo.isKeyframeMotion(KeyframeMotionRequest::sitDownKeeper))
  {
    Vector2f inImageLowPoint;
    Vector2f inImageUpPoint;
    if(Transformation::robotToImage(theRobotPose.inverse() * Vector2f(theFieldDimensions.xPosOwnGoalArea + 350.f, 0.f), theCameraMatrix, theCameraInfo, inImageLowPoint)
       && Transformation::robotToImage(theRobotPose.inverse() * Vector2f(theFieldDimensions.xPosOwnPenaltyMark, 0.f), theCameraMatrix, theCameraInfo, inImageUpPoint))
    {
      const int lowerY = std::min(static_cast<int>(inImageLowPoint.y()), theCameraInfo.height);
      const int upperY = std::max(static_cast<int>(inImageUpPoint.y()), 0);

      std::vector<Vector2i> sortedBallSpots = theBallSpots.ballSpots;
      std::sort(sortedBallSpots.begin(), sortedBallSpots.end(), [&](const Vector2i& a, const Vector2i& b) {return a.y() > b.y(); });

      for(const Vector2i& spot : sortedBallSpots)
        if(spot.y() < lowerY && spot.y() > upperY)
        {
          theBallPercept.positionInImage = spot.cast<float>();
          // TL: Function is called with hardcoded numbers here, as this parameter is only for backward compatibility to previous approach and should vanish hopefully soon.
          if(theMeasurementCovariance.transformWithCovLegacy(theBallPercept.positionInImage, theBallSpecification.radius, Vector2f(0.04f, 0.06f),
                                                             theBallPercept.positionOnField, theBallPercept.covarianceOnField))
          {
            theBallPercept.status = BallPercept::seen;
          }

          theBallPercept.radiusInImage = 30.f;
        }
    }
  }
}

float BallPerceptor::apply(const Vector2i& ballSpot, Vector2f& ballPosition, float& predRadius)
{
  Vector2f relativePoint;
  Geometry::Circle ball;
  if(!(Transformation::imageToRobotHorizontalPlane(ballSpot.cast<float>(), theBallSpecification.radius, theCameraMatrix, theCameraInfo, relativePoint)
       && Projection::calculateBallInImage(relativePoint, theCameraMatrix, theCameraInfo, theBallSpecification.radius, ball)))
    return -1.f;

  int ballArea = static_cast<int>(ball.radius * ballAreaFactor);
  ballArea += 4 - (ballArea % 4);

  RECTANGLE("module:BallPerceptor:spots", static_cast<int>(ballSpot.x() - ballArea / 2), static_cast<int>(ballSpot.y() - ballArea / 2), static_cast<int>(ballSpot.x() + ballArea / 2), static_cast<int>(ballSpot.y() + ballArea / 2), 2, Drawings::PenStyle::solidPen, ColorRGBA::black);

  STOPWATCH("module:BallPerceptor:getImageSection")
    if(useFloat)
    {
      PatchUtilities::extractPatch(ballSpot, Vector2i(ballArea, ballArea), Vector2i(patchSize, patchSize), theECImage.grayscaled, encoder.input(0).data(), extractionMode);
      switch(normalizationMode)
      {
        case BallPerceptorModule::Params::normalizeContrast:
          PatchUtilities::normalizeContrast(encoder.input(0).data(), Vector2i(patchSize, patchSize), normalizationOutlierRatio);
          break;
        case BallPerceptorModule::Params::normalizeBrightness:
          PatchUtilities::normalizeBrightness(encoder.input(0).data(), Vector2i(patchSize, patchSize), normalizationOutlierRatio);
      }
    }
    else
    {
      PatchUtilities::extractPatch(ballSpot, Vector2i(ballArea, ballArea), Vector2i(patchSize, patchSize), theECImage.grayscaled, reinterpret_cast<unsigned char*>(encoder.input(0).data()), extractionMode);
      switch(normalizationMode)
      {
        case BallPerceptorModule::Params::normalizeContrast:
          PatchUtilities::normalizeContrast(reinterpret_cast<unsigned char*>(encoder.input(0).data()), Vector2i(patchSize, patchSize), normalizationOutlierRatio);
          break;
        case BallPerceptorModule::Params::normalizeBrightness:
          PatchUtilities::normalizeBrightness(reinterpret_cast<unsigned char*>(encoder.input(0).data()), Vector2i(patchSize, patchSize), normalizationOutlierRatio);
      }
    }
  const float stepSize = static_cast<float>(ballArea) / static_cast<float>(patchSize);

  // encode patch
  encoder.apply();

  // classify
  classifier.input(0) = encoder.output(0);
  classifier.apply();
  const float pred = classifier.output(0)[0];

  // predict ball position if poss for ball is high enough
  if(pred > guessedThreshold)
  {
    corrector.input(0) = encoder.output(0);
    corrector.apply();
    ballPosition.x() = (corrector.output(0)[0] - patchSize / 2) * stepSize + ballSpot.x();
    ballPosition.y() = (corrector.output(0)[1] - patchSize / 2) * stepSize + ballSpot.y();
    predRadius = corrector.output(0)[2] * stepSize;
  }

  return pred;
}

void BallPerceptor::compile()
{
  const std::string baseDir = std::string(File::getBHDir()) + "/Config/NeuralNets/BallPerceptor/";
  encModel = std::make_unique<NeuralNetwork::Model>(baseDir + encoderName);
  clModel = std::make_unique<NeuralNetwork::Model>(baseDir + classifierName);
  corModel = std::make_unique<NeuralNetwork::Model>(baseDir + correctorName);

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
