/**
 * @file BallAndPenaltyMarkPerceptor.cpp
 *
 * This file implements a module that detects balls and penalty marks in images
 * with a neural network.
 *
 * @author Bernd Poppinga
 * @author Felix Thielke
 * @author Gerrit Felsch
 * @author Lukas Gittner
 */

#include "BallAndPenaltyMarkPerceptor.h"
#include "Platform/File.h"
#include "Platform/SystemCall.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/Stopwatch.h"
#include "Streaming/Global.h"
#include "Tools/Math/Projection.h"
#include "Tools/Math/Transformation.h"
#include <filesystem>

MAKE_MODULE(BallAndPenaltyMarkPerceptor);

BallAndPenaltyMarkPerceptor::BallAndPenaltyMarkPerceptor() :
  multihead(&Global::getAsmjitRuntime())
{
  compile();
}

void BallAndPenaltyMarkPerceptor::update(PenaltyMarkPercept& thePenaltyMarkPercept)
{
  thePenaltyMarkPercept.wasSeen = false;
  updateBallAndPenaltyMarkPerceptor();
  if(theGameState.isPenaltyShootout())
  {
    if(theBallPercept.status == BallPercept::Status::seen)
    {
      thePenaltyMarkPercept.positionInImage = theBallPercept.positionInImage.cast<int>();

      if(theMeasurementCovariance.transformPointWithCov(theBallPercept.positionInImage, theBallSpecification.radius, thePenaltyMarkPercept.positionOnField,
                                                        thePenaltyMarkPercept.covarianceOnField))
        thePenaltyMarkPercept.wasSeen = true;
      return;
    }
  }
  if(bestProbPenalty >= penaltyThreshold && theBodyContour.isValidPoint(bestPenaltyPosition.cast<int>()))
  {
    thePenaltyMarkPercept.positionInImage = bestPenaltyPosition.cast<int>();
    if(theMeasurementCovariance.transformWithCovLegacy(bestPenaltyPosition, 0.f, Vector2f(0.04f, 0.06f),
                                                       thePenaltyMarkPercept.positionOnField, thePenaltyMarkPercept.covarianceOnField))
    {
      thePenaltyMarkPercept.wasSeen = true;
      return;
    }
  }
}

void BallAndPenaltyMarkPerceptor::update(BallPercept& theBallPercept)
{
  theBallPercept.status = BallPercept::notSeen;
  updateBallAndPenaltyMarkPerceptor();
  if(bestProbBall >= guessedThreshold)
  {
    theBallPercept.positionInImage = bestBallPosition;
    theBallPercept.radiusInImage = bestRadius;

    if(theMeasurementCovariance.transformWithCovLegacy(theBallPercept.positionInImage, theBallSpecification.radius, Vector2f(0.04f, 0.06f),
                                                       theBallPercept.positionOnField, theBallPercept.covarianceOnField))
    {
      theBallPercept.status = bestProbBall >= acceptThreshold ? BallPercept::seen : BallPercept::guessed;
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

void BallAndPenaltyMarkPerceptor::updateBallAndPenaltyMarkPerceptor()
{
  DECLARE_DEBUG_DRAWING("module:BallAndPenaltyMarkPerceptor:spots", "drawingOnImage");

  if(lastFrameTime == theFrameInfo.time)
    return;
  lastFrameTime = theFrameInfo.time;

  bestProbBall = 0.f;
  bestProbPenalty = 0.f;

  if(!multihead.valid())
    return;

  std::vector<Vector2i> ballSpots = theBallSpots.ballSpots;
  for(const Boundaryi& region : thePenaltyMarkRegions.regions)
  {
    const Vector2f center(region.x.getCenter(), region.y.getCenter());
    Vector2i centerI = center.cast<int>();
    ballSpots.push_back(centerI);
  }
  if(ballSpots.empty())
    return;

  float probBall, probPenalty;
  Vector2f ballPosition, penaltyPosition;

  float radius;
  std::pair<float, float> prob;

  // iterates over all ball spots in a frame and calculates the probability for ball, penalty or none
  for(std::size_t i = 0; i < ballSpots.size(); ++i)
  {
    prob = apply(ballSpots[i], ballPosition, penaltyPosition, radius);
    probBall = prob.first;
    probPenalty = prob.second;

    COMPLEX_DRAWING("module:BallAndPenaltyMarkPerceptor:spots")
    {
      std::stringstream ss;
      ss << i << ": " << static_cast<int>(probBall * 100);
      DRAW_TEXT("module:BallAndPenaltyMarkPerceptor:spots", ballSpots[i].x(), ballSpots[i].y(), 15, ColorRGBA::red, ss.str());
    }

    if(probBall > bestProbBall)
    {
      bestProbBall = probBall;
      bestBallPosition = ballPosition;
      bestRadius = radius;
      if(SystemCall::getMode() == SystemCall::physicalRobot && probBall >= ensureThreshold)
        break;
    }

    if(probPenalty > bestProbPenalty)
    {
      bestProbPenalty = probPenalty;
      bestPenaltyPosition = penaltyPosition;
    }
  }
}

std::pair<float, float> BallAndPenaltyMarkPerceptor::apply(const Vector2i& ballSpot, Vector2f& ballPosition, Vector2f& penaltyPosition, float& predRadius)
{
  Vector2f relativePoint;
  Geometry::Circle ball;
  if(!(Transformation::imageToRobotHorizontalPlane(ballSpot.cast<float>(), theBallSpecification.radius, theCameraMatrix, theCameraInfo, relativePoint)
       && Projection::calculateBallInImage(relativePoint, theCameraMatrix, theCameraInfo, theBallSpecification.radius, ball)))
    return std::make_pair(-1.f, -1.f);

  int ballArea = static_cast<int>(ball.radius * ballAreaFactor);
  ballArea += 4 - (ballArea % 4);

  RECTANGLE("module:BallAndPenaltyMarkPerceptor:spots", static_cast<int>(ballSpot.x() - ballArea / 2), static_cast<int>(ballSpot.y() - ballArea / 2), static_cast<int>(ballSpot.x() + ballArea / 2), static_cast<int>(ballSpot.y() + ballArea / 2), 2, Drawings::PenStyle::solidPen, ColorRGBA::black);
  std::vector<float> patchWithoutNormalization;
  STOPWATCH("module:BallAndPenaltyMarkPerceptor:getImageSection")
  if(useFloat)
  {
    PatchUtilities::extractPatch(ballSpot, Vector2i(ballArea, ballArea), Vector2i(patchSize, patchSize), theECImage.grayscaled, multihead.input(0).data(), extractionMode);
    switch(normalizationMode)
    {
      case normalizeContrast:
        PatchUtilities::normalizeContrast(multihead.input(0).data(), Vector2i(patchSize, patchSize), normalizationOutlierRatio);
        break;
      case normalizeBrightness:
        PatchUtilities::normalizeBrightness(multihead.input(0).data(), Vector2i(patchSize, patchSize), normalizationOutlierRatio);
    }
  }
  else
  {
    PatchUtilities::extractPatch(ballSpot, Vector2i(ballArea, ballArea), Vector2i(patchSize, patchSize), theECImage.grayscaled, reinterpret_cast<unsigned char*>(multihead.input(0).data()), extractionMode);
    switch(normalizationMode)
    {
      case normalizeContrast:
        PatchUtilities::normalizeContrast(reinterpret_cast<unsigned char*>(multihead.input(0).data()), Vector2i(patchSize, patchSize), normalizationOutlierRatio);
        break;
      case normalizeBrightness:
        PatchUtilities::normalizeBrightness(reinterpret_cast<unsigned char*>(multihead.input(0).data()), Vector2i(patchSize, patchSize), normalizationOutlierRatio);
    }
  }
  const float stepSize = static_cast<float>(ballArea) / static_cast<float>(patchSize);

  std::vector<float> patchWithNormalization;

  multihead.apply();
  const float predNegatives = multihead.output(0)[0];
  const float predPenalty = multihead.output(0)[1];
  const float predBall = multihead.output(0)[2];

  // predict ball position if poss for ball is high enough
  if(predBall >= guessedThreshold && predBall - predNegatives >= 0)
  {
    ballPosition.x() = (multihead.output(0)[3] - patchSize / 2) * stepSize + ballSpot.x();
    ballPosition.y() = (multihead.output(0)[4] - patchSize / 2) * stepSize + ballSpot.y();
    predRadius = (multihead.output(0)[5] * stepSize);
    ASSERT(predRadius > 0.f);
  }
  // ballspot position is penalty position if poss for penalty is high enough
  else if(predPenalty >= penaltyThreshold)
  {
    penaltyPosition.x() = static_cast<float>(ballSpot.x());
    penaltyPosition.y() = static_cast<float>(ballSpot.y());
    ASSERT(penaltyPosition.x() >= 0.f || penaltyPosition.y() >= 0.f);
  }
  return std::make_pair(predBall, predPenalty);
}

void BallAndPenaltyMarkPerceptor::savePatch(std::vector<float> data, const std::string filename)
{
  const std::string frameTime = std::to_string(theFrameInfo.time);
  const std::string camera = theCameraInfo.camera == CameraInfo::lower ? "lower" : "upper";
  const std::string saveDir = std::string(File::getBHDir()) + "/blub";
  std::filesystem::create_directories(saveDir);

  const std::string savePath = saveDir + "/" + frameTime + "_" + camera  + "_" + filename + ".txt";

  OutBinaryFile stream(savePath);

  std::string patchString;
  for(unsigned int i = 0; i < data.size(); i++)
  {
    stream << std::to_string(data[i]) << "\n";
  }
}

void BallAndPenaltyMarkPerceptor::compile()
{
  const std::string baseDir = std::string(File::getBHDir()) + "/Config/NeuralNets/BallAndPenaltyMarkPerceptor/";
  multiheadModel = std::make_unique<NeuralNetwork::Model>(baseDir + multiheadName);

  if(!useFloat)
  {
    multiheadModel->setInputUInt8(0);
  }

  multihead.compile(*multiheadModel);

  ASSERT(multihead.numOfInputs() == 1);

  patchSize = 32;
}
