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
  //TODO: PENALTY SHOOTOUT from penaltyMarkPercept
  thePenaltyMarkPercept.wasSeen = false;
  ballAndPenaltyMarkUpdate();
  if(theGameState.isPenaltyShootout())
  {
    if(theBallPercept.status == BallPercept::Status::seen)
    {
      thePenaltyMarkPercept.positionInImage = theBallPercept.positionInImage.cast<int>();

      // Todo use covariance of ball percept
      if(theMeasurementCovariance.transformPointWithCov(theBallPercept.positionInImage, theBallSpecification.radius, thePenaltyMarkPercept.positionOnField,
                                                        thePenaltyMarkPercept.covarianceOnField))
        thePenaltyMarkPercept.wasSeen = true;
      return;
    }
  }
  if(bestProbPenalty >= penaltyThreshold && theBodyContour.isValidPoint(bestPenaltyPosition.cast<int>()))
  {
    thePenaltyMarkPercept.positionInImage = bestPenaltyPosition.cast<int>();
    // TL: Function is called with hardcoded numbers here, as this parameter is only for backward compatibility to previous approach and should vanish hopefully soon.
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
  theBallPercept.radiusOnField = theBallSpecification.radius;
  theBallPercept.status = BallPercept::notSeen;
  ballAndPenaltyMarkUpdate();
  if(bestProbBall >= guessedThreshold)
  {
    theBallPercept.positionInImage = bestBallPosition;
    theBallPercept.radiusInImage = bestRadius;

    // TL: Function is called with hardcoded numbers here, as this parameter is only for backward compatibility to previous approach and should vanish hopefully soon.
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

void BallAndPenaltyMarkPerceptor::ballAndPenaltyMarkUpdate()
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
    //OUTPUT_TEXT("PenaltyMarkRegion: " << centerI);
    ballSpots.push_back(centerI);
  }
  if(ballSpots.empty())
    return;

  float probBall, probPenalty;
  Vector2f ballPosition, penaltyPosition;

  float radius;
  std::pair<float, float> prob;
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

  Image<PixelTypes::GrayscaledPixel> grayscaledPatch, blueChromaPatch, redChromaPatch;
  PatchUtilities::extractPatch(ballSpot, Vector2i(ballArea, ballArea), Vector2i(patchSize, patchSize), theECImage.grayscaled, grayscaledPatch, extractionMode);


  Vector2i chromaCenter = (ballSpot.array() / 2).matrix();

  Vector2i chromaInsize = (Vector2i(ballArea, ballArea).array() / 2).matrix();


  PatchUtilities::extractPatch(chromaCenter, chromaInsize, Vector2i(patchSize, patchSize), theECImage.blueChromaticity, blueChromaPatch, extractionMode);
  PatchUtilities::extractPatch(chromaCenter, chromaInsize, Vector2i(patchSize, patchSize), theECImage.redChromaticity, redChromaPatch, extractionMode);

  if(savePatches)
  {
    std::vector<float> grayVectorPatch, blueVectorPatch, redVectorPatch;
    for (unsigned int y = 0; y < patchSize; ++y)
    {
      for (unsigned int x = 0; x < patchSize; ++x)
      {
        grayVectorPatch.push_back(static_cast<float>(grayscaledPatch(x, y)));
        blueVectorPatch.push_back(static_cast<float>(blueChromaPatch(x, y)));
        redVectorPatch.push_back(static_cast<float>(redChromaPatch(x, y)));
      }
    }
    ASSERT(grayVectorPatch.size() == patchSize * patchSize);
    ASSERT(blueVectorPatch.size() == patchSize * patchSize);
    ASSERT(redVectorPatch.size() == patchSize * patchSize);
    savePatch(grayVectorPatch, blueVectorPatch, redVectorPatch, "", ballSpot);
  }
  if(emergencyLabelMode)
  {
    return std::make_pair(0.f, 0.f);
  }


  const float stepSize = static_cast<float>(ballArea) / static_cast<float>(patchSize);



  if(useGrayScaledImage)
  {
    //PatchUtilities::extractPatch(ballSpot, Vector2i(ballArea, ballArea), Vector2i(32, 32), theECImage.grayscaled, multihead.input(0).data(), extractionMode);
    std::memcpy(multihead.input(0).data(), grayscaledPatch[0], grayscaledPatch.width * grayscaledPatch.height * sizeof(PixelTypes::GrayscaledPixel));
    PatchUtilities::normalizeBrightness(reinterpret_cast<unsigned char*>(multihead.input(0).data()), Vector2i(patchSize, patchSize), normalizationOutlierRatio);
  }
  else
  {
    PixelTypes::GrayscaledPixel* yPos = grayscaledPatch[0];
    PixelTypes::GrayscaledPixel* uPos = blueChromaPatch[0];
    PixelTypes::GrayscaledPixel* vPos = redChromaPatch[0];
    PixelTypes::GrayscaledPixel* inputPos = reinterpret_cast<PixelTypes::GrayscaledPixel*>(multihead.input(0).data());

    for (unsigned int pos = 0; pos < patchSize * patchSize; ++pos, ++yPos, ++uPos, ++vPos, inputPos += 3)
    {
      inputPos[0] = yPos[0];
      inputPos[1] = uPos[0];
      inputPos[2] = vPos[0];
    }
  }


  STOPWATCH("module:BallAndPenaltyMarkPerceptor:apply")
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
    //OUTPUT_TEXT("RADIUS: " << (multihead.output(0)[5] * stepSize));
    ASSERT(predRadius > 0.f);
  }
  else if(predPenalty >= penaltyThreshold)
  {
    penaltyPosition.x() = static_cast<float>(ballSpot.x());
    penaltyPosition.y() = static_cast<float>(ballSpot.y());
    ASSERT(penaltyPosition.x() >= 0.f || penaltyPosition.y() >= 0.f);
  }
  //OUTPUT_TEXT("BallAndPenaltyMarkPerceptor: Prediction: " << predBall << "|" << predPenalty << "|" << predNegatives);

  return std::make_pair(predBall, predPenalty);
}

void BallAndPenaltyMarkPerceptor::savePatch(std::vector<float>& grayData, std::vector<float>& blueData, std::vector<float>& redData, const std::string& suffix, const Vector2i& spot)
{
  const std::string saveDir = std::string(File::getBHDir()) + "/../MachineLearning/BallTrainerPenaltyMarkClassifier/data/raw";
  std::filesystem::create_directories(saveDir);
  std::filesystem::create_directories(saveDir + "/labels");
  const std::string frameTime = std::to_string(theFrameInfo.time);


  OutTextFile file(std::string(saveDir + suffix + ".txt"), false, true);
  file << frameTime + suffix << endl;
  file << spot << endl;
  file << theCameraMatrix << endl;
  file << theCameraInfo << endl;
  file << "###" << endl;

  const std::string savePatchPath = saveDir + "/" + frameTime + " " + std::to_string(spot.x()) + " " + std::to_string(spot.y()) + suffix + ".txt";

  OutBinaryFile stream(savePatchPath);

  for(float i : grayData)
  {
    stream << i;
  }
  for (float i : blueData)
  {
    stream << i;
  }
  for (float i : redData)
  {
    stream << i;
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

  ASSERT(multihead.numOfInputs() == 1); //TODO

  //  ASSERT(encoder.input(0).rank() == 3);
  //  ASSERT(encoder.input(0).dims(0) == encoder.input(0).dims(1));
  //  ASSERT(encoder.input(0).dims(2) == 1);

  //  ASSERT(multihead.output(0).rank() == 1);
  //  ASSERT(multihead.output(0).dims(0) == 1 && corrector.output(0).dims(0) == 3);
}
