/**
 * @file JerseyClassifierProvider.cpp
 *
 * This file implements a module that provides functionality to classify jerseys
 * by sampling pixels in an estimated area and classifying each of them as
 * belonging to the own team or to the opponent team.
 *
 * @author Thomas Röfer (the algorithm)
 * @author Lukas Malte Monnerjahn (the algorithm)
 * @author Arne Hasselbring (the module)
 * @author Tim Laue (the updates for 2023 and 2026)
 */

#include "JerseyClassifierProvider.h"
#include "Debugging/DebugDrawings.h"
#include "Tools/Math/Transformation.h"
#include "Math/Random.h"
#include <algorithm>
#include <cmath>

MAKE_MODULE(JerseyClassifierProvider);

void JerseyClassifierProvider::update(JerseyClassifier& jerseyClassifier)
{
  DECLARE_DEBUG_DRAWING("module:JerseyClassifierProvider:jerseyAndRobotRange", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:JerseyClassifierProvider:jerseyClassification", "drawingOnImage");

  jerseyClassifier.detectJersey = [this](RobotPercept::Robot& robotOnField)
  {
    return detectJersey(robotOnField);
  };
}

/**
 * Hue values for the different team colors. The indices are enum constants from Settings::TeamColor.
 */
static int jerseyHues[] =
{
  255 * 7 / 8, // blue + cyan
  255 * 2 / 8, // red
  255 * 4 / 8, // yellow
  0, // black (unused)
  0, // white (unused)
  255 * 5 / 8, // green
  255 * 3 / 8, // orange
  32, // purple
  255 * 7 / 16, // brown
  0, // gray (unused)
};

void JerseyClassifierProvider::detectJersey(RobotPercept::Robot& robotOnField)
{
  // Step 0: If no jersey is found, the default remains:
  robotOnField.type = RobotPercept::unknown;

  // Step 1: Determine corner points of perceived robot.
  Vector2f bottomLeftInImage;
  Vector2f bottomRightInImage;
  Vector2f topCenterInImage;
  //TODO: Make this a parameter?
  const Settings::RobotType robotType = Global::getSettings().robotType;
  const float assumedRobotHeight = robotType == Settings::k1 ? 950.f : 1180.f;

  if(!Transformation::robotToImage(robotOnField.left, theCameraMatrix, theCameraInfo, bottomLeftInImage) ||
     !Transformation::robotToImage(robotOnField.right, theCameraMatrix, theCameraInfo, bottomRightInImage) ||
     !Transformation::robotToImage(Vector3f(robotOnField.center.x(), robotOnField.center.y(), assumedRobotHeight), theCameraMatrix, theCameraInfo, topCenterInImage))
    return;

  // Step 2: Create a set of samples in potential jersey region
  RECTANGLE("module:JerseyClassifierProvider:jerseyAndRobotRange",
            bottomLeftInImage.x(), bottomLeftInImage.y(), bottomRightInImage.x(), topCenterInImage.y(), 3, Drawings::solidPen, ColorRGBA::violet);
  jSamples.clear();
  for(unsigned int i=0; i<numberOfSamples; i++)
  {
    Vector3f jPointWorld(robotOnField.center.x(), robotOnField.center.y(), jerseyCenterZ);
    Vector2f jPointImage;
    jPointWorld.x() += Random::uniform(-maxSampleOffset, maxSampleOffset);
    jPointWorld.y() += Random::uniform(-maxSampleOffset, maxSampleOffset);
    jPointWorld.z() += Random::uniform(-maxSampleOffset, maxSampleOffset);
    if(Transformation::robotToImage(jPointWorld, theCameraMatrix, theCameraInfo, jPointImage))
    {
      int x = static_cast<int>(std::round(jPointImage.x()));
      int y = static_cast<int>(std::round(jPointImage.y()));
      if(x < 0 || x >= theCameraInfo.width || y >= theCameraInfo.height || y < 0)
        continue;
      jSamples.emplace_back(x,y);
      CROSS("module:JerseyClassifierProvider:jerseyAndRobotRange", jPointImage.x(), jPointImage.y(), 2, 1, Drawings::solidPen, ColorRGBA::violet);
    }
  }
  // Too few sample points in image:
  if(jSamples.size() < numberOfSamples/2)
    return;

  // Step 3: Analyze color at sample positions:
  // Get pixel classifier
  const std::function<bool(int, int)>& isOwnFieldPlayer = getPixelClassifier(theGameState.ownTeam.fieldPlayerColor, theGameState.opponentTeam.fieldPlayerColor, theGameState.opponentTeam.goalkeeperColor, theGameState.ownTeam.goalkeeperColor);
  const std::function<bool(int, int)>& isOpponentFieldPlayer = getPixelClassifier(theGameState.opponentTeam.fieldPlayerColor, theGameState.ownTeam.fieldPlayerColor, theGameState.opponentTeam.goalkeeperColor, theGameState.ownTeam.goalkeeperColor);
  const std::function<bool(int, int)>& isOwnGoalkeeper = getPixelClassifier(theGameState.ownTeam.goalkeeperColor, theGameState.ownTeam.fieldPlayerColor, theGameState.opponentTeam.fieldPlayerColor, theGameState.opponentTeam.goalkeeperColor);
  const std::function<bool(int, int)>& isOpponentGoalkeeper = getPixelClassifier(theGameState.opponentTeam.goalkeeperColor, theGameState.opponentTeam.fieldPlayerColor, theGameState.ownTeam.fieldPlayerColor, theGameState.ownTeam.goalkeeperColor);

  float ownFieldPlayerPixels = 0;
  float opponentFieldPlayerPixels = 0;
  float ownGoalkeeperPixels = 0;
  float opponentGoalkeeperPixels = 0;
  float weight = 1.f;
  for(const auto& js : jSamples)
  {
    if(isOwnFieldPlayer(js.x(), js.y()))
      ownFieldPlayerPixels += weight;
    else if(isOpponentFieldPlayer(js.x(), js.y()))
      opponentFieldPlayerPixels += weight;
    else if(isOpponentGoalkeeper(js.x(), js.y()))
      opponentGoalkeeperPixels += weight;
    else if(isOwnGoalkeeper(js.x(), js.y()))
      ownGoalkeeperPixels += weight;
  }

  // Step 4: Determine the final result:

  // threshold to counter white robot parts or green field background being classified as opponent jersey
  if((theGameState.opponentTeam.fieldPlayerColor == GameState::Team::Color::white || theGameState.opponentTeam.fieldPlayerColor == GameState::Team::Color::green))
  {
    const float threshold = 1.5f * (ownFieldPlayerPixels + ownGoalkeeperPixels) + (jSamples.size()) / 5;
    opponentFieldPlayerPixels = opponentFieldPlayerPixels <= threshold ? 0 : opponentFieldPlayerPixels - threshold;
  }
  if((theGameState.opponentTeam.goalkeeperColor == GameState::Team::Color::white || theGameState.opponentTeam.goalkeeperColor == GameState::Team::Color::green))
  {
    const float threshold = 1.5f * (ownFieldPlayerPixels + ownGoalkeeperPixels) + (jSamples.size()) / 5;
    opponentGoalkeeperPixels = opponentGoalkeeperPixels <= threshold ? 0 : opponentGoalkeeperPixels - threshold;
  }
  DRAW_TEXT("module:JerseyClassifierProvider:jerseyClassification", bottomLeftInImage.x(), topCenterInImage.y() - 14, 10, ColorRGBA::black, "own: " << ownFieldPlayerPixels);
  DRAW_TEXT("module:JerseyClassifierProvider:jerseyClassification", bottomLeftInImage.x(), topCenterInImage.y() - 4, 10, ColorRGBA::black, "opponent: " << opponentFieldPlayerPixels);
  DRAW_TEXT("module:JerseyClassifierProvider:jerseyClassification", bottomLeftInImage.x(), topCenterInImage.y() - 24, 10, ColorRGBA::black, "own goalie: " << ownGoalkeeperPixels);
  DRAW_TEXT("module:JerseyClassifierProvider:jerseyClassification", bottomLeftInImage.x(), topCenterInImage.y() - 34, 10, ColorRGBA::black, "opponent goalie: " << opponentGoalkeeperPixels);
  const float minJerseyWeight = jSamples.size() * minJerseyWeightRatio;
  const float overallPixels = ownFieldPlayerPixels + ownGoalkeeperPixels + opponentGoalkeeperPixels + opponentFieldPlayerPixels;
  if((ownFieldPlayerPixels > minJerseyWeight) && (ownFieldPlayerPixels > overallPixels * minJerseyRatio))
  {
    robotOnField.type = RobotPercept::ownPlayer;
    DRAW_TEXT("module:JerseyClassifierProvider:jerseyClassification", bottomLeftInImage.x(), topCenterInImage.y() + 10, 10, ColorRGBA::black, "class: own field player");
  }
  else if((opponentFieldPlayerPixels > minJerseyWeight) && (opponentFieldPlayerPixels > overallPixels * minJerseyRatio))
  {
    robotOnField.type = RobotPercept::opponentPlayer;
    DRAW_TEXT("module:JerseyClassifierProvider:jerseyClassification", bottomLeftInImage.x(), topCenterInImage.y() + 10, 10, ColorRGBA::black, "class: opponent field player");
  }
  else if((ownGoalkeeperPixels > minJerseyWeight) && (ownGoalkeeperPixels > overallPixels * minJerseyRatio))
  {
    robotOnField.type = RobotPercept::ownGoalkeeper;
    DRAW_TEXT("module:JerseyClassifierProvider:jerseyClassification", bottomLeftInImage.x(), topCenterInImage.y() + 10, 10, ColorRGBA::black, "class: own goalkeeper");
  }
  else if((opponentGoalkeeperPixels > minJerseyWeight) && (opponentGoalkeeperPixels > overallPixels * minJerseyRatio))
  {
    robotOnField.type = RobotPercept::opponentGoalkeeper;
    DRAW_TEXT("module:JerseyClassifierProvider:jerseyClassification", bottomLeftInImage.x(), topCenterInImage.y() + 10, 10, ColorRGBA::black, "class: opponent goalkeeper");
  }
  else
  {
    robotOnField.type = RobotPercept::unknown;
    DRAW_TEXT("module:JerseyClassifierProvider:jerseyClassification", bottomLeftInImage.x(), topCenterInImage.y() + 12, 10, ColorRGBA::black, "class: unknown");
  }
}

std::function<bool(int, int)> JerseyClassifierProvider::getPixelClassifier(const GameState::Team::Color checkColor,
                                                                                      const GameState::Team::Color o1,
                                                                                      const GameState::Team::Color o2,
                                                                                      const GameState::Team::Color o3) const
{
  const int checkHue = jerseyHues[checkColor];
  const int o1Hue = jerseyHues[o1];
  const int o2Hue = jerseyHues[o2];
  const int o3Hue = jerseyHues[o3];

  bool checkIsWhite = checkColor == GameState::Team::Color::white;
  bool othersAreAllColorful = o1 != GameState::Team::Color::black && o1 != GameState::Team::Color::gray && o1 != GameState::Team::Color::white &&
                              o2 != GameState::Team::Color::black && o2 != GameState::Team::Color::gray && o2 != GameState::Team::Color::white &&
                              o3 != GameState::Team::Color::black && o3 != GameState::Team::Color::gray && o3 != GameState::Team::Color::white;

  // White jersey is compared to colorful options:
  if(checkIsWhite)
    return [this, o1Hue, o2Hue, o3Hue](const int x, const int y)
    {
      return theECImage.saturated[y][x] < satThreshold && theECImage.grayscaled[y][x] >= whiteThreshold &&
             std::abs(static_cast<signed char>(theECImage.hued[y][x] - o1Hue)) > hueSimilarityThreshold &&
             std::abs(static_cast<signed char>(theECImage.hued[y][x] - o2Hue)) > hueSimilarityThreshold &&
             std::abs(static_cast<signed char>(theECImage.hued[y][x] - o3Hue)) > hueSimilarityThreshold;
    };
  // All jerseys are colorful, no black/gray/white involved at all:
  else if(!checkIsWhite && othersAreAllColorful)
    return [this, checkHue, o1Hue, o2Hue, o3Hue](const int x, const int y)
    {
      const int hue = theECImage.hued[y][x];
      return std::abs(static_cast<signed char>(hue - checkHue)) < std::min(std::abs(static_cast<signed char>(hue - o1Hue)), hueSimilarityThreshold) &&
             std::abs(static_cast<signed char>(hue - checkHue)) < std::min(std::abs(static_cast<signed char>(hue - o2Hue)), hueSimilarityThreshold) &&
             std::abs(static_cast<signed char>(hue - checkHue)) < std::min(std::abs(static_cast<signed char>(hue - o3Hue)), hueSimilarityThreshold);
    };
  // We check a colored jersey against a set of others that contain at least once white or gray or black
  else //if(!checkIsWhite && !othersAreAllColorful)
    return [this, checkHue, o1Hue, o2Hue, o3Hue, o1, o2, o3](const int x, const int y)
    {
      const int hue = theECImage.hued[y][x];
      bool checkO1 = false;
      if(o1 == GameState::Team::Color::black || o1 == GameState::Team::Color::gray || o1 == GameState::Team::Color::white)
        checkO1 = std::abs(static_cast<signed char>(theECImage.hued[y][x] - checkHue)) <= hueSimilarityThreshold && theECImage.saturated[y][x] > satThreshold;
      else
        checkO1 = std::abs(static_cast<signed char>(hue - checkHue)) < std::min(std::abs(static_cast<signed char>(hue - o1Hue)), hueSimilarityThreshold);
      if(!checkO1)
        return false;

      bool checkO2 = false;
      if(o2 == GameState::Team::Color::black || o2 == GameState::Team::Color::gray || o2 == GameState::Team::Color::white)
        checkO2 = std::abs(static_cast<signed char>(theECImage.hued[y][x] - checkHue)) <= hueSimilarityThreshold && theECImage.saturated[y][x] > satThreshold;
      else
        checkO2 = std::abs(static_cast<signed char>(hue - checkHue)) < std::min(std::abs(static_cast<signed char>(hue - o2Hue)), hueSimilarityThreshold);
      if(!checkO2)
        return false;

      bool checkO3 = false;
      if(o3 == GameState::Team::Color::black || o3 == GameState::Team::Color::gray || o3 == GameState::Team::Color::white)
        checkO3 = std::abs(static_cast<signed char>(theECImage.hued[y][x] - checkHue)) <= hueSimilarityThreshold && theECImage.saturated[y][x] > satThreshold;
      else
        checkO3 = std::abs(static_cast<signed char>(hue - checkHue)) < std::min(std::abs(static_cast<signed char>(hue - o3Hue)), hueSimilarityThreshold);

      return checkO3;
    };
}
