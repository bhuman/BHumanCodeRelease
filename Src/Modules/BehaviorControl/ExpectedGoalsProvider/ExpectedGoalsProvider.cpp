/**
 * @file ExpectedGoalsProvider.cpp
 *
 * This file implements a module that estimates the probability of scoring a goal when shooting from a hypothetical ball position.
 *
 * @author Arne Hasselbring
 * @author Jo Lienhoop
 */

#include "ExpectedGoalsProvider.h"

MAKE_MODULE(ExpectedGoalsProvider);

ExpectedGoalsProvider::ExpectedGoalsProvider()
{
  // Create a grid on the field with a certain cell size for drawing the heatmap
  cellsNumber = ((gridCornerUpper - gridCornerLower).array() / Vector2f(cellSize, cellSize).array()).cast<int>();
  cellsNumber += Vector2i(1, 1);
  cellColors.reserve(cellsNumber.x() * cellsNumber.y());
}

void ExpectedGoalsProvider::update(ExpectedGoals& theExpectedGoals)
{
  theExpectedGoals.xG = [this](const Vector2f& pointOnField) -> float
  {
    return xG(pointOnField);
  };

  theExpectedGoals.xGA = [this](const Vector2f& pointOnField) -> float
  {
    return xGA(pointOnField);
  };

  theExpectedGoals.getRating = [this](const Vector2f& pointOnField, const bool isPositioning) -> float
  {
    return getRating(pointOnField, isPositioning);
  };

  theExpectedGoals.getOpponentRating = [this](const Vector2f& pointOnField) -> float
  {
    return getOpponentRating(pointOnField);
  };

  DECLARE_DEBUG_DRAWING("module:ExpectedGoalsProvider:heatmap", "drawingOnField");
  MODIFY_ONCE("module:ExpectedGoalsProvider:calcOpeningAngle", calcOpeningAngle);
  MODIFY_ONCE("module:ExpectedGoalsProvider:calcShotDistance", calcShotDistance);
  MODIFY_ONCE("module:ExpectedGoalsProvider:drawHeatmap", drawHeatmap);
  MODIFY_ONCE("module:ExpectedGoalsProvider:isPositioningDrawing", isPositioningDrawing);
  if(drawHeatmap)
    draw();
}

float ExpectedGoalsProvider::xG(const Vector2f& pointOnField) const
{
  if((pointOnField - goalCenter).squaredNorm() < sqr(theFieldDimensions.yPosLeftGoal - theFieldDimensions.goalPostRadius))
    return 1.f;

  std::array<float, 2> features{};
  features[0] = (pointOnField - goalCenter).norm() / 1000.f;

  const float minBallGoalPostOffset = theFieldDimensions.goalPostRadius + theBallSpecification.radius;
  const Angle leftAngleOffset = std::asin(std::min(1.f, minBallGoalPostOffset / (leftGoalPost - pointOnField).norm()));
  const Angle rightAngleOffset = std::asin(std::min(1.f, minBallGoalPostOffset / (rightGoalPost - pointOnField).norm()));
  const Angle angleToLeftPost = (leftGoalPost - pointOnField).angle() - leftAngleOffset;
  const Angle angleToRightPost = (rightGoalPost - pointOnField).angle() + rightAngleOffset;
  features[1] = angleToLeftPost - angleToRightPost;

  std::array<float, 2> hidden{};
  hidden[0] = std::tanh(-0.0182828f + 0.0995936f * features[0] + -2.10128f * features[1]);
  hidden[1] = std::tanh(-0.137664f + 0.903687f * features[0] + -2.10898f * features[1]);

  return 1.f / (1.f + std::exp(-(0.153333f + hidden[0] * -2.7232f + hidden[1] * -1.07426f)));
}

float ExpectedGoalsProvider::xGA(const Vector2f& pointOnField) const
{
  return 1.f - xG(-pointOnField);
}

float ExpectedGoalsProvider::getRating(const Vector2f& pointOnField, const bool isPositioning) const
{
  // Estimated probability that there is a wide enough opening angle on the opponent's goal in order to score
  const float openingAngleCriterion = calcOpeningAngle ? mapToRange(getOpeningAngle(pointOnField, isPositioning), minOpeningAngle, maxOpeningAngle, 0_deg, Angle(1.f)) : Angle(1.f);
  // Estimated probability that the kick would be successful based on the distance to the opponent's goal
  const float shotDistanceCriterion = calcShotDistance ? xG(Vector2f(goalCenter.x() - (goalCenter - pointOnField).norm(), 0.f)) : 1.f;
  // Estimated probability that the goal shot would be successful based on the combined criteria
  return std::max(minValue, openingAngleCriterion * shotDistanceCriterion);
}

float ExpectedGoalsProvider::getOpponentRating(const Vector2f& pointOnField) const
{
  // Estimated probability that the goal shot would not be successful based on the opening angle on the own goal
  return mapToRange(getOpponentOpeningAngle(-pointOnField), minOpeningAngle, maxOpeningAngleOpponenent, Angle(1.f), 0_deg);
}

Angle ExpectedGoalsProvider::getOpeningAngle(const Vector2f& pointOnField, const bool isPositioning) const
{
  const float minBallGoalPostOffset = theFieldDimensions.goalPostRadius + theBallSpecification.radius;
  const Angle leftAngleOffset = std::asin(std::min(1.f, minBallGoalPostOffset / (leftGoalPost - pointOnField).norm()));
  const Angle rightAngleOffset = std::asin(std::min(1.f, minBallGoalPostOffset / (rightGoalPost - pointOnField).norm()));
  const Angle angleToLeftPost = (leftGoalPost - pointOnField).angle() - leftAngleOffset;
  const Angle angleToRightPost = (rightGoalPost - pointOnField).angle() + rightAngleOffset;

  // Handle cases where the position is behind the opponent's goalposts (in regard to the x-coordinates)
  if(angleToLeftPost < angleToRightPost)
    return 0_deg;
  if(pointOnField.x() >= theFieldDimensions.xPosOpponentGoalPost - minBallGoalPostOffset)
    return (std::abs(pointOnField.y()) >= theFieldDimensions.yPosLeftGoal + minBallGoalPostOffset) ? 0_deg : 180_deg;

  ASSERT(std::abs(angleToLeftPost) <= pi_2);
  ASSERT(std::abs(angleToRightPost) <= pi_2);

  // Construct a sector wheel relative to the position and add the goal sector between the two goal posts
  SectorWheel wheel;
  std::list<SectorWheel::Sector> sectors;
  Angle openingAngle = 0_deg;
  wheel.begin(pointOnField);
  wheel.addSector(Rangea(angleToRightPost, angleToLeftPost), std::numeric_limits<float>::max(), SectorWheel::Sector::goal);

  for(const auto& obstacle : theGlobalOpponentsModel.opponents)
  {
    const Vector2f& obstacleOnField = obstacle.position;
    // Skip opponents inside of their goal (behind the goal line)
    if(obstacleOnField.x() > (theFieldDimensions.xPosOpponentGoalLine + theFieldDimensions.xPosOpponentGoal) * 0.5f)
      continue;

    // Check if the opponent is inside of the goal sector
    const float width = (obstacle.left - obstacle.right).norm() + 4.f * theBallSpecification.radius;
    const float distance = std::sqrt(std::max((obstacleOnField - pointOnField).squaredNorm() - sqr(width / 2.f), 1.f));
    const float ratio = !isPositioning ? 1.f : mapToRange(distance, 300.f, 1500.f, 1.f, 0.f);
    const float radius = ratio * std::atan(width / (2.f * distance));

    const Angle direction = (obstacleOnField - pointOnField).angle();
    if(direction - radius > angleToLeftPost ||
       direction + radius < angleToRightPost ||
       radius == 0.f)
      continue;

    // Add the opponent to the sector wheel
    wheel.addSector(Rangea(Angle::normalize(direction - radius), Angle::normalize(direction + radius)), distance, SectorWheel::Sector::obstacle);
  }

  sectors = wheel.finish();
  // Find the maximum opening angle i.e. the free sector with the largest size
  for(const SectorWheel::Sector& sector : sectors)
    if(sector.type == SectorWheel::Sector::goal &&
       sector.angleRange.getSize() >= openingAngle)
      openingAngle = sector.angleRange.getSize();

  return openingAngle;
}

Angle ExpectedGoalsProvider::getOpponentOpeningAngle(const Vector2f& pointOnField) const
{
  const float minBallGoalPostOffset = theFieldDimensions.goalPostRadius + theBallSpecification.radius;
  const Angle leftAngleOffset = std::asin(std::min(1.f, minBallGoalPostOffset / (leftGoalPost - pointOnField).norm()));
  const Angle rightAngleOffset = std::asin(std::min(1.f, minBallGoalPostOffset / (rightGoalPost - pointOnField).norm()));
  const Angle angleToLeftPost = (leftGoalPost - pointOnField).angle() - leftAngleOffset;
  const Angle angleToRightPost = (rightGoalPost - pointOnField).angle() + rightAngleOffset;

  // Handle cases where the position is behind the opponent's goalposts (in regard to the x-coordinates)
  if(angleToLeftPost < angleToRightPost)
    return 0_deg;
  if(pointOnField.x() >= theFieldDimensions.xPosOpponentGoalPost - minBallGoalPostOffset)
    return (std::abs(pointOnField.y()) >= theFieldDimensions.yPosLeftGoal + minBallGoalPostOffset) ? 0_deg : 180_deg;

  // Return the size of the goal sector without considering any obstacles
  return Rangea(angleToRightPost, angleToLeftPost).getSize();
}

void ExpectedGoalsProvider::draw()
{
  cellColors.clear();
  for(float y = gridCornerLower.y(); y <= gridCornerUpper.y(); y += cellSize)
  {
    for(float x = gridCornerLower.x(); x <= gridCornerUpper.x(); x += cellSize)
    {
      // Linear interpolation of rating in [0, 1] between the two colors for the heatmap
      ColorRGBA cellColor = worstRatingColor.interpolate(getRating(Vector2f(x, y), isPositioningDrawing), bestRatingColor);
      cellColor.a = heatmapAlpha;
      cellColors.push_back(cellColor);
    }
  }
  GRID_RGBA("module:ExpectedGoalsProvider:heatmap", 0, 0, cellSize, cellsNumber.x(), cellsNumber.y(), cellColors.data());
}
