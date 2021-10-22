/**
 * This module provides a rating over the current field, and information about which field positions are currently better.
 * This is then later used to decide the dribble direction, kick direction in a duel or possible pass targets
 *
 * @file FieldRatingProvider.h
 * @author Philip Reichenberg
 */

#pragma once

#include "Representations/BehaviorControl/FieldRating.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Tools/Module/Module.h"
#include <vector>

MODULE(FieldRatingProvider,
{,
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(ObstacleModel),
  REQUIRES(RobotPose),
  REQUIRES(TeamData),
  PROVIDES(FieldRating),
  DEFINES_PARAMETERS(
  {,
    // field border
    (float)(3.f) fieldBorderValue, // high value to make sure the ball is not kicked outside
    (float)(50.f) fieldBorderRange, // if distance exceeds this range, the potentialfield has no influence

    // opponent, ownGoal
    (float)(1.f) repelValue, // max possible repel value for the potentialfield
    (float)(125.f) minRepelDifferenceRange, // min width of min and max value
    (float)(1500) maxRepelDifferenceRange, // max width of min and max value
    (float)(2000.f) opponentDistanceToGoal,
    (float)(0.075f) malusForOpponentTurnFactor,

    // opponentGoal
    (float)(1.5f) attractValue, // max possible attract value for the potentialfield
    (float) attractRange, // if distance exceeds this range, the potentialfield has no influence

    // teammate
    (float)(0.6f) teammateValue, // max possible attract value for a teammate
    (float)(800.f) teammateAttractRange, // max distance a teammate influences the potential field.
    (float)(500.f) bestRelativePose, // best relativ pass pose relative from the teammate in direction of the goal
    (float)(1000.f) minTeammatePassDistance, // teammate must stand at least this distance far away from us
    (float)(2000.f) maxTeammatePassDistance, // if teammate stands this far away (or more) then the rating is not reduced

    // other side
    (float)(0.5f) percentOnSide,
    (float) otherSideYCoordinate,
    (float)(1800.f) otherSideWidth,
    (float)(0.5f) otherSideMaxRating,

    // get better goal angle
    (float)(0.5f) betterGoalAngleValue,
    (float) betterGoalAngleRange,

    // robot direction
    (Angle)(45_deg) maxFacingAngle,
    (float)(3000.f) facingRange,
    (float)(0.35f) facingValue,
    (float)(100.f) facingBackShift,
    (float)(1.f) lowPassFilterFactorPerSecond, // filter factor (per second)

    // ball distance
    (float)(750.f) bestDistanceForBall,
    (float)(250.f) bestDistanceWidth, // bestDistanceForBall +- bestDistanceWidth shall produce the best rating
    (float)(500.f) ballRange,
    (float)(0.5f) ballRating,

    // drawing
    (float) drawMinX,
    (float) drawMaxX,
    (float) drawMinY,
    (float) drawMaxY,
    (Vector2f)(Vector2f(50.f, 50.f)) drawGrid, // size of the draw grid
  }),
});

ENUM(PotentialType,
{,
  fieldBorder,
  opponentGoal,
  opponent,
  teammate,
  ownGoal,
});

class FieldRatingProvider : public FieldRatingProviderBase
{
public:
  void update(FieldRating& fieldRating) override;
  FieldRatingProvider();

private:

  bool fieldBorderDrawing;
  bool goalDrawing;
  bool goalAngleDrawing;
  bool opponentDrawing;
  bool teammateDrawing;
  bool otherSideDrawing;
  bool facingDrawing;
  bool ballNearDrawing;

  float fieldBorderRTV;
  float attractRTV;
  float teammateRTV;
  float betterGoalAngleRTV;
  float facingRTV;
  float ballRTV;
  Rangef bestBallPositionRange;
  float lowPassFilterFactor;

  std::vector<Angle> teammateAngleToGoal;
  std::vector<Vector2f> teammateOffsetToGoal;
  std::vector<float> teammateRangeInterpolation;
  std::vector<float> teammateObstacleInfluenceRange;
  std::vector<std::vector<Vector2f>> teammateObstacleOnField;
  std::vector<Vector2f> obstaclesOnField;
  unsigned int lastTeammateUpdate;
  unsigned int lastObstacleOnFieldUpdate;

  Vector2f robotRotation;

  Vector2f rightInnerGoalPost;
  Vector2f leftInnerGoalPost;

  float leftInnerGoalPostY;
  float rightInnerGoalPostY;
  float outerGoalPostX;

  float functionLinear(const float distance, const float radius, const float radiusTimesValue);
  Vector2f functionLinearDer(const Vector2f& distanceVector, const float distance, const float valueSign);

  float functionCone(const float distance, const Angle dribbleAngle, const Angle maxAngle, const float radius, const float value);
  Vector2f functionConeDer(const Vector2f& distanceVector, const Angle dribbleAngle, const Angle maxAngle, const float radius, const float value);

  void draw();

  void updateParameters();

  PotentialValue getFieldBorderPotential(const float x, const float y, const bool calculateFieldDirection);

  PotentialValue getObstaclePotential(const float x, const float y, const bool calculateFieldDirection);

  PotentialValue getGoalPotential(const float x, const float y, const bool calculateFieldDirection);

  PotentialValue getGoalAnglePotential(const float x, const float y, const bool calculateFieldDirection);

  PotentialValue getTeammatesPotential(const float x, const float y, const bool calculateFieldDirection);

  PotentialValue getPotentialOtherSide(const float x, const float y, const bool calculateFieldDirection);

  PotentialValue getRobotFacingPotential(const float x, const float y, const bool calculateFieldDirection);

  PotentialValue getBallNearPotential(const float x, const float y, const bool calculateFieldDirection);
};
