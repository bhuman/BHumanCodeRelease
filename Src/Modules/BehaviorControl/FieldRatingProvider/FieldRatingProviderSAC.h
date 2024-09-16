/**
 * This module provides a rating over the current field, and information about which field positions are currently better.
 * This is then later used to decide the dribble direction, kick direction in a duel or possible pass targets
 *
 * @file FieldRatingProviderSAC.h
 * @author Philip Reichenberg
 */

#pragma once

#include "Representations/BehaviorControl/FieldInterceptBall.h"
#include "Representations/BehaviorControl/FieldRating.h"
#include "Representations/BehaviorControl/IndirectKick.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/GlobalTeammatesModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Framework/Module.h"
#include <vector>
#include "Debugging/ColorRGBA.h"

MODULE(FieldRatingProviderSAC,
{,
  REQUIRES(FieldInterceptBall),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GlobalTeammatesModel),
  REQUIRES(IndirectKick),
  REQUIRES(ObstacleModel),
  REQUIRES(RobotPose),
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
    (float)(1000.f) goalPostMaxRadius,
    (float)(0.5f) malusForOpponentTurnFactor, // point behind the opponent are weaker and better for us
    (float)(0.7f) selfAndOpponentShiftFactor, // A point between us and an opponent is better for us, as long as the distance is lower than 70% of overall distance to the opponent.
    (float)(100.f) opponentBackRangeY, // Scale obstacle back rating over this range (in mm)
    (float)(200.f) opponentBackShiftY, // Shift obstacle y-coordinate by this range (in mm) in direction of the ball
    (float)(300.f) opponentBackRangeX, // Scale the obstacle back rating down to 0 over this x-coordinate range
    (float)(0.3f) opponentBackValue, // Max obstacle back rating

    // opponentGoal
    (float)(1.5f) attractValue, // max possible attract value for the potentialfield
    (float)(-750) indirectGoalOffset, // If a goal shot is not allowed, shift the target position by this amount in the x-axis
    (float) attractRange, // if distance exceeds this range, the potentialfield has no influence

    // teammate
    (float)(0.f) bestRelativePose, // best relative pass pose relative from the teammate in direction of the goal
    (float)(500.f) minTeammatePassDistance, // teammate must stand at least this distance far away from us
    (float)(2000.f) maxTeammatePassDistance, // if teammate stands this far away (or more) then the rating is not reduced
    (int)(500) estimateTeammateIntoFuture, /**< Estimate the teammates position this much into the future (in ms). */
    (float)(250.f) interpolationZoneInOwnHalf,

    // pass target
    (float)(2.f) passTargetValue, // pass target attract value
    (float)(1500.f) passAttractRange, // max distance a teammate influences the potential field.
    (float)(1000.f) teammateAttractRangeMin, // Reduce the pass rating range to this minimum.

    // get better goal angle
    (float)(0.5f) betterGoalAngleValue,
    (float) betterGoalAngleRange,

    // ball distance
    (float)(1000.f) bestDistanceForBall,
    (float)(500.f) bestDistanceWidth, // bestDistanceForBall +- bestDistanceWidth shall produce the best rating
    (float)(500.f) ballRange,
    (float)(0.5f) ballRating,
    (Angle)(110_deg) ballGoalSectorWidth, // Only direction from the ball to the goal +- 110 degrees are allowed
    (Angle)(10_deg) ballGoalSectorBorderWidth,

    // drawing
    (ColorRGBA)(213, 17, 48, 125) badRatingColor,
    (ColorRGBA)(0, 140, 0, 125) middleRatingColor,
    (ColorRGBA)(0, 104, 180, 125) goodRatingColor,

    (float) drawMinX,
    (float) drawMaxX,
    (float) drawMinY,
    (float) drawMaxY,
    (Vector2f)(50.f, 50.f) drawGrid, // size of the draw grid
    (Vector2f)(Vector2f(20.f, 20.f)) drawGridArrow, // size of the draw grid
    (int)(20) arrowwidth,
    (float)(0.5f) arrowlenght,
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

class FieldRatingProviderSAC : public FieldRatingProviderSACBase
{
public:
  void update(FieldRating& fieldRating) override;
  FieldRatingProviderSAC();

private:

  bool fieldBorderDrawing = false;
  bool goalDrawing = false;
  bool goalAngleDrawing = false;
  bool opponentDrawing = false;
  bool teammateDrawing = false;
  bool ballNearDrawing = false;
  bool passTargetDrawing = false;
  bool modifyDrawingRating = false;

  // TODO what does RTV mean? I wrote this code, but have no idea, lol
  float fieldBorderRTV;
  float attractRTV;
  float passRTV;
  float betterGoalAngleRTV;
  float ballRTV;
  float obstacleBackRTV;
  Rangef bestBallPositionRange;
  float minXCoordinateForPass;

  std::vector<Angle> teammateAngleToGoal;
  std::vector<Angle> teammateBallAngle;
  std::vector<Vector2f> teammateOffsetToGoal;
  std::vector<Vector2f> teammateInField;
  std::vector<float> teammateRangeInterpolation;
  std::vector<float> teammateObstacleInfluenceRange;
  std::vector<Vector2f> obstacleOnField;

  struct ObstacleOnField
  {
    Vector2f position;
    bool isGoalPost;
  };

  std::vector<ObstacleOnField> obstaclesOnField;
  unsigned int lastTeammateUpdate;
  unsigned int lastObstacleOnFieldUpdate;
  unsigned int lastTeammateInFieldUpdate;

  Vector2f rightInnerGoalPost;
  Vector2f leftInnerGoalPost;

  Obstacle leftGoalPost;
  Obstacle rightGoalPost;

  float leftInnerGoalPostY;
  float rightInnerGoalPostY;
  float outerGoalPostX;

  Rangef drawMinMax = Rangef(-1.f, 1.f);

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

  PotentialValue getBallNearPotential(const float x, const float y, const bool calculateFieldDirection);

  void updateTeammateData(const Vector2f& useBallPose);

  std::vector<Vector2f> getPossiblePassTargets();
};
