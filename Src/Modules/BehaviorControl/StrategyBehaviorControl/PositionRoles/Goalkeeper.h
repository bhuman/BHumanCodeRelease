/**
 * @file Goalkeeper.h
 *
 * This file declares the goalkeeper role.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Math/Pose2f.h"
#include "Streaming/AutoStreamable.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Configuration/BehaviorParameters.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Tools/BehaviorControl/Strategy/PositionRole.h"

class Goalkeeper : public PositionRole
{
  STREAMABLE(Parameters,
  {,
    (Rangef) ballDistanceInterpolationRange,
    (Rangea) rotationThresholdRange,     //Interpolate rotation threshold between these values
    (Rangef) translationXThresholdRange,  //Interpolate translation threshold between these values
    (Rangef) translationYThresholdRange,  //Interpolate translation threshold between these values
    (Angle) shouldStopRotation,                          //Stop walking if orientation to the target is less than this value
    (Vector2f) shouldStopTranslation,              //Stop walking is translation to the target is less than this value
    //former GoaliePoseProvider parameters
    (float) goalieBaseLine,                             //The base line of a goalkeeper (the max height for the goalie in most instances). This marks the basic guarding spot for the goal.
    (float) distanceToGoalLineForMinimumXValue,
    (Angle) positionAngle,
    (int) timeOutBallLastSeen,
    (int) timeOutBallDisappear,
    (Pose2f) goaliePoseField,
    (Pose2f) goaliePoseRel,
  });

  void preProcess() override;
  Pose2f position(Side side, const Pose2f& basePose, const std::vector<Vector2f>& baseArea, const Agent& self, const Agents& teammates) override;

  Pose2f calcGlobalBobPose() const;
  Pose2f calcGlobalPoseOnBobLine() const;
  Pose2f calcGlobalPoseOnBobLineAndCut() const;
  void draw() const;
  void verify() const;
  Pose2f tolerance() const override;
  bool shouldStop(const Pose2f& target) const override;

  Parameters p;

public:
  Goalkeeper();
};
