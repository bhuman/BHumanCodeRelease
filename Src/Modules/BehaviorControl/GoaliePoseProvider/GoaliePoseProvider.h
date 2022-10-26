/**
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Framework/Module.h"
#include "Math/Eigen.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/GoaliePose.h"
#include "Representations/Configuration/BehaviorParameters.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"

MODULE(GoaliePoseProvider,
{,
  REQUIRES(BehaviorParameters),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  PROVIDES(GoaliePose),
  DEFINES_PARAMETERS(
  {,
    (float)(-150.f) goalieLine,             /**< The offset to the front goal area line that is the maximum x coordinate on which the Keeper will stand to guard the goal */
    (float)(200.f) distanceToGroundLineForMinimumXValue,
    (Angle)(65_deg) jumpAngleThresholdMin,  /**< Minimum angle relative to goal post at which keeper won't jump to avoid jumping into the post */
    (Angle)(100_deg) jumpAngleThresholdMax, /**< Maximum angle relative to goal post at which keeper won't jump to avoid jumping into the post */
    (Angle)(120_deg) positionAngle,
    (float)(600.f) jumpDistanceThreshold,   /**< Maximum distance to goal post at which keeper won't jump to avoid jumping into the post */
    (int)(3000) timeOutBallLastSeen,
    (int)(100) timeOutBallDisappear,
  }),
});

class GoaliePoseProvider : public GoaliePoseProviderBase
{
public:
  void update(GoaliePose& goaliePose) override;
  Pose2f calcGlobalBobPose() const;
  Pose2f calcGlobalPoseOnBobLine() const;
  Pose2f calcGlobalPoseOnBobLineAndCut() const;
};
