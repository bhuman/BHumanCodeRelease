/**
 * @file KickSelection.h
 *
 * This file defines some functions to select a kick for a given target angle range.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Math/Angle.h"
#include "Math/BHMath.h"
#include "Math/Eigen.h"
#include "Math/Pose2f.h"
#include "Math/Range.h"

namespace KickSelection
{
  /**
   * This function is a heuristic to get the kick pose which moves a ball fastest into a given target sector with a given kick.
   * @param targetAngleRange A (normalized) angle range which represents the target sector (in field coordinates).
   * @param robotPose The pose of the robot on the field.
   * @param ballPosition The position of the ball on the field.
   * @param ballOffset The ball offset for the kick from the KickInfo.
   * @param rotationOffset The rotation offset for the kick from the KickInfo.
   * @return The pose that moves the ball fastest (among all possible kick poses with this kick to the specified sector).
   */
  inline Pose2f calcOptimalKickPoseForTargetAngleRange(const Rangea& targetAngleRange, const Pose2f& robotPose, const Vector2f& ballPosition, const Vector2f& ballOffset, Angle rotationOffset)
  {
    Angle targetAngle = Angle::normalize(robotPose.rotation - rotationOffset);
    if(!targetAngleRange.isInside(targetAngle))
      targetAngle = std::abs(Angle::normalize(targetAngle - targetAngleRange.min)) < std::abs(Angle::normalize(targetAngle - targetAngleRange.max)) ? targetAngleRange.min : targetAngleRange.max;
    Pose2f kickPose(targetAngle, ballPosition);
    kickPose.rotate(rotationOffset);
    kickPose.translate(ballOffset);
    return kickPose;
  }

  /**
   * Calculates an approximation of the time needed to reach a target pose.
   * @param poseRelative The target pose in robot-relative coordinates.
   * @param maxSpeed The maximum speed of the walking engine.
   * @return An approximation of the time to reach the target pose (in milliseconds).
   *
   * @author Andreas Stolpmann
   */
  inline float calcTTRP(const Pose2f& poseRelative, const Pose2f& maxSpeed)
  {
    const Angle angleToPose = poseRelative.translation.angle();
    const float distanceToPose = poseRelative.translation.norm();

    float ttrp = 0.f;

    if(distanceToPose > 400.f)
    {
      ttrp += distanceToPose / (maxSpeed.translation.x() / 1000.f);
    }
    else
    {
      ttrp += std::abs(poseRelative.translation.x()) / (maxSpeed.translation.x() / 1000.f);
      ttrp += std::abs(poseRelative.translation.y()) / (maxSpeed.translation.y() / 1000.f);
    }

    ASSERT(std::isfinite(ttrp));

    const float turnTTRP1 = std::abs(poseRelative.rotation) / (maxSpeed.rotation / 1000.f);
    ASSERT(std::isfinite(turnTTRP1));
    const float turnTTRP2 = std::abs(angleToPose) / (maxSpeed.rotation / 1000.f) +
                            std::abs(Angle::normalize(poseRelative.rotation - angleToPose)) / (maxSpeed.rotation / 1000.f);
    ASSERT(std::isfinite(turnTTRP2));

    const float interpolation = mapToRange(distanceToPose, 100.f, 1000.f, 0.f, 1.f);
    ttrp += turnTTRP1 * (1.f - interpolation) + turnTTRP2 * interpolation;
    ASSERT(std::isfinite(ttrp));

    return ttrp;
  }

  /**
   * Calculates an absolute rotation for the robot at receiverPosition that allows it to receive a pass from ballPosition and redirect the ball to goalPosition if possible.
   * @param ballPosition The position of the ball on the field.
   * @param receiverPosition The position of the robot that receives the pass on the field.
   * @param goalPosition The position of the opponent goal on the field.
   * @param maxVisionAngle Points of interest should be inside the range of the robot's orientation +- this angle.
   * @param magicalFactor Robot should be oriented with this percentage in the angular range between the ball and the opponent goal in order to receive and redirect the ball. Higher means more in the direction of the goal.
   * @return Absolute rotation on field.
   *
   * @author Jo Lienhoop
   * @author Yannik Meinken
   */
  inline Angle
  calculateTargetRotation(const Vector2f& ballPosition, const Vector2f& receiverPosition, const Vector2f& goalPosition, Angle maxVisionAngle = 60_deg,
                          const float magicalFactor = 0.5f)
  {
    Angle receiverToGoal = (goalPosition - receiverPosition).angle();
    Angle receiverToBall = (ballPosition - receiverPosition).angle();

    // make sure the inner angle is used
    if(std::abs(receiverToGoal - receiverToBall) > 180_deg)
    {
      receiverToGoal += receiverToGoal < 0_deg ? 360_deg : 0_deg;
      receiverToBall += receiverToBall < 0_deg ? 360_deg : 0_deg;
    }
    ASSERT(std::abs(receiverToGoal - receiverToBall) <= 180_deg);

    Angle proposedRotation = Angle::normalize(receiverToGoal * magicalFactor + receiverToBall * (1.f - magicalFactor));

    // Accept the calculated angle, if the robot would be able to observe the ball. Otherwise rotate toward the ball
    const Angle proposedToBall = Angle::normalize(receiverToBall - proposedRotation);

    if(std::abs(proposedToBall) <= maxVisionAngle)
      return proposedRotation;

    const Angle maxProposedToBall = 180_deg * magicalFactor;

    // interpolate between looking directly at the ball if standing at the line from ball to goal to maxVisionAngle in case the ball could nearly been seen with the proposed angle
    // this ensures a continues function
    maxVisionAngle = mapToRange(Angle(std::abs(proposedToBall)), maxVisionAngle, maxProposedToBall, maxVisionAngle, 0_deg);

    const Angle proposedToAcceptable = proposedToBall - (proposedToBall > Angle(0) ? maxVisionAngle : - maxVisionAngle);
    proposedRotation = Angle::normalize(proposedRotation + proposedToAcceptable);
    ASSERT(Angle::normalize(receiverToBall - proposedRotation) <= maxVisionAngle + 1_deg);

    return proposedRotation;
  }
}
