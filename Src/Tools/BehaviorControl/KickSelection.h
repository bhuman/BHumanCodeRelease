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
    const Vector2f ballInRobot = robotPose.inverse() * ballPosition + ballOffset;
    const float distance = ballInRobot.norm();
    Angle targetAngle = 0.f;
    if(distance > 600.f)
      targetAngle = ballInRobot.angle();
    else if(distance > 100.f)
    {
      const float rotationRatio = std::sqrt((distance - 100.f) / (600.f - 100.f));
      targetAngle = (Vector2f(1.f - rotationRatio, 0.f) + ballInRobot.normalized() * rotationRatio).angle();
    }
    targetAngle = Angle::normalize(robotPose.rotation + targetAngle - rotationOffset);
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
   * @param other The position of the ball in robot-relative coordinates.
   * @param maxSpeed The maximum speed of the walking engine.
   * @return An approximation of the time to reach the target pose (in milliseconds).
   *
   * @author Andreas Stolpmann
   */
  inline float calcTTRP(const Pose2f& poseRelative, const Vector2f& other, const Pose2f& maxSpeed)
  {
    const Angle angleToPose = poseRelative.translation.angle();
    const float distanceToPose = poseRelative.translation.norm();
    static_cast<void>(other);

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
}
