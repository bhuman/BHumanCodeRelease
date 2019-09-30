/**
 * @file Skills.h
 *
 * This file declares all skills that are used by the current behavior.
 *
 * @author Probably many people who will not add themselves to this declaration.
 */

#pragma once

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/MotionControl/ArmKeyFrameRequest.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/KickRequest.h"
#include "Representations/MotionControl/SpecialActionRequest.h"
#include "Representations/MotionControl/WalkKickGenerator.h"
#include "Tools/BehaviorControl/Framework/Skill/Skill.h"
#include "Tools/Math/Angle.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/RobotParts/Arms.h"
#include <string>

namespace Skills
{
  /** This skill executes the get up engine. */
  SKILL_INTERFACE(GetUpEngine);

  /**
   * This skill executes an in walk kick.
   * @param walkKick The walk kick variant that should be executed
   * @param kickPose The pose at which the kick should be executed in robot-relative coordinates
   */
  SKILL_INTERFACE(InWalkKick, (const WalkKickVariant&) walkKick, (const Pose2f&) kickPose);

  /**
   * This skill executes a kick.
   * @param kickType The kick motion ID of the kick that should be executed
   * @param mirror Whether the kick should be mirrored
   * @param length The desired length of the kick (i.e. distance that the ball moves)
   * @param armsBackFix Use inverse elbow yaw if backLikeDevil is active (motion must be designed for that)
   */
  SKILL_INTERFACE(Kick, (KickRequest::KickMotionID) kickType, (bool) mirror, (float) length, (bool)(true) armsBackFix);

  /**
   * This skill walks to a target using a path planner.
   * @param speed The walking speed as ratio of the maximum speed in [0, 1]
   * @param target The target pose in absolute field coordinates
   */
  SKILL_INTERFACE(PathToTarget, (float) speed, (const Pose2f&) target);

  /**
   * This skill lets the robot execute a special action.
   * @param id The ID of the motion
   * @param mirror Whether the motion should be mirrored
   */
  SKILL_INTERFACE(SpecialAction, (SpecialActionRequest::SpecialActionID) id, (bool)(false) mirror);

  /** This skill makes the robot stand. */
  SKILL_INTERFACE(Stand);

  /**
   * This skill walks with a specified speed.
   * @param speed The walking speed in radians/s for the rotation and mm/s for the translation
   */
  SKILL_INTERFACE(WalkAtAbsoluteSpeed, (const Pose2f&) speed);

  /**
   * This skill walks with a specified speed relative to the configured maximum.
   * @param speed The walking speed as ratio of the maximum speed in [0, 1]
   */
  SKILL_INTERFACE(WalkAtRelativeSpeed, (const Pose2f&) speed);

  /**
   * This skill walks to a (relative) target.
   * @param speed The walking speed as ratio of the maximum speed in [0, 1]
   * @param target The target pose in robot-relative coordinates
   */
  SKILL_INTERFACE(WalkToTarget, (const Pose2f&) speed, (const Pose2f&) target);

  /**
   * This skill executes a key frame motion with both arms.
   * @param motion The motion that the arm should execute
   * @param fast Whether states should not be interpolated
   */
  SKILL_INTERFACE(KeyFrameArms, (ArmKeyFrameRequest::ArmKeyFrameId) motion, (bool)(false) fast);

  /**
   * This skill executes a key frame motion with a single arm.
   * @param motion The motion that the arm should execute
   * @param arm The arm that should execute the motion
   * @param fast Whether states should not be interpolated
   */
  SKILL_INTERFACE(KeyFrameSingleArm, (ArmKeyFrameRequest::ArmKeyFrameId) motion, (Arms::Arm) arm, (bool)(false) fast);

  /**
   * This skill lets one arm point at some point.
   * @param localPoint The point in robot-relative coordinates
   */
  SKILL_INTERFACE(PointAt, (const Vector3f&) localPoint);

  /**
   * This skill lets a specific arm point at some point.
   * @param localPoint The point in robot-relative coordinates
   * @param arm The arm that shall be used for pointing
   */
  SKILL_INTERFACE(PointAtWithArm, (const Vector3f&) localPoint, (Arms::Arm) arm);

  /**
   * This skill moves the head so that a camera looks at specified angles.
   * @param pan The target pan angle
   * @param tilt The target tilt angle
   * @param speed The speed with which to move the head
   * @param camera The camera which should have the specified angles
   */
  SKILL_INTERFACE(LookAtAngles, (Angle) pan, (Angle) tilt, (float)(180_deg) speed, (HeadMotionRequest::CameraControlMode)(HeadMotionRequest::autoCamera) camera);

  /**
   * This skill moves the head such that a specified (robot-relative) point is focused by one camera.
   * @param target The point to look at in robot-relative coordinates
   * @param camera The camera which should look at the point
   * @param speed The speed with which to move the head
   */
  SKILL_INTERFACE(LookAtPoint, (const Vector3f&) target, (HeadMotionRequest::CameraControlMode)(HeadMotionRequest::autoCamera) camera, (Angle)(180_deg) speed);

  /** This skill moves the head so that it looks forward.  */
  SKILL_INTERFACE(LookForward);

  /**
   * This skill sets the passTarget member of the BehaviorStatus.
   * @param passTarget The passTarget to set
   * @param shootingTo Optionally, the position where the ball should end up in robot-relative coordinates
   */
  SKILL_INTERFACE(PassTarget, (int) passTarget, (const Vector2f&)(Vector2f::Zero()) ballTarget);

  /**
   * This skill sets the activity member of the BehaviorStatus.
   * @param activity The activity to set
   */
  SKILL_INTERFACE(Activity, (BehaviorStatus::Activity) activity);

  /**
   * This skill adds an annotation if it differs from the one that has been added in the last frame.
   * @param annotation The annotation message
   */
  SKILL_INTERFACE(Annotation, (const std::string&) annotation);

  /**
   * This skill plays a sound file if it differs from the one that has been played in the last frame.
   * @param name The name of the sound file
   */
  SKILL_INTERFACE(PlaySound, (const std::string&) name);

  /**
   * This skill makes the Nao say something if it differs from what was said in the last frame.
   * @param name The text to be synthesized and pronounced
   */
  SKILL_INTERFACE(Say, (const std::string&) text);
}
