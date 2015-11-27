/**
 * @file InverseKinematic.h
 * @author Alexander Härtl
 * @author jeff
 */

#pragma once

#include "Tools/Math/Eigen.h"

struct CameraCalibration;
struct JointAngles;
struct JointCalibration;
struct Pose3f;
struct RobotDimensions;

namespace InverseKinematic
{
  /**
   * This method calculates the joint angles for the legs of the robot from a Pose3f for each leg.
   * @param positionLeft The desired position (translation + rotation) of the left foots ankle point.
   * @param positionRight The desired position (translation + rotation) of the right foots ankle point.
   * @param jointAngles The instance of JointAngles where the resulting joint angles are written into.
   * @param robotDimensions The Robot Dimensions needed for calculation.
   * @param ratio The ratio between the left and right yaw angle.
   * @return Whether the target position was reachable or not (if the given target position is not reachable the computation proceeds using the closest reachable position near the target).
   */
  bool calcLegJoints(const Pose3f& positionLeft, const Pose3f& positionRight, JointAngles& jointAngles,
                     const RobotDimensions& robotDimensions, float ratio = 0.5f) WARN_UNUSED_RESULT;

  /**
   * This method calculates the joint angles for the legs of the robot from a Pose3f for each leg and the body ptch and roll.
   * @param positionLeft The desired position (translation + rotation) of the left foots point
   * @param positionRight The desired position (translation + rotation) of the right foots point
   * @param bodyRotation The rotation of the body around the x-Axis and y-Axis
   * @param jointAngles The instance of JointAngles where the resulting joint angles are written into.
   * @param robotDimensions The RobotDimensions needed for calculation
   * @param ratio The ratio between the left and right yaw angle
   * @return Whether the target position was reachable or not (if the given target position is not reachable the computation proceeds using the closest reachable position near the target)
   */
  bool calcLegJoints(const Pose3f& positionLeft, const Pose3f& positionRight, const Vector2f& bodyRotation, JointAngles& jointAngles,
                     const RobotDimensions& robotDimensions, float ratio = 0.5f) WARN_UNUSED_RESULT;
  bool calcLegJoints(const Pose3f& positionLeft, const Pose3f& positionRight, const Quaternionf& bodyRotation, JointAngles& jointAngles,
                const RobotDimensions& robotDimensions, float ratio = 0.5f) WARN_UNUSED_RESULT;

  /**
   * Solves the inverse kinematics for the head of the Nao such that the camera looks at a certain point.
   * @param position Point the camera should look at in cartesian space relative to the robot origin.
   * @param imageTilt Tilt angle at which the point should appear in the image (pi/2: center of image, less than pi/2 => closer to the top of the image.)
   * @param panTilt Vector [pan, tilt] containing the resulting joint angles.
   * @param lowerCamera true if the joint angles are to be determined for the lower camera, false for the upper camera.
   * @param robotDimensions The robot dimensions needed for the calculation.
   * @param cameraCalibration The camera calibration
   */
  void calcHeadJoints(const Vector3f& position, const float imageTilt, const RobotDimensions& robotDimensions,
                      const bool lowerCamera, Vector2f& panTilt, const CameraCalibration& cameraCalibration);
};
