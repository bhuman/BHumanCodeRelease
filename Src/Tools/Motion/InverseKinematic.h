/**
 * @file InverseKinematic.h
 * @author Alexander Härtl
 * @author jeff
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Representations/Infrastructure/CameraInfo.h"
#include "Tools/Math/Eigen.h"
#include "Tools/RobotParts/Arms.h"
#include "Tools/RobotParts/Legs.h"
#include "Tools/Streams/Enum.h"

struct CameraCalibration;
struct JointAngles;
struct JointLimits;
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
  [[nodiscard]] bool calcLegJoints(const Pose3f& positionLeft, const Pose3f& positionRight, JointAngles& jointAngles,
                                   const RobotDimensions& robotDimensions, float ratio = 0.5f);

  /**
   * This method calculates the joint angles for the legs of the robot from a Pose3f for each leg and the body pitch and roll.
   * @param positionLeft The desired position (translation + rotation) of the left foots point
   * @param positionRight The desired position (translation + rotation) of the right foots point
   * @param bodyRotation The rotation of the body around the x-Axis and y-Axis
   * @param jointAngles The instance of JointAngles where the resulting joint angles are written into.
   * @param robotDimensions The RobotDimensions needed for calculation
   * @param ratio The ratio between the left and right yaw angle
   * @return Whether the target position was reachable or not (if the given target position is not reachable the computation proceeds using the closest reachable position near the target)
   */
  [[nodiscard]] bool calcLegJoints(const Pose3f& positionLeft, const Pose3f& positionRight, const Vector2f& bodyRotation, JointAngles& jointAngles,
                                   const RobotDimensions& robotDimensions, float ratio = 0.5f);
  [[nodiscard]] bool calcLegJoints(const Pose3f& positionLeft, const Pose3f& positionRight, const Quaternionf& bodyRotation, JointAngles& jointAngles,
                                   const RobotDimensions& robotDimensions, float ratio = 0.5f);

  /**
   * Solves the inverse kinematics for the head of the Nao such that the camera looks at a certain point.
   * @param position Point the camera should look at in cartesian space relative to the robot origin.
   * @param imageTilt Tilt angle at which the point should appear in the image (pi/2: center of image, less than pi/2 => closer to the top of the image.)
   * @param camera The joint angles are to be determined for this camera.
   * @param robotDimensions The robot dimensions needed for the calculation.
   * @param panTilt Vector [pan, tilt] containing the resulting joint angles.
   * @param cameraCalibration The camera calibration
   */
  void calcHeadJoints(const Vector3f& position, Angle imageTilt, const RobotDimensions& robotDimensions,
                      CameraInfo::Camera camera, Vector2a& panTilt, const CameraCalibration& cameraCalibration);

  ENUM(IKAError,
  {,
    shoulderPitchOutOfRange,
    shoulderRollOutOfRange,
    elbowYawOutOfRange,
    elbowRollOutOfRange,
    armToShort,
    noPossibleElbowPosition,
  });

  /**
   * This method calculates the joint angles for an arm of the robot from an elbow and hand position.
   *
   * @param arm specifies the arm (left or right) to calculate for
   * @param elBowPosition the elbow position to calc from (relative to the robots origin)
   * @param handPostion the hand position to calc from (relative to the robots origin)
   * @param robotDimensions the dimensions of the robot
   * @param jointLimits the joint limits of the robot
   * @param jointAngle the result: the calculated joint angles for the given parameters
   * @return An ENUMSET of IKAError's. All marked errors are occurred -> 0 means no errors
   *      note: the lengths will not be checked
   */
  [[nodiscard]] unsigned calcArmJoints(const Arms::Arm arm, const Vector3f& elBowPosition, const Vector3f& handPosition,
                                       const RobotDimensions& robotDimensions, const JointLimits& jointLimits,
                                       JointAngles& jointAngles);

  /**
   * This method calculates the joint angles for an arm of the robot from a hand pose.
   *
   * @param arm specifies the arm (left or right) to calculate for
   * @param handPostion the hand pose to calc from (relative to the robots origin)
   * @param robotDimensions the dimensions of the robot
   * @param jointLimits the joint limits of the robot
   * @param jointAngle the result: the calculated joint angles for the given parameters
   * @return An ENUMSET of IKAError's. All marked errors are occurred -> 0 means no errors
   *      note: the lengths will not be checked
   */
  [[nodiscard]] unsigned calcArmJoints(const Arms::Arm arm, const Pose3f& handPose, const RobotDimensions& robotDimensions,
                                       const JointLimits& jointLimits, JointAngles& jointAngles);

  /**
   * This method calculates the joint angles for an upper arm of the robot from an elbow position.
   *
   * @param arm specifies the arm (left or right) to calculate for
   * @param elBowPosition the elbow position to calc from (relative to the robots origin)
   * @param robotDimensions the dimensions of the robot
   * @param jointLimits the joint limits of the robot
   * @param jointAngle the result: the calculated joint angles for the given parameters
   * @return An ENUMSET of IKAError's. All marked errors are occurred -> 0 means no errors
   *      note: the length will not be checked
   */
  unsigned calcShoulderJoints(const Arms::Arm arm, const Vector3f& elBowPosition, const RobotDimensions& robotDimensions,
                              const JointLimits& jointLimits, JointAngles& jointAngles);
};
