/**
 * @file RobotDimensions.h
 * Description of the dimensions of the NAO robot.
 * @author Cord Niehaus
 * @author Thomas Röfer
 */

#pragma once

#include "Math/Angle.h"
#include "Math/Eigen.h"
#include "RobotParts/FsrSensors.h"
#include "Streaming/EnumIndexedArray.h"

/**
 * This representation contains all necessary dimensions of the robot.
 * The torso coordinate frame is considered to be in the middle between the hip joints.
 */
STREAMABLE(RobotDimensions,
{
  /**
   * Offset between the neck joint and current camera.
   * @param lowerCamera true, if lower camera is in use, false otherwise.
   */
  const Vector3f& getOffsetNeckToCamera(bool lowerCamera) const { return lowerCamera ? offsetNeckToLowerCamera : offsetNeckToUpperCamera; }

  /**
   * Tilt of current camera against neck.
   * @param lowerCamera true, if lower camera is in use, false otherwise.
   */
  Angle getTiltNeckToCamera(bool lowerCamera) const { return lowerCamera ? tiltNeckToLowerCamera : tiltNeckToUpperCamera; },

  (float) yHipOffset,                 //!< The y offset of the left hip.
  (Vector3f) hipPitchToRollOffset,    //!< The offset between leg joints HipPitch and HipRoll.
  (float) upperLegLength,             //!< Length between leg joints HipPitch and KneePitch in z-direction.
  (float) xOffsetHipToKnee,           //!< The x offset between the last hip joint and KneePitch
  (float) lowerLegLength,             //!< Length between leg joints KneePitch and AnklePitch in z-direction.
  (float) zOffsetAnklePitchToRoll,    //!< The z offset between the leg joints AnklePitch and AnkleRoll.
  (float) footHeight,                 //!< Height between the sole of the foot and the foot joint AnkleRoll.
  (float) footLength,                 //!< Length from the ankle joint to the tip of the foot.
  (float) soleToFrontEdgeLength,      //!< Length from sole origin(point below ankle joint) to the front edge of the foot.
  (float) soleToInnerEdgeLength,      //!< Length from sole origin to the inner edge of the foot.
  (float) soleToOuterEdgeLength,      //!< Length from sole origin to the outer edge of the foot.
  (float) soleToBackEdgeLength,       //!< Length from sole origin to the back edge of the foot.
  (std::vector<Vector2f>) soleShape,  //!< The contour of the sole of the left foot. The right one has mirrored y coordinates.
  (Vector2f) bumperInnerEdge,         //!< Position offset for the inner foot bumper edge relative to the sole origin (without sign)
  (Vector2f) bumperOuterEdge,         //!< Position offset for the outer foot bumper edge relative to the sole origin (without sign)
  (Vector3f) hipToNeckOffset,         //!< Offset between hip and joint headYaw.

  (Vector3f) offsetNeckToLowerCamera, //!< Offset between joint headPitch and lower camera.
  (Angle) tiltNeckToLowerCamera,      //!< Tilt of lower camera against joint headPitch.

  (Vector3f) offsetNeckToUpperCamera, //!< Offset between joint headPitch and upper camera.
  (Angle) tiltNeckToUpperCamera,      //!< Tilt of upper camera against joint headPitch.

  (Vector3f) armOffset,               //!< The offset of joint lShoulderPitch relative to the torso coordinate frame (y must be negated for right arm).
  (float) yOffsetElbowToShoulder,     //!< The offset between the joints lShoulderRoll and lElbowYaw in y (must be negated for right arm).
  (float) upperArmLength,             //!< The length between the joints ShoulderRoll and ElbowYaw in x-direction.
  (float) lowerArmLength,             //!< The length of the lower arm starting at ElbowRoll.
  (float) xOffsetElbowToWrist,        //!< The length from Elbow to WristJoint.
  (Vector3f) handOffset,              //!< The offset of a hand relative to his wrist coordinate frame.
  (float) handRadius,                 //!< The radius of a virtual sphere a hand can span.
  (float) armRadius,                  //!< The radius of an arm.

  (Vector3f) imuOffset,               //!< The offset of the imu relative to the torso coordinate frame.

  (ENUM_INDEXED_ARRAY(Vector2f, FsrSensors::FsrSensor)) leftFsrPositions, //!< The positions of the fsr on the left foot.
  (ENUM_INDEXED_ARRAY(Vector2f, FsrSensors::FsrSensor)) rightFsrPositions, //!< The positions of the fsr on the right foot.
  (float) simOriginHeight, /**< The height above ground of the origin of the simulation model. */
  (float) robotDepth, /**< The assumed depth of a robot in obstacle detection. */
});
