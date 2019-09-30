/**
 * @file RobotDimensions.h
 * Description of the dimensions of the NAO robot.
 * @author Cord Niehaus
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Math/Angle.h"
#include "Tools/Math/Eigen.h"
#include "Tools/RobotParts/FsrSensors.h"
#include "Tools/Streams/EnumIndexedArray.h"

/**
 * This representation contains all necessary dimensions of the robot.
 * The torso coordinate frame is considert to be in the middle between the hip joints.
 */
STREAMABLE(RobotDimensions,
{
  /**
   * x-offset between the neck joint and current camera.
   * @param lowerCamera true, if lower camera is in use, false otherwise.
   */
  float getXOffsetNeckToCamera(bool lowerCamera) const { return lowerCamera ? xOffsetNeckToLowerCamera : xOffsetNeckToUpperCamera; }

  /**
   * Height offset between the neck joint and current camera.
   * @param lowerCamera true, if lower camera is in use, false otherwise.
   */
  float getZOffsetNeckToCamera(bool lowerCamera) const { return lowerCamera ? zOffsetNeckToLowerCamera : zOffsetNeckToUpperCamera; }

  /**
   * Tilt of current camera against neck.
   * @param lowerCamera true, if lower camera is in use, false otherwise.
   */
  Angle getTiltNeckToCamera(bool lowerCamera) const { return lowerCamera ? tiltNeckToLowerCamera : tiltNeckToUpperCamera; },

  (float) yHipOffset,               //!< The y offset of the left hip.
  (float) upperLegLength,           //!< Length between leg joints HipPitch and KneePitch in z-direction.
  (float) lowerLegLength,           //!< Length between leg joints KneePitch and AnklePitch in z-direction.
  (float) footHeight,               //!< Height between the sole of the foot and the foot joint AnkleRoll.
  (float) footLength,               //!< Length from the ankle joint to the tip of the foot.
  (float) soleToFrontEdgeLength,    //!< Length from sole middle point to the front edge of the foot.
  (float) soleToInnerEdgeLength,    //!< Length from sole middle point to the inner edge of the foot.
  (float) soleToOuterEdgeLength,    //!< Length from sole middle point to the outer edge of the foot.
  (float) soleToBackEdgeLength,     //!< Length from sole middle point to the back edge of the foot.
  (Vector2f) bumperInnerEdge,       //!< Position offset for the inner foot bumper edge relativ to the sole origion (without sign)
  (Vector2f) bumperOuterEdge,       //!< Position offset for the outer foot bumper edge relativ to the sole origion (without sign)
  (float) hipToNeckLength,          //!< Height offset between hip and joint headYaw.

  (float) xOffsetNeckToLowerCamera, //!< Forward offset between joint headPitch and lower camera.
  (float) zOffsetNeckToLowerCamera, //!< Height offset between joint headPitch and lower camera.
  (Angle) tiltNeckToLowerCamera,    //!< Tilt of lower camera against joint headPitch.

  (float) xOffsetNeckToUpperCamera, //!< Forward offset between joint headPitch and upper camera.
  (float) zOffsetNeckToUpperCamera, //!< Height offset between joint headPitch and upper camera.
  (Angle) tiltNeckToUpperCamera,    //!< Tilt of upper camera against joint headPitch.

  (Vector3f) armOffset,             //!< The offset of joint lShoulderPitch relative to the torso coordinate frame (y must be negated for right arm).
  (float) yOffsetElbowToShoulder,   //!< The offset between the joints lShoulderRoll and lElbowYaw in y (must be negated for right arm).
  (float) upperArmLength,           //!< The length between the joints ShoulderRoll and ElbowYaw in x-direction.
  (float) lowerArmLength,           //!< The length of the lower arm starting at ElbowRoll.
  (float) xOffsetElbowToWrist,      //!< The length from Elbow to WristJoint.
  (Vector3f) handOffset,            //!< The offset of a hand relative to his wrist coordinate frame.
  (float) handRadius,               //!< The radius of a virtuel sphere a hand can span.
  (float) armRadius,                //!< The radius of an arm.

  (Vector3f) imuOffset,             //!< The offset of the imu relative to the torso coordinate frame.

  (ENUM_INDEXED_ARRAY(Vector2f, FsrSensors::FsrSensor)) leftFsrPositions, //!< The positions of the fsr on the left foot.
  (ENUM_INDEXED_ARRAY(Vector2f, FsrSensors::FsrSensor)) rightFsrPositions, //!< The positions of the fsr on the right foot.
});
