/**
 * @file GroundContactDetector.h
 * Declaration of module GroundContactDetector.
 * @author Colin Graf
 */

#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Tools/RingBufferWithSum.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/RotationMatrix.h"
#include "Tools/Module/Module.h"

MODULE(GroundContactDetector,
{,
  REQUIRES(FrameInfo),
  REQUIRES(InertialData),
  REQUIRES(MotionRequest),
  REQUIRES(RobotModel),
  USES(MotionInfo),
  USES(TorsoMatrix), // TODO: This can be replaced with InertialData
  PROVIDES(GroundContactState),
  DEFINES_PARAMETERS(
  {,
    (float)(0.08f) noContactMinAccNoise,
    (float)(0.04f) noContactMinGyroNoise,
    (float)(0.01f) contactMaxAngleNoise,
    (float)(0.007f) contactAngleActivationNoise,
    (float)(20.f) contactMaxAccZ, // deactivate
  }),
});

/**
 * @class GroundContactDetector
 * A module for sensor data filtering.
 */
class GroundContactDetector : public GroundContactDetectorBase
{
private:
  bool contact = false; /**< Whether the robot has ground contact or not */
  unsigned int contactStartTime = 0; /**< Time when the robot started having ground contact */
  bool useAngle = false; /**< Whether the estimated angle will be used to detect ground contact loss */

  RotationMatrix expectedRotationInv;
  RingBufferWithSum<Vector2f, 60> angleNoises = {Vector2f::Zero()};
  RingBufferWithSum<Vector3f, 60> accNoises = {Vector3f::Zero()};
  RingBufferWithSum<Vector2f, 60> gyroNoises = {Vector2f::Zero()};
  RingBufferWithSum<Vector3f, 60> accValues = {Vector3f::Zero()};
  RingBufferWithSum<Vector2f, 60> gyroValues = {Vector2f::Zero()};
  RingBufferWithSum<float, 5> calibratedAccZValues;

  /**
   * Updates the GroundContactState representation .
   * @param groundContactState The ground contact representation which is updated by this module.
   */
  void update(GroundContactState& groundContactState);
};
