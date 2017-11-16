/**
 * @file OdometryProvider.h
 *
 * This file declares a module that calculates walking engine odometry. The
 * calculations are taken (with small refactoring) from Colin Graf's walking
 * engine.
 *
 * @author Jonas Kuball
 */

#pragma once

#include "TorsoAngleEstimator.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Representations/Sensing/OdometryOffset.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBuffer.h"

MODULE(OdometryProvider,
{,
  REQUIRES(InertialSensorData),
  REQUIRES(MotionRequest),
  REQUIRES(RobotModel),
  REQUIRES(WalkGenerator),
  PROVIDES(OdometryOffset),
  LOADS_PARAMETERS(
  {,
    (Pose2f) odometryScale,
  }),
});

class OdometryProvider : public OdometryProviderBase
{
private:
  TorsoAngleEstimator torsoAngleEstimator; /**< Estimates the pitch and roll angles of the torso. */
  RingBuffer<bool, 3> swingPhases; /**< Whether the left leg was the swing leg in the past three frames. */
  RingBuffer<Pose2f, 3> forwardOdometryOffsets; /**< The odometry offsets computed by the walk generator for the last three frames. */
  Pose3f lastFootLeft; /**< The pose of the left foot in the previous frame. */
  Pose3f lastFootRight; /**< The pose of the right foot in the previous frame. */
  Vector3f lastOdometryOrigin = Vector3f::Zero(); /**< The odometry origin in the previous frame. */
  Pose2f odometry; /**< Continous odometry since the start of this module. */
  Pose2f lastCombinedOdometry; /**< The combined odometry from the previous frame. */

  /**
   * Computes the torso matrix for the current frame.
   * @return The torso matrix, i.e. the pose of the torso relative to the field plane.
   */
  Pose3f computeTorsoMatrix();

  /**
   * Computes the measured odometry offset from the current sensor data. Technically,
   * this is the robot's motion from three frames ago, since the sensors lag.
   * @return The relative motion of the point between the feet.
   */
  Pose2f computeOdometryOffset(const Pose3f& torsoMatrix);

  /**
   * Updates the representation provided by this module. It is an estimated odmetry
   * offset combined from the measured odometry that is three frames old and the
   * forward odoemtry from the past three frames provided by the walk generator.
   * @param theOdometryOffset The estimated motion of the robot in the current frame.
   */
  void update(OdometryOffset& theOdometryOffset);

public:
  OdometryProvider();
};
