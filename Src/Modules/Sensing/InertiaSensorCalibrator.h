/**
* @file InertiaSensorCalibrator.h
* Declaration of module InertiaSensorCalibrator.
* @author Colin Graf
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/RingBuffer.h"
#include "Tools/RingBufferWithSum.h"
#include "Tools/Math/Kalman.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Sensing/InertiaSensorData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Configuration/SensorCalibration.h"

MODULE(InertiaSensorCalibrator,
{,
  REQUIRES(SensorData),
  REQUIRES(RobotModel),
  REQUIRES(FrameInfo),
  REQUIRES(RobotInfo),
  REQUIRES(GameInfo),
  REQUIRES(GroundContactState),
  REQUIRES(JointCalibration),
  REQUIRES(SensorCalibration),
  USES(MotionSelection),
  USES(MotionInfo),
  USES(WalkingEngineOutput),
  PROVIDES_WITH_MODIFY(InertiaSensorData),
  DEFINES_PARAMETERS(
  {,
    (unsigned)(1500) timeFrame, /**< The time frame within unstable situations lead to dropping averaged gyro and acceleration measurements. (in ms) */
    (unsigned)(400) penalizedTimeFrame, /**< Time frame expansion after receiving a penalty or before getting unpanlized (in ms) */
    (Vector2<>)(0.05f, 0.05f) gyroBiasProcessNoise, /**< The process noise of the gyro offset estimator. */
    (Vector2<>)(0.01f, 0.01f) gyroBiasStandMeasurementNoise, /**< The noise of gyro measurements and the gyro offset while standing. */
    (Vector2<>)(0.1f, 0.1f) gyroBiasWalkMeasurementNoise, /**< The noise of gyro measurements and the gyro offset while walking. */
    (Vector3<>)(0.1f, 0.1f, 0.1f) accBiasProcessNoise, /**< The process noise of the acceleration sensor offset estimator. */
    (Vector3<>)(0.1f, 0.1f, 0.1f) accBiasStandMeasurementNoise, /**< The noise of acceleration sensor measurements and the acceleration sensor offset while standing. */
    (Vector3<>)(1.f, 1.f, 1.f) accBiasWalkMeasurementNoise, /**< The noise of acceleration sensor measurements and the acceleration sensor offset while walking. */
  }),
});

/**
* @class InertiaSensorCalibrator
* A module for determining the bias of the inertia sensor readings.
*/
class InertiaSensorCalibrator : public InertiaSensorCalibratorBase
{
public:
  /** Default constructor. */
  InertiaSensorCalibrator();

private:
  /**
  * Class for buffering averaged gyro and acceleration sensor readings.
  */
  class Collection
  {
  public:
    Vector3<> accAvg; /**< The average of acceleration sensor readings of one walking phase or 1 sec. */
    Vector2<> gyroAvg; /**< The average of gyro sensor eadings of one walking phase or 1 sec. */
    unsigned int timeStamp; /**< When this collection was created. */
    bool standing; /**< Whether the robot was standing while collecting sensor readings. */

    /**
    * Constructs a collection.
    */
    Collection(const Vector3<>& accAvg, const Vector2<>& gyroAvg, unsigned int timeStamp, bool standing) :
      accAvg(accAvg), gyroAvg(gyroAvg), timeStamp(timeStamp), standing(standing) {}

    /**
    * Default constructor.
    */
    Collection() = default;
  };

  bool calibrated; /**< Whether the filters are initialized. */
  Kalman<float> accXBias; /**< The calibration bias of accX. */
  Kalman<float> accYBias; /**< The calibration bias of accY. */
  Kalman<float> accZBias; /**< The calibration bias of accZ. */
  Kalman<float> gyroXBias; /**< The calibration bias of gyroX. */
  Kalman<float> gyroYBias; /**< The calibration bias of gyroY. */

  unsigned timeWhenPenalized; /**< When the robot received the last penalty */
  unsigned int collectionStartTime; /**< When the current collection was started. */
  unsigned int cleanCollectionStartTime; /**< When the last unstable situation was over. */

  RingBufferWithSum<Vector3<>, 300> accValues; /**< Ringbuffer for collecting the acceleration sensor values of one walking phase or 1 sec. */
  RingBufferWithSum<Vector2<>, 300> gyroValues; /**< Ringbuffer for collecting the gyro sensor values of one walking phase or 1 sec. */

  RingBuffer<Collection, 50> collections; /**< Buffered averaged gyro and accleration sensor readings. */

  unsigned int lastTime; /**< The time of the previous iteration. */
  MotionRequest::Motion lastMotion; /**< The executed motion of the previous iteration. */
  double lastPositionInWalkCycle; /**< The walk cycle position of the previous iteration. */
  unsigned lastPenalty; /** The penalty state of the previous iteration. */

  RotationMatrix calculatedRotation; /**< Body rotation, which was calculated using kinematics. */

#ifndef RELEASE
  JointCalibration lastJointCalibration; /**< Some parts of the joint calibration of the previous iteration. */
#endif

  /**
  * Resets all internal values (including determined calibration) of this module.
  */
  void reset();

  /**
  * Updates the InertiaSensorData representation.
  * @param inertiaSensorData The inertia sensor data representation which is updated by this module.
  */
  void update(InertiaSensorData& inertiaSensorData);
};
