/**
 * @file GyroOffsetProvider.h
 * This file declares a module that shall detect if the gyro values of the IMU have offsets.
 * The gyros are sampled over a period of time and if their deviation is low but the values not close to 0, we assume they have offsets.
 * This happens once a while on a V6 NAO.
 * @author Philip Reichenberg
 */

#pragma once

#include "Representations/Sensing/GyroOffset.h"
#include "Representations/Sensing/IMUValueState.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/SensorData/RawInertialSensorData.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Framework/Module.h"
#include "Math/RingBufferWithSum.h"

MODULE(GyroOffsetProvider,
{,
  REQUIRES(FrameInfo),
  USES(GameState),
  REQUIRES(GroundContactState),
  REQUIRES(IMUValueState),
  USES(MotionInfo),
  REQUIRES(RawInertialSensorData),
  REQUIRES(MotionRobotHealth),
  PROVIDES(GyroOffset),
  DEFINES_PARAMETERS(
  {,
    (Angle)(3_deg) thresholdZero, // if the NAO is not moving, the gyros should be lower than this value
    (Rangea)(Rangea(-0.5_deg, 0.5_deg)) bodyDisconnectGyroRange, // To detect a body disconnect, the gyros must have values outside of this range
    (int)(2000) waitTimeBeforeSampling, // Wait this much before sampling
    (int)(1000) maxGyroDelay, // gyros need to update at least once in this time, in ms
    (unsigned int)(101000) startTimestamp, // check stuck gyro after the software is running for at least for 1 second
    (int)(1500) bodyDisconnectWaitTime, // Wait this much time, until the body connection is back. The joints need about one seconds, until the stiffness is back
    (int)(10000) gyroOffsetWarningTime, // Repeat the sound, that the robot needs a reboot, every 10secs
    (int)(20000) gyroNotCheckedWarningTime, // Repeat that the gyro was not checked after this duration
    (int)(1000) waitForInitialGroundContactTime, // The robot shall not warn us, if it never had ground contact
  }),
});

class GyroOffsetProvider : public GyroOffsetProviderBase
{
public:
  GyroOffsetProvider();
private:

  ENUM(State,
  {,
    waiting,
    sampling,
    set,
    off,
  });

  // RingBuffer for the last 3 gyro mean (x) and deviation (y) values, from GyroState
  RingBuffer<Angle, 3> gyroMeanX;
  RingBuffer<Angle, 3> gyroMeanY;
  RingBuffer<Angle, 3> gyroMeanZ;

  // State of the gyro offset check
  State state = State::waiting;

  // Last unique gyro values. They should change every second motion frame
  Vector3a lastGyros = Vector3a::Zero();

  unsigned int lastGyroStateUpdate = 0;
  unsigned int lastGyroChange = 0;
  unsigned int samplingStart = 0;
  unsigned int gyroStuckTimestamp = 0;
  unsigned int gyroStuckSoundTimestamp = 0;
  unsigned int gyroOffsetSoundTimestamp = 0;
  unsigned int gyroNotCheckedTimestamp = 0;
  unsigned int timeStampStart = 0;
  bool hadGroundContactOnce = false;

  void update(GyroOffset& gyroOffset) override;

  /**
   * This method checks if the gyro values did not change over a longer period of time.
   * In such a case, a sound is played and the flag gyroIsStuck is set to true.
   * The flag is used in the KeyframeMotionEngine, to prevent a get up try while the orientation of the torso can not be calculated
   * and the InertialDataProvider, to prevent a miscalculation in the torso orientation.
   * When the flag is switched to false, the orientation from LOLA is used as a initial value.
   */
  void checkBodyDisconnection(GyroOffset& gyroOffset);
};
