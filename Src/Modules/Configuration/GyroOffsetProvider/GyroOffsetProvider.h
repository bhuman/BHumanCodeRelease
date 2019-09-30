/**
 * @file GyroOffsetProvider.h
 * This file declares a module that shall detect if the gyro values of the IMU have offsets.
 * The gyros are sampled over a periode of time and if their deviation is low but the values not close to 0, we assume they have offsets.
 * This happens once a while on a V6 NAO.
 * @author Philip Reichenberg
 */

#pragma once

#include "Platform/SystemCall.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Configuration/GyroOffset.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBufferWithSum.h"
#include "Tools/Settings.h"

MODULE(GyroOffsetProvider,
{,
  USES(FrameInfo),
  USES(GameInfo),
  USES(GroundContactState),
  USES(InertialData),
  USES(MotionRequest),
  USES(RobotInfo),
  PROVIDES(GyroOffset),
  DEFINES_PARAMETERS(
  {,
    (Angle)(3_deg) thresholdGyroDeviation, // if the NAO is not moving, the gyros should vary max by this value. It must be high, because the gyros have a high deviation over a long time interval
    (Angle)(1_deg) thresholdZero, // if the NAO is not moving, the gyros should be lower than this value
    (int)(100) thresholdTime, // if a check was positive, wait this time until the next check
    (int)(10) minChecks, // to many checks in a row must be positive befor we do a calibration
    (int)(2000) waitTime,
    (int)(100)  maxGyroDelay, // gyros need to update at least once in this time, in ms
    (unsigned int)(101000) startTimestamp, // check stucked gyro after the software is running for at least for 1 second
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
  //RingBuffer for the last 99 gyro values
  RingBufferWithSum<Angle, 99> gyroValuesX;
  RingBufferWithSum<Angle, 99> gyroValuesY;
  RingBufferWithSum<Angle, 99> gyroValuesZ;
  Vector3a lastGyros;
  unsigned int lastGyroChange;
  bool wasPlayingOnce;
  State state;
  int samplingCounter;
  std::vector<int> gyroChecks;
  unsigned int samplingStart;
  unsigned int gyroStuckTimestamp;
  std::vector<unsigned int> timestamps; //timestamps, so the checks are done once per thresholdTime

  void update(GyroOffset& gyroOffset) override;

  /**
   * This method calculates the deviation in the sampled gyro values
   * @param buffer The ring buffer with the sampled gyro value
   */
  Angle calcDeviation(RingBufferWithSum<Angle, 33>& buffer);

  /**
   * This method checks if the gyro values did not change over a longer periode of time.
   * In such a case, a sound is played and the flag gyroIsStuck is set to true.
   * The flag is used in the GetUpEngine, to prevent a get up try while the orientation of the torso can not be calculated
                         and the InertialDataProvider, to prevent a miscalculation in the torso orientation. When the flag is switched to false, the orientation from LOLA is used as a initial value.
   */
  void checkGyroDelay(GyroOffset& gyroOffset);
};
