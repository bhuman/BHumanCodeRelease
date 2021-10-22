/**
 * @file GyroOffsetProvider.h
 * This file declares a module that shall detect if the gyro values of the IMU have offsets.
 * The gyros are sampled over a period of time and if their deviation is low but the values not close to 0, we assume they have offsets.
 * This happens once a while on a V6 NAO.
 * @author Philip Reichenberg
 */

#pragma once

#include "Platform/SystemCall.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Sensing/GyroOffset.h"
#include "Representations/Sensing/GyroState.h"
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
  REQUIRES(GyroState),
  PROVIDES(GyroOffset),
  DEFINES_PARAMETERS(
  {,
    (Angle)(3_deg) thresholdGyroDeviation, // if the NAO is not moving, the gyros should vary max by this value. It must be high, because the gyros have a high deviation over a long time interval
    (Angle)(1_deg) thresholdZero, // if the NAO is not moving, the gyros should be lower than this value
    (int)(100) thresholdTime, // if a check was positive, wait this time until the next check
    (int)(10) minChecks, // so many checks in a row must be positive before we do a calibration
    (int)(2000) waitTimeBeforeSampling, // Wait this much befor sampling
    (int)(100) maxGyroDelay, // gyros need to update at least once in this time, in ms
    (unsigned int)(101000) startTimestamp, // check stuck gyro after the software is running for at least for 1 second
    (int)(1500) bodyDisconnectWaitTime, // Wait this much time, until the body connection is back. The joints need about one seconds, until the stiffness is back
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
  //RingBuffer for the last 3 gyro mean (x) and deviation (y) values, from GyroState
  RingBuffer<Angle, 3> gyroMeanX;
  RingBuffer<Angle, 3> gyroMeanY;
  RingBuffer<Angle, 3> gyroMeanZ;
  RingBuffer<Angle, 3> gyroDeviationX;
  RingBuffer<Angle, 3> gyroDeviationY;
  RingBuffer<Angle, 3> gyroDeviationZ;

  int samplingCounter;
  unsigned int lastGyroStateUpdate;
  Vector3a lastGyros;
  unsigned int lastGyroChange;
  bool wasPlayingOnce;
  State state;
  std::vector<int> gyroChecks;
  unsigned int samplingStart;
  unsigned int gyroStuckTimestamp;
  unsigned int gyroStuckSoundTimestamp;
  std::vector<unsigned int> timestamps; //timestamps, so the checks are done once per thresholdTime

  void update(GyroOffset& gyroOffset) override;

  /**
   * This method checks if the gyro values did not change over a longer period of time.
   * In such a case, a sound is played and the flag gyroIsStuck is set to true.
   * The flag is used in the GetUpEngine, to prevent a get up try while the orientation of the torso can not be calculated
                         and the InertialDataProvider, to prevent a miscalculation in the torso orientation. When the flag is switched to false, the orientation from LOLA is used as a initial value.
   */
  void checkBodyDisconnection(GyroOffset& gyroOffset);
};
