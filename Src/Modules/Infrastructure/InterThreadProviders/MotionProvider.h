/**
 * @file MotionProvider.h
 *
 * This file declares a module that provides representations from the motion
 * thread for the current frame.
 *
 * @author Philip Reichenberg
 * @author Thomas Röfer
 */

#include "Framework/Module.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/SensorDataHistory.h"

MODULE(MotionProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(MassCalibration),
  REQUIRES(RobotDimensions),
  REQUIRES(SensorDataHistory),
  REQUIRES(JointAngles),
  PROVIDES(JointAngles),
  PROVIDES(OdometryData),
  PROVIDES(RobotModel),
  PROVIDES(TorsoMatrix),
});

class MotionProvider : public MotionProviderBase
{
  const SensorDataHistory::SensorData* motionData = nullptr; /**< The best sensor data for the current time. \c nullptr if there is none. */

  void update(JointAngles& jointAngles);
  void update(OdometryData& odometryData);
  void update(RobotModel& robotModel);
  void update(TorsoMatrix& torsoMatrix);
};
