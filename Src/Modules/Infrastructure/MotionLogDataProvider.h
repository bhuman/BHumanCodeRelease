/**
 * @file MotionLogDataProvider.h
 * This file declares a module that provides data replayed from a log file.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Infrastructure/SensorData/UsSensorData.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Tools/MessageQueue/InMessage.h"
#include "Tools/Module/Module.h"
#include "LogDataProvider.h"

MODULE(MotionLogDataProvider,
{,
  PROVIDES(FrameInfo),
  PROVIDES(GroundTruthOdometryData),
  PROVIDES(InertialSensorData),
  PROVIDES(JointAngles),
  PROVIDES(JointSensorData),
  PROVIDES(KeyStates),
  PROVIDES(OdometryData),
  PROVIDES(OpponentTeamInfo),
  PROVIDES(OwnTeamInfo),
  PROVIDES(RawGameInfo),
  PROVIDES(RobotInfo),
  PROVIDES(UsSensorData),
});

class MotionLogDataProvider : public MotionLogDataProviderBase, public LogDataProvider
{
private:
  static PROCESS_LOCAL MotionLogDataProvider* theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none. */
  bool frameDataComplete; /**< Were all messages of the current frame received? */
  OdometryData lastOdometryData;

  /**
   * The method is called for every incoming debug message by handleMessage.
   * @param message An interface to read the message from the queue.
   * @return Was the message handled?
   */
  bool handleMessage2(InMessage& message);

public:
  MotionLogDataProvider();
  ~MotionLogDataProvider();

  void update(FrameInfo&) {}
  void update(GroundTruthOdometryData&);
  void update(InertialSensorData&) {}
  void update(JointAngles&) {}
  void update(JointSensorData&) {}
  void update(KeyStates&) {}
  void update(OdometryData&) {}
  void update(OpponentTeamInfo&) {}
  void update(OwnTeamInfo&) {}
  void update(RawGameInfo&) {}
  void update(RobotInfo&) {}
  void update(UsSensorData&) {}

  /**
   * The method is called for every incoming debug message.
   * @param message An interface to read the message from the queue.
   * @return Was the message handled?
   */
  static bool handleMessage(InMessage& message);

  /**
   * The method returns whether idProcessFinished was received.
   * @return Were all messages of the current frame received?
   */
  static bool isFrameDataComplete();
};
