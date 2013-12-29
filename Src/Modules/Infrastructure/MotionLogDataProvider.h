/**
* @file MotionLogDataProvider.h
* This file declares a module that provides data replayed from a log file.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Infrastructure/JointDataDeg.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Infrastructure/KeyStates.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Sensing/OrientationData.h"
#include "Tools/MessageQueue/InMessage.h"
#include "LogDataProvider.h"

MODULE(MotionLogDataProvider)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(JointData)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(SensorData)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(KeyStates)
  PROVIDES_WITH_MODIFY(FrameInfo)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(OdometryData)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(GroundTruthOdometryData)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(OrientationData)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(GroundTruthOrientationData)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(GameInfo)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(OwnTeamInfo)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(OpponentTeamInfo)
  PROVIDES_WITH_MODIFY(RobotInfo)
  PROVIDES(FilteredSensorData)
  PROVIDES(FilteredJointData)
END_MODULE

class MotionLogDataProvider : public MotionLogDataProviderBase, public LogDataProvider
{
private:
  static PROCESS_WIDE_STORAGE(MotionLogDataProvider) theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none. */
  bool frameDataComplete; /**< Were all messages of the current frame received? */

#ifndef RELEASE
  OdometryData lastOdometryData;
#endif

  /**
  * The method is called for every incoming debug message by handleMessage.
  * @param message An interface to read the message from the queue.
  * @return Was the message handled?
  */
  bool handleMessage2(InMessage& message);

public:
  /**
  * Default constructor.
  */
  MotionLogDataProvider();

  /**
  * Destructor.
  */
  ~MotionLogDataProvider();

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

  UPDATE(FrameInfo)
  UPDATE2(JointData,
  {
    JointDataDeg jointDataDeg(_JointData);
    MODIFY("representation:JointDataDeg", jointDataDeg);
  })
  UPDATE(KeyStates)
  UPDATE(OdometryData)
  UPDATE(SensorData)
  UPDATE(FilteredSensorData)
  UPDATE(FilteredJointData)
  UPDATE(OrientationData)
  UPDATE(GameInfo)
  UPDATE(OwnTeamInfo)
  UPDATE(OpponentTeamInfo)
  UPDATE(RobotInfo)

  void update(GroundTruthOdometryData&);
  void update(GroundTruthOrientationData&);
};
