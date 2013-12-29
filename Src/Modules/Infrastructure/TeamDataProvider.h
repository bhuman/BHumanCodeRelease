/**
 * @file TeamDataProvider.h
 * This file implements a module that provides the data received by team communication.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/MessageQueue/MessageQueue.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Tools/NTP.h"

MODULE(TeamDataProvider)
  REQUIRES(FrameInfo)
  REQUIRES(RobotInfo)
  REQUIRES(OwnTeamInfo)
  REQUIRES(MotionInfo)
  USES(MotionRequest)
  PROVIDES_WITH_MODIFY_AND_DRAW(BallModel)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(GroundTruthRobotPose)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(GroundTruthBallModel)
  PROVIDES_WITH_MODIFY_AND_DRAW(TeamMateData)
END_MODULE

class TeamDataProvider : public TeamDataProviderBase, public MessageHandler
{
private:
  static PROCESS_WIDE_STORAGE(TeamDataProvider) theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none. */

  unsigned timeStamp; /**< The time when the messages currently processed were received. */
  int robotNumber; /**< The number of the robot the messages of which are currently processed. */
  unsigned lastSentTimeStamp; /**< The time when the last package to teammates was sent. */

  NTP ntp; /**< The Network Time Protocol. */
  GroundTruthRobotPose theGroundTruthRobotPose; /**< The last ground truth pose of this robot received. */
  GroundTruthBallModel theGroundTruthBallModel; /**< The last ground truth ball model of this robot received. */
  TeamMateData theTeamMateData; /**< The last received team mate data (ball model, robot pose, etc.). */

  void update(BallModel& ballModel) {ballModel = theGroundTruthBallModel;}
  void update(GroundTruthBallModel& groundTruthBallModel) {groundTruthBallModel = theGroundTruthBallModel;}
  void update(GroundTruthRobotPose& groundTruthRobotPose) {groundTruthRobotPose = theGroundTruthRobotPose;}
  void update(TeamMateData& teamMateData);

  /**
   * The method is called for every incoming team message by handleMessages.
   * @param message An interface to read the message from the queue.
   * @return true Was the message handled?
   */
  bool handleMessage(InMessage& message);

public:
  /**
   * Default constructor.
   */
  TeamDataProvider();

  /**
   * Destructor.
   */
  ~TeamDataProvider();

  /**
   * The method is called to handle all incoming team messages.
   * @param teamReceiver The message queue containing all team messages received.
   */
  static void handleMessages(MessageQueue& teamReceiver);
};
