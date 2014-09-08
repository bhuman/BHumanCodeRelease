/**
 * @file TeamDataProvider.h
 * This file implements a module that provides the data received by team communication.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/MessageQueue/MessageQueue.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Modeling/TeammateReliability.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/FallDownState.h"
#include "Tools/NTP.h"
#include <cstdint>

MODULE(TeamDataProvider,
{,
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(MotionInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RobotInfo),
  REQUIRES(TeammateReliability),
  USES(CameraMatrix),
  USES(MotionRequest),
  //-To draw the own data ----------
  USES(RobotPose),
  USES(SideConfidence),
  USES(BallModel),
  USES(GroundContactState),
  USES(FallDownState),
  //--------------------------------
  PROVIDES_WITH_MODIFY_AND_DRAW(TeammateData),
  REQUIRES(TeammateData),
  PROVIDES(TeammateDataCompressed),
  DEFINES_PARAMETERS(
  {,
    (int)(200) sendInterval,
    (float)(0.2f) dropInSpeedOfRobot, /** < Speed of a DropIn Player in mm/ms*/
    (float)(0.065f) dropInRotationSpeedOfRobot, /** < Rotation speed of a DropIn Player in degree/ms */
    (float)(1000.f) dropInBonusForTheStriker, /** < the time bonus should stable the striker role */
    (float)(1500.f) dropInRelibilityOK, /** < time is added to timeToReachBall for relibility OK (should be greater than zero )*/
    (float)(0.f) dropInRelibilityGOOD, /** < time is added to timeToReachBall for relibility GOOD (should be smaller than dropInRelibilityOK and greater than zero)*/
    (float)(2000.f) dropInKeeperToGoalThreshold, /** < DropIn player becomes keeper if distance to goal is shorter as threshold*/
  }),
});

class TeamDataProvider : public TeamDataProviderBase, public MessageHandler
{
private:
  static PROCESS_WIDE_STORAGE(TeamDataProvider) theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none. */

  unsigned timeStamp; /**< The time when the messages currently processed were received. */
  int robotNumber; /**< The number of the robot the messages of which are currently processed. */
  unsigned lastSentTimeStamp; /**< The time when the last package to teammates was sent. */
  int ballAge;

  NTP ntp; /**< The Network Time Protocol. */
  TeammateData theTeammateData; /**< The last received team mate data (ball model, robot pose, etc.). */

  void update(TeammateData& teammateData);
  void update(TeammateDataCompressed& teammateDatacompressed) { teammateDatacompressed = TeammateDataCompressed(theTeammateData);}

  /** 
   * This method updates the teammateData with the own data in order to get better drawings.
   */
  void fillOwnData(TeammateData& teammateData);

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
  void setRolesForTeammates(TeammateData& teammateData);
  float estimatedDropInTimeToReachBall(TeammateData& teammateData, int robotNumber);
};
