/**
 * @file OwnSideModelProvider.cpp
 * The file implements a module that determines whether the robot cannot have left its own
 * side since the last kick-off.
 *
 * @author Thomas Röfer
 */

#include "OwnSideModelProvider.h"
#include "Tools/Settings.h"

OwnSideModelProvider::OwnSideModelProvider() :
  distanceWalkedAtKnownPosition(0),
  largestXPossibleAtKnownPosition(0),
  manuallyPlaced(false),
  timeWhenPenalized(0),
  timeWhenPenaltyEnded(0),
  receivedGameControllerPacket(false)
{}

void OwnSideModelProvider::update(OwnSideModel& ownSideModel)
{
  if(theCognitionStateChanges.lastPenalty != PENALTY_NONE && theRobotInfo.penalty == PENALTY_NONE)
    timeWhenPenaltyEnded = theFrameInfo.time;
  if(theGameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT && theGameInfo.state == STATE_SET && theRobotInfo.penalty == PENALTY_NONE &&
     theFrameInfo.getTimeSince(timeWhenPenaltyEnded) > 5000 && theFallDownState.state == FallDownState::pickedUp)
    manuallyPlaced = true;

  if(theCognitionStateChanges.lastPenalty == PENALTY_NONE && theRobotInfo.penalty != PENALTY_NONE)
  {
    // If this is the first GameController packet, it might be that the software
    // just started and the robot is actually penalized for a longer time.
    if(!receivedGameControllerPacket && theRobotInfo.penalty != PENALTY_MANUAL)
      timeWhenPenalized = 0;
    else
      timeWhenPenalized = theFrameInfo.time;
  }
  receivedGameControllerPacket = theFrameInfo.getTimeSince(theGameInfo.timeLastPacketReceived) < 45000 - minPenaltyTime;

  ownSideModel.returnFromGameControllerPenalty = false;
  ownSideModel.returnFromManualPenalty = false;

  if(theGameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT)
  {
    if(theCognitionStateChanges.lastGameState == STATE_INITIAL && theGameInfo.state == STATE_READY)
    {
      distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
      largestXPossibleAtKnownPosition = largestXInInitial;
    }
    else if(theCognitionStateChanges.lastPenalty == PENALTY_MANUAL && theRobotInfo.penalty == PENALTY_NONE)
    {
      largestXPossibleAtKnownPosition = -theFieldDimensions.centerCircleRadius - awayFromLineDistance;
      distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
      ownSideModel.returnFromManualPenalty = true;
    }
    else if(theCognitionStateChanges.lastPenalty != PENALTY_NONE && theCognitionStateChanges.lastPenalty != PENALTY_SPL_ILLEGAL_MOTION_IN_SET &&
            theRobotInfo.penalty == PENALTY_NONE && theFrameInfo.getTimeSince(timeWhenPenalized) > (theCognitionStateChanges.lastPenalty == PENALTY_SPL_ILLEGAL_POSITIONING ? minPenaltyTimeIP : minPenaltyTime))
    {
      distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
      largestXPossibleAtKnownPosition = theFieldDimensions.xPosOwnPenaltyMark + 700;
      ownSideModel.returnFromGameControllerPenalty = true;
    }
    else if(theCognitionStateChanges.lastGameState == STATE_SET && theGameInfo.state == STATE_PLAYING)
    {
      distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
      if(manuallyPlaced)
      {
        if(theTeamBehaviorStatus.role.isGoalkeeper)
          largestXPossibleAtKnownPosition = theFieldDimensions.xPosOwnGroundline;
        else if(theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber)
          largestXPossibleAtKnownPosition = -theFieldDimensions.centerCircleRadius - awayFromLineDistance;
        else
          largestXPossibleAtKnownPosition = theFieldDimensions.xPosOwnPenaltyArea + awayFromLineDistance;
      }
      else
        largestXPossibleAtKnownPosition = -awayFromLineDistance;
    }
  }
  else if(theCognitionStateChanges.lastGameState == STATE_SET)
  {
    distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
    if(theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber)
      largestXPossibleAtKnownPosition = theFieldDimensions.xPosPenaltyStrikerStartPosition;
    else
      largestXPossibleAtKnownPosition = theFieldDimensions.xPosOwnGroundline;
  }

  if(theStaticInitialPose.isActive && theRobotInfo.penalty == PENALTY_NONE
     && (theCognitionStateChanges.lastPenalty == PENALTY_SPL_PLAYER_PUSHING || theCognitionStateChanges.lastPenalty == PENALTY_MANUAL))
    largestXPossibleAtKnownPosition = theStaticInitialPose.staticPoseOnField.translation.x();

  ownSideModel.largestXPossible = largestXPossibleAtKnownPosition + distanceUncertaintyOffset +
                                  (theOdometer.distanceWalked - distanceWalkedAtKnownPosition) * distanceUncertaintyFactor;
  ownSideModel.stillInOwnSide = ownSideModel.largestXPossible < 0;

  if(theGameInfo.state != STATE_SET)
    manuallyPlaced = false;

  ownSideModel.manuallyPlaced = manuallyPlaced;
}

MAKE_MODULE(OwnSideModelProvider, modeling)
