/**
 * @file OwnSideModelProvider.h
 * The file implements a module that determines whether the robot cannot have left its own
 * side since the last kick-off.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "OwnSideModelProvider.h"
#include "Tools/Settings.h"

OwnSideModelProvider::OwnSideModelProvider() :
  distanceWalkedAtKnownPosition(0),
  largestXPossibleAtKnownPosition(0),
  manuallyPlaced(false),
  timeWhenPenalized(0),
  gameStateWhenPenalized(STATE_INITIAL)
{}

void OwnSideModelProvider::update(OwnSideModel& ownSideModel)
{
  if(theGameInfo.state == STATE_SET && theFallDownState.state == FallDownState::pickedUp)
    manuallyPlaced = true;

  if(theCognitionStateChanges.lastPenalty == PENALTY_NONE && theRobotInfo.penalty != PENALTY_NONE)
  {
    timeWhenPenalized = theFrameInfo.time;
    gameStateWhenPenalized = theGameInfo.state;
  }

  ownSideModel.returnFromGameControllerPenalty = false;
  ownSideModel.returnFromManualPenalty = false;

  if(theGameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT)
  {
    if(theCognitionStateChanges.lastGameState == STATE_INITIAL && theGameInfo.state != STATE_INITIAL)
    {
      distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
      largestXPossibleAtKnownPosition = largestXInInitial;
    }
    else if(theCognitionStateChanges.lastPenalty == PENALTY_MANUAL && theRobotInfo.penalty == PENALTY_NONE)
    {
      distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
      largestXPossibleAtKnownPosition = -theFieldDimensions.centerCircleRadius - awayFromLineDistance;
      ownSideModel.returnFromManualPenalty = true;
    }
    else if(theCognitionStateChanges.lastPenalty != PENALTY_NONE && theCognitionStateChanges.lastPenalty != PENALTY_SPL_ILLEGAL_MOTION_IN_SET &&
            theRobotInfo.penalty == PENALTY_NONE &&
            (theFrameInfo.getTimeSince(timeWhenPenalized) > minPenaltyTime || theGameInfo.state != gameStateWhenPenalized))
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
        if(theRole.isGoalkeeper())
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
  ownSideModel.largestXPossible = largestXPossibleAtKnownPosition + distanceUncertaintyOffset +
                                  (theOdometer.distanceWalked - distanceWalkedAtKnownPosition) * distanceUncertaintyFactor;
  ownSideModel.stillInOwnSide = ownSideModel.largestXPossible < 0;

  if(theGameInfo.state != STATE_SET)
    manuallyPlaced = false;

  ownSideModel.manuallyPlaced = manuallyPlaced;
}

MAKE_MODULE(OwnSideModelProvider, modeling)
