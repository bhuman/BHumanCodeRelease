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
  lastPenalty(PENALTY_NONE),
  lastGameState(STATE_INITIAL),
  distanceWalkedAtKnownPosition(0),
  largestXPossibleAtKnownPosition(0),
  manuallyPlaced(false),
  timeWhenPenalized(0),
  gameStateWhenPenalized(STATE_INITIAL)
{}

void OwnSideModelProvider::update(OwnSideModel& ownSideModel)
{
  if(theGameInfo.state == STATE_SET && !theGroundContactState.contact)
    manuallyPlaced = true;

  if(lastPenalty == PENALTY_NONE && theRobotInfo.penalty != PENALTY_NONE)
  {
    timeWhenPenalized = theFrameInfo.time;
    gameStateWhenPenalized = theGameInfo.state;
  }

  ownSideModel.returnFromGameControllerPenalty = false;
  ownSideModel.returnFromManualPenalty = false;

  if(theGameInfo.secondaryState != STATE2_PENALTYSHOOT)
  {
    if(lastGameState == STATE_INITIAL && theGameInfo.state != STATE_INITIAL)
    {
      distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
      largestXPossibleAtKnownPosition = largestXInInitial;
    }
    else if(lastPenalty == PENALTY_MANUAL && theRobotInfo.penalty == PENALTY_NONE)
    {
      distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
      largestXPossibleAtKnownPosition = -theFieldDimensions.centerCircleRadius - awayFromLineDistance;
      ownSideModel.returnFromManualPenalty = true;
    }
    else if(lastPenalty != PENALTY_NONE && lastPenalty != PENALTY_SPL_ILLEGAL_MOTION_IN_SET &&
            theRobotInfo.penalty == PENALTY_NONE &&
            (theFrameInfo.getTimeSince(timeWhenPenalized) > minPenaltyTime || theGameInfo.state != gameStateWhenPenalized))
    {
      distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
      largestXPossibleAtKnownPosition = theFieldDimensions.xPosOwnPenaltyMark;
      ownSideModel.returnFromGameControllerPenalty = true;
    }
    else if(lastGameState == STATE_SET && theGameInfo.state == STATE_PLAYING)
    {
      distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
      if(manuallyPlaced)
      {
        if(Global::getSettings().isGoalkeeper)
          largestXPossibleAtKnownPosition = theFieldDimensions.xPosOwnGroundline;
        else if(theGameInfo.kickOffTeam == theOwnTeamInfo.teamNumber)
          largestXPossibleAtKnownPosition = -theFieldDimensions.centerCircleRadius - awayFromLineDistance;
        else
          largestXPossibleAtKnownPosition = theFieldDimensions.xPosOwnPenaltyArea + awayFromLineDistance;
      }
      else
        largestXPossibleAtKnownPosition = -awayFromLineDistance;
    }
  }
  else if(lastGameState == STATE_SET)
  {
    distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
    if(theGameInfo.kickOffTeam == theOwnTeamInfo.teamNumber)
      largestXPossibleAtKnownPosition = theFieldDimensions.xPosPenaltyStrikerStartPosition;
    else
      largestXPossibleAtKnownPosition = theFieldDimensions.xPosOwnGroundline;
  }
  ownSideModel.largestXPossible = largestXPossibleAtKnownPosition + distanceUncertaintyOffset +
                                  (theOdometer.distanceWalked - distanceWalkedAtKnownPosition) * distanceUncertaintyFactor;
  ownSideModel.stillInOwnSide = ownSideModel.largestXPossible < 0;
  lastPenalty = theRobotInfo.penalty;
  lastGameState = theGameInfo.state;

  if(theGameInfo.state != STATE_SET)
    manuallyPlaced = false;
}

MAKE_MODULE(OwnSideModelProvider, modeling)
