/**
* @file SideInformationProvider.cpp
*
* Implementation of the module SideInformationProvider.
* The SPL field is symmetric and perceiving the field elements alone does not
* allow to infer the correct playing direction.
* This module computes some information
* that might help to solve this problem in some situations.
* One part is to compute, if a robot MUST be inside its ow half given the walked distance and the current rules of the game.
* The other part (a flip detection based on observations) is currently removed.
*
* @author Tim Laue
*/

#include "SideInformationProvider.h"
#include "Debugging/Debugging.h"
#include "Platform/SystemCall.h"

MAKE_MODULE(SideInformationProvider);

SideInformationProvider::SideInformationProvider()
{
  distanceWalkedAtKnownPosition = 0.f;
  largestXPossibleAtKnownPosition = 0.f;
}

void SideInformationProvider::update(SideInformation& sideInformation)
{
  if(!theGameState.isPenaltyShootout())
  {
    if(theExtendedGameState.wasInitial() && theGameState.isReady())
    {
      distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
      // HACK: In the simulator, robots can be anywhere in their own half when switching to READY.
      largestXPossibleAtKnownPosition = SystemCall::getMode() == SystemCall::simulatedRobot ?
                                        -awayFromLineDistance :
                                        theSetupPoses.getPoseOfRobot(theGameState.playerNumber).position.x();
    }
    else if(theExtendedGameState.returnFromManualPenalty)
    {
      largestXPossibleAtKnownPosition = -theFieldDimensions.centerCircleRadius - awayFromLineDistance;
      distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
    }
    else if(theExtendedGameState.returnFromGameControllerPenalty)
    {
      distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
      largestXPossibleAtKnownPosition = theFieldDimensions.xPosReturnFromPenalty + 700;
    }
    else if(theExtendedGameState.wasSet() && theGameState.isPlaying() && !theGameState.isPenaltyKick())
    {
      distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
      largestXPossibleAtKnownPosition = (GameState::isForOwnTeam(theExtendedGameState.stateLastFrame) ? theFieldDimensions.centerCircleRadius : 0.f) - awayFromLineDistance;
    }
  }
  else if(theExtendedGameState.wasSet())
  {
    distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
    if(theGameState.isForOwnTeam())
      largestXPossibleAtKnownPosition = theFieldDimensions.xPosPenaltyStrikerStartPosition;
    else
      largestXPossibleAtKnownPosition = theFieldDimensions.xPosOwnGoalLine;
  }

  if(theStaticInitialPose.isActive && !theGameState.isPenalized()
     && (theExtendedGameState.playerStateLastFrame == GameState::penalizedPlayerPushing || theExtendedGameState.playerStateLastFrame == GameState::penalizedManual))
    largestXPossibleAtKnownPosition = theStaticInitialPose.staticPoseOnField.translation.x();

  sideInformation.largestXCoordinatePossible = largestXPossibleAtKnownPosition + distanceUncertaintyOffset +
                                              (theOdometer.distanceWalked - distanceWalkedAtKnownPosition) * distanceUncertaintyFactor;
  sideInformation.robotMustBeInOwnHalf = (sideInformation.largestXCoordinatePossible < 0 && !forceOpponentHalf) || forceOwnHalf;
  sideInformation.robotMustBeInOpponentHalf = forceOpponentHalf;
}
