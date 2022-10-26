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

MAKE_MODULE(SideInformationProvider, modeling);

SideInformationProvider::SideInformationProvider()
{
  distanceWalkedAtKnownPosition = 0.f;
  largestXPossibleAtKnownPosition = 0.f;
}

void SideInformationProvider::update(SideInformation& sideInformation)
{
  computeBasicOwnSideInformation(sideInformation);
  computeMirrorFlag(sideInformation);

  // Release the annoying chicken!
  if(sideInformation.mirror)
    SystemCall::playSound("theFlippingChicken.wav");
}

void SideInformationProvider::computeMirrorFlag(SideInformation& sideInformation)
{
  // Function is currently empty as the previous approach does not work
  // anymore from 2022 on.

  // Put new code here!
  // Please keep in mind that the basic idea, on which the following computations in the
  // SelfLocator (that are still in place) rely, is to set the mirror flag once and not for
  // a number of consecutive frames. When the flag is set, the SelfLocator will mirror the pose
  // estimate immediately. Thus, the internal state of this module has to be reset after setting the
  // mirror flag to avoid some fliping flipping back and forth.
  // If you have questions, ask Tim.

  // Check this first, before doing anything else:
  if(sideInformation.mirror) {
    // Reset your stuff.
  }

  // Set the flag:
  sideInformation.mirror = false;
  DEBUG_RESPONSE_ONCE("module:SideInformationProvider:forceMirror")
    sideInformation.mirror = true;
}

void SideInformationProvider::computeBasicOwnSideInformation(SideInformation& sideInformation)
{
  if(!theGameState.isPenaltyShootout())
  {
    if(theExtendedGameState.wasInitial() && theGameState.isReady())
    {
      distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
      largestXPossibleAtKnownPosition = theSetupPoses.getPoseOfRobot(theGameState.ownTeam.getSubstitutedPlayerNumber(theGameState.playerNumber)).position.x();
    }
    else if(theExtendedGameState.returnFromManualPenalty)
    {
      largestXPossibleAtKnownPosition = -theFieldDimensions.centerCircleRadius - awayFromLineDistance;
      distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
    }
    else if(theExtendedGameState.returnFromGameControllerPenalty)
    {
      distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
      largestXPossibleAtKnownPosition = theFieldDimensions.xPosOwnPenaltyMark + 700;
    }
    else if(theExtendedGameState.wasSet() && theGameState.isPlaying() && !theGameState.isPenaltyKick())
    {
      distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
      if(theExtendedGameState.manuallyPlaced)
      {
        if(theGameState.isGoalkeeper())
          largestXPossibleAtKnownPosition = theFieldDimensions.xPosOwnGroundLine + 52.f;
        else if(theGameState.isForOwnTeam())
          largestXPossibleAtKnownPosition = -theFieldDimensions.centerCircleRadius - awayFromLineDistance;
        else
          largestXPossibleAtKnownPosition = theFieldDimensions.xPosOwnPenaltyArea + 127.f;
      }
      else
        largestXPossibleAtKnownPosition = (GameState::isForOwnTeam(theExtendedGameState.stateLastFrame) ? theFieldDimensions.centerCircleRadius : 0.f) - awayFromLineDistance;
    }
  }
  else if(theExtendedGameState.wasSet())
  {
    distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
    if(theGameState.isForOwnTeam())
      largestXPossibleAtKnownPosition = theFieldDimensions.xPosPenaltyStrikerStartPosition;
    else
      largestXPossibleAtKnownPosition = theFieldDimensions.xPosOwnGroundLine;
  }

  if(theStaticInitialPose.isActive && !theGameState.isPenalized()
     && (theExtendedGameState.playerStateLastFrame == GameState::penalizedPlayerPushing || theExtendedGameState.playerStateLastFrame == GameState::penalizedManual))
    largestXPossibleAtKnownPosition = theStaticInitialPose.staticPoseOnField.translation.x();

  sideInformation.largestXCoordinatePossible = largestXPossibleAtKnownPosition + distanceUncertaintyOffset +
                                              (theOdometer.distanceWalked - distanceWalkedAtKnownPosition) * distanceUncertaintyFactor;
  sideInformation.robotMustBeInOwnHalf = sideInformation.largestXCoordinatePossible < 0 || forceOwnHalf;
  sideInformation.robotMustBeInOpponentHalf = forceOpponentHalf;
}
