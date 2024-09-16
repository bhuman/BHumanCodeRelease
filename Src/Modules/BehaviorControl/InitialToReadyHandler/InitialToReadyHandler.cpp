/**
 * @file InitialToReadyHandler.cpp
 *
 * This file implements a module that handles the transition from initial state to the ready state
 * during the state standby.
 *
 * @author Kevin Dehmlow
 *
 */

#include <iterator>
#include "InitialToReadyHandler.h"
#include "Platform/SystemCall.h"

MAKE_MODULE(InitialToReadyHandler);

InitialToReadyHandler::InitialToReadyHandler() = default;

void InitialToReadyHandler::update(InitialToReady& theInitialToReady)
{
  // this module is only useful during initial state and when robot is active
  if(theGameState.state != GameState::standby ||
     theGameState.playerState != GameState::PlayerState::active)
  {
    // after transition restart Handler for next iteration
    restart(theInitialToReady);
    return;
  }

  theInitialToReady.gestureDetected = false;
  theInitialToReady.detectedBy = 0;

  bool enableSound = false;
  DEBUG_RESPONSE("module:InitialToReadyHandler:enableSoundResponse")
    enableSound = !enableSound;

  const auto threshold = static_cast<unsigned>(thresholdForIndependentDetections);
  const auto& timeWhenStateStarted = theGameState.timeWhenStateStarted;
  auto transitionToReady = [&theInitialToReady, &timeWhenStateStarted, &threshold](const Teammate& mate) {
    return mate.theInitialToReady.gestureDetected &&
                                                                        // was mates detection faster (within threshold)
        (theInitialToReady.timestamp > mate.theInitialToReady.timestamp ? theInitialToReady.timestamp - mate.theInitialToReady.timestamp < threshold
                                                                        // or is mates detection newer than last detection
                                                                        : mate.theInitialToReady.timestamp - theInitialToReady.timestamp > threshold &&
                                                                          // when detection was shortly before a timeout
                                                                          mate.theInitialToReady.timestamp >= timeWhenStateStarted);
  };

  if(pawnsLeft > 0 && theInitialToReady.state == InitialToReady::State::observing)
  {
    int detectByMate = Settings::lowestValidPlayerNumber - 1;
    bool forceTransition = false;
    DEBUG_RESPONSE_ONCE("module:InitialToReadyHandler:initiateTransition")
      forceTransition = true;
    if((theRefereePercept.gesture == RefereePercept::Gesture::initialToReady && refereeInSight()) ||
      forceTransition)
    {
      // save detection only when threshold to last detection is met
      if(theFrameInfo.getTimeSince(theInitialToReady.timestamp) > thresholdForIndependentDetections)
      {
        theInitialToReady.gestureDetected = true;
        theInitialToReady.detectedBy = detectByMate = theGameState.playerNumber;
        theInitialToReady.timestamp = theFrameInfo.time;
        if(enableSound)
          SystemCall::playSound("sirene.wav");
      }
    }

    auto firstMateTransitioned = std::find_if(theTeamData.teammates.begin(), theTeamData.teammates.end(), transitionToReady);
    if(firstMateTransitioned != theTeamData.teammates.end())
    {
      // when we are here, following is implied
      // - this robot detected referee gesture, but a mate was faster in detection and sending this information around
      // - this robot not detected referee gesture, but a mate detected it and the threshold is met
      theInitialToReady.gestureDetected = false;
      theInitialToReady.timestamp = firstMateTransitioned->theInitialToReady.timestamp;
      detectByMate = firstMateTransitioned->theInitialToReady.detectedBy;
      if(enableSound)
        SystemCall::playSound("theValidityDuck.wav");
    }

    // some mate detect the referee gesture
    if(detectByMate >= Settings::lowestValidPlayerNumber)
    {
      int pawn;
      if(setNextPawn(pawn))
      {
        pawnsLeft--;
        if(theGameState.playerNumber == pawn)
        {
          theInitialToReady.state = InitialToReady::State::transition;
          theInitialToReady.isPawn = true;
        }
        else
        {
          theInitialToReady.state = InitialToReady::State::waiting;
          observedPawn = pawn;
        }
      }
      else
        pawnsLeft = 0;
    }
  }

  if(theInitialToReady.state == InitialToReady::State::waiting)
  {
    if(isPawnPenalized(observedPawn))
      theInitialToReady.state = InitialToReady::State::observing;
    else if(theFrameInfo.getTimeSince(theInitialToReady.timestamp) > waitForPawnSacrifice)
    {
      theInitialToReady.state = InitialToReady::State::transition;
      if(enableSound)
        SystemCall::playSound("wyld.wav");
    }
  }
}

bool InitialToReadyHandler::getIndexOfRobotOnOtherSide(int& index)
{
  int prevPlayerNumber = Settings::lowestValidPlayerNumber - 1;
  for(int i = 0; i < static_cast<int>(theSetupPoses.poses.size()); i++)
  {
    int playerNumber = theSetupPoses.poses[i].playerNumber;
    // corner case: when in SetupPoses gaps has been filled because of missing robots,
    // but no substitute robots has been added to the lineup, so in the lineup the last positions are empty
    if(prevPlayerNumber >= playerNumber)
      return false;
    else if(theSetupPoses.poses[i].position.y() < 0)
    {
      index = i;
      return true;
    }
    else
      prevPlayerNumber = playerNumber;
  }
  return false;
}

bool InitialToReadyHandler::refereeInSight()
{
  int index;
  if(getIndexOfRobotOnOtherSide(index))
    return theGameState.leftHandTeam ? theGameState.playerNumber >= theSetupPoses.poses[index].playerNumber
                                     : theGameState.playerNumber < theSetupPoses.poses[index].playerNumber;
  else
    return false;
}

bool InitialToReadyHandler::setNextPawn(int& pawn)
{
  pawn = Settings::lowestValidPlayerNumber - 1;

  int firstOnOtherSideIndex;
  if(getIndexOfRobotOnOtherSide(firstOnOtherSideIndex))
  {
    auto nextPawn = [this](const int index, int& pawn)
    {
      const int playerNumber = theSetupPoses.poses[index].playerNumber;
      if(playerNumber < Settings::lowestValidPlayerNumber || playerNumber > Settings::highestValidPlayerNumber ||
        theGameState.ownTeam.isGoalkeeper(playerNumber) || isPenalized(playerNumber))
        return false;
      else
      {
        pawn = playerNumber;
        return true;
      }
    };

    if(theGameState.leftHandTeam)
      for(int i = firstOnOtherSideIndex - 2; i >= 0 && !nextPawn(i, pawn); i--);
    else
    {
      int prevPlayerNumber = theSetupPoses.poses[firstOnOtherSideIndex].playerNumber;
      for(int i = firstOnOtherSideIndex + 1; i < static_cast<int>(theSetupPoses.poses.size()); i++)
      {
        // corner case: when in SetupPoses gaps has been filled because of missing robots,
        // but no substitute robots has been added to the lineup, so in the lineup the last positions are empty
        if(prevPlayerNumber >= theSetupPoses.poses[i].playerNumber || nextPawn(i, pawn))
          break;
        else
          prevPlayerNumber = theSetupPoses.poses[i].playerNumber;
      }
    }
  }

  return pawn >= Settings::lowestValidPlayerNumber;
}

bool InitialToReadyHandler::isPawnPenalized(const int playerNumber) const
{
  return playerNumber < Settings::lowestValidPlayerNumber || playerNumber > Settings::highestValidPlayerNumber ||
            theGameState.ownTeam.playerStates[playerNumber - Settings::lowestValidPlayerNumber] == GameState::penalizedIllegalMotionInStandby;
}

bool InitialToReadyHandler::isPenalized(const int playerNumber) const
{
  return playerNumber < Settings::lowestValidPlayerNumber || playerNumber > Settings::highestValidPlayerNumber ||
         GameState::isPenalized(theGameState.ownTeam.playerStates[playerNumber - Settings::lowestValidPlayerNumber]);
}

void InitialToReadyHandler::restart(InitialToReady& theInitialToReady)
{
  theInitialToReady.state = InitialToReady::State::observing;
  theInitialToReady.timestamp = 0;
  theInitialToReady.gestureDetected = false;
  theInitialToReady.detectedBy = 0;
  theInitialToReady.isPawn = false;

  pawnsLeft = pawnsLeftToSacrifice;
  observedPawn = Settings::lowestValidPlayerNumber - 1;
}
