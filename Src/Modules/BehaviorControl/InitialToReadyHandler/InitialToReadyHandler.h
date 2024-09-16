/**
 * @file InitialToReadyHandler.h
 *
 * This file implements a module that handles the transition from initial state to the ready state
 * during the state standby.
 *
 * @author Kevin Dehmlow
 */

#pragma once

#include <queue>
#include "Framework/Module.h"
#include "Representations/BehaviorControl/InitialToReady.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Configuration/SetupPoses.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Perception/RefereePercept/RefereePercept.h"

MODULE(InitialToReadyHandler,
{,
  USES(GameState),
  REQUIRES(FrameInfo),
  REQUIRES(RefereePercept),
  USES(SetupPoses),
  REQUIRES(TeamData),
  PROVIDES(InitialToReady),
  LOADS_PARAMETERS(
  {,
    (int) waitForPawnSacrifice, /**< wait time in ms for pawn sacrifice to be penalized */
    (unsigned) pawnsLeftToSacrifice, /**< amount of robots to sacrifice before giving up in detection of referee gesture */
    (int) thresholdForIndependentDetections, /**< threshold in ms for different referee gesture detections */
  }),
});

class InitialToReadyHandler : public InitialToReadyHandlerBase
{
public:
  /** Constructor */
  InitialToReadyHandler();

private:
  void update(InitialToReady& theInitialToReady) override;

  /**
   * @param playerNumber robot with number to check on penalization
   * @return true, when robot with playerNumber is penalized
   */
  bool isPenalized(const int playerNumber) const;

  /**
   * @param playerNumber robot with number to check on penalization
   * @return true, when robot with playerNumber is penalized for Illegal Motion in Initial
   */
  bool isPawnPenalized(const int playerNumber) const;

  /**
   * Restart this Handler and reset the InitialToReady representation to be ready for a new iteration
   * @param theInitialToReady
   */
  void restart(InitialToReady& theInitialToReady);

  /**
   * Set the player number of the next pawn to sacrifice
   * @param pawn set on reference the player number of the next pawn or undefined, when no pawn found
   * @return true, when appropriate pawn has been set
   */
  bool setNextPawn(int& pawn);

  /**
   * @return true, when referee could be in sight of robot
   */
  bool refereeInSight();

  /**
   * Set the index of the first robot in the SetupPose.poses vector which stand on the opposite side from the goalkeeper.
   * Meaning standing on the negative y-side in field coordinates.
   * @param index reference to set the index on
   * @return true, when appropriate robot has been found and index has been set
   */
  bool getIndexOfRobotOnOtherSide(int& index);

  int observedPawn = Settings::lowestValidPlayerNumber - 1;
  int pawnsLeft = pawnsLeftToSacrifice;
};
