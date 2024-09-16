/**
 * @file StrategyStatus.h
 *
 * This file declares a representation of the status of the strategy.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Tools/BehaviorControl/Strategy/Role.h"
#include "Tools/BehaviorControl/Strategy/SetPlay.h"
#include "Tools/BehaviorControl/Strategy/Strategy.h"
#include "Tools/BehaviorControl/Strategy/Tactic.h"
#include "Tools/Communication/BHumanMessageParticle.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE(StrategyStatus, COMMA public BHumanCompressedMessageParticle<StrategyStatus>
{
  void draw() const,

  (Tactic::Type)(Tactic::none) proposedTactic, /**< The tactic that this player votes for. */
  (Tactic::Type)(Tactic::none) acceptedTactic, /**< The actually used tactic. */
  (bool)(false) proposedMirror, /**< Whether this player wants that everything in the tactic/set play is mirrored. */
  (bool)(false) acceptedMirror, /**< Whether everything in the tactic/set play is actually mirrored. */
  (SetPlay::Type)(SetPlay::none) proposedSetPlay, /**< The set play that this player votes for. */
  (SetPlay::Type)(SetPlay::none) acceptedSetPlay, /**< The actually used set play. */
  (int)(-1) setPlayStep, /**< The current step within the set play. */
  (Tactic::Position::Type)(Tactic::Position::none) position, /**< The current position. */
  (Role::Type)(Role::none) role, /**< The current role. */
});
