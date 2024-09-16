/**
 * @file Agent.h
 *
 * This file defines a struct that represents an agent in the team.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Role.h"
#include "SetPlay.h"
#include "Tactic.h"
#include "Math/Eigen.h"
#include "Math/Pose2f.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE(Agent,
{,
  (int)(-1) number, /**< The player number of the agent. */
  (bool)(false) isGoalkeeper, /**< Whether the agent is designated to be the goalkeeper. */

  (unsigned)(0) lastKnownTimestamp, /**< Timestamp when the following `lastKnown...` members were calculated. */
  (Pose2f) lastKnownPose, /**< The pose of this agent that all (*) other agents know. (* except for those who didn't get the memo) */
  (Vector2f)(Vector2f::Zero()) lastKnownTarget, /**< The target of this agent that all other agents know. */
  (float)(0.f) lastKnownSpeed, /**< The speed of this agent that all other agents know. */

  (unsigned)(0) timestamp, /**< The timestamp of most of the following information. */
  (Pose2f) pose, /**< The estimated pose of this agent when it was last updated. */
  (Vector2f)(Vector2f::Zero()) currentPosition, /**< The estimated position of this agent propagated to now. */

  (Vector2f)(Vector2f::Zero()) ballPosition, /**< The estimated ball position of this agent. */
  (Vector2f)(Vector2f::Zero()) ballVelocity, /**< The estimated ball velocity of this agent. */
  (unsigned)(0) timeWhenBallLastSeen, /**< The most recent timestamp when this agent saw the ball. */
  (unsigned)(0) timeWhenBallDisappeared, /**< The most recent timestamp when the ball disappeared. */
  (bool)(false) disagreeOnBall, /**< Whether this agent disagrees with the local ball model (at the time when its last message was sent). */

  (bool)(false) isUpright, /**< Whether this agent is upright. */
  (unsigned)(0) timeWhenLastUpright, /**< The most recent timestamp when this agent was upright. */

  (Tactic::Type)(Tactic::none) proposedTactic, /**< The tactic that this agent votes for. */
  (Tactic::Type)(Tactic::none) acceptedTactic, /**< The tactic that this agent actually uses. */
  (bool)(false) proposedMirror, /**< Whether this agent wants that everything in the tactic/set play is mirrored. */
  (bool)(false) acceptedMirror, /**< Whether everything in the tactic/set play is actually mirrored. */
  (SetPlay::Type)(SetPlay::none) proposedSetPlay, /**< The set play that this agent votes for. */
  (SetPlay::Type)(SetPlay::none) acceptedSetPlay, /**< The set play that this agent actually uses. */
  (int)(-1) setPlayStep, /**< The current step within the set play. */
  (Tactic::Position::Type)(Tactic::Position::none) position, /**< The position of this agent in the tactic. */
  (Pose2f) basePose, /**< The pose from the tactic position. */
  (std::vector<Vector2f>) baseArea, /**< The region in the Voronoi diagram of the base poses in the tactic. */
  (Role::Type)(Role::none) role, /**< The role of this agent. */
  (Role::Type)(Role::none) nextRole, /**< The role of this agent in the next step (because the old role is still needed during its computation). */
});

class Agents : public std::vector<const Agent*>
{
public:
  [[nodiscard]] const Agent* byNumber(int number) const
  {
    for(const Agent* agent : *this)
      if(agent->number == number)
        return agent;
    return nullptr;
  }

  [[nodiscard]] const Agent* byPosition(Tactic::Position::Type position) const
  {
    for(const Agent* agent : *this)
      if(agent->position == position)
        return agent;
    return nullptr;
  }
};
