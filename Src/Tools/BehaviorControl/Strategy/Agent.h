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

struct Agent
{
  int number = -1; /**< The player number of the agent. */
  bool isGoalkeeper = false; /**< Whether the agent is designated to be the goalkeeper. */

  unsigned lastKnownTimestamp = 0; /**< Timestamp when the following `lastKnown...` members were calculated. */
  Pose2f lastKnownPose; /**< The pose of this agent that all (*) other agents know. (* except for those who didn't get the memo) */
  Vector2f lastKnownTarget = Vector2f::Zero(); /**< The target of this agent that all other agents know. */
  float lastKnownSpeed = 0.f; /**< The speed of this agent that all other agents know. */

  unsigned timestamp = 0; /**< The timestamp of most of the following information. */
  Pose2f pose; /**< The estimated pose of this agent when it was last updated. */
  Vector2f currentPosition = Vector2f::Zero(); /**< The estimated position of this agent propagated to now. */

  Vector2f ballPosition = Vector2f::Zero(); /**< The estimated ball position of this agent. */
  Vector2f ballVelocity = Vector2f::Zero(); /**< The estimated ball velocity of this agent. */
  unsigned timeWhenBallLastSeen = 0; /**< The most recent timestamp when this agent saw the ball. */
  unsigned timeWhenBallDisappeared = 0; /**< The most recent timestamp when the ball disappeared. */
  bool disagreeOnBall = false; /**< Whether this agent disagrees with the local ball model (at the time when its last message was sent). */

  bool isUpright = false; /**< Whether this agent is upright. */
  unsigned timeWhenLastUpright = 0; /**< The most recent timestamp when this agent was upright. */

  Tactic::Type proposedTactic = Tactic::none; /**< The tactic that this agent votes for. */
  Tactic::Type acceptedTactic = Tactic::none; /**< The tactic that this agent actually uses. */
  bool proposedMirror = false; /**< Whether this agent wants that everything in the tactic/set play is mirrored. */
  bool acceptedMirror = false; /**< Whether everything in the tactic/set play is actually mirrored. */
  SetPlay::Type proposedSetPlay = SetPlay::none; /**< The set play that this agent votes for. */
  SetPlay::Type acceptedSetPlay = SetPlay::none; /**< The set play that this agent actually uses. */
  int setPlayStep = -1; /**< The current step within the set play. */
  Tactic::Position::Type position = Tactic::Position::none; /**< The position of this agent in the tactic. */
  Pose2f basePose; /**< The pose from the tactic position. */
  std::vector<Vector2f> baseArea; /**< The region in the Voronoi diagram of the base poses in the tactic. */
  Role::Type role = Role::none; /**< The role of this agent. */
  mutable Role::Type nextRole = Role::none; /**< The role of this agent in the next step (because the old role is still needed during its computation). */
};

class Agents : public std::vector<const Agent*>
{
public:
  const Agent* byNumber(int number) const
  {
    for(const Agent* agent : *this)
      if(agent->number == number)
        return agent;
    return nullptr;
  }

  const Agent* byPosition(Tactic::Position::Type position) const
  {
    for(const Agent* agent : *this)
      if(agent->position == position)
        return agent;
    return nullptr;
  }

  const Agent* byRole(Role::Type role) const
  {
    for (const Agent* agent : *this)
      if (agent->role == role)
        return agent;
    return nullptr;
  }
};
