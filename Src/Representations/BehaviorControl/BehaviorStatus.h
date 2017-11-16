/**
 * @file Representations/BehaviorControl/BehaviorStatus.h
 * The file declares a struct that contains data about the current behavior state.
 * @author Andreas Stolpmann
 */

#pragma once

#include "Representations/BehaviorControl/Role.h"
#include "Representations/BehaviorControl/TimeToReachBall.h"
#include "Representations/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"

/**
 * @struct BehaviorStatus
 * A struct that contains data about the current behavior state.
 */
STREAMABLE(BehaviorStatus, COMMA public BHumanMessageParticle<idBehaviorStatus>
{
  ENUM(Activity,
  {,
    unknown,
    blocking,
    duel,
    dribble,
    dribbleDuel,
    searchForBall,
    searchForBallAtRecentPosition,
    goToBall,
    takingPosition,
    kick,
    guardGoal,
    catchBall,
    standAndWait,
    passing,
    gettingUp,
    turn,
    walkNextToKeeper,
    kickoff,

    waving,
  });
  /** BHumanMessageParticle functions */
  void operator >> (BHumanMessage& m) const override;
  void operator << (const BHumanMessage& m) override;
  bool handleArbitraryMessage(InMessage& m, const std::function<unsigned(unsigned)>& toLocalTimestamp) override,

  ((Role) RoleType)(undefined) role,
  (Activity)(unknown) activity, /**< What is the robot doing in general? */
  (TimeToReachBall) timeToReachBall,
  (int)(-1) passTarget,
});

inline void BehaviorStatus::operator >> (BHumanMessage& m) const
{
  m.theBHULKsStandardMessage.currentlyPerfomingRole = Role::toBHulksRole(role);
  m.theBHumanArbitraryMessage.queue.out.bin << activity;
  m.theBHumanArbitraryMessage.queue.out.finishMessage(id());
  timeToReachBall >> m;
  m.theBHULKsStandardMessage.passTarget = static_cast<int8_t>(passTarget);

  m.theBHULKsStandardMessage.kingIsPlayingBall = Role::toBHulksRole(role) == B_HULKs::Role::King&&
      (activity == BehaviorStatus::goToBall || activity == BehaviorStatus::kick || activity == BehaviorStatus::duel);
}

inline void BehaviorStatus::operator << (const BHumanMessage& m)
{
  role = Role::fromBHulksRole(m.theBHULKsStandardMessage.currentlyPerfomingRole);
  activity = m.theBHULKsStandardMessage.kingIsPlayingBall && m.theBSPLStandardMessage.playerNum == 1 ? BehaviorStatus::goToBall : unknown;
  timeToReachBall << m;
  passTarget = static_cast<int>(m.theBHULKsStandardMessage.passTarget);
}

inline bool BehaviorStatus::handleArbitraryMessage(InMessage& m, const std::function<unsigned(unsigned)>& toLocalTimestamp)
{
  ASSERT(m.getMessageID() == id());
  m.bin >> activity;
  return true;
}
