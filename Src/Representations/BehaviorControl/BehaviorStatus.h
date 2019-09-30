/**
 * @file Representations/BehaviorControl/BehaviorStatus.h
 * The file declares a struct that contains data about the current behavior state.
 * @author Andreas Stolpmann
 */

#pragma once

#include "Representations/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"
#include "Representations/Communication/GameInfo.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"

/**
 * @struct BehaviorStatus
 * A struct that contains data about the current behavior state.
 */
STREAMABLE(BehaviorStatus, COMMA public BHumanMessageParticle<idBehaviorStatus>
{
  ENUM(Activity,
  {,
    unknown,

    fallen,
    finished,
    initial,
    set,

    codeReleaseKickAtGoal,
    codeReleasePositionForKickOff,
  });

  /**
   * Checks whether the activity indicates that the robot is going to the ball.
   * @return Whether the activity indicates that the robot is going to the ball.
   */
  bool goesToBall() const
  {
    return activity == codeReleaseKickAtGoal;
  }

  /** BHumanMessageParticle functions */
  void operator>>(BHumanMessage& m) const override;
  void operator<<(const BHumanMessage& m) override,

  (Activity)(unknown) activity, /**< What is the robot doing in general? */
  (int)(-1) passTarget,
  (Vector2f)(Vector2f::Zero()) walkingTo,
  (Vector2f)(Vector2f::Zero()) shootingTo,
});

inline void BehaviorStatus::operator>>(BHumanMessage& m) const
{
  m.theMixedTeamHeader.wantsToPlayBall = goesToBall();
  m.theMixedTeamHeader.someBehaviorNumber = -1; // TODO
  m.theBHumanStandardMessage.activity = static_cast<unsigned char>(activity);
  m.theBHumanStandardMessage.passTarget = passTarget;
  m.theBHumanStandardMessage.walkingTo = walkingTo;
  m.theBHumanStandardMessage.shootingTo = shootingTo;
}

inline void BehaviorStatus::operator<<(const BHumanMessage& m)
{
  if(!m.hasBHumanParts)
  {
    activity = BehaviorStatus::unknown;
    const GameInfo& theGameInfo = static_cast<GameInfo&>(Blackboard::getInstance()["GameInfo"]);
    if(theGameInfo.state == STATE_INITIAL)
      activity = BehaviorStatus::initial;
    else if(theGameInfo.state == STATE_SET)
      activity = BehaviorStatus::set;
    else if(theGameInfo.state == STATE_FINISHED)
      activity = BehaviorStatus::finished;
    walkingTo = Vector2f::Zero();
    shootingTo = Vector2f::Zero();
    return;
  }
  activity = static_cast<Activity>(m.theBHumanStandardMessage.activity);
  passTarget = m.theBHumanStandardMessage.passTarget;
  walkingTo = m.theBHumanStandardMessage.walkingTo;
  shootingTo = m.theBHumanStandardMessage.shootingTo;
}
