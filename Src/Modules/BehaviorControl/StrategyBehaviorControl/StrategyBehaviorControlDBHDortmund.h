/**
 * @file StrategyBehaviorControlDBHDefender.h
 *
 * This file declares a module that determines the strategy of the team.
 *
 * @author Yannik Meinken
 */

#pragma once

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/SkillRequest.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/RobotPose.h"
#include "Framework/Module.h"
#include "Math/Angle.h"

MODULE(StrategyBehaviorControlDBHDortmund,
{,
  REQUIRES(FieldDimensions),
  REQUIRES(GameState),
  REQUIRES(RobotPose),
  REQUIRES(FieldBall),
  PROVIDES(SkillRequest),
  DEFINES_PARAMETERS(
  {,
  }),
});

class StrategyBehaviorControlDBHDortmund : public StrategyBehaviorControlDBHDortmundBase
{
  /**
   * Updates the skill request.
   * @param skillRequest The provided skill request.
   */
  void update(SkillRequest& skillRequest) override;

  SkillRequest behave();

  SkillRequest kickBall();

  Angle angleToBall = 0.f;
};
