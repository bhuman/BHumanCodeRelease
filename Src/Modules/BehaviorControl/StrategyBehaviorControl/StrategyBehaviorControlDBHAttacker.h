/**
 * @file StrategyBehaviorControlDBHAttacker.h
 *
 * This file declares a module that determines the strategy of the team.
 *
 * @author Yannik Meinken
 * @author Sina Schreiber
 * @author Michelle Gusev
 */

#pragma once

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/SkillRequest.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/RobotPose.h"
#include "Framework/Module.h"

MODULE(StrategyBehaviorControlDBHAttacker,
{,
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(RobotPose),
  REQUIRES(FieldBall),
  REQUIRES(PassEvaluation),
  REQUIRES(TeamData),
  REQUIRES(ExpectedGoals),
  PROVIDES(SkillRequest),
  DEFINES_PARAMETERS(
  {,
    (unsigned)(000) timeWaiting,/**< time we want to wait before start with the behavior*/
    (float)(1000) maxDistanceToBall,/**< maximum distance from a robot to the ball */
    (Vector2f)(2800.0f, 1500.f) rightShootPosition,
    (Vector2f)(2800.0f, -1500.f) leftShootPosition,
    (float)(0.1) passHysteresis,
    (float)(100.f) delta,
    (float)(100.f) step,
  }
  ),
});

class StrategyBehaviorControlDBHAttacker : public StrategyBehaviorControlDBHAttackerBase
{
  /**
   * Updates the skill request.
   * @param skillRequest The provided skill request.
   */
  void update(SkillRequest& skillRequest) override;

  SkillRequest behave();

  bool secondRobot = false; // bool if the second robot plays the ball
  bool thirdRobot = false; // bool if the third robot plays the ball

  int passTarget = 2;

  float rating(const Vector2f pos) const;

  void gradientAscent(Vector2f& pos, const int numIterations) const;

  Vector2f pos = theRobotPose.translation;

  Vector2f upperFieldCorner = Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosLeftSideline);
  Vector2f gridCornerLower; /**< Corner point with smallest coordinates for the heatmap to be to be drawn */
  Vector2f gridCornerUpper; /**< Corner point with largest coordinates for the heatmap to be drawn */
  Vector2i cellsNumber; /**< Number of grid cells on the corresponding axis, or resolution for the heatmap */
  int totalCellsNumber;
  std::vector<ColorRGBA> cellColors;

  void drawRating();
  SkillRequest smashOrPass(Teammate other);

  bool shoot = true;
};
