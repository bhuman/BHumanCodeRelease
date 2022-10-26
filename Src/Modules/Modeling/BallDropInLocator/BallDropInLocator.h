/**
 * @file BallDropInLocator.h
 *
 * This file declares a module that computes the position where the ball is put after it goes out.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/BallDropInModel.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeammatesBallModel.h"
#include "Math/Eigen.h"
#include "Framework/Module.h"
#include "Streaming/Enum.h"

MODULE(BallDropInLocator,
{,
  REQUIRES(BallModel),
  REQUIRES(BallSpecification),
  REQUIRES(ExtendedGameState),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(RobotPose),
  REQUIRES(TeammatesBallModel),
  PROVIDES(BallDropInModel),
  DEFINES_PARAMETERS(
  {,
    (float)(50.f) safetyMargin, /**< A safety margin that is added for the decision that a ball is out. */
    (int)(4000) useOutPositionTimeout, /**< If the GameController has not started a set play within this time after observing the ball leaving the field, \c useOutPosition is reset. */
  }),
});

class BallDropInLocator : public BallDropInLocatorBase
{
  void update(BallDropInModel& ballDropInModel) override;

  /**
   * Updates the ball state.
   * @param ballDropInModel The representation that is filled.
   */
  void updateBall(BallDropInModel& ballDropInModel);

  /*
   * Updates the state according to GameController data.
   * @param ballDropInModel The representation that is filled.
   */
  void updateGameControllerData(BallDropInModel& ballDropInModel);

  /** Draws drawings for this module. */
  void draw() const;

  Vector2f predictedOutPosition = Vector2f::Zero(); /**< The position where the ball will go out if it moves on a straight line. */

  bool useOutPosition = false; /**< Whether the position where the ball went out shall be used. */
  bool ballWasOnField = true; /**< Whether the ball was inside the field during the last frame. */
};
