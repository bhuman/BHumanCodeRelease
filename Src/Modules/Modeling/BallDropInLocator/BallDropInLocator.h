/**
 * @file BallDropInLocator.h
 *
 * This file declares a module that computes the position where the ball is put after it goes out.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Representations/Communication/TeamData.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CognitionStateChanges.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Modeling/BallDropInModel.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Module/Module.h"
#include "Tools/Streams/Enum.h"

MODULE(BallDropInLocator,
{,
  REQUIRES(BallModel),
  REQUIRES(BallSpecification),
  REQUIRES(CognitionStateChanges),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(OpponentTeamInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RobotPose),
  REQUIRES(TeamData),
  REQUIRES(TeamBallModel),
  REQUIRES(TeamPlayersModel),
  PROVIDES(BallDropInModel),
  DEFINES_PARAMETERS(
  {,
    (float)(1000.f) dropInPenaltyDistance, /**< How far the ball is put behind the robot / ball. This is dictated by the rules. */
    (float)(sqr(300.f)) ballTouchThresholdSquared, /**< The radius around robots at which balls are accepted as touched. */
    (int)(15000) rememberEventDuration, /**< The maximum duration to use events before favoring the hypothesis that the relevant touch has not been seen. */
    (int)(3000) timeUntilPenalizedRobotsAreRemoved, /**< The time after which it can be assumed that penalized robots have been removed (referee parameter). */
    (float)(50.f) safetyMargin, /**< A safety margin that is added for the decision that a ball is out. */
  }),
});

class BallDropInLocator : public BallDropInLocatorBase
{
public:
  BallDropInLocator();

private:
  ENUM(TouchedBy,
  {,
    ownTeam,
    opponentTeam,
    unknown,
  });

  struct BallTouchEvent
  {
    Vector2f positionOnField; /**< The position where the robot was when it touched the ball. */
    unsigned timestamp; /**< The time when the touch happened. */
  };

  void update(BallDropInModel& ballDropInModel) override;

  /** Updates the last touch events. */
  void updateTouchPositions();

  /**
   * Updates the ball state.
   * @brief ballDropInModel The representation that is filled.
   */
  void updateBall(BallDropInModel& ballDropInModel);

  /*
   * Updates the state according to GameController data.
   * @brief ballDropInModel The representation that is filled.
   */
  void updateGameControllerData(BallDropInModel& ballDropInModel);

  /**
   * Determines the position where the ball has been touched last.
   * @param lastTouchPosition The position where the ball has been touched last.
   * @return Whether an appropriate position has been found.
   */
  bool determineLastTouchPosition(Vector2f& lastTouchPosition) const;

  /** Draws drawings for this module. */
  void draw() const;

  Vector2f lastBallPosition = Vector2f::Zero(); /**< The team ball position of the last frame. */
  Vector2f lastBallPositionLyingInsideField = Vector2f::Zero(); /**< The team ball position of the last time it was resting and inside the field. */

  bool useOutPosition = false; /**< Whether the position where the ball went out shall be used. */
  bool ownTeamTouchedLast = false; /**< Whether the own team touched the ball last. */

  BallTouchEvent lastTouchEvents[numOfTouchedBys]; /**< A time-ordered list of recent ball touch events. */

  unsigned short lastDropInTime = static_cast<unsigned short>(-1); /**< The value of theGameInfo.dropInTime during the last frame. */
};
