/**
 * @file LibTeamProvider.h
 *
 * Contains information about the team
 */

#include "Framework/Module.h"
#include "Debugging/Debugging.h"
#include "Debugging/DebugDrawings.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Libraries/LibTeam.h"
#include "Representations/BehaviorControl/Libraries/LibPosition.h"
#include "Representations/BehaviorControl/StrategyStatus.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"

MODULE(LibTeamProvider,
{,
  USES(BehaviorStatus),
  REQUIRES(BallModel),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(LibPosition),
  REQUIRES(RobotPose),
  REQUIRES(StrategyStatus),
  REQUIRES(TeamData),
  PROVIDES(LibTeam),
});

class LibTeamProvider : public LibTeamProviderBase
{
private:
  void update(LibTeam& libTeam) override;

  /** returns player number of striker or -1 if striker was not found */
  int getStrikerPlayerNumber() const;

  /** returns the position of a team mate in field coordinates
      @player player number of team mate */
  Vector2f getTeammatePosition(int player) const;

  /** Returns the ball position estimate of the specified player */
  Vector2f getBallPosition(int player) const;

  /** Returns the number of the teammates in the own goal area */
  int numberOfNonKeeperTeammateInOwnGoalArea(const float distanceThreshold = 50.f) const;

  bool iAmClosestToBall() const;

  float getMinTeammateDistanceToBall() const;
};
