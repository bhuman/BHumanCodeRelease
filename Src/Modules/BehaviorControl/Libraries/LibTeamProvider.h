/**
 * @file LibTeamProvider.h
 *
 * Contains information about the team
 */

#include "Tools/Module/Module.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Libraries/LibTeam.h"
#include "Representations/BehaviorControl/Libraries/LibPosition.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/ExtendedGameInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"

MODULE(LibTeamProvider,
{,
  USES(BehaviorStatus),
  REQUIRES(BallModel),
  REQUIRES(ExtendedGameInfo),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(LibPosition),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  REQUIRES(TeamBehaviorStatus),
  REQUIRES(TeamData),
  PROVIDES(LibTeam),
});

class LibTeamProvider : public LibTeamProviderBase
{
private:
  void update(LibTeam& libTeam) override;

  int getKeeperPlayerNumber() const;

  /** returns player number of striker or -1 if striker was not found */
  int getStrikerPlayerNumber() const;

  /** returns the pose of the keeper */
  Pose2f getKeeperPose() const;

  /** returns the pose of the striker */
  Pose2f getStrikerPose() const;

  /** returns the pose of a team mate in field coordinates
      @player player number of team mate */
  Pose2f getTeammatePose(int player) const;

  /** Returns the activity of the specified player */
  BehaviorStatus::Activity getActivity(int player) const;

  /** Returns the status of the specified player */
  Teammate::Status getStatus(int player) const;

  /** Returns the ball position estimate of the specified player */
  Vector2f getBallPosition(int player) const;

  /** Returns the time to reach ball of the specified player. */
  const TimeToReachBall* getTimeToReachBall(int player) const;

  /** Returns the number of the teammates in the own goal area */
  int numberOfNonKeeperTeammateInOwnGoalArea(const float distanceThreshold = 50.f) const;

  bool iAmClosestToBall() const;

  float getMinTeammateDistanceToBall() const;
};
