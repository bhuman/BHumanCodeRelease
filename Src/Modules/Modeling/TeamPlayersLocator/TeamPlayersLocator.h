/**
 * @file TeamPlayersLocator.h
 *
 *
 * @author Florian
 */

#pragma once

#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Tools/Modeling/Obstacle.h"
#include "Tools/Module/Module.h"
#include <map>
#include <vector>

MODULE(TeamPlayersLocator,
{,
  REQUIRES(RobotPose),
  REQUIRES(RobotInfo),
  REQUIRES(BallModel),
  REQUIRES(TeamData),
  REQUIRES(FieldDimensions),
  REQUIRES(FallDownState),
  REQUIRES(GameInfo),
  REQUIRES(GroundContactState),
  REQUIRES(ObstacleModel),
  REQUIRES(FrameInfo),
  PROVIDES(TeamPlayersModel),
  LOADS_PARAMETERS(
  {,
    (float) squaredDistanceGoalPostThreshold,
    (float) squaredDistanceThreshold,
    (float) selfDetectionOnlyRadius,
    (float) teammatePoseDeviation,
    (int) obstacleAgeThreshold,
  }),
});

/**
 * @class TeamPlayersLocator
 * A combined world model
 */
class TeamPlayersLocator : public TeamPlayersLocatorBase
{
public:
  TeamPlayersLocator();

private:
  /**
   * Provides the world representation of obstacles
   */
  void update(TeamPlayersModel& teamPlayersModel) override;

  bool isInsideOwnDetectionArea(const Vector2f& position, int robotNumber, float& distance, int lastSeen) const;
  bool isInsideOwnDetectionArea(const Vector2f& position, int robotNumber, int lastSeen) const;
  bool collideOtherDetectionArea(const Vector2f& position, int robotNumber, std::map<int, RobotPose>& ownTeam,
                                 const float distance) const;
  bool isGoalPost(const Vector2f& position) const;
  bool isTeammate(const Vector2f& position, const float radius, int ignoreRobotNumber) const;
  bool isTeammate(const Vector2f& position, const float radius) const;
  void setType(Obstacle& one, const Obstacle& other) const;

  std::map<int, RobotPose> ownTeam;
  std::vector<Obstacle, Eigen::aligned_allocator<Obstacle>> goalPosts;
};
