/**
 * @file ScoreDirectionsProvider.cpp
 * The file implements a module that determines sectors of directions in which the ball can be
 * kicked and hit the goal. This is based on the obstacle model. The problem is solved using
 * a sweep line method. All obstacles are described as angular sectors surrounding the ball.
 * The two edges of each sector, i.e. the direction in which an obstacle begins and the
 * direction in which it ends, are added to an array that is then sorted by the angles. The
 * goal posts are added as well. After that, the array is traversed while updating an
 * obstacle counter. Whenever a starting edge is encountered, the counter is increased, and
 * when an ending edge is found, the counter is lowered. Whenever the counter is zero, a
 * free sector was found.
 * @author Thomas Röfer
 */

#include "ScoreDirectionsProvider.h"
#include <algorithm>

#define DRAWING_EDGE Vector2<>(12000.f, 0)

void ScoreDirectionsProvider::update(ScoreDirections& scoreDirections)
{
  DECLARE_DEBUG_DRAWING("module:ScoreDirectionsProvider:sectors", "drawingOnField");

  const Vector2<> ballPosition = theRobotPose * theBallModel.estimate.position;
  const Vector2<> leftGoalPostOffset(Vector2<>(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal) - ballPosition);
  const Vector2<> rightGoalPostOffset(Vector2<>(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal) - ballPosition);
  const float angleToLeftGoalPost = leftGoalPostOffset.angle();
  const float angleToRightGoalPost = rightGoalPostOffset.angle();
  const Pose2D origin(theRobotPose.rotation);

  // Describes one of the two edges of an obstacle sector.
  struct Edge
  {
    float distance; // The distance to the obstacle border that is the reason for this edge.
    float direction; // The direction in which this edge is pointing (in radians).
    int step; // Increase (+1) or decrease (-1) the obstacle counter?

    Edge() = default;
    Edge(float distance, float direction, int step)
    : distance(distance), direction(direction), step(step) {}
  };

  std::vector<Edge> edges;

  // Insert obstacles including seen goal posts
  for(const auto& obstacle : theExpObstacleModel.eobs)
  {
    Vector2<> position = theRobotPose * obstacle.center;
    if(obstacle.type != ExpObstacleModel::ExpObstacle::US &&
       position.x < (theFieldDimensions.xPosOpponentGoal + theFieldDimensions.xPosOpponentGoalPost) / 2.f)
    {
      const Vector2<> offsetToBall = position - ballPosition;
      const float direction = offsetToBall.angle();
      const float width = (obstacle.type == ExpObstacleModel::ExpObstacle::GOALPOST
                          ? theFieldDimensions.goalPostRadius * 2.f
                          : (obstacle.getLeftFoot() - obstacle.getRightFoot()).abs()) + 2.f * theFieldDimensions.ballRadius;
      const float distance = std::sqrt(std::max(offsetToBall.squareAbs() - sqr(width / 2.f), 1.f));
      const float radius = std::atan2(width / 2.f, distance);
      const float left = direction + radius;
      const float right = direction - radius;
      if(right < angleToLeftGoalPost && left > angleToRightGoalPost)
      {
        edges.push_back(Edge(distance, right, 1));
        edges.push_back(Edge(distance, left, -1));
        COMPLEX_DRAWING("module:ScoreDirectionsProvider:sectors",
        {
          const Vector2<> points[3] =
          {
            theBallModel.estimate.position,
            Pose2D(right - theRobotPose.rotation) * DRAWING_EDGE + theBallModel.estimate.position,
            Pose2D(left - theRobotPose.rotation) * DRAWING_EDGE + theBallModel.estimate.position
          };
          POLYGON("module:ScoreDirectionsProvider:sectors",
                  3, points, 0, Drawings::ps_null, ColorRGBA(), Drawings::bs_solid, ColorRGBA(255, 0, 0, 64));
        });
      }
    }
  }

  // Insert localization-based goal posts
  const float leftGoalPostDistance = std::sqrt(std::max(leftGoalPostOffset.squareAbs() - sqr(theFieldDimensions.goalPostRadius + theFieldDimensions.ballRadius), 1.f));
  const float leftGoalPostRadius = std::atan2(theFieldDimensions.goalPostRadius + theFieldDimensions.ballRadius, leftGoalPostDistance);
  edges.push_back(Edge(leftGoalPostDistance, angleToLeftGoalPost - leftGoalPostRadius, 1));
  edges.push_back(Edge(leftGoalPostDistance, angleToLeftGoalPost + leftGoalPostRadius, -1));
  COMPLEX_DRAWING("module:ScoreDirectionsProvider:sectors",
  {
    const Vector2<> points[3] =
    {
      theBallModel.estimate.position,
      Pose2D(angleToLeftGoalPost - theRobotPose.rotation - leftGoalPostRadius) * DRAWING_EDGE + theBallModel.estimate.position,
      Pose2D(angleToLeftGoalPost - theRobotPose.rotation + leftGoalPostRadius) * DRAWING_EDGE + theBallModel.estimate.position
    };
    POLYGON("module:ScoreDirectionsProvider:sectors",
            3, points, 0, Drawings::ps_null, ColorRGBA(), Drawings::bs_solid, ColorRGBA(255, 255, 0, 64));
  });

  const float rightGoalPostDistance = std::sqrt(std::max(rightGoalPostOffset.squareAbs() - sqr(theFieldDimensions.goalPostRadius + theFieldDimensions.ballRadius), 1.f));
  const float rightGoalPostRadius = std::atan2(theFieldDimensions.goalPostRadius + theFieldDimensions.ballRadius, rightGoalPostDistance);
  edges.push_back(Edge(rightGoalPostDistance, angleToRightGoalPost - rightGoalPostRadius, 1));
  edges.push_back(Edge(rightGoalPostDistance, angleToRightGoalPost + rightGoalPostRadius, -1));
  COMPLEX_DRAWING("module:ScoreDirectionsProvider:sectors",
  {
    const Vector2<> points[3] =
    {
      theBallModel.estimate.position,
      Pose2D(angleToRightGoalPost - theRobotPose.rotation - rightGoalPostRadius) * DRAWING_EDGE + theBallModel.estimate.position,
      Pose2D(angleToRightGoalPost - theRobotPose.rotation + rightGoalPostRadius) * DRAWING_EDGE + theBallModel.estimate.position
    };
    POLYGON("module:ScoreDirectionsProvider:sectors",
            3, points, 0, Drawings::ps_null, ColorRGBA(), Drawings::bs_solid, ColorRGBA(255, 255, 0, 64));
  });

  float lastCenter = 1000.f;
  if(!scoreDirections.sectors.empty())
  {
    const ScoreDirections::Sector& s(scoreDirections.sectors.front());
    const float left = (s.leftLimit - ballPosition).angle();
    const float right = (s.rightLimit - ballPosition).angle();
    lastCenter = (left + right) / 2.f;
  }

  // Fill representation
  scoreDirections.sectors.clear();

  if(!edges.empty())
  {
    // Sort by angle. When two directions are equal, the closer obstacle is started first,
    // but ended last.
    std::sort(edges.begin(), edges.end(),
              [](const Edge& e1, const Edge& e2) -> bool
    {
      return e1.direction < e2.direction ||
             (e1.direction == e2.direction && e1.distance * e1.step > e2.distance * e2.step);
    });

    // Sweep through obstacles. Whenever the counter is zero, the sector is free.
    int counter = edges.front().step;
    for(auto i = edges.begin(), j = i++; i != edges.end(); ++i, ++j)
    {
      ASSERT(counter >= 0);
      float range = i->direction - j->direction;
      if(counter == 0 && range > 0)
        scoreDirections.sectors.push_back(ScoreDirections::Sector(
          Pose2D(i->direction, ballPosition) * Vector2<>(i->distance, 0),
          Pose2D(j->direction, ballPosition) * Vector2<>(j->distance, 0)));
      counter += i->step;
    }

    // Sort by angular size in decending order
    std::sort(scoreDirections.sectors.begin(), scoreDirections.sectors.end(),
              [&](const ScoreDirections::Sector& s1, const ScoreDirections::Sector& s2) -> bool
    {
      const float left1 = (s1.leftLimit - ballPosition).angle();
      const float right1 = (s1.rightLimit - ballPosition).angle();
      const float left2 = (s2.leftLimit - ballPosition).angle();
      const float right2 = (s2.rightLimit - ballPosition).angle();
      const float range1 = (left1 - right1) * (left1 >= lastCenter && right1 <= lastCenter ? bonusRatio : 1.f);
      const float range2 = (left2 - right2) * (left2 >= lastCenter && right2 <= lastCenter ? bonusRatio : 1.f);
      return range1 > range2;
    });
  }
}

MAKE_MODULE(ScoreDirectionsProvider, Modeling)
