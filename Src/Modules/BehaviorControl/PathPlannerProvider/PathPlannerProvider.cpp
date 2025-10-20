/**
 * @file Modules/BehaviorControl/PathPlannerProvider/PathPlannerProvider.cpp
 *
 * This file implements a module that provides a representation that allows to
 * determine a motion request that brings the robot closer to a given target
 * based on path planning. The planing problem is modeled using a visibility
 * graph.
 *
 * @author Thomas Röfer
 */

#include "PathPlannerProvider.h"
#include "Platform/SystemCall.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/DebugDrawings3D.h"
#include "Debugging/Annotation.h"
#include "Tools/BehaviorControl/Strategy/ActiveRole.h"
#include <algorithm>

/**
 * Draw a 3D circle parallel to the field plane.
 * @param id The drawing id.
 * @param xCenter The x coordinate of the center.
 * @param yCenter The y coordinate of the center.
 * @param zCenter The z coordinate of the center.
 * @param radius The radius.
 * @param thickness The line thickness.
 * @param color The color of the circle.
 */
#undef CIRCLE3D
#define CIRCLE3D(id, xCenter, yCenter, zCenter, radius, thickness, color) \
  ARC3D(id, xCenter, yCenter, zCenter, radius, 0, pi2, thickness, color)

/**
 * Draw a 3D arc parallel to the field plane.
 * @param id The drawing id.
 * @param xCenter The x coordinate of the center.
 * @param yCenter The y coordinate of the center.
 * @param zCenter The z coordinate of the center.
 * @param radius The radius.
 * @param fromAngle The start angle.
 * @param angleSize The angular size of the arc. The arc is in the range
 *                  [fromAngle ... fromAngle + angleSize].
 * @param thickness The line thickness.
 * @param color The color of the circle.
 */
#define ARC3D(id, xCenter, yCenter, zCenter, radius, fromAngle, angleSize, thickness, color) \
  do \
  { \
    constexpr Angle _angleStep = pi2 / 32.f; \
    Vector2f _from((xCenter) + std::cos(fromAngle) * (radius), (yCenter) + std::sin(fromAngle) * (radius)); \
    for(Angle _angle = _angleStep; _angle <= (angleSize) - _angleStep; _angle += _angleStep) \
    { \
      Vector2f _to((xCenter) + std::cos(_angle + (fromAngle)) * (radius), (yCenter) + std::sin(_angle + (fromAngle)) * (radius)); \
      LINE3D(id, _from.x(), _from.y(), (zCenter), _to.x(), _to.y(), (zCenter), thickness, color); \
      _from = _to; \
    } \
    Vector2f _to((xCenter) + std::cos((fromAngle) + (angleSize)) * (radius), (yCenter) + std::sin((fromAngle) + (angleSize)) * (radius)); \
    LINE3D(id, _from.x(), _from.y(), (zCenter), _to.x(), _to.y(), (zCenter), thickness, color); \
  } \
  while(false)

MAKE_MODULE(PathPlannerProvider);

static const float epsilon = 0.1f; /**< Small offset in mm. */

PathPlannerProvider::PathPlannerProvider()
  : borders({{theFieldDimensions.xPosOpponentGoalLine + fieldBorderLimit, 0.f, 0.f, -1.f},
             {theFieldDimensions.xPosOwnGoalLine - fieldBorderLimit, 0.f, 0.f, 1.f},
             {0.f, theFieldDimensions.yPosLeftTouchline + fieldBorderLimit, 1.f, 0.f},
             {0.f, theFieldDimensions.yPosRightTouchline - fieldBorderLimit, -1.f, 0.f}})
{
}

void PathPlannerProvider::update(PathPlanner& pathPlanner)
{
  DECLARE_DEBUG_DRAWING("module:PathPlannerProvider:barriers", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:PathPlannerProvider:obstacles", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:PathPlannerProvider:expanded", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:PathPlannerProvider:path", "drawingOnField");
  DECLARE_DEBUG_DRAWING3D("module:PathPlannerProvider:barriers", "field");
  DECLARE_DEBUG_DRAWING3D("module:PathPlannerProvider:obstacles", "field");
  DECLARE_DEBUG_DRAWING3D("module:PathPlannerProvider:expanded", "field");
  DECLARE_DEBUG_DRAWING3D("module:PathPlannerProvider:path", "field");

  pathPlanner.plan = [this](const Pose2f& target, const Pose2f& speed) -> MotionRequest::ObstacleAvoidance
  {
    const bool excludeOwnPenaltyArea = theIllegalAreas.illegal & bit(IllegalAreas::ownPenaltyArea);
    const bool excludeOpponentPenaltyArea = theIllegalAreas.illegal & bit(IllegalAreas::opponentPenaltyArea);
    const bool excludeCenterCircle = (theGameState.isReady() && theIllegalAreas.anticipatedTimestamp && -theFrameInfo.getTimeSince(theIllegalAreas.anticipatedTimestamp) < static_cast<int>((theGameState.timeWhenStateEnds - theGameState.timeWhenStateStarted) / 2))
                                     ? theIllegalAreas.anticipatedIllegal & bit(IllegalAreas::centerCircle)
                                     : theIllegalAreas.illegal & bit(IllegalAreas::centerCircle);

    pathPlannerWasActive = true;
    createBarriers(target, excludeOwnPenaltyArea, excludeOpponentPenaltyArea);
    createNodes(target, excludeOwnPenaltyArea, excludeOpponentPenaltyArea, excludeCenterCircle);
    plan(nodes[0], nodes[1], speed.translation.x() / speed.rotation);

    bool foundPath = false;
    MotionRequest::ObstacleAvoidance obstacleAvoidance;
    FOREACH_ENUM(Rotation, rotation)
      if(nodes[1].fromEdge[rotation])
      {
        Edge* edge;
        for(edge = nodes[1].fromEdge[rotation]; edge->fromNode != &nodes[0]; edge = edge->fromNode->fromEdge[edge->fromRotation])
        {
          obstacleAvoidance.path.emplace_back();
          obstacleAvoidance.path.back().obstacle = Geometry::Circle(theRobotPose.inverse() * edge->fromNode->center, edge->fromNode->radius + radiusControlOffset);
          obstacleAvoidance.path.back().clockwise = edge->fromRotation == cw;
        }
        lastDir = edge->toRotation;

        std::reverse(obstacleAvoidance.path.begin(), obstacleAvoidance.path.end());
        obstacleAvoidance.avoidance = calcAvoidanceVector(edge->toNode);

        foundPath = true;
        break;
      }

    if(!foundPath)
    {
      // Walk straight to target (but still avoid close obstacles)
      obstacleAvoidance.avoidance = calcAvoidanceVector(&nodes[1]);

      if(theFrameInfo.getTimeSince(timeWhenLastPlayedSound) > 5000)
      {
        ANNOTATION("PathPlannerProvider", "No path to target");
        SystemCall::playSound("theValidityDuck.wav");
        timeWhenLastPlayedSound = theFrameInfo.time;
      }
    }
    draw();

    return obstacleAvoidance;
  };

  if(!pathPlannerWasActive)
    lastDir = numOfRotations;
  else
    pathPlannerWasActive = false;
}

void PathPlannerProvider::createBarriers(const Pose2f& target, bool excludeOwnPenaltyArea, bool excludeOpponentPenaltyArea)
{
  barriers.clear();
  barriers.reserve(20);

  // Add sides of the goal nets
  barriers.emplace_back(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal,
                        theFieldDimensions.xPosOpponentFieldBorder, theFieldDimensions.yPosLeftGoal);
  barriers.emplace_back(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal,
                        theFieldDimensions.xPosOpponentFieldBorder, theFieldDimensions.yPosRightGoal);
  barriers.emplace_back(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosLeftGoal,
                        theFieldDimensions.xPosOwnFieldBorder, theFieldDimensions.yPosLeftGoal);
  barriers.emplace_back(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosRightGoal,
                        theFieldDimensions.xPosOwnFieldBorder, theFieldDimensions.yPosRightGoal);

  if(useFastestWalkLeaderboardBarriers)
  {
    barriers.emplace_back(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea,
                          theFieldDimensions.xPosOwnFieldBorder, theFieldDimensions.yPosLeftPenaltyArea);
    barriers.emplace_back(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosRightGoalArea,
                          theFieldDimensions.xPosOpponentFieldBorder, theFieldDimensions.yPosRightGoalArea);
    barriers.emplace_back(theFieldDimensions.xPosOwnPenaltyMark, 0.f,
                          theFieldDimensions.xPosOwnFieldBorder, 0.f);
    barriers.emplace_back(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosLeftGoalArea,
                          theFieldDimensions.xPosOpponentFieldBorder, theFieldDimensions.yPosLeftGoalArea);
  }

  if(excludeOwnPenaltyArea)
  {
    float left = theFieldDimensions.yPosLeftPenaltyArea + penaltyAreaRadius - radiusControlOffset;
    float right = theFieldDimensions.yPosRightPenaltyArea - penaltyAreaRadius + radiusControlOffset;
    float front = theFieldDimensions.xPosOwnPenaltyArea + penaltyAreaRadius - radiusControlOffset;

    clipPenaltyArea(theRobotPose.translation, left, right, front);
    clipPenaltyArea(target.translation, left, right, front);

    barriers.emplace_back(theFieldDimensions.xPosOwnPenaltyArea, left,
                          theFieldDimensions.xPosOwnGoalLine, left);
    barriers.emplace_back(theFieldDimensions.xPosOwnPenaltyArea, right,
                          theFieldDimensions.xPosOwnGoalLine, right);
    barriers.emplace_back(front, theFieldDimensions.yPosLeftPenaltyArea,
                          front, theFieldDimensions.yPosRightPenaltyArea);
  }

  if(excludeOpponentPenaltyArea)
  {
    float left = theFieldDimensions.yPosLeftPenaltyArea + penaltyAreaRadius - radiusControlOffset;
    float right = theFieldDimensions.yPosRightPenaltyArea - penaltyAreaRadius + radiusControlOffset;
    float front = theFieldDimensions.xPosOpponentPenaltyArea + penaltyAreaRadius - radiusControlOffset;

    clipPenaltyArea(theRobotPose.translation, left, right, front);
    clipPenaltyArea(target.translation, left, right, front);

    barriers.emplace_back(theFieldDimensions.xPosOpponentPenaltyArea, left,
                          theFieldDimensions.xPosOpponentGoalLine, left);
    barriers.emplace_back(theFieldDimensions.xPosOpponentPenaltyArea, right,
                          theFieldDimensions.xPosOpponentGoalLine, right);
    barriers.emplace_back(front, theFieldDimensions.yPosLeftPenaltyArea,
                          front, theFieldDimensions.yPosRightPenaltyArea);
  }

  if(theGameState.isPlaying() && wrongBallSideCostFactor > 0.f)
  {
    const Vector2f& ballPosition = theStrategyStatus.role == ActiveRole::toRole(ActiveRole::playBall) ? theFieldBall.recentBallEndPositionOnField() : theFieldBall.recentBallPositionOnField();
    if((ballPosition - target.translation).norm() >= 1.f)
    {
      Vector2f end = ballPosition + (ballPosition - Vector2f(theFieldDimensions.xPosOwnGoal, 0)).normalized(wrongBallSideRadius);
      barriers.emplace_back(ballPosition.x(), ballPosition.y(), end.x(), end.y(), (ballDistance + theBallSpecification.radius) * pi2 * wrongBallSideCostFactor);
    }
  }
}

void PathPlannerProvider::clipPenaltyArea(const Vector2f& position, float& left, float& right, float& front) const
{
  if(position.x() <= front && position.y() >= right && position.y() <= left)
  {
    // If the robot is inside the penalty area, move the closest barrier so the robot is still outside
    const float distanceLeft = left - position.y();
    const float distanceRight = position.y() - right;
    const float distanceFront = front - position.x();
    if(distanceLeft < std::min(distanceRight, distanceFront))
      left = position.y() - epsilon;
    else if(distanceRight < std::min(distanceLeft, distanceFront))
      right = position.y() + epsilon;
    else
      front = position.x() - epsilon;
  }
}

void PathPlannerProvider::createNodes(const Pose2f& target, bool excludeOwnPenaltyArea, bool excludeOpponentPenaltyArea, bool excludeCenterCircle)
{
  nodes.clear();

  // Reserve enough space that prevents any reallocation, because the addresses of entries are used.
  nodes.reserve(sqr((excludeOwnPenaltyArea ? 4 : 0) + (excludeOpponentPenaltyArea ? 4 : 0) +
                    (excludeCenterCircle ? 1 : 0) + 7 + theObstacleModel.obstacles.size()));

  // Insert start and target
  nodes.emplace_back(theRobotPose.translation, 0.f);
  nodes.emplace_back(target.translation, 0.f);
  Node& from(nodes.front());
  Node& to(nodes.back());

  // Insert goalposts
  const float goalPostRadius = goalPostDistance + theFieldDimensions.goalPostRadius;
  nodes.emplace_back(Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal), goalPostRadius - radiusControlOffset);
  nodes.emplace_back(Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal), goalPostRadius - radiusControlOffset);
  nodes.emplace_back(Vector2f(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosLeftGoal), goalPostRadius - radiusControlOffset);
  nodes.emplace_back(Vector2f(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosRightGoal), goalPostRadius - radiusControlOffset);

  if(excludeOwnPenaltyArea)
  {
    // The nodes around the penalty area will be intersected by barriers. Therefore, they can
    // be reached from two sides and must be cloneable once.
    nodes.emplace_back(Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea), penaltyAreaRadius - radiusControlOffset + epsilon);
    nodes.back().allowedClones = 1;
    nodes.emplace_back(Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosRightPenaltyArea), penaltyAreaRadius - radiusControlOffset + epsilon);
    nodes.back().allowedClones = 1;
    nodes.emplace_back(Vector2f(theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.yPosLeftPenaltyArea), penaltyAreaRadius - radiusControlOffset + epsilon);
    nodes.back().allowedClones = 1;
    nodes.emplace_back(Vector2f(theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.yPosRightPenaltyArea), penaltyAreaRadius - radiusControlOffset + epsilon);
    nodes.back().allowedClones = 1;
  }

  if(excludeOpponentPenaltyArea)
  {
    // The nodes around the penalty area will be intersected by barriers. Therefore, they can
    // be reached from two sides and must be cloneable once.
    nodes.emplace_back(Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea), penaltyAreaRadius - radiusControlOffset + epsilon);
    nodes.back().allowedClones = 1;
    nodes.emplace_back(Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosRightPenaltyArea), penaltyAreaRadius - radiusControlOffset + epsilon);
    nodes.back().allowedClones = 1;
    nodes.emplace_back(Vector2f(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosLeftPenaltyArea), penaltyAreaRadius - radiusControlOffset + epsilon);
    nodes.back().allowedClones = 1;
    nodes.emplace_back(Vector2f(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosRightPenaltyArea), penaltyAreaRadius - radiusControlOffset + epsilon);
    nodes.back().allowedClones = 1;
  }

  // Insert obstacles if they are on the field.
  for(const auto& obstacle : theObstacleModel.obstacles)
    addObstacle(theRobotPose * obstacle.center, getRadius(obstacle.type));

  // Add ball in playing if it is not the target
  if(theGameState.isPlaying())
  {
    const Vector2f& ballPosition = theStrategyStatus.role == ActiveRole::toRole(ActiveRole::playBall) ? theFieldBall.recentBallEndPositionOnField() : theFieldBall.recentBallPositionOnField();
    if(theGameState.isFreeKick() && !theGameState.isForOwnTeam())
      addObstacle(ballPosition, freeKickRadius);
    else if((ballPosition - to.center).norm() >= 1.f)
      addObstacle(ballPosition, ballDistance + theBallSpecification.radius);
  }

  if(centerCircleDistance != 0.f && excludeCenterCircle)
    addObstacle(Vector2f::Zero(), centerCircleDistance + theFieldDimensions.centerCircleRadius);

  // If other nodes surround start or target, shrink them.
  for(auto node = nodes.begin(); node != nodes.begin() + 2; ++node)
    for(auto other = nodes.begin() + 2; other != nodes.end(); ++other)
    {
      if((other->center - node->center).squaredNorm() <= sqr(other->radius))
      {
        Vector2f diff = other->center - node->center;
        const float distance = diff.norm();
        other->radius = (other->radius + distance - epsilon) / 2.f;
        other->center = node->center + (other->center - node->center).normalized(other->radius + epsilon);
        ASSERT(other->radius >= 0.f);
      }
    }

  // If start and target are both inside the field, prevent passing obstacles outside the field.
  Boundaryf border(Rangef(borders[1].base.x(), borders[0].base.x()),
                   Rangef(borders[3].base.y(), borders[2].base.y()));
  if(border.isInside(from.center) && border.isInside(to.center))
  {
    Vector2f p1;
    Vector2f p2;
    for(auto node = nodes.begin() + 2; node != nodes.end(); ++node)
      for(const auto& border : borders)
        if(Geometry::getIntersectionOfLineAndCircle(border, *node, p1, p2) == 2)
          node->blockedSectors.emplace_back((p1 - node->center).angle(), (p2 - node->center).angle());
  }

  // Whenever a barrier intersects a node, add a blocking sector.
  for(auto& node : nodes)
  {
    for(auto& barrier : barriers)
    {
      Geometry::Line line(barrier.from, barrier.to - barrier.from);
      Vector2f p1;
      Vector2f p2;
      if(Geometry::getIntersectionOfLineAndCircle(line, node, p1, p2) == 2)
      {
        if(Geometry::getDistanceToEdge(line, p1) == 0.f)
        {
          const float angle = (p1 - node.center).angle();
          node.blockedSectors.emplace_back(angle, angle, barrier.costs);
        }
        if(Geometry::getDistanceToEdge(line, p2) == 0.f)
        {
          const float angle = (p2 - node.center).angle();
          node.blockedSectors.emplace_back(angle, angle, barrier.costs);
        }
      }
    }
  }
}

float PathPlannerProvider::getRadius(Obstacle::Type type) const
{
  if(theGameState.isReady())
    return readyRobotRadius - radiusControlOffset;
  else
    switch(type)
    {
      case Obstacle::fallenSomeRobot:
      case Obstacle::fallenOpponent:
      case Obstacle::fallenTeammate:
        return fallenRobotRadius - radiusControlOffset;
      default:
        return uprightRobotRadius - radiusControlOffset;
    }
}

void PathPlannerProvider::addObstacle(const Vector2f& center, float radius)
{
  if(radius != 0.f &&
     borders[0].base.x() > center.x() - radius &&
     borders[1].base.x() < center.x() + radius &&
     borders[2].base.y() > center.y() - radius &&
     borders[3].base.y() < center.y() + radius)
    nodes.emplace_back(center, radius);
}

void PathPlannerProvider::plan(Node& from, Node& to, float speedRatio)
{
  candidates.clear();
  candidates.reserve(nodes.size() * nodes.size() * 4);

  expand(from, to, cw, speedRatio);
  expand(from, to, ccw, speedRatio);

  // Do A* search
  while(!candidates.empty())
  {
    Candidate candidate = candidates.front();

    std::pop_heap(candidates.begin(), candidates.end());
    candidates.pop_back();

    // Clone target node if it was already reached and clones are allowed.
    if(candidate.edge->toNode->fromEdge[candidate.edge->toRotation] &&
       candidate.edge->toNode->allowedClones > 0)
    {
      nodes.emplace_back(*candidate.edge->toNode);
      --candidate.edge->toNode->allowedClones;
      candidate.edge->toNode = &nodes.back();
    }
    if(!candidate.edge->toNode->fromEdge[candidate.edge->toRotation])
    {
      candidate.edge->toNode->fromEdge[candidate.edge->toRotation] = candidate.edge;
      if(candidate.edge->toNode == &to)
        break;
      else
        expand(*candidate.edge->toNode, to, candidate.edge->toRotation, speedRatio);
    }
  }
}

void PathPlannerProvider::expand(Node& node, const Node& to, Rotation rotation, float speedRatio)
{
  if(!node.expanded)
  {
    findNeighbors(node);
    node.expanded = true;
  }

  for(auto& edge : node.edges[rotation])
  {
    edge.pathLength = edge.length;
    if(node.fromEdge[rotation])
    {
      // This is not the first node, i.e. we arrived here at fromEdge->toPoint.
      edge.pathLength += node.fromEdge[rotation]->pathLength;
      const float toAngle = (node.fromEdge[rotation]->toPoint - node.center).angle();

      // The sector used on the circle is from toAngle of the incoming edge
      // to fromAngle of the outgoing edge.
      Rangef interval;
      if(rotation == cw)
      {
        interval.min = edge.fromAngle;
        interval.max = toAngle;
      }
      else
      {
        interval.min = toAngle;
        interval.max = edge.fromAngle;
      }

      // If an blocking sector overlaps with this interval, at least one limit
      // of one interval must be inside the other interval.
      // If it is, reaching the outgoing edge is not possible.
      for(const auto& sector : node.blockedSectors)
        if(interval.isInside(sector.min) || interval.isInside(sector.max) ||
           sector.isInside(interval.min) || sector.isInside(interval.max))
        {
          if(sector.costs == std::numeric_limits<float>::infinity())
            goto continueOuterLoop;
          else
            edge.pathLength += sector.costs;
        }

      // Compute the positive angle from incoming to outgoing edge in the fixed direction (cw/ccw).
      float angle = interval.max - interval.min;
      if(angle < 0.f)
        angle += pi2;

      edge.pathLength += angle * node.radius;
    }
    else
    {
      // This is the first node. Add penalty for rotating to outgoing edge.
      const float toRotate = std::abs((theRobotPose.translation - edge.toNode->center).norm() > edge.toNode->radius
                                      ? (Pose2f(edge.toPoint) - theRobotPose).translation.angle()
                                      : Angle::normalize((edge.toPoint - edge.toNode->center).angle() + (edge.toRotation == cw ? -pi_2 : pi_2) - theRobotPose.rotation));
      const float distanceRatio = toRotate * speedRatio;
      edge.pathLength += distanceRatio * rotationPenalty + (lastDir == edge.toRotation ? 0.f : switchPenalty);
    }

    candidates.emplace_back(&edge, (to.center - edge.toPoint).norm());
    std::push_heap(candidates.begin(), candidates.end());

  continueOuterLoop:
    ;
  }
}

void PathPlannerProvider::findNeighbors(Node& node)
{
  Tangents tangents;
  createTangents(node, tangents);
  addNeighborsFromTangents(node, tangents);
}

void PathPlannerProvider::createTangents(Node& node, Tangents& tangents)
{
  // search all nodes except for start node
  for(auto neighbor = nodes.begin() + 1; neighbor != nodes.end(); ++neighbor)
  {
    Vector2f v = neighbor->center - node.center;
    const float d2 = v.squaredNorm();
    if(d2 > (node.radius - neighbor->radius) * (node.radius - neighbor->radius))
    {
      const float d = std::sqrt(d2);
      v /= d;

      // http://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Tangents_between_two_circles
      //
      // Let A, B be the centers, and C, D be points at which the tangent
      // touches first and second circle, and n be the normal vector to it.
      //
      // We have the system:
      //   n * n = 1          (n is a unit vector)
      //   C = A + r1 * n
      //   D = B +/- r2 * n
      //   n * CD = 0         (common orthogonality)
      //
      // n * CD = n * (AB +/- r2*n - r1*n) = AB*n - (r1 -/+ r2) = 0,  <=>
      // AB * n = (r1 -/+ r2), <=>
      // v * n = (r1 -/+ r2) / d,  where v = AB/|AB| = AB/d
      // This is a linear equation in unknown vector n.
      FOREACH_ENUM(Rotation, i)
      {
        const float sign1 = i ? -1.f : 1.f;
        const float c = (node.radius - sign1 * neighbor->radius) / d;

        if(c * c <= 1.f)
        {
          // If one of the circles is just a point, the second pair of tangents is skipped,
          // because they would be duplicates of the first pair.
          if(i && (node.radius == 0.f || neighbor->radius == 0.f))
          {
            // However, if the current node is a point (and the other one is not),
            // the other one still hides nodes further away. Therefore,
            // it needs a second (dummy) tangent for both rotations.
            if(node.radius == 0.f)
            {
              tangents[0].emplace_back(tangents[1].back());
              tangents[0].back().dummy = true;
              tangents[1].emplace_back(tangents[0][tangents[0].size() - 2]);
              tangents[1].back().dummy = true;
            }
          }
          else
          {
            // Now we're just intersecting a line with a circle: v*n=c, n*n=1
            const float h = std::sqrt(1.f - c * c);
            FOREACH_ENUM(Rotation, j)
            {
              float sign2 = j ? -1.f : 1.f;
              const Vector2f n(v.x() * c - sign2 * h * v.y(), v.y() * c + sign2 * h * v.x());
              const Vector2f p1 = node.center + n * node.radius;
              const Vector2f p2 = neighbor->center + n * sign1 * neighbor->radius;
              float distance = (p2 - p1).norm();
              const float fromAngle = node.radius == 0.f ? (v * d + n * sign1 * neighbor->radius).angle() : n.angle();
              bool dummy = neighbor->fromEdge[i ^ j] != nullptr;
              if(dummy)
              {
                // Clone target node if it was already reached and clones are allowed.
                if(neighbor->allowedClones > 0)
                {
                  nodes.push_back(*neighbor);
                  --neighbor->allowedClones;
                }
              }
              else
                for(const auto& barrier : barriers)
                  if(barrier.intersects(p1, p2))
                  {
                    if(barrier.costs == std::numeric_limits<float>::infinity())
                    {
                      dummy = true;
                      break;
                    }
                    else
                      distance += barrier.costs;
                  }

              tangents[j].emplace_back(Edge(&node, &*neighbor, fromAngle, p2, static_cast<Rotation>(j), static_cast<Rotation>(i ^ j), distance),
                                       neighbor->radius == 0.f ? Tangent::none : i ^ j ? Tangent::right : Tangent::left, d - neighbor->radius, dummy);

              // If both nodes are points, there is only a single connection. Skip the rest.
              if(node.radius == 0.f && neighbor->radius == 0.f)
                goto exitBothLoops;
            }
          }
        }
        else if(i)
        {
          // The circles overlap and no second pair can be computed.
          // However, the other node still hides all other nodes within an angular range.
          // Add (dummy) tangents as end points for these ranges.
          for(auto& t : tangents)
          {
            const float d1 = 0.5f * (d + (sqr(node.radius) - sqr(neighbor->radius)) / d);
            const float a = std::acos(d1 / node.radius);
            const float dir = v.angle();
            t.emplace_back(t.back());
            BlockedSector blocked(Angle::normalize(dir - a), Angle::normalize(dir + a));
            if(t.back().side == Tangent::left)
            {
              node.blockedSectors.emplace_back(blocked);
              if(neighbor->radius < node.radius)
                ++node.allowedClones;
              t.back().side = Tangent::right;
              t.back().fromAngle = blocked.min;
              t.back().dummy = true;
            }
            else
            {
              t.back().side = Tangent::left;
              t.back().fromAngle = blocked.max;
              t.back().dummy = true;
            }
          }
        }

        // If this is the second tangent for a rotation, check for wraparound.
        // If right tangent is on the wrong side of left tangent, add a second
        // (dummy) right tangent 2pi earlier.
        // For each left tangent, set the index of the matching right tangent.
        if(neighbor->radius != 0.f && i)
        {
          for(auto& t : tangents)
          {
            ASSERT(t.size() >= 2);
            if(t.back().side == Tangent::left)
            {
              if(t.back().fromAngle < t[t.size() - 2].fromAngle)
              {
                t.emplace_back(t[t.size() - 2]);
                t.back().fromAngle -= pi2;
                t.back().dummy = true;
                t[t.size() - 2].matchingRightTangent = static_cast<int>(t.size() - 1);
              }
              else
                t.back().matchingRightTangent = static_cast<int>(t.size() - 2);
            }
            else
            {
              if(t.back().fromAngle > t[t.size() - 2].fromAngle)
              {
                t.emplace_back(t.back());
                t.back().fromAngle -= pi2;
                t.back().dummy = true;
                t[t.size() - 3].matchingRightTangent = static_cast<int>(t.size() - 1);
              }
              else
                t[t.size() - 2].matchingRightTangent = static_cast<int>(t.size() - 1);
            }
          }
        }
      }
    exitBothLoops:
      ;
    }
  }
}

void PathPlannerProvider::addNeighborsFromTangents(Node& node, Tangents& tangents)
{
  FOREACH_ENUM(Rotation, rotation)
  {
    auto& t = tangents[rotation];

    // Create index for tangents sorted by angle.
    // Since indices are used to reference between tangents,
    // the original vector of tangents must stay unchanged.
    std::vector<Tangent*> index;
    index.reserve(t.size());
    for(auto& tangent : t)
      index.push_back(&tangent);
    std::sort(index.begin(), index.end(), [](const Tangent* t1, const Tangent* t2) -> bool
    {
      return t1->fromAngle < t2->fromAngle;
    });

    // Sweep through all tangents, managing a set of current nodes sorted by their distance.
    std::vector<Tangent*> sweepLine;
    sweepLine.reserve(t.size());
    const auto byDistance = [](const Tangent* t1, const Tangent* t2) -> bool
    {
      return t1->circleDistance > t2->circleDistance;
    };
    for(auto& tangent : index)
    {
      // In general, if the node of the current tangent is not further away than the closest node
      // in the sweep line, it is a neighbor. However, since the obstacles are modeled as circles,
      // all obstacles must be checked until one is found that is actually further away or the target
      // point is actually inside another obstacle.
      if(!tangent->dummy)
      {
        for(const auto& s : sweepLine)
          if(!s->ended)
          {
            if(s->circleDistance >= tangent->circleDistance)
              break;
            else if(s->circleDistance + 2.f * s->toNode->radius < tangent->circleDistance)
              goto doNotAddTangent;
            else
            {
              Vector2f fromPoint = Pose2f(tangent->fromAngle, node.center) * Vector2f(node.radius, 0.f);
              Geometry::Line line(fromPoint, tangent->toPoint - fromPoint);
              Vector2f p1;
              Vector2f p2;
              const int numOfIntersections = Geometry::getIntersectionOfLineAndCircle(line, *s->toNode, p1, p2);
              if(numOfIntersections > 0)
              {
                const float distance = line.direction.squaredNorm();
                if(distance >= (p1 - fromPoint).squaredNorm() ||
                   (numOfIntersections == 2 && distance >= (p2 - fromPoint).squaredNorm()))
                  goto doNotAddTangent;
              }
            }
          }
        node.edges[rotation].emplace_back(*tangent);

      doNotAddTangent:
        ;
      }

      if(tangent->side == Tangent::right)
      {
        // If the current tangent is a right edge, add it to the sweep line
        sweepLine.push_back(tangent);
        std::push_heap(sweepLine.begin(), sweepLine.end(), byDistance);
      }
      else if(tangent->side == Tangent::left)
      {
        // If the current tangent is a left edge, mark it as to be deleted
        // from the sweep line. Delete all closest entries from the sweep line
        // as long as they can be deleted.
        if(tangent->matchingRightTangent != -1)
          t[tangent->matchingRightTangent].ended = true;
        else
        {
          ANNOTATION("PathPlannerProvider", "ASSERT(tangent->matchingRightTangent != -1)");
#ifndef NDEBUG
          {
            OutBinaryFile stream("PathPlannerProvider.log");
            stream << theRobotPose << theObstacleModel << lastDir;
          }
          FAIL("Send Config/PathPlannerProvider.log to Thomas.Roefer@dfki.de.");
#endif
        }
        while(!sweepLine.empty() && sweepLine.front()->ended)
        {
          std::pop_heap(sweepLine.begin(), sweepLine.end(), byDistance);
          sweepLine.pop_back();
        }
      }
    }
  }
}

Vector2f PathPlannerProvider::calcAvoidanceVector(const Node* nextNode) const
{
  // Determine general avoidance direction.
  Vector2f avoidance = Vector2f::Zero();
  bool closeToNextNode = false;
  bool closeToOtherNode = false;
  bool inFront = false;
  for(auto node = nodes.begin() + 2; node != nodes.end(); ++node)
  {
    const Vector2f offset = node->center - theRobotPose.translation;
    if(offset.squaredNorm() < sqr(node->originalRadius + radiusControlOffset))
    {
      inFront |= (Pose2f(node->center) - theRobotPose).translation.x() >= 0.f;
      const float factor = std::min(1.f, (node->originalRadius + radiusControlOffset - offset.norm()) / radiusAvoidanceTolerance);
      avoidance += -(Pose2f(node->center) - theRobotPose).translation.normalized(factor);
      closeToNextNode |= &*node == nextNode;
      closeToOtherNode |= &*node != nextNode;
    }
  }

  // Normalize if another node than the current one must (also) be avoided.
  // Reset if only the current node must be avoided (this will happen implicitly).
  if(inFront && closeToOtherNode && avoidance.squaredNorm() > 1.f)
    avoidance.normalize();
  else if(!inFront || (closeToNextNode && !closeToOtherNode))
    avoidance = Vector2f::Zero();

  return avoidance;
}

void PathPlannerProvider::draw() const
{
  COMPLEX_DRAWING("module:PathPlannerProvider:barriers")
  {
    for(const auto& barrier : barriers)
    {
      LINE("module:PathPlannerProvider:barriers", barrier.from.x(), barrier.from.y(), barrier.to.x(), barrier.to.y(),
           10, Drawings::solidPen, ColorRGBA::red);
    }
  }

  COMPLEX_DRAWING3D("module:PathPlannerProvider:barriers")
  {
    for(const auto& barrier : barriers)
    {
      LINE3D("module:PathPlannerProvider:barriers", barrier.from.x(), barrier.from.y(), 0, barrier.to.x(), barrier.to.y(), 0,
             5, ColorRGBA::red);
    }
  }

  COMPLEX_DRAWING("module:PathPlannerProvider:obstacles")
  {
    for(const auto& node : nodes)
    {
      CIRCLE("module:PathPlannerProvider:obstacles", node.center.x(), node.center.y(), node.radius > 0.f ? node.radius : 50.f,
             10, Drawings::solidPen, ColorRGBA::yellow, Drawings::noPen, ColorRGBA());
      for(const auto& blockedSector : node.blockedSectors)
      {
        float span = blockedSector.max - blockedSector.min;
        if(span < 0.f)
          span += pi2;
        ARC("module:PathPlannerProvider:obstacles", node.center.x(), node.center.y(), node.radius > 0.f ? node.radius : 50.f, blockedSector.min, span,
            10, Drawings::solidPen, ColorRGBA::red, Drawings::noPen, ColorRGBA());
      }
    }
  }

  COMPLEX_DRAWING3D("module:PathPlannerProvider:obstacles")
  {
    for(const auto& node : nodes)
    {
      CIRCLE3D("module:PathPlannerProvider:obstacles", node.center.x(), node.center.y(), 0, node.radius > 0.f ? node.radius : 20.f, 5, ColorRGBA::yellow);
      for(const auto& blockedSector : node.blockedSectors)
      {
        float span = blockedSector.max - blockedSector.min;
        if(span < 0.f)
          span += pi2;
        ARC3D("module:PathPlannerProvider:obstacles", node.center.x(), node.center.y(), 3.f, node.radius, blockedSector.min, span,
              6, ColorRGBA::red);
      }
    }
  }

  COMPLEX_DRAWING("module:PathPlannerProvider:expanded")
  {
    for(const auto& node : nodes)
    {
      FOREACH_ENUM(Rotation, rotation)
        for(const auto& edge : node.edges[rotation])
        {
          Vector2f p1 = Pose2f(edge.fromAngle, node.center) * Vector2f(node.radius, 0.f);
          const Vector2f& p2 = edge.toPoint;
          LINE("module:PathPlannerProvider:expanded", p1.x(), p1.y(), p2.x(), p2.y(), 10, Drawings::solidPen, ColorRGBA::yellow);
        }
    }
  }

  COMPLEX_DRAWING3D("module:PathPlannerProvider:expanded")
  {
    for(const auto& node : nodes)
    {
      FOREACH_ENUM(Rotation, rotation)
        for(const auto& edge : node.edges[rotation])
        {
          Vector2f p1 = Pose2f(edge.fromAngle, node.center) * Vector2f(node.radius, 0.f);
          const Vector2f& p2 = edge.toPoint;
          LINE3D("module:PathPlannerProvider:expanded", p1.x(), p1.y(), 0, p2.x(), p2.y(), 0, 5, ColorRGBA::yellow);
        }
    }
  }

  COMPLEX_DRAWING("module:PathPlannerProvider:path")
  {
    FOREACH_ENUM(Rotation, rotation)
      if(nodes[1].fromEdge[rotation])
      {
        for(const Edge* edge = nodes[1].fromEdge[rotation]; edge; edge = edge->fromNode->fromEdge[edge->fromRotation])
        {
          Vector2f p1 = Pose2f(edge->fromAngle, edge->fromNode->center) * Vector2f(edge->fromNode->radius, 0.f);
          const Vector2f& p2 = edge->toPoint;
          LINE("module:PathPlannerProvider:path", p1.x(), p1.y(), p2.x(), p2.y(), 20, Drawings::solidPen, ColorRGBA::green);

          if(edge->fromNode->fromEdge[edge->fromRotation])
          {
            const Node& node = *edge->fromNode;
            // This is not the first node, i.e. we arrived here at fromEdge->toPoint.
            const float toAngle = (node.fromEdge[edge->fromRotation]->toPoint - node.center).angle();

            // The sector used on the circle is from toAngle of the incoming edge
            // to fromAngle of the outgoing edge.
            Rangef interval;
            if(edge->fromRotation == cw)
            {
              interval.min = edge->fromAngle;
              interval.max = toAngle;
            }
            else
            {
              interval.min = toAngle;
              interval.max = edge->fromAngle;
            }

            // Compute the positive angle from incoming to outgoing edge in the fixed direction (cw/ccw).
            float angle = interval.max - interval.min;
            if(angle < 0.f)
              angle += pi2;

            ARC("module:PathPlannerProvider:path", node.center.x(), node.center.y(), node.radius, interval.min, angle,
                20, Drawings::solidPen, ColorRGBA::green, Drawings::noPen, ColorRGBA());
          }
        }
      }
  }

  COMPLEX_DRAWING3D("module:PathPlannerProvider:path")
  {
    FOREACH_ENUM(Rotation, rotation)
      if(nodes[1].fromEdge[rotation])
      {
        for(const Edge* edge = nodes[1].fromEdge[rotation]; edge; edge = edge->fromNode->fromEdge[edge->fromRotation])
        {
          Vector2f p1 = Pose2f(edge->fromAngle, edge->fromNode->center) * Vector2f(edge->fromNode->radius, 0.f);
          const Vector2f& p2 = edge->toPoint;
          LINE3D("module:PathPlannerProvider:path", p1.x(), p1.y(), 3.f, p2.x(), p2.y(), 3.f, 10, ColorRGBA::green);

          if(edge->fromNode->fromEdge[edge->fromRotation])
          {
            const Node& node = *edge->fromNode;
            // This is not the first node, i.e. we arrived here at fromEdge->toPoint.
            const float toAngle = (node.fromEdge[edge->fromRotation]->toPoint - node.center).angle();

            // The sector used on the circle is from toAngle of the incoming edge
            // to fromAngle of the outgoing edge.
            Rangef interval;
            if(edge->fromRotation == cw)
            {
              interval.min = edge->fromAngle;
              interval.max = toAngle;
            }
            else
            {
              interval.min = toAngle;
              interval.max = edge->fromAngle;
            }

            // Compute the positive angle from incoming to outgoing edge in the fixed direction (cw/ccw).
            float angle = interval.max - interval.min;
            if(angle < 0.f)
              angle += pi2;

            ARC3D("module:PathPlannerProvider:path", node.center.x(), node.center.y(), 3.f, node.radius, interval.min, angle,
                  10, ColorRGBA::green);
          }
        }
      }
  }
}
