/**
 * @file Modules/BehaviorControl/PathPlannerProvider/PathPlannerProvider.h
 *
 * This file declares a module that provides a representation that allows to
 * determine a motion request that brings the robot closer to a given target
 * based on path planning. The planing problem is modeled using a visibility
 * graph.
 *
 * @author Thomas Röfer
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/PathPlanner.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Tools/Module/Module.h"
#include <limits>

MODULE(PathPlannerProvider,
{,
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(ObstacleModel),
  REQUIRES(RobotPose),
  REQUIRES(OwnTeamInfo),
  REQUIRES(TeamBehaviorStatus),
  REQUIRES(TeamPlayersModel),
  PROVIDES(PathPlanner),
  LOADS_PARAMETERS(
  {,
    (bool) useObstacles, /**< Use TeamPlayersModel or ObstacleModel? */
    (float) goalPostRadius, /**< Radius to walk around a goal post (in mm). */
    (float) uprightRobotRadius, /**< Radius to walk around an upright robot (in mm). */
    (float) fallenRobotRadius, /**< Radius to walk around a fallen robot (in mm). */
    (float) readyRobotRadius, /**< Radius to walk around a robot in ready state (in mm). */
    (float) centerCircleRadius, /**< If != 0: Radius to walk around a the center circle in ready state in defensive kickoff (in mm). */
    (float) penaltyAreaRadius, /**< Radius to walk around a corner of the own penalty area (in mm). */
    (float) ballRadius, /**< Radius to walk around the ball (in mm). */
    (float) freeKickRadius, /**< Radius to walk around the ball when defending a free kick (in mm). */
    (float) wrongBallSideCostFactor, /**< How much of a full circle is it more expensive to pass the ball on the wrong side? */
    (float) wrongBallSideRadius, /**< How far from the ball is passing it on the wrong side penalized? */
    (float) fieldBorderLimit, /**< Distance outside the side lines that is still used for walking (in mm). */
    (float) startTurningBeforeTargetDistance, /**< How far before the target is started to turn to final direction (in mm)? */
    (float) startTurningBeforeCircleDistance, /**< How far before the next circle is switched to walking a curve (in mm)? */
    (float) radiusControlOffset, /**< Plan closer to obstacles by this offset, but keep original distance when executing plan (in mm). */
    (float) radiusAvoidanceTolerance, /**< Radius range in which robot is partially pushed away (in mm). */
    (float) fullTranslationThreshold, /**< Plan closer to obstacles by this offset, but keep original distance when executing plan (in mm). */
    (float) noTranslationThreshold, /**< How wide is the corridor between surrounding and avoiding obstacles behavior (in mm)? */
    (float) controlAheadAngle, /**< Put control point how far ahead on circles (in radians). */
    (float) rotationPenalty, /**< Penalty factor for rotating towards first intermediate target in mm/radian. Stabilizes path selection. */
    (float) switchPenalty, /**< Penalty for selecting a different turn direction around first obstacle in mm. */
    (float) pFactor, /**< Proportional feedback from turn angle to speed. */
    (float) iFactor, /**< Integral feedback from turn angle to speed. */
    (float) antiWindupFactor, /**< How much of the saturated control input to subtract from the integrated error. */
    (float) antiWindupSaturation, /**< Assumed saturation. */
  }),
});

class PathPlannerProvider : public PathPlannerProviderBase
{
  /** Obstacles can be surrounded in clockwise or counterclockwise direction. */
  ENUM(Rotation,
  {,
    cw,
    ccw,
  });

  struct Node;

  /** The edges of the visibility graph. */
  struct Edge
  {
    Node* fromNode; /**< The node from which this edge starts. */
    Node* toNode; /**< The node at which this edge ends. */
    float fromAngle; /**< The angle where this edge touches the circle around fromNode. */
    Vector2f toPoint; /**< The point where this edge touches the circle of toNode. */
    Rotation fromRotation; /**< The rotation with which fromNode was surrounded. */
    Rotation toRotation;  /**< The rotation with which toNode will be surrounded. */
    float length; /**< The length of this edge. */
    float pathLength; /**< The overall length of the path until arriving at toNode. Will be set by A* search. */

    /**
     * Constructor.
     * @param fromNode The node from which this edge starts.
     * @param toNode The node at which this edge ends.
     * @param fromAngle The angle where this edge touches the circle around fromNode.
     * @param toPoint The point where this edge touches the circle of toNode.
     * @param fromRotation The rotation with which fromNode was surrounded.
     * @param toRotation The rotation with which toNode will be surrounded.
     * @param length The length of this edge.
     */
    Edge(Node* fromNode, Node* toNode, float fromAngle, const Vector2f& toPoint, Rotation fromRotation, Rotation toRotation, float length)
      : fromNode(fromNode), toNode(toNode), fromAngle(fromAngle), toPoint(toPoint), fromRotation(fromRotation), toRotation(toRotation), length(length) {}
  };

  /**
   * A sector of a circle surrounding an obstacle that creates higher costs when
   * it is traversed.
   */
  struct BlockedSector : public Rangef
  {
    float costs; /**< costs for passing this circle segment. */
    BlockedSector(float min, float max, float costs = std::numeric_limits<float>::infinity()) : Rangef(min, max), costs(costs) {}
  };

  /** The nodes of the visibility graph, i.e. the obstacles. */
  struct Node : public Geometry::Circle
  {
    std::vector<Edge> edges[numOfRotations]; /** The outgoing edges per rotation. */
    std::vector<BlockedSector> blockedSectors; /**< Angular sectors that are blocked by overlapping other obstacles. */
    Edge* fromEdge[numOfRotations]; /**< From which edge was this node reached first (per rotation) during the A* search? */
    bool expanded = false; /**< Were the outgoing edges of this node already expanded? */
    int allowedClones = 0; /**< The number of times this node can be cloned. */
    float originalRadius; /**< The original radius of this node before it was reduced (in mm). */

    /**
     * Constructor.
     * @param center The center of the obstacle.
     * @param radius The radius of the obstacle.
     */
    Node(const Vector2f& center, float radius) : Circle(center, radius), originalRadius(radius)
    {
      fromEdge[cw] = fromEdge[ccw] = nullptr;
    }

    /**
     * The copy constructor copies all edges, but sets this node as their origin.
     * The new node has not been reached by from an edge yet.
     * @param other The other node that is copied.
     */
    Node(const Node& other) : Node(other.center, other.radius)
    {
      FOREACH_ENUM(Rotation, rotation)
        for(auto& edge : other.edges[rotation])
        {
          edges[rotation].emplace_back(this, edge.toNode, edge.fromAngle, edge.toPoint, edge.fromRotation, edge.toRotation, edge.length);
          edges[rotation].back().pathLength = edge.pathLength;
        }
      blockedSectors = other.blockedSectors;
      expanded = other.expanded;
      originalRadius = other.originalRadius;
    }
  };

  /** A structure to manage the open edges during the A* search. */
  struct Candidate
  {
    Edge* edge; /**< The corresponding edge. */
    float estimatedPathLength; /**< The estimated path length including the heuristic. */

    /**
     * Constructor.
     * @param edge The corresponding edge.
     * @param heuristic The estimated path length from the end of the edge to the target position.
     */
    Candidate(Edge* edge, float heuristic)
      : edge(edge), estimatedPathLength(edge->pathLength + heuristic) {}

    /**
     * Comparison operator for the heap that manages the open edges.
     * @param other The other candidate that is compared with.
     * @param Which one should be taken out of the heap later?
     */
    bool operator<(const Candidate& other) const
    {
      return estimatedPathLength > other.estimatedPathLength;
    }
  };

  /** Barrier lines that cannot be crossed during planning. */
  struct Barrier
  {
    Vector2f from; /**< First end point of barrier. */
    Vector2f to; /**< Second end point of barrier. */
    float costs; /**< Costs for crossing this barrier. */

    /**
     * Constructor.
     * @param x1 The x coordinate of first end point of the barrier.
     * @param y1 The y coordinate of first end point of the barrier.
     * @param x2 The x coordinate of second end point of the barrier.
     * @param y2 The y coordinate of second end point of the barrier.
     */
    Barrier(float x1, float y1, float x2, float y2, float costs = std::numeric_limits<float>::infinity())
      : from(x1, y1), to(x2, y2), costs(costs) {}

    /**
     * Does a line intersect this barrier?
     * @param p1 The first end point of the line.
     * @param p2 The second end point of the line.
     * @return Do they intersect?
     */
    bool intersects(const Vector2f& p1, const Vector2f& p2) const
    {
      return Geometry::checkIntersectionOfLines(from, to, p1, p2);
    }
  };

  struct Tangent : public Edge
  {
    ENUM(Side,
    {,
      none,
      left,
      right,
    });

    Side side; /**< Is this the left or right side of the corridor to the other node? */
    float circleDistance; /**< The closest distance between the borders of the two node connected by this tangent. */
    bool dummy; /**< Is this just a helper and should not be transformed into a real edge? */
    bool ended = false; /**< Has the matching left tangent already processed for this right tangent? */
    int matchingRightTangent = -1; /**< The index of the matching right tangent for this left tangent. */

    /**
     * Constructor.
     * @param edge The edge that might be added to the graph if is not blocked by obstacles.
     * @param side Is this the left or right side of the corridor to the other node?
     * @param circleDistance The closest distance between the borders of the two node connected by this tangent.
     * @param dummy Is this just a helper and should not be transformed into a real edge?
     */
    Tangent(const Edge& edge, Side side, float circleDistance, bool dummy)
    : Edge(edge), side(side), circleDistance(circleDistance), dummy(dummy) {}
  };

  using Tangents = std::array<std::vector<Tangent>, numOfRotations>;

  std::vector<Node> nodes; /**< All nodes of the visibility graph, i.e. all obstacles, and starting point (1st entry) and target (2nd entry). */
  std::vector<Candidate> candidates; /**< All open edges during the A* search. */
  std::vector<Barrier> barriers; /**< Barrier lines that cannot be crossed during planning. */
  std::vector<Geometry::Line> borders; /**< The border of the field plus a tolerance. */
  Rotation lastDir = cw; /**< Last direction selected when walking around first obstacle. */
  float turnAngleIntegrator = 0.f; /**< An integrator over the angle to the next node. Unclear which unit this has. */
  unsigned timeWhenLastPlayedSound = 0; /**< Used to limit frequency of sound playback. */
  bool pathPlannerWasActive = false; /**< Was the path planner active in previous frame? */

  /**
   * Provide a representation that is able to plan a path using this module.
   * @param pathPlanner The representation that is provided.
   */
  void update(PathPlanner& pathPlanner) override;

  /**
   * Compute barrier lines that cannot be crossed during planning.
   * @param target The target the robot tries to reach.
   * @param excludePenaltyArea Also generate barriers for the own penalty area.
   */
  void createBarriers(const Pose2f& target, bool excludePenaltyArea);

  /**
   * Clip penalty area barriers to make a position reachable.
   * @param position The position that should be reachable.
   * @param left The y coordinate of the left barrier.
   * @param right The y coordinate of the right barrier.
   * @param front The x coordinate of the front barrier.
   */
  void clipPenaltyArea(const Vector2f& position, float& left, float& right, float& front) const;

  /**
   * Create the nodes from obstacles.
   * @param target The target the robot tries to reach.
   * @param excludePenaltyArea Filter out obstacles inside the own penalty area and the own goal.
   */
  void createNodes(const Pose2f& target, bool excludePenaltyArea);

  /**
   * Determine the radius of an obstacle.
   * @param type The type of the obstacle.
   * @return The radius with which the obstacle can be surrounded in mm. 0 if the obstacle should be ignored.
   */
  float getRadius(Obstacle::Type type) const;

  /**
   * Adds a node for an obstacle if it is inside the field and valid.
   * @param center The center of the obstacle.
   * @param radius The radius of the obstacle. If 0, it is ignored.
   */
  void addObstacle(const Vector2f& center, float radius);

  /**
   * Plan a shortest path. The result can be tracked backwards from the target node.
   * @param from The starting node. It is implicitly assumed that this is also the first entry in the vector "nodes".
   * @param to The target node.
   * @param speedRatio The ratio between forward speed and turn speed.
   */
  void plan(Node& from, Node& to, float speedRatio);

  /**
   * Expand a node during the A* search and add all suitable outgoing edges to the set of open edges.
   * @param node The node that is expanded.
   * @param to The overall target node. Required to calculate the heuristic.
   * @param rotation Only the outgoing edges with the same rotation are expanded.
   * @param speedRatio The ratio between forward speed and turn speed.
   */
  void expand(Node& node, const Node& to, Rotation rotation, float speedRatio);

  /**
   * Find all nodes reachable from this node without intersecting with other nodes, i.e. determine the outgoing edges.
   * @param node The node the outgoing edges of which are determined.
   */
  void findNeighbors(Node& node);

  /**
   * Create all tangents from one node to all other nodes. The number of tangents created per other node depends
   * on whether the nodes are circles or points (one for point to point, two for a point and a circle, four for two
   * circles) and whether they overlap (none if one node is inside the other one, two if they intersect, four if two
   * circles do not overlap). If another circle overlaps, the angular range of the overlap is also marked as being
   * blocked in the node passed, i.e. no tangents can start from this ranges.
   * @param node The node from which the tangents to all neighbors are created.
   * @param tangents The tangents found are returned here. Must be empty when passed. There are two sets of tangents,
   *                 i.e. the ones that start in clockwise direction and the ones that start in counter clockwise
   *                 direction. In addition, some tangents might be marked as dummies, because they are copies of
   *                 tangents in the other direction, but are needed by the sweepline algorithm that is later used.
   */
  void createTangents(Node& node, Tangents& tangents);

  /**
   * Add all outgoing edges of a node to that node based on the tangents to all other nodes. Do not add edges that
   * intersect with other nodes in between. This is determined using a sweepline algorithm that go through all
   * tangents in ascending angular direction and keeps track of all nodes in the current direction ordered by their
   * distance. Only the tangents to the closest node in each direction are accepted as outgoing edges. The is done
   * separately for outgoing edges in clockwise and counterclockwise directions.
   * @param node The starting node of the edges that are created.
   * @param tangents The tangents as produced by the method "createTangents".
   */
  void addNeighborsFromTangents(Node& node, Tangents& tangents);

  /**
   * Calculate motion request to follow the planed path.
   * @param target The target the robot tries to reach.
   * @param speed The speed the robot should walk with in ratios of the maximum speed.
   * @param edge The current edge that is traveled.
   * @param nextEdge The next edge to be traveled after the next node. This is nullptr if
   *                 there is no further edge anymore.
   * @param motionRequest The motion request that is calculated.
   */
  void calcMotionRequest(const Pose2f& target, const Pose2f& speed,
                         const Edge* edge, const Edge* nextEdge, MotionRequest& motionRequest);

  /** Some visualizations. */
  void draw() const;

public:
  /** The default constructor constructs the borders from the field dimensions. */
  PathPlannerProvider();
};
