/**
 * @file Modules/BehaviorControl/VGPathPlanner/VGPathPlanner.h
 *
 * A module that plans a path from the current position to a target position if requested
 * so by the behavior. Otherwise, it just passes through the motion request.
 * It models the planing problem using a visibility graph.
 *
 * @author Thomas Röfer
 */

#include "Tools/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/MotionControl/MotionRequest.h"

MODULE(VGPathPlanner,
{,
  REQUIRES(BehaviorMotionRequest),
  REQUIRES(BallModel),
  REQUIRES(FrameInfo),
  REQUIRES(ObstacleModel),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(TeamPlayersModel),
  PROVIDES(MotionRequest),
  DEFINES_PARAMETERS(
  {,
    (bool)(false) useObstacles, /**< Use TeamPlayersModel or ObstacleModel? */
    (float)(350.f) goalPostRadius, /**< Radius to walk around a goal post (in mm). */
    (float)(500.f) uprightRobotRadius, /**< Radius to walk around an upright robot (in mm). */
    (float)(550.f) fallenRobotRadius, /**< Radius to walk around a fallen robot (in mm). */
    (float)(150.f) penaltyAreaRadius, /**< Radius to walk around a corner of the own penaly area (in mm). */
    (float)(250.f) ballRadius, /**< Radius to walk around the ball (in mm). */
    (int)(3000) ballValidDelay, /**< How long is the ball avoided after its last perception (in ms). */
    (float)(350.f) fieldBorderLimit, /**< Distance outside the side lines that is still used for walking (in mm). */
    (float)(100.f) startTurningBeforeTargetDistance, /**< How far before the target is started to turn to final direction (in mm)? */
    (float)(200.f) startTurningBeforeCircleDistance, /**< How far before the next circle is switched to walking a curve (in mm)? */
    (float)(100.f) radiusControlOffset, /**< Plan closer to obstacles by this offset, but keep original distance when executing plan (in mm). */
    (float)(100.f) radiusAvoidanceTolerance, /**< Radius range in which robot is partially pushed away (in mm). */
    (float)(0.4f) fullTranslationThreshold, /**< Plan closer to obstacles by this offset, but keep original distance when executing plan (in mm). */
    (float)(0.9f) noTranslationThreshold, /**< How wide is the corridor between surrounding and avoiding obstacles behavior (in mm)? */
    (float)(0.3f) controlAheadAngle, /**< Put control point how far ahead on circles (in radians). */
    (float)(150.f) rotationPenalty, /**< Penalty factor for rotating towards first intermediate target in mm/radian. Stabilizes path selection. */
    (float)(400.f) switchPenalty, /**< Penalty for selecting a different turn direction around first obstacle in mm. */
    (float)(0.5f) alignAgainThreshold, /**< Above this deviation from the desired direction, the robot turns. */
    (float)(0.1f) alignedThreshold, /**< Below this deviation from the desired direction, the robot stops turning. */
    (float)(1.f) pFactor, /**< How fast to turn? */
  }),
});

class VGPathPlanner : public VGPathPlannerBase
{
  /** Obstacles can be surrounded in clockwise or counterclockwise direction. */
  ENUM(Rotation,
  {,
    cw,
    ccw,
  });

  struct Node;

  /** The edges of the visiblity graph. */
  struct Edge
  {
    Node* fromNode; /**< The node from which this edge starts. */
    Node* toNode; /**< The node at which this edge ends. */
    float fromAngle; /**< The angle where this edge touches the circle around fromNode. */
    Vector2f toPoint; /**< The point where this edge touches the circle of toNode. */
    Rotation fromRotation; /**< The rotation with which fromNode was surrounded. */
    Rotation toRotation;  /**< The rotation with which toNode will be surrounded. */
    float length; /**< The length of this edge. */
    float pathLength; /**< The overall length of the path until arriving at toNode. Will be set by A* seach. */

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

  /** The nodes of the visibility graph, i.e. the obstacles. */
  struct Node : public Geometry::Circle
  {
    std::vector<Edge> edges[numOfRotations]; /** The outgoing edges per rotation. */
    std::vector<Rangef> blockedSectors; /**< Angular sectors that are blocked by overlapping other obstacles. */
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

    /**
     * Constructor.
     * @param x1 The x coordinate of first end point of the barrier.
     * @param y1 The y coordinate of first end point of the barrier.
     * @param x2 The x coordinate of second end point of the barrier.
     * @param y2 The y coordinate of second end point of the barrier.
     */
    Barrier(float x1, float y1, float x2, float y2) : from(x1, y1), to(x2, y2) {}

    /**
     * Does a line intesect this barrier?
     * @param p1 The first end point of the line.
     * @param p2 The second end point of the line.
     * @return Do they intersect?
     */
    bool intersects(const Vector2f& p1, const Vector2f& p2) const
    {
      return Geometry::checkIntersectionOfLines(from, to, p1, p2);
    }
  };

  std::vector<Node> nodes; /**< All nodes of the visibility graph, i.e. all obstacles, and starting point (1st entry) and target (2nd entry). */
  std::vector<Candidate> candidates; /**< All open edges during the A* search. */
  std::vector<Barrier> barriers; /**< Barrier lines that cannot be crossed during planning. */
  std::vector<Geometry::Line> borders; /**< The border of the field plus a tolerance. */
  bool walkStraight = false; /**< Currently walking straight? */
  Rotation lastDir = cw; /**< Last direction selected when walking around first obstacle. */
  unsigned timeWhenLastPlayedSound = 0; /**< Used to limit frequency of sound playback. */

  /**
   * Either pass through the BehaviorMotionRequest or plan a path to the target and
   * execute the first step of it.
   * @param motionRequest The MotionRequest that is provided.
   */
  void update(MotionRequest& motionRequest);

  /**
   * Compute barrier lines that cannot be crossed during planning.
   * @param excludePenaltyArea Also generate barriers for the own penalty area.
   */
  void createBarriers(bool excludePenaltyArea);

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
   * @param excludePenaltyArea Filter out obstacles inside the own penalty area and the own goal.
   */
  void createNodes(bool excludePenaltyArea);

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
   * @param from The starting node. It is implicitely assumed that this is also the first entry in the vector "nodes".
   * @param to The target node.
   */
  void plan(Node& from, Node& to);

  /**
   * Expand a node during the A* search and add all suitable outgoing edges to the set of open edges.
   * @param node The node that is expanded.
   * @param to The overall target node. Required to calculate the heuristic.
   * @param rotation Only the outgoing edges with the same rotation are expanded.
   */
  void expand(Node& node, const Node& to, Rotation rotation);

  /**
   * Find all nodes reachable from this node without intersecting with other nodes, i.e. determine the outgoing edges.
   * @param node The node the outgoing edges of which are determined.
   */
  void findNeighbors(Node& node);

  /**
   * Calculate motion request to follow the planed path.
   * @param edge The current edge that is traveled.
   * @param nextEdge The next edge to be traveled after the next node. This is nullptr if
   *                 there is no further edge anymore.
   * @param motionRequest The motion request that is calculated.
   */
  void calcMotionRequest(const Edge* edge, const Edge* nextEdge, MotionRequest& motionRequest);

  /** Some visualizations. */
  void draw() const;

public:
  /** The default constructor constructs the borders from the field dimensions. */
  VGPathPlanner();
};
