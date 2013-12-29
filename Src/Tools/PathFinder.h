/**
 * @file PathFinder.h
 * @author Katharina Gillmann
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Modeling/CombinedWorldModel.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Configuration/FieldDimensions.h"


class PathFinder
{
public:

  /**
  * @class Node
  * a node of a path
  */
  class Node
  {
  public:
    Vector2<> position; //position of the current node
    int indexPreviousNode; // index of the previous node in the path

    Node() : indexPreviousNode(-1) {} // Constructor
    Node(const Vector2<>& position) : position(position), indexPreviousNode(-1) {}
  };

  PathFinder(const CombinedWorldModel& combinedWorldModel, const FieldDimensions& fieldDimensions,
    const RobotInfo& robotInfo, const GameInfo& gameInfo) :
    theCombinedWorldModel(combinedWorldModel),
    theFieldDimensions(fieldDimensions),
    theRobotInfo(robotInfo),
    theGameInfo(gameInfo),
    countNoPathFound(0)
  {
  }

  std::vector< Vector2<> > path; // current complete path
  float pathLength; // length of the path

  std::vector<Node> firstTree;
  std::vector<Node> secondTree;

  void findPath(const Vector2<>& startOfPath, const Vector2<>& endOfPath);
  void loadParameters();

  /**
  * @class Parameters
  * The parameters of the module
  */
  STREAMABLE(Parameters,
  {,
    (float)(200.f) stepSize,                  /**< The length of the new added edge */
    (float)(400.f) distancePathCombination,   /**<max distance between two pathes for combining them > */
    (float)(500.f) distanceToObstacle,        /**<needed distance between robot and obstacle> */
    (float)(200.f) distanceCloseToObstacle,   /**<close distance of positions edge and obstacle> */
    (float)(0.3f) targetProbability,         /**<probability for using target as random position> */
    (float)(0.7f) wayPointsProbability,      /**<probability for using old wayPoints as random position> */
    (int)(1000) countOfCycles,               /**<count of cycles for creating one way> */
    (float)(1.5f) searchSpaceFactor,         /**<factor for the search area in which random positions can be found>*/
    (int)(200) counterUseOldPath,           /**<counter until old path is used if no new one was found>*/
    (bool)(false) enterPenaltyAreaInReady,    /**<flag the enables/disables the penalty area obstacle*/
  });

  Parameters parameters; /**< The parameters of this module */

private:
  const CombinedWorldModel& theCombinedWorldModel;
  const FieldDimensions& theFieldDimensions;
  const RobotInfo& theRobotInfo;
  const GameInfo& theGameInfo;

  std::vector<Node> completePath; // the complete path found in the last run
  int countNoPathFound;

  void createRandomPosition(Vector2<>& randomPosition, const std::vector<Node>& currentNotUsedTree);
  bool checkForObstaclesNearPosition(const std::vector<Node>& allObstacles, Vector2<>& positionOfObstacle, const Vector2<>& position, float& smallestDistance);
  void calculateNearestNode(const std::vector<Node>& currentUsedTree, int& indexNearestNode, const Vector2<>& randomPosition);
  bool checkForCollision(const std::vector<Node>& allObstacles, const Vector2<>& nearestNode, const Vector2<>& randomPosition, bool usePosition, float& distanceToObstacle, Vector2<>& positionObstacle, const Vector2<>& target);
  void checkForFoundPath(const Vector2<>& currentUsedNode, const std::vector<Node>& currentNotUsedTree, bool& foundPath, bool& foundPathInFirstTree, int& indexOtherTree, const bool useFirstTree);
  void createNewNode(std::vector<Node>& currentUsedTree, const Vector2<> randomPosition, const int indexNearestNode);
  void createCompletePath(std::vector<Node>& completePath, const bool foundPathInFirstTree, const std::vector<Node>& firstTree, const std::vector<Node>& secondTree, const int indexOtherTree, const bool ObstacleStart, const bool ObstacleEnd, const Vector2<>& oldStart, const Vector2<>& oldEnd);
  void addAvoidingPoints(const std::vector<Node>& allObstacles, std::vector<Node>& currentUsedTree, const Vector2<>& position, const Vector2<>& positionOfObstacle, bool nearestObstacle, const Node& target, const bool startIsUsed);

  bool checkForObstaclesNearPositionMMX(const std::vector<Node>& allObstacles, Vector2<>& positionOfObstacle, const Vector2<>& position, float& smallestDistance);
  void checkForFoundPathMMX(const Vector2<>& currentUsedNode, const std::vector<Node>& currentNotUsedTree, bool& foundPath, bool& foundPathInFirstTree, int& indexOtherTree, const bool useFirstTree);
  bool checkForCollisionMMX(const std::vector<Node>& allObstacles, const Vector2<>& nearestNode, const Vector2<>& randomPosition, bool usePosition, float& distanceToObstacle, Vector2<>& positionObstacle, const Vector2<>& target);
  void calculateNearestNodeMMX(const std::vector<Node>& currentUsedTree, int& indexNearestNode, const Vector2<>& randomPosition);

};
