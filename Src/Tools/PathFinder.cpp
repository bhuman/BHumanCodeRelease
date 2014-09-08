/**
* @file Tools/PathFinder.cpp
* Implementation of a class that provides methodes for planing a path.
* @author Katharina Gillmann
*/

#include "Tools/PathFinder.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Random.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Streams/InStreams.h"
#include <cstdlib>
#include <cfloat>
#include <algorithm>

using namespace std;

void PathFinder::loadParameters()
{
  InMapFile stream("pathFinder.cfg");
  if(stream.exists())
  {
    stream >> parameters;
  }
}

void PathFinder::findPath(const Vector2<>& startOfPath, const Vector2<>& endOfPath)
{
  MODIFY("tools:PathFinder:parameters", parameters);

  bool foundPath = false;
  bool useFirstTree = true;
  bool foundPathInFirstTree = true;
  int indexOtherTree = 0;

  firstTree.clear();
  secondTree.clear();
  path.clear();
  pathLength = 0.0;

  // original start and end position. Needed if position is near obstacle so that start or/and end position for path are moved
  Vector2<> oldStart = startOfPath;
  Vector2<> oldEnd = endOfPath;

  Node startPosition(startOfPath);
  Node endPosition(endOfPath);

  Vector2<> positionOfObstacleStart;
  Vector2<> positionOfObstacleEnd;
  float smallestDistanceStart;
  float smallestDistanceEnd;

  vector<Node> allObstacles; // puts all obstacles into one vector
  for(std::vector<GaussianPositionDistribution>::const_iterator i = theCombinedWorldModel.positionsOpponentTeam.begin(), end = theCombinedWorldModel.positionsOpponentTeam.end(); i != end; ++i)
  {
    allObstacles.push_back(Node(i->robotPosition));
  }
  for(std::vector<Pose2D>::const_iterator i = theCombinedWorldModel.positionsOwnTeam.begin(), end = theCombinedWorldModel.positionsOwnTeam.end(); i != end; ++i)
  {
    if((i->translation - startOfPath).squareAbs() > sqr(100) && (i->translation - endOfPath).squareAbs() > sqr(100)) // own robot is not added to all obstacles
    {
      allObstacles.push_back(Node(i->translation));
    }
  }

  bool startNearObstacle;
  bool targetNearObstacle;
  startNearObstacle = checkForObstaclesNearPosition(allObstacles, positionOfObstacleStart, startPosition.position, smallestDistanceStart); // checks if start position is near obstacle
  targetNearObstacle = checkForObstaclesNearPosition(allObstacles, positionOfObstacleEnd, endPosition.position, smallestDistanceEnd); // checks if end position is near obstacle
  bool smallDistance = (startPosition.position - endPosition.position).abs() <= parameters.distancePathCombination; // if distance between positions is very small
  bool inSameObstacle = startNearObstacle && targetNearObstacle && positionOfObstacleStart == positionOfObstacleEnd; // if positions are inside the same direct obstacle and connection is possible
  bool inOverlappingObstacles = startNearObstacle && targetNearObstacle && positionOfObstacleStart != positionOfObstacleEnd && (positionOfObstacleStart - positionOfObstacleEnd).abs() < 2 * parameters.distanceToObstacle; // if positions are in different connected obstacles and direct position is possible
  if(smallDistance || inSameObstacle || inOverlappingObstacles) // if start and end position can still be connected
  {
    bool collides = false;
    float distanceToObstacle;
    Vector2<> positionObstacle;
    collides = checkForCollision(allObstacles, startPosition.position, endPosition.position, true, distanceToObstacle, positionObstacle, endPosition.position);
    Vector2<> directionObstacle = positionObstacle - startPosition.position;
    Vector2<> directionPositions = endPosition.position - startPosition.position;
    float angleStart = toDegrees(acos(directionObstacle * directionPositions / (directionObstacle.abs() * directionPositions.abs())));// calculate angle of position to found obstacle.
    directionObstacle = positionObstacle - endPosition.position;
    directionPositions = startPosition.position - endPosition.position;
    float angleEnd = toDegrees(acos(directionObstacle * directionPositions / (directionObstacle.abs() * directionPositions.abs())));
    foundPath = !collides || (collides && (distanceToObstacle > parameters.distanceCloseToObstacle || angleStart > 80 || angleEnd > 80)); // direct connection is made if no collision exists, the distance to obstacle is high enough or if position is close to the obstacle and the angle to the obstacle is > 80° (because obstacle doesn't disturb the connection, although collision is detected)
  }

  if(foundPath) // use normal start and end position
  {
    firstTree.push_back(startPosition);
    secondTree.push_back(endPosition);
    startNearObstacle = false;
    targetNearObstacle = false;
  }
  else // if necessary use positions outside the obstacle
  {
    for(unsigned int i = 0; i < 2; i++) // aid for avoiding diplicated code
    {
      Vector2<> positionOfObstacle = i == 0 ? positionOfObstacleStart : positionOfObstacleEnd;
      Node position = i == 0 ? startPosition : endPosition;
      Node otherPosition = i == 0 ? endPosition : startPosition;
      vector<Node>& usedTree = i == 0 ? firstTree : secondTree;
      bool& nearObstacle = i == 0 ? startNearObstacle : targetNearObstacle;
      if(nearObstacle)
      {
        addAvoidingPoints(allObstacles, usedTree, position.position, positionOfObstacle, otherPosition, i == 0); // calculate avoiding points if position is inside obstacle
      }
      else
      {
        usedTree.push_back(position); // use position itself if it is not near an obstacle
      }
    }
  }
  int countOfCycles = 0;
  while(!foundPath && countOfCycles <= parameters.countOfCycles) // found path from start to end position
  {
    vector<Node>& currentUsedTree = useFirstTree ? firstTree : secondTree; // decision wich tree will be expanded
    vector<Node>& currentNotUsedTree = useFirstTree ? secondTree : firstTree;
    Vector2<> randomPosition;
    createRandomPosition(randomPosition, currentNotUsedTree); // creates random position
    int indexNearestNode = 0;
    float distanceToObstacle;
    Vector2<> positionObstacle;
    calculateNearestNode(currentUsedTree, indexNearestNode, randomPosition); // calculates the nearest node of the tree towards the random position
    bool collides;
    collides = checkForCollision(allObstacles, currentUsedTree[indexNearestNode].position, randomPosition, false, distanceToObstacle, positionObstacle, endPosition.position); // checks if the new node collidates with an obstacle
    if(!collides)
    {
      createNewNode(currentUsedTree, randomPosition, indexNearestNode); // creates the new node
      checkForFoundPath(currentUsedTree.back().position, currentNotUsedTree, foundPath, foundPathInFirstTree, indexOtherTree, useFirstTree); // checks if a complete path was found
      useFirstTree = !useFirstTree; // is changed to use the other tree during the next cycle
    }
    countOfCycles++;
  }

  if(foundPath) // use new path if one was found
  {
    countNoPathFound = 0;
    createCompletePath(completePath, foundPathInFirstTree, firstTree, secondTree, indexOtherTree, startNearObstacle, targetNearObstacle, oldStart, oldEnd); // creates found path
  }
  else if(countNoPathFound < parameters.counterUseOldPath && completePath.size() != 0) // use old path if no new path was found
  {
    countNoPathFound++;
    vector<PathFinder::Node> temp = completePath;
    int index = -1;
    float currentDistance = FLT_MAX;
    int size = completePath.size() <= 11 ? (int) completePath.size() - 2 : 10;
    for(int i = 0; i <= size; i++)
    {
      Geometry::Line line(completePath[i].position, completePath[i + 1].position - completePath[i].position);
      Vector2<> rotatedDirection = line.direction;
      rotatedDirection.rotateLeft();

      Vector2<> positionRelToBase = startOfPath - line.base;
      bool firstNodeIsRelevant = (rotatedDirection.x * positionRelToBase.y - positionRelToBase.x * rotatedDirection.y) >= 0;
      float intersect = Geometry::getDistanceToEdge(line, startOfPath);
      if(intersect < currentDistance)
      {
        if(firstNodeIsRelevant)
        {
          index = i;
        }
        else
        {
          index = i + 1;
        }
        currentDistance = intersect;
      }
    }
    completePath.clear();
    for(unsigned int i = index; i < temp.size(); i++)
    {
      completePath.push_back(temp[i]);
    }
  }
  else // if no path is available
  {
    completePath.clear();
    completePath.push_back(startOfPath);
    completePath.push_back(endOfPath);
  }
  for(unsigned int i = 0; i < completePath.size(); i++)
  {
    path.push_back(completePath[i].position);// adds complete path to the representation
    if(i != 0)
    {
      pathLength += (completePath[i].position - completePath[i - 1].position).abs();
    }
  }
}

void PathFinder::createRandomPosition(Vector2<>& randomPosition, const vector<Node>& currentNotUsedTree)
{
  float probabilityTarget = parameters.targetProbability;
  float probabilityWayPoint = parameters.wayPointsProbability;
  float rand = randomFloat();
  if(rand <= probabilityTarget) // use target as random point
  {
    randomPosition = currentNotUsedTree[0].position;
  }
  else if(rand <= probabilityWayPoint && completePath.size() != 0) // use old waypoint as random point if it exists
  {
    int rand = random((short)completePath.size());
    randomPosition = completePath[rand].position;
  }
  else
  {
    randomPosition.x = randomFloat() * ((theFieldDimensions.xPosOpponentFieldBorder * parameters.searchSpaceFactor) - (theFieldDimensions.xPosOwnFieldBorder * parameters.searchSpaceFactor)) + (theFieldDimensions.xPosOwnFieldBorder * parameters.searchSpaceFactor); // finds a random position on field, random points outside the field are used, so that the whole field can be expanded. Path is clipped on field later
    randomPosition.y = randomFloat() * ((theFieldDimensions.yPosLeftFieldBorder * parameters.searchSpaceFactor) - (theFieldDimensions.yPosRightFieldBorder * parameters.searchSpaceFactor)) + (theFieldDimensions.yPosRightFieldBorder * parameters.searchSpaceFactor);
  }
}

void PathFinder::calculateNearestNode(const vector<Node>& currentUsedTree, int& indexNearestNode, const Vector2<>& randomPosition)
{
  float distance = FLT_MAX;
  for(unsigned int i = 0; i < currentUsedTree.size(); i++) // finds the node in the tree which is nearest to the random position
  {
    float distanceBetweenPositions = (randomPosition - currentUsedTree[i].position).abs();
    if(distanceBetweenPositions < distance)
    {
      indexNearestNode = i;
      distance = distanceBetweenPositions;
    }
  }
}

bool PathFinder::checkForCollision(const vector<Node>& allObstacles, const Vector2<>& nearestNode, const Vector2<>& randomPosition, bool usePosition, float& distanceToObstacle, Vector2<>& positionObstacle, const Vector2<>& target)
{
  bool collides = false;
  float currentDistance = parameters.distanceToObstacle;
  Vector2<> newPosition;
  if((nearestNode - randomPosition).abs() < parameters.stepSize || usePosition) // use hand over/ random position itself
  {
    newPosition = randomPosition;
  }
  else
  {
    newPosition = nearestNode + (randomPosition - nearestNode).normalize(parameters.stepSize); // calculate endPosition
  }
  if(std::abs(newPosition.x) > (theFieldDimensions.xPosOpponentGroundline + (theFieldDimensions.xPosOpponentFieldBorder - theFieldDimensions.xPosOpponentGroundline) / 2) || std::abs(newPosition.y) > (theFieldDimensions.yPosLeftSideline + (theFieldDimensions.yPosLeftFieldBorder - theFieldDimensions.yPosLeftSideline) / 2)) // if new node is outside the field
  {
    collides = true;
    return collides;
  }
  // path should be around the own penalty area for avoiding illegal defenders (Only for field players):
  if(newPosition.x < theFieldDimensions.xPosOwnPenaltyArea + 200 && std::abs(newPosition.y) < theFieldDimensions.yPosLeftPenaltyArea + 200 && theRobotInfo.number != 1)
  {
    if(!(theGameInfo.state == STATE_READY && parameters.enterPenaltyAreaInReady && newPosition.x > theFieldDimensions.xPosOwnPenaltyArea - 200))
    {
      if(theRole.role != Role::striker || Global::getSettings().isDropInGame)
      {
        collides = true;
        return collides;
      }
      else
      {
        // Striker is allowed to go to the penalty area if there is no other nonKeeper inside
        for(int i = TeammateData::firstPlayer; i < TeammateData::numOfPlayers; ++i)
        {
          if(i != theRobotInfo.number && theTeammateData.isActive[i] && theTeammateData.behaviorStatus[i].role != Role::keeper
             && theTeammateData.robotPoses[i].translation.x < theFieldDimensions.xPosOwnPenaltyArea + 50.f
             && std::abs(theTeammateData.robotPoses[i].translation.y) < theFieldDimensions.yPosLeftPenaltyArea + 50.f)
          {
            collides = true;
            return collides;
          }
        }
      }
    }
  }
  Geometry::Line segmentOfPath(nearestNode, newPosition - nearestNode); // calculate line between used points
  Vector2<> rotatedDirection = segmentOfPath.direction;
  rotatedDirection.rotateLeft();
  for(std::vector<Node>::const_iterator i = allObstacles.begin(), end = allObstacles.end(); i != end; ++i)
  {
    Vector2<> obstacleRelToBase = i->position - segmentOfPath.base;
    bool obstacleIsRelevant = (rotatedDirection.x * obstacleRelToBase.y - obstacleRelToBase.x * rotatedDirection.y) < 0; // if obstacle is between the positions. Obstacles behind the posititons (not between) are not relevant
    float obstacleDistance = Geometry::getDistanceToEdge(segmentOfPath, i->position);
    if(obstacleDistance < currentDistance && obstacleIsRelevant) // finds the nearest relevant obstacle
    {
      currentDistance = obstacleDistance;
      positionObstacle = i->position;
      collides = true;
    }
  }
  distanceToObstacle = currentDistance;
  return collides;
}

void PathFinder::checkForFoundPath(const Vector2<>& currentUsedNode, const vector<Node>& currentNotUsedTree, bool& foundPath, bool& foundPathInFirstTree, int& indexOtherTree, const bool useFirstTree)
{
  float distanceBetweenTrees = sqr(parameters.distancePathCombination);  // squared for faster comparison
  for(unsigned int i = 0; i < currentNotUsedTree.size(); i++) // checks if the distance between two nodes of both trees is small enough to connect the trees.
  {
    const float distance = (currentNotUsedTree[i].position - currentUsedNode).squareAbs();
    if(distance < distanceBetweenTrees)
    {
      distanceBetweenTrees = distance;
      indexOtherTree = i;
      foundPath = true;
      foundPathInFirstTree = useFirstTree;
    }
  }
}

void PathFinder::createNewNode(vector<Node>& currentUsedTree, const Vector2<> randomPosition, const int indexNearestNode)
{
  Vector2<> positionNewNode;
  if((currentUsedTree[indexNearestNode].position - randomPosition).abs() < parameters.stepSize)
  {
    positionNewNode = randomPosition;
  }
  else
  {
    positionNewNode = currentUsedTree[indexNearestNode].position + (randomPosition - currentUsedTree[indexNearestNode].position).normalize(parameters.stepSize);
  }

  Node newNode(positionNewNode); // creates new node for the tree. The position of the new node is between the nearest node and the random position
  newNode.indexPreviousNode = indexNearestNode;
  currentUsedTree.push_back(newNode);
}

void PathFinder::createCompletePath(vector<Node>& completePath, const bool foundPathInFirstTree, const vector<Node>& firstTree, const vector<Node>& secondTree, const int indexOtherTree, const bool obstacleStart, const bool obstacleEnd, const Vector2<>& oldStart, const Vector2<>& oldEnd)
{
  const Node* currentNodeFirstTree = foundPathInFirstTree ? &firstTree.back() : &firstTree[indexOtherTree];
  const Node* currentNodeSecondTree = foundPathInFirstTree ? &secondTree[indexOtherTree] : &secondTree.back();
  completePath.clear();

  while(currentNodeFirstTree->indexPreviousNode != -1) // adds the found path within the first tree to the complete path
  {
    completePath.push_back(*currentNodeFirstTree);
    currentNodeFirstTree = &firstTree[currentNodeFirstTree->indexPreviousNode];
  }
  completePath.push_back(*currentNodeFirstTree);

  if(obstacleStart)
  {
    completePath.push_back(oldStart); // add original start position
  }

  std::reverse(completePath.begin(), completePath.end()); // current complete path needs to be reversed because the first path was added in reversed order

  while(currentNodeSecondTree->indexPreviousNode != -1) // adds the found path within the second tree to the complete path
  {
    completePath.push_back(*currentNodeSecondTree);
    currentNodeSecondTree = &secondTree[currentNodeSecondTree->indexPreviousNode];
  }
  completePath.push_back(*currentNodeSecondTree);

  if(obstacleEnd)
  {
    completePath.push_back(oldEnd);
  }
}

bool PathFinder::checkForObstaclesNearPosition(const vector<Node>& allObstacles, Vector2<>& positionOfObstacle, const Vector2<>& position, float& smallestDistance)
{
  float currentDistance = parameters.distanceToObstacle;
  bool foundObstacle = false;
  float distance;
  for(std::vector<Node>::const_iterator i = allObstacles.begin(), end = allObstacles.end(); i != end; ++i)
  {
    distance = (position - i->position).abs();
    if(distance < currentDistance && i->position != position) // finds the nearest obstacle to the current position
    {
      positionOfObstacle = i->position;
      currentDistance = distance;
      foundObstacle = true;
    }
  }
  smallestDistance = currentDistance;
  return foundObstacle;
}

void PathFinder::addAvoidingPoints(const vector<Node>& allObstacles, vector<Node>& currentUsedTree, const Vector2<>& position, const Vector2<>& positionOfObstacle, const Node& target, const bool startIsUsed)
{
  int degree = 60;
  Vector2<> originalPosition = position;
  float length;
  Vector2<> difference;
  if(position == positionOfObstacle)// special handling if obstacle = position
  {
    length = parameters.distanceToObstacle;
    difference = Vector2<>(parameters.distanceToObstacle, 0.0f);
  }
  else
  {
    difference = position - positionOfObstacle;
    difference.normalize(parameters.distanceToObstacle - difference.abs());
    float hypotenuse = (position - positionOfObstacle).abs();
    float opposite = sin(fromDegrees(degree)) * hypotenuse;
    float adjacent = sqrt(sqr(hypotenuse) - sqr(opposite));
    length = sqrt((parameters.distanceToObstacle - opposite) * (parameters.distanceToObstacle + opposite)); // length based on "Höhensatz des Euklids"
    length -= adjacent;
    //length = sqrt((2 * parameters.distanceToObstacle - difference.abs()) * difference.abs()); // length based on "Höhensatz des Euklid"
  }

  const int arraySize = 3;
  float distances[arraySize];
  Node nodes[arraySize];
  bool collisions[arraySize];
  Vector2<> obstacles[arraySize];

  nodes[0].position = (position + difference);//own direction
  difference.rotate(fromDegrees(degree));
  difference.normalize(length);
  nodes[1].position = position + difference;// rotated 70° from original avoiding position, based on start/end position
  difference.rotate(fromDegrees(-(degree * 2)));
  nodes[2].position = position + difference;// rotated -70° from original avoiding position, 180° is not used because obstacle would be between position and new node

  collisions[0] = checkForObstaclesNearPosition(allObstacles, obstacles[0], nodes[0].position, distances[0]);// check for new collisions for all avoiding points
  collisions[1] = checkForObstaclesNearPosition(allObstacles, obstacles[1], nodes[1].position, distances[1]);
  collisions[2] = checkForObstaclesNearPosition(allObstacles, obstacles[2], nodes[2].position, distances[2]);

  if(collisions[0] && collisions[1] && collisions[2] && obstacles[0] != positionOfObstacle && obstacles[1] != positionOfObstacle && obstacles[2] != positionOfObstacle && obstacles[0] != originalPosition && obstacles[1] != originalPosition && obstacles[2] != originalPosition) // if all avoiding points lead in new obstacles use all points
  {
    for(int i = 0; i < arraySize; i++)
    {
      currentUsedTree.push_back(nodes[i]);
    }
  }
  else if(startIsUsed) // use special avoiding points for moving point of path
  {
    Node newPosition;
    float currentDistance = FLT_MAX;
    for(int i = 0; i < arraySize; i++) // use only points which don't lead in another obstacle or where the distance to the new obstacle is high enough. Position is also used if collision exists but only whith the avoided obstacle or the own position
    {
      if(!collisions[i] || (collisions[i] && (obstacles[i] == positionOfObstacle || obstacles[i] == originalPosition || distances[i] > parameters.distanceCloseToObstacle)))
      {
        float distance = (nodes[i].position - target.position).abs();
        if(distance < currentDistance)
        {
          currentDistance = distance;
          newPosition = nodes[i];
        }
      }
    }
    currentUsedTree.push_back(newPosition);
  }
  else
  {
    for(int i = 0; i < arraySize; i++) // use only points which don't lead in another obstacle or where the distance to the new obstacle is high enough. Position is also used if collision exists but only whith the avoided obstacle or the own position
    {
      if(!collisions[i] || (collisions[i] && (obstacles[i] == positionOfObstacle || obstacles[i] == originalPosition || distances[i] > parameters.distanceCloseToObstacle)))
      {
        currentUsedTree.push_back(nodes[i]);
      }
    }
  }
}
