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
#undef TARGET_ROBOT
#ifdef TARGET_ROBOT
#include <mm3dnow.h>
#endif

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
  bool nearestNeighbourMMX = true;
  bool foundPathMMX = true;
  bool checkCollisionsMMX = true;
  bool nearestObstacleMMX = true;
  MODIFY("tools:Pathfinder:useMMX:NearestNeighbourMMX", nearestNeighbourMMX);
  MODIFY("tools:Pathfinder:useMMX:foundPathMMX", foundPathMMX);
  MODIFY("tools:Pathfinder:useMMX:checkCollisionsMMX", checkCollisionsMMX);
  MODIFY("tools:Pathfinder:useMMX:NearestObstacleMMX", nearestObstacleMMX);
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
#ifdef TARGET_ROBOT
  if(nearestObstacleMMX)
  {
    startNearObstacle = checkForObstaclesNearPositionMMX(allObstacles, positionOfObstacleStart, startPosition.position, smallestDistanceStart); // checks if start position is near obstacle
    targetNearObstacle = checkForObstaclesNearPositionMMX(allObstacles, positionOfObstacleEnd, endPosition.position, smallestDistanceEnd); // checks if end position is near obstacle
  }
  else
  {
    startNearObstacle = checkForObstaclesNearPosition(allObstacles, positionOfObstacleStart, startPosition.position, smallestDistanceStart); // checks if start position is near obstacle
    targetNearObstacle = checkForObstaclesNearPosition(allObstacles, positionOfObstacleEnd, endPosition.position, smallestDistanceEnd); // checks if end position is near obstacle
  }
#else
  startNearObstacle = checkForObstaclesNearPosition(allObstacles, positionOfObstacleStart, startPosition.position, smallestDistanceStart); // checks if start position is near obstacle
  targetNearObstacle = checkForObstaclesNearPosition(allObstacles, positionOfObstacleEnd, endPosition.position, smallestDistanceEnd); // checks if end position is near obstacle
#endif
  bool smallDistance = (startPosition.position - endPosition.position).abs() <= parameters.distancePathCombination; // if distance between positions is very small
  bool inSameObstacle = startNearObstacle && targetNearObstacle && positionOfObstacleStart == positionOfObstacleEnd; // if positions are inside the same direct obstacle and connection is possible
  bool inOverlappingObstacles = startNearObstacle && targetNearObstacle && positionOfObstacleStart != positionOfObstacleEnd && (positionOfObstacleStart - positionOfObstacleEnd).abs() < 2 * parameters.distanceToObstacle; // if positions are in different connected obstacles and direct position is possible
  if(smallDistance || inSameObstacle || inOverlappingObstacles) // if start and end position can still be connected
  {
    bool collides = false;
    float distanceToObstacle;
    Vector2<> positionObstacle;
#ifdef TARGET_ROBOT
    if(checkCollisionsMMX)
    {
      collides = checkForCollisionMMX(allObstacles, startPosition.position, endPosition.position, true, distanceToObstacle, positionObstacle, endPosition.position);
    }
    else
    {
      collides = checkForCollision(allObstacles, startPosition.position, endPosition.position, true, distanceToObstacle, positionObstacle, endPosition.position);
    }
#else
    collides = checkForCollision(allObstacles, startPosition.position, endPosition.position, true, distanceToObstacle, positionObstacle, endPosition.position);
#endif
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
        addAvoidingPoints(allObstacles, usedTree, position.position, positionOfObstacle, nearestObstacleMMX, otherPosition, i == 0); // calculate avoiding points if position is inside obstacle
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
#ifdef TARGET_ROBOT
    if(nearestNeighbourMMX)
    {
      calculateNearestNodeMMX(currentUsedTree, indexNearestNode, randomPosition); // calculates the nearest node of the tree towards the random position
    }
    else
    {
      calculateNearestNode(currentUsedTree, indexNearestNode, randomPosition); // calculates the nearest node of the tree towards the random position
    }
#else
    calculateNearestNode(currentUsedTree, indexNearestNode, randomPosition); // calculates the nearest node of the tree towards the random position
#endif
    bool collides;
#ifdef TARGET_ROBOT
    if(checkCollisionsMMX)
    {
      collides = checkForCollisionMMX(allObstacles, currentUsedTree[indexNearestNode].position, randomPosition, false, distanceToObstacle, positionObstacle, endPosition.position); // checks if the new node collidates with an obstacle
    }
    else
    {
      collides = checkForCollision(allObstacles, currentUsedTree[indexNearestNode].position, randomPosition, false, distanceToObstacle, positionObstacle, endPosition.position); // checks if the new node collidates with an obstacle
    }
#else
    collides = checkForCollision(allObstacles, currentUsedTree[indexNearestNode].position, randomPosition, false, distanceToObstacle, positionObstacle, endPosition.position); // checks if the new node collidates with an obstacle
#endif
    if(!collides)
    {
      createNewNode(currentUsedTree, randomPosition, indexNearestNode); // creates the new node
#ifdef TARGET_ROBOT
      if(foundPathMMX)
      {
        checkForFoundPathMMX(currentUsedTree.back().position, currentNotUsedTree, foundPath, foundPathInFirstTree, indexOtherTree, useFirstTree); // checks if a complete path was found
      }
      else
      {
        checkForFoundPath(currentUsedTree.back().position, currentNotUsedTree, foundPath, foundPathInFirstTree, indexOtherTree, useFirstTree); // checks if a complete path was found
      }
#else
      checkForFoundPath(currentUsedTree.back().position, currentNotUsedTree, foundPath, foundPathInFirstTree, indexOtherTree, useFirstTree); // checks if a complete path was found
#endif
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
    int size = completePath.size() <= 11 ? completePath.size() - 2 : 10;
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

void PathFinder::calculateNearestNodeMMX(const vector<Node>& currentUsedTree, int& indexNearestNode, const Vector2<>& randomPosition)
{
#ifdef TARGET_ROBOT
  _m_femms();
  const unsigned int vectorSize = currentUsedTree.size();
  ASSERT(vectorSize > 0);
  if(vectorSize == 1)
  {
    indexNearestNode = 0;
    return;
  }
  const Node* firstElem = &currentUsedTree[0];
  __m64 mRandomPosition = *((__m64*) & (randomPosition.x)); // loads randomPositon into register, *((__m64*)&(randomPosition.x)) is the beginning of the vector
  __m64 m0thElement = *((__m64*) & (firstElem[0].position.x)); // loads first element into register
  __m64 m1thElement = *((__m64*) & (firstElem[1].position.x)); // loads second element into register
  __m64 dist0th = _m_pfsub(m0thElement, mRandomPosition); // difference between random position and first element
  __m64 dist1th = _m_pfsub(m1thElement, mRandomPosition);
  dist0th = _m_pfmul(dist0th, dist0th); // multiplication of the vector
  dist1th = _m_pfmul(dist1th, dist1th);
  __m64 mMinimum = _m_pfacc(dist0th, dist1th); // dist0th.x + dist0th.y and dist1th.x + dist1th.y
  __m64 mIndex = _mm_set_pi32(1, 0); // index of the first two elements. dist0th is written in the second part
  for(unsigned int i = 2; i < vectorSize - 1; i += 2) // finds the node in the tree which is nearest to the random position, size - 1 for odd number of nodes
  {
    __m64 evenElem = *((__m64*) & (firstElem[i].position.x));
    __m64 oddElem = *((__m64*) & (firstElem[i + 1].position.x));
    __m64 evenDist = _m_pfsub(evenElem, mRandomPosition); // difference between random position and first element
    __m64 oddDist = _m_pfsub(oddElem, mRandomPosition);
    evenDist = _m_pfmul(evenDist, evenDist); // multiplication of the vector
    oddDist = _m_pfmul(oddDist, oddDist);
    __m64 mDistances = _m_pfacc(evenDist, oddDist);
    __m64 isLess = _m_pfcmpgt(mMinimum, mDistances); // checks if minimum is greater than current distance
    mMinimum = _m_pfmin(mMinimum, mDistances); // refreshes the minimum
    __m64 indexNew = _mm_set_pi32(i + 1, i); // loads current indices
    mIndex = _mm_or_si64(_mm_andnot_si64(isLess, mIndex), _mm_and_si64(indexNew, isLess)); //mIndex & not isLess | mindexNew & isLess
  }
  __m64 shiftedMinimum = _mm_srli_si64(mMinimum, 32); // shift to the left, needed for later subtraction to find the minimum value
  __m64 isLess = _m_pfcmpgt(mMinimum, shiftedMinimum);
  mMinimum = _m_pfmin(mMinimum, shiftedMinimum); // refreshes the minimum, necessary for odd number of nodes
  __m64 shiftedIndex = _mm_srli_si64(mIndex, 32);
  mIndex = _mm_or_si64(_mm_andnot_si64(isLess, mIndex), _mm_and_si64(shiftedIndex, isLess)); //mIndex & not isLess | mindexNew & isLess
  if(vectorSize % 2 == 1)
  {
    __m64 lastElem = *((__m64*) & (currentUsedTree[vectorSize - 1].position.x)); // loads first element into register
    __m64 dist = _m_pfsub(lastElem, mRandomPosition);
    dist = _m_pfmul(dist, dist);
    __m64 lastMinimum = _m_pfacc(dist, dist);
    __m64 lastIndex = _mm_cvtsi32_si64(vectorSize - 1);
    isLess = _m_pfcmpgt(mMinimum, lastMinimum);
    mIndex = _mm_or_si64(_mm_andnot_si64(isLess, mIndex), _mm_and_si64(lastIndex, isLess));
  }
  indexNearestNode = _mm_cvtsi64_si32(mIndex);
  _m_femms();
#endif
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
      collides = true;
      return collides;
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

bool PathFinder::checkForCollisionMMX(const vector<Node>& allObstacles, const Vector2<>& nearestNode, const Vector2<>& randomPosition, bool usePosition, float& distanceToObstacle, Vector2<>& positionObstacle, const Vector2<>& target)
{
  bool collides = false;
#ifdef TARGET_ROBOT
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
  if(std::abs(newPosition.x) > (theFieldDimensions.xPosOpponentGroundline + (theFieldDimensions.xPosOpponentFieldBorder - theFieldDimensions.xPosOpponentGroundline) / 2) || std::abs(newPosition.y) > (theFieldDimensions.yPosLeftGroundline + (theFieldDimensions.yPosLeftFieldBorder - theFieldDimensions.yPosLeftGroundline) / 2)) // if new node is outside the field
  {
    collides = true;
    return collides;
  }
  // path should be around the own penalty area for avoiding illegal defenders (Only for field players):
  if(newPosition.x < theFieldDimensions.xPosOwnPenaltyArea + 200 && std::abs(newPosition.y) < theFieldDimensions.yPosLeftPenaltyArea + 200 && theRobotInfo.number != 1)
  {
    if(!(theGameInfo.state == STATE_READY && parameters.enterPenaltyAreaInReady && newPosition.x > theFieldDimensions.xPosOwnPenaltyArea - 200))
    {
      collides = true;
      return collides;
    }
  }
  if(allObstacles.size() == 0)
  {
    collides = false;
    return collides;
  }
  Geometry::Line segmentOfPath(nearestNode, newPosition - nearestNode); // calculate line between used points
  Vector2<> rotatedDirection = segmentOfPath.direction;
  rotatedDirection.rotateLeft();

  Vector2<> middlePosition = (nearestNode + newPosition) / 2; // calculates the middle of the segment
  Node middleNode = Node(middlePosition);
  float nearestDistance = sqr(((newPosition - nearestNode).abs() / 2) + parameters.distanceToObstacle);
  Vector2<> vectorNearestDistance = Vector2<>(nearestDistance, nearestDistance);
  vector<int> nearObstacles(allObstacles.size()); // vector for saving if obstacle is near the segment of path
  int vectorSize = allObstacles.size();

  _m_femms();
  const Node* firstElem = &allObstacles[0];
  int* firstNearObstacles = &nearObstacles[0];
  __m64 mMiddleOfSegment = *((__m64*) & (middleNode.position.x)); // loads middle position into register, *((__m64*)&(middlePosition.x)) is the beginning of the vector
  __m64 mMaxDistance = *((__m64*) & (vectorNearestDistance.x)); // distance to middle point which is relevant
  for(int i = 0; i < vectorSize - 1; i += 2) // finds the node in the tree which is nearest to the random position, size - 1 for odd number of nodes
  {
    __m64 evenElem = *((__m64*) & (firstElem[i].position.x));
    __m64 oddElem = *((__m64*) & (firstElem[i + 1].position.x));
    //calculates the square distance between the element and the middle position (is the same as squareAbs() from Vector2<>)
    __m64 evenDist = _m_pfsub(evenElem, mMiddleOfSegment); // difference between random position and first element
    __m64 oddDist = _m_pfsub(oddElem, mMiddleOfSegment);
    evenDist = _m_pfmul(evenDist, evenDist); // multiplication of the vector
    oddDist = _m_pfmul(oddDist, oddDist);
    __m64 mDistances = _m_pfacc(evenDist, oddDist);
    __m64 isLess = _m_pfcmpgt(mMaxDistance, mDistances); // checks if minimum is greater than current distance
    *((__m64*)(firstNearObstacles + i)) = isLess; // wrotes result
  }
  if(vectorSize % 2 == 1) // calculates the distance for the last element
  {
    __m64 lastElem = *((__m64*) & (allObstacles[vectorSize - 1].position.x)); // loads first element into register
    __m64 dist = _m_pfsub(lastElem, mMiddleOfSegment);
    dist = _m_pfmul(dist, dist);
    __m64 lastDistance = _m_pfacc(dist, dist);
    __m64 isLess = _m_pfcmpgt(mMaxDistance, lastDistance);

    firstNearObstacles[vectorSize - 1] = _mm_cvtsi64_si32(isLess);
  }
  _m_femms();

  for(unsigned int i = 0; i < nearObstacles.size(); i++)
  {
    if(nearObstacles[i])
    {
      Vector2<> obstacleRelToBase = allObstacles[i].position - segmentOfPath.base;
      bool obstacleIsRelevant = (rotatedDirection.x * obstacleRelToBase.y - obstacleRelToBase.x * rotatedDirection.y) < 0; // if obstacle is between the positions. Obstacles behind the posititons (not between) are not relevant
      float obstacleDistance = Geometry::getDistanceToEdge(segmentOfPath, allObstacles[i].position);
      if(obstacleDistance < currentDistance && obstacleIsRelevant) // finds the nearest relevant obstacle
      {
        currentDistance = obstacleDistance;
        positionObstacle = allObstacles[i].position;
        collides = true;
      }
    }
  }

  distanceToObstacle = currentDistance;
#endif
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


void PathFinder::checkForFoundPathMMX(const Vector2<>& currentUsedNode, const vector<Node>& currentNotUsedTree, bool& foundPath, bool& foundPathInFirstTree, int& indexOtherTree, const bool useFirstTree)
{
#ifdef TARGET_ROBOT
  _m_femms();
  int indexNearestNode = -1;
  const unsigned int vectorSize = currentNotUsedTree.size();
  if(vectorSize == 1)
  {
    float distanceBetweenTrees = sqr(parameters.distancePathCombination);
    const float distance = (currentNotUsedTree[0].position - currentUsedNode).squareAbs();
    if(distance < distanceBetweenTrees)
    {
      indexOtherTree = 0;
      foundPath = true;
      foundPathInFirstTree = useFirstTree;
    }
    return;
  }
  const Node* firstElem = &currentNotUsedTree[0];
  __m64 mOwnNode = *((__m64*) & (currentUsedNode.x)); // loads own node into register, *((__m64*)&(currentUsedNode.x)) is the beginning of the vector
  __m64 m0thElement = *((__m64*) & (firstElem[0].position.x)); // loads first element into register
  __m64 m1thElement = *((__m64*) & (firstElem[1].position.x)); // loads second element into register
  __m64 dist0th = _m_pfsub(m0thElement, mOwnNode); // difference between random position and first element
  __m64 dist1th = _m_pfsub(m1thElement, mOwnNode);
  dist0th = _m_pfmul(dist0th, dist0th); // multiplication of the vector
  dist1th = _m_pfmul(dist1th, dist1th);
  __m64 mMinimum = _m_pfacc(dist0th, dist1th); // dist0th.x + dist0th.y and dist1th.x + dist1th.y
  __m64 mIndex = _mm_set_pi32(1, 0); // index of the first two elements. dist0th is written in the second part
  for(unsigned int i = 2; i < vectorSize - 1; i += 2) // finds the node in the tree which is nearest to the own node, size - 1 for odd number of nodes
  {
    __m64 evenElem = *((__m64*) & (firstElem[i].position.x));
    __m64 oddElem = *((__m64*) & (firstElem[i + 1].position.x));
    __m64 evenDist = _m_pfsub(evenElem, mOwnNode); // difference between own node and first element
    __m64 oddDist = _m_pfsub(oddElem, mOwnNode);
    evenDist = _m_pfmul(evenDist, evenDist); // multiplication of the vector
    oddDist = _m_pfmul(oddDist, oddDist);
    __m64 mDistances = _m_pfacc(evenDist, oddDist);
    __m64 isLess = _m_pfcmpgt(mMinimum, mDistances); // checks if minimum is greater than current distance
    mMinimum = _m_pfmin(mMinimum, mDistances); // refreshes the minimum
    __m64 indexNew = _mm_set_pi32(i + 1, i); // loads current indices
    mIndex = _mm_or_si64(_mm_andnot_si64(isLess, mIndex), _mm_and_si64(indexNew, isLess)); //mIndex & not isLess | mindexNew & isLess
  }
  __m64 shiftedMinimum = _mm_srli_si64(mMinimum, 32); // shift to the left, needed for later subtraction to find the minimum value
  __m64 isLess = _m_pfcmpgt(mMinimum, shiftedMinimum);
  mMinimum = _m_pfmin(mMinimum, shiftedMinimum); // refreshes the minimum, necessary for odd number of nodes
  __m64 shiftedIndex = _mm_srli_si64(mIndex, 32);
  mIndex = _mm_or_si64(_mm_andnot_si64(isLess, mIndex), _mm_and_si64(shiftedIndex, isLess)); //mIndex & not isLess | mindexNew & isLess
  if(vectorSize % 2 == 1)
  {
    __m64 lastElem = *((__m64*) & (currentNotUsedTree[vectorSize - 1].position.x)); // loads first element into register
    __m64 dist = _m_pfsub(lastElem, mOwnNode);
    dist = _m_pfmul(dist, dist);
    __m64 lastMinimum = _m_pfacc(dist, dist);
    __m64 lastIndex = _mm_cvtsi32_si64(vectorSize - 1);
    isLess = _m_pfcmpgt(mMinimum, lastMinimum);
    mIndex = _mm_or_si64(_mm_andnot_si64(isLess, mIndex), _mm_and_si64(lastIndex, isLess));
  }
  indexNearestNode = _mm_cvtsi64_si32(mIndex);
  _m_femms(); // switch from mmx usage to floating point
  if((currentNotUsedTree[indexNearestNode].position - currentUsedNode).squareAbs() <= sqr(parameters.distancePathCombination)) // check if distance of the nearest node is small enough
  {
    indexOtherTree = indexNearestNode;
    foundPath = true;
    foundPathInFirstTree = useFirstTree;
  }
  else
  {
    foundPath = false;
  }
#endif
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

bool PathFinder::checkForObstaclesNearPositionMMX(const vector<Node>& allObstacles, Vector2<>& positionOfObstacle, const Vector2<>& position, float& smallestDistance)
{
  bool foundObstacle = false;
#ifdef TARGET_ROBOT
  int indexNearestObstacle;
  float currentDistance = parameters.distanceToObstacle;

  const unsigned int vectorSize = allObstacles.size();
  if(vectorSize == 0)
  {
    foundObstacle = false;
    return foundObstacle;
  }
  else if(vectorSize == 1)
  {
    float distanceToObstacle = (allObstacles[0].position - position).abs();
    if(distanceToObstacle < currentDistance)
    {
      smallestDistance = distanceToObstacle;
      foundObstacle = true;
      positionOfObstacle = allObstacles[0].position;
    }
    return foundObstacle;
  }
  _m_femms();
  const Node* firstElem = &allObstacles[0];
  __m64 mOwnPosition = *((__m64*) & (position.x)); // loads randomPositon into register, *((__m64*)&(randomPosition.x)) is the beginning of the vector
  __m64 m0thElement = *((__m64*) & (firstElem[0].position.x)); // loads first element into register
  __m64 m1thElement = *((__m64*) & (firstElem[1].position.x)); // loads second element into register
  __m64 dist0th = _m_pfsub(m0thElement, mOwnPosition); // difference between random position and first element
  __m64 dist1th = _m_pfsub(m1thElement, mOwnPosition);
  dist0th = _m_pfmul(dist0th, dist0th); // multiplication of the vector
  dist1th = _m_pfmul(dist1th, dist1th);
  __m64 mMinimum = _m_pfacc(dist0th, dist1th); // dist0th.x + dist0th.y and dist1th.x + dist1th.y
  __m64 mIndex = _mm_set_pi32(1, 0); // index of the first two elements. dist0th is written in the second part
  for(unsigned int i = 2; i < vectorSize - 1; i += 2) // finds the node in the tree which is nearest to the random position, size - 1 for odd number of nodes
  {
    __m64 evenElem = *((__m64*) & (firstElem[i].position.x));
    __m64 oddElem = *((__m64*) & (firstElem[i + 1].position.x));
    __m64 evenDist = _m_pfsub(evenElem, mOwnPosition); // difference between random position and first element
    __m64 oddDist = _m_pfsub(oddElem, mOwnPosition);
    evenDist = _m_pfmul(evenDist, evenDist); // multiplication of the vector
    oddDist = _m_pfmul(oddDist, oddDist);
    __m64 mDistances = _m_pfacc(evenDist, oddDist);
    __m64 isLess = _m_pfcmpgt(mMinimum, mDistances); // checks if minimum is greater than current distance
    mMinimum = _m_pfmin(mMinimum, mDistances); // refreshes the minimum
    __m64 indexNew = _mm_set_pi32(i + 1, i); // loads current indices
    mIndex = _mm_or_si64(_mm_andnot_si64(isLess, mIndex), _mm_and_si64(indexNew, isLess)); //mIndex & not isLess | mindexNew & isLess
  }
  __m64 shiftedMinimum = _mm_srli_si64(mMinimum, 32); // shift to the left, needed for later subtraction to find the minimum value
  __m64 isLess = _m_pfcmpgt(mMinimum, shiftedMinimum);
  mMinimum = _m_pfmin(mMinimum, shiftedMinimum); // refreshes the minimum, necessary for odd number of nodes
  __m64 shiftedIndex = _mm_srli_si64(mIndex, 32);
  mIndex = _mm_or_si64(_mm_andnot_si64(isLess, mIndex), _mm_and_si64(shiftedIndex, isLess)); //mIndex & not isLess | mindexNew & isLess
  if(vectorSize % 2 == 1)
  {
    __m64 lastElem = *((__m64*) & (allObstacles[vectorSize - 1].position.x)); // loads first element into register
    __m64 dist = _m_pfsub(lastElem, mOwnPosition);
    dist = _m_pfmul(dist, dist);
    __m64 lastMinimum = _m_pfacc(dist, dist);
    __m64 lastIndex = _mm_cvtsi32_si64(vectorSize - 1);
    isLess = _m_pfcmpgt(mMinimum, lastMinimum);
    mIndex = _mm_or_si64(_mm_andnot_si64(isLess, mIndex), _mm_and_si64(lastIndex, isLess));
  }
  indexNearestObstacle = _mm_cvtsi64_si32(mIndex);
  _m_femms();
  float distanceToObstacle = (allObstacles[indexNearestObstacle].position - position).abs();
  if(distanceToObstacle < currentDistance)
  {
    smallestDistance = distanceToObstacle;
    foundObstacle = true;
    positionOfObstacle = allObstacles[indexNearestObstacle].position;
  }
#endif
  return foundObstacle;
}

void PathFinder::addAvoidingPoints(const vector<Node>& allObstacles, vector<Node>& currentUsedTree, const Vector2<>& position, const Vector2<>& positionOfObstacle, bool nearestObstacleMMX, const Node& target, const bool startIsUsed)
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

#ifdef TARGET_ROBOT
  if(nearestObstacleMMX)
  {
    collisions[0] = checkForObstaclesNearPositionMMX(allObstacles, obstacles[0], nodes[0].position, distances[0]);// check for new collisions for all avoiding points
    collisions[1] = checkForObstaclesNearPositionMMX(allObstacles, obstacles[1], nodes[1].position, distances[1]);
    collisions[2] = checkForObstaclesNearPositionMMX(allObstacles, obstacles[2], nodes[2].position, distances[2]);
  }
  else
  {
    collisions[0] = checkForObstaclesNearPosition(allObstacles, obstacles[0], nodes[0].position, distances[0]);// check for new collisions for all avoiding points
    collisions[1] = checkForObstaclesNearPosition(allObstacles, obstacles[1], nodes[1].position, distances[1]);
    collisions[2] = checkForObstaclesNearPosition(allObstacles, obstacles[2], nodes[2].position, distances[2]);
  }
#else
  collisions[0] = checkForObstaclesNearPosition(allObstacles, obstacles[0], nodes[0].position, distances[0]);// check for new collisions for all avoiding points
  collisions[1] = checkForObstaclesNearPosition(allObstacles, obstacles[1], nodes[1].position, distances[1]);
  collisions[2] = checkForObstaclesNearPosition(allObstacles, obstacles[2], nodes[2].position, distances[2]);
#endif


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
