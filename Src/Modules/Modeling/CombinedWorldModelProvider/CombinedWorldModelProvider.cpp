/**
* @file CombinedWorldModelProvider.cpp
* Implementation of the CombinedWorldModelProvider module.
* @author Katharina Gillmann
*/

#include "CombinedWorldModelProvider.h"
#include "Tools/Debugging/DebugDrawings.h"
#include <algorithm>
#include <limits>
#include "Tools/Streams/InStreams.h"

using namespace std;

MAKE_MODULE(CombinedWorldModelProvider, Modeling)

void CombinedWorldModelProvider::update(CombinedWorldModel& combinedWorldModel)
{
  combinedWorldModel.positionsOwnTeam.clear();
  combinedWorldModel.positionsOpponentTeam.clear();

  // calculates all robot poses of the own team. But the position of the robot is only chosen, when the robot is not penalized, has ground contact and has not leapt the network timeout.
  for(int i = TeammateData::firstPlayer; i < TeammateData::numOfPlayers; i++)
  {
    if(i == theRobotInfo.number && theRobotInfo.penalty == PENALTY_NONE && theGroundContactState.contact && theOwnTeamInfo.players[i - 1].penalty == PENALTY_NONE)
    {
      combinedWorldModel.positionsOwnTeam.push_back(Pose2D(theRobotPose.rotation, theRobotPose.translation)); // pose of the robot itself
    }
    else if(i != theRobotInfo.number && theOwnTeamInfo.players[i - 1].penalty == PENALTY_NONE && !theTeammateData.isPenalized[i] && theTeammateData.hasGroundContact[i] && theFrameInfo.getTimeSince(theTeammateData.timeStamps[i]) < static_cast<int>(theTeammateData.networkTimeout) && theTeammateData.robotPoses[i].deviation < 50) // the "penalized" state is checked twice (by checking the message of the team mate itself and by checking the state set by the GameController). The index of theOwnTeamInfo.players is i-1 because it is "zero based" whereas theTeammateData is "one based"
    {
      combinedWorldModel.positionsOwnTeam.push_back(Pose2D(theTeammateData.robotPoses[i].rotation, theTeammateData.robotPoses[i].translation)); // Pose of team mates
    }
  }

  // adds last BallModel of each player to corresponding RingBuffers
  for(int i = TeammateData::firstPlayer; i < TeammateData::numOfPlayers; i++)
  {
    if(i == theRobotInfo.number)
    {
      ballModelsAllPlayers[i].add(ExtendedBallModel(theBallModel, theRobotPose, theCameraMatrix.translation.z)); // adds current BallModel to the buffer
    }
    else if(ballModelsAllPlayers[i].getNumberOfEntries() == 0 || ballModelsAllPlayers[i][0].timeWhenLastSeen != theTeammateData.ballModels[i].timeWhenLastSeen)
    {
      ballModelsAllPlayers[i].add(ExtendedBallModel(theTeammateData.ballModels[i], theTeammateData.robotPoses[i], theTeammateData.cameraHeights[i]));
    }
    //if robot is penalized, not upright or has no ground contact
    if((i == theRobotInfo.number && (theOwnTeamInfo.players[i - 1].penalty != PENALTY_NONE || theRobotInfo.penalty != PENALTY_NONE || !theGroundContactState.contact || theFallDownState.state != theFallDownState.upright))
       || (i != theRobotInfo.number && (theOwnTeamInfo.players[i - 1].penalty != PENALTY_NONE || !theTeammateData.isUpright[i] || theTeammateData.isPenalized[i] || !theTeammateData.hasGroundContact[i] || theFrameInfo.getTimeSince(theTeammateData.timeStamps[i]) > static_cast<int>(theTeammateData.networkTimeout))))
    {
      if(!oldBallModelUsed[i])// if old BallModel was not already saved
      {
        int ringBufferIndex;
        for(ringBufferIndex = 0; ringBufferIndex < ballModelsAllPlayers[i].getNumberOfEntries(); ringBufferIndex++)
        {
          if(theFrameInfo.getTimeSince(ballModelsAllPlayers[i][ringBufferIndex].timeWhenLastSeen) > ballModelAge) // save the first BallModel which is older than x sec.
          {
            break;
          }
        }
        if(ringBufferIndex >= ballModelsAllPlayers[i].getNumberOfEntries()) // if no BallModel is older than x sec.
        {
          ringBufferIndex = ballModelsAllPlayers[i].getNumberOfEntries() - 1;
        }
        lastValidBallModel[i] = ballModelsAllPlayers[i][ringBufferIndex]; // save last valid BallModel
        oldBallModelUsed[i] = true;
      }
    }
    else // if robot is valid the last BallModel is used
    {
      lastValidBallModel[i] = ballModelsAllPlayers[i][0];
      oldBallModelUsed[i] = false;
    }
  }

  // calculates the combined ball model. It is based on the weighted sum of the ball models of all players of the own team.
  bool ballIsValid = true;
  BallState ballState = getCombinedBallPosition(ballIsValid);
  combinedWorldModel.ballIsValid = ballIsValid; // if the ball state is valid
  if(ballIsValid) // if the seen ball is valid the ball position is updated. Otherwise the last valid position is used.
  {
    combinedWorldModel.ballState = ballState;
    combinedWorldModel.expectedEndPosition = combinedWorldModel.ballState.getEndPosition(theFieldDimensions.ballFriction);
  }

  // collects all (by vision and ultrasonic) detected robots of all players of the own team
  //The positions of the seen robots by an own player are only used when the own robot is not penalized, has ground contact, stands upright and has not leapt the network timeout.
  allDetectedRobots.clear();
  allCluster.clear();

  for(int i = TeammateData::firstPlayer; i < TeammateData::numOfPlayers; i++)
  {
    if(i == theRobotInfo.number && theOwnTeamInfo.players[i - 1].penalty == PENALTY_NONE && theRobotInfo.penalty == PENALTY_NONE && theGroundContactState.contact && theFallDownState.state == theFallDownState.upright) // robots seen by the own robot itself.
    {
      for(std::vector<ObstacleModel::Obstacle>::const_iterator j = theObstacleModel.obstacles.begin(); j != theObstacleModel.obstacles.end(); ++j) // collects all (by ultrasonic) detected robots of the own robot itself
      {
        if(std::abs((theRobotPose * j->center).x) <= theFieldDimensions.xPosOpponentFieldBorder && std::abs((theRobotPose * j->center).y) <= theFieldDimensions.yPosLeftFieldBorder && !ownTeammatesAreMeasured(theRobotPose * j->center, combinedWorldModel.positionsOwnTeam, theRobotPose.translation)) // if seen robots are of the opponent team and are not detected outside the field
        {
          allDetectedRobots.push_back(DetectedRobot(theRobotPose, j->center, j->covariance, true));
        }
      }
      for(const GaussianPositionDistribution& d : theObstacleClusters.obstacles)
      {
        if(std::abs((theRobotPose * d.robotPosition).x) <= theFieldDimensions.xPosOpponentFieldBorder && std::abs((theRobotPose * d.robotPosition).y) <= theFieldDimensions.yPosLeftFieldBorder && !ownTeammatesAreMeasured(theRobotPose * d.robotPosition, combinedWorldModel.positionsOwnTeam, theRobotPose.translation)) // if seen robots are of the opponent team and are not detected outside the field
        {
          allDetectedRobots.push_back(DetectedRobot(theRobotPose, d.robotPosition, d.covariance, true));
        }
      }
    }
    else if(i != theRobotInfo.number && theTeammateData.isUpright[i] && theOwnTeamInfo.players[i - 1].penalty == PENALTY_NONE && !theTeammateData.isPenalized[i] && theTeammateData.hasGroundContact[i] && theFrameInfo.getTimeSince(theTeammateData.timeStamps[i]) < static_cast<int>(theTeammateData.networkTimeout)) // robots seen by own team mates (by vision)
    {
      for(std::vector<ObstacleModel::Obstacle>::const_iterator j = theTeammateData.obstacleModels[i].obstacles.begin(); j != theTeammateData.obstacleModels[i].obstacles.end(); ++j) // collects all (by ultrasonic) detected robots of all players of the own team mates
      {
        if(std::abs((theTeammateData.robotPoses[i] * j->center).x) <= theFieldDimensions.xPosOpponentFieldBorder && std::abs((theTeammateData.robotPoses[i] * j->center).y) <= theFieldDimensions.yPosLeftFieldBorder && !ownTeammatesAreMeasured(theTeammateData.robotPoses[i] * j->center, combinedWorldModel.positionsOwnTeam, theTeammateData.robotPoses[i].translation)) // if seen robots are of the opponent team and are not detected outside the field
        {
          allDetectedRobots.push_back(DetectedRobot(theTeammateData.robotPoses[i], j->center, j->covariance));
        }
      }
      for(const GaussianPositionDistribution& d : theTeammateData.obstacleClusters[i].obstacles)
      {
        if(std::abs((theTeammateData.robotPoses[i] * d.robotPosition).x) <= theFieldDimensions.xPosOpponentFieldBorder && std::abs((theTeammateData.robotPoses[i] * d.robotPosition).y) <= theFieldDimensions.yPosLeftFieldBorder && !ownTeammatesAreMeasured(theTeammateData.robotPoses[i] * d.robotPosition, combinedWorldModel.positionsOwnTeam, theTeammateData.robotPoses[i].translation)) // if seen robots are of the opponent team and are not detected outside the field
        {
          allDetectedRobots.push_back(DetectedRobot(theTeammateData.robotPoses[i], d.robotPosition, d.covariance, false));
        }
      }
    }
  }

  clusterAllDetectedRobots(); // clusters all detected robots
  combinedWorldModel.positionsOpponentTeam = getPositionOfOpponentRobots();// computes the positions out of the clusters
}

BallState CombinedWorldModelProvider::getCombinedBallPosition(bool& ballIsValid)
{
  float weights[TeammateData::numOfPlayers]; // own robot and team mates
  int timeSinceBallLastSeen = std::numeric_limits<int>::max();
  int ownBallLastSeen;
  //BallStates of all players
  for(int i = TeammateData::firstPlayer; i < TeammateData::numOfPlayers; i++)
  {
    //weight is 0 if ball is seen outside the field.
    if(fabs((lastValidBallModel[i].robotPose * lastValidBallModel[i].estimate.position).x) > theFieldDimensions.xPosOpponentFieldBorder ||
       fabs((lastValidBallModel[i].robotPose * lastValidBallModel[i].estimate.position).y) > theFieldDimensions.yPosLeftFieldBorder)
    {
      weights[i] = 0;
      ownBallLastSeen = std::numeric_limits<int>::max();
    }
    else
    {
      weights[i] = computeWeights(lastValidBallModel[i], lastValidBallModel[i].robotPose, lastValidBallModel[i].timeWhenDisappeared, theTeammateReliability.states[i]);
      ownBallLastSeen = theFrameInfo.getTimeSince(lastValidBallModel[i].timeWhenLastSeen); // time since ball was last seen
    }
    if(ownBallLastSeen < timeSinceBallLastSeen) // if the time since ball last seen is lower than the current min time, it is set as current min time
    {
      timeSinceBallLastSeen = ownBallLastSeen;
    }
  }

  BallState combinedBallStateAbsolute;

  for(int i = TeammateData::firstPlayer; i < TeammateData::numOfPlayers; i++) //sum of all ball states
  {
    if(i == theRobotInfo.number) // own robot
    {
      combinedBallStateAbsolute.velocity += (Vector2<>(theBallModel.estimate.velocity).rotate(theRobotPose.rotation) * weights[i]); // ball velocity * weight
      combinedBallStateAbsolute.position += ((theRobotPose * theBallModel.estimate.position) * weights[i]); // ball position * weight
    }
    else // team mates
    {
      combinedBallStateAbsolute.position += ((theTeammateData.robotPoses[i] * theTeammateData.ballModels[i].estimate.position) * weights[i]);
      combinedBallStateAbsolute.velocity += (Vector2<>(theTeammateData.ballModels[i].estimate.velocity).rotate(theTeammateData.robotPoses[i].rotation) * weights[i]);
    }
  }
  //sum of all weights
  float weightSum = 0;
  for(int i = 1; i < TeammateData::numOfPlayers; i++)
  {
    weightSum += weights[i];
  }
  // weighted sum of ball states
  if(weightSum != 0.0f)
  {
    combinedBallStateAbsolute.position /= weightSum;
    combinedBallStateAbsolute.velocity /= weightSum;
  }
  else
    ballIsValid = false;

  if(timeSinceBallLastSeen >= movementFactorBallSinceLastSeen) // if the minimum seen time is too high the ball position is not valid
  {
    ballIsValid = false;
  }
  return combinedBallStateAbsolute;
}

float CombinedWorldModelProvider::computeWeights(const ExtendedBallModel& ballModel, const RobotPose& robotPose, unsigned timeWhenBallDisappeared, TeammateReliability::ReliabilityState state) const
{
  if(ballModel.cameraHeight < 100 || (Global::getSettings().isDropInGame && state != TeammateReliability::GOOD)) // to handle degenerated cases
  {
    return 0.0f;
  }
  float weight = 1.0f; // default value for weight of one ball state
  weight *= robotPose.validity; // validity of Selflocalisation
  weight *= 1.0f - (1.0f / (1.0f + exp(-(theFrameInfo.getTimeSince(ballModel.timeWhenLastSeen) - movementFactorBallSinceLastSeen) / scalingFactorBallSinceLastSeen)));  // sigmoid function based on ball_time_since_last_seen
  weight *= 1.0f - (1.0f / (1.0f + exp(-(theFrameInfo.getTimeSince(timeWhenBallDisappeared) - movementFactorBallDisappeared) / scalingFactorBallDisappeared)));  // sigmoid function based on ball_time_when_disappeared

  const float angleOfCamera = atan(ballModel.estimate.position.abs() / ballModel.cameraHeight); // angle of camera when looking at the ball
  const float ballPositionWithPositiveDeviation = tan(angleOfCamera + fromDegrees(1.0f)) * ballModel.cameraHeight; // ball position when angle of camera is a bit different in position direction
  const float ballPositionWithNegativeDeviation = tan(angleOfCamera - fromDegrees(1.0f)) * ballModel.cameraHeight; // ball position when angle of camera is a bit different in negative direction
  const float ballDeviation = (ballPositionWithNegativeDeviation - ballPositionWithPositiveDeviation) / 2; // averaged devition of the ball position
  weight *= 1.0f / ballDeviation;

  weight = std::abs(weight);

  return weight;
}

void CombinedWorldModelProvider::clusterAllDetectedRobots()
{
  const float distance = sqr(clusteringDistance); // use square to save a square root in caluclating the distance between two robots (less time for calculation)

  // detects all neighbors (dependent on the distance) of one detected robot. Epsilon-Neighborhood
  for(std::vector<DetectedRobot>::iterator i = allDetectedRobots.begin(); i != allDetectedRobots.end(); ++i)
  {
    for(std::vector<DetectedRobot>::iterator j = allDetectedRobots.begin(); j != allDetectedRobots.end(); ++j)
    {
      const float currentDistance = (j->meanAndCovariance.robotPosition - i->meanAndCovariance.robotPosition).squareAbs(); // calculated distance between two current robots, not abs() but squareAbs() for faster calculation
      if(i != j && currentDistance <= distance)// if distance to the other robot is smaller than prescribed distance
      {
        i->measurementsSmallerThanDistance.push_back(&*j); // Using an adjacency list, adds pointer of the neighboring robot to the list of neighbors
      }
    }
  }

  int clusterId = 0;
  //clusterIds are set recursive for each robot
  for(std::vector<DetectedRobot>::iterator j = allDetectedRobots.begin(); j != allDetectedRobots.end(); ++j)
  {
    if(j->clusterId == -1)
    {
      recursiveClustering(*j, clusterId);
      clusterId++;
    }
  }

  Cluster clusterTemp; // a temporary cluster to merge all measured robots of one cluster

  // merges measurements to clusters
  for(int i = 0; i < clusterId; i++)
  {
    clusterTemp.detectedRobots.clear();
    for(std::vector<DetectedRobot>::iterator j = allDetectedRobots.begin(); j != allDetectedRobots.end(); ++j)
    {
      if(j->clusterId == i) // all measurements with the same clusterId are assigned to the same cluster
      {
        clusterTemp.detectedRobots.push_back(&*j); // pointer of the detected robot inside this cluster
      }
    }
    allCluster.push_back(clusterTemp);
  }
}

vector<GaussianPositionDistribution> CombinedWorldModelProvider::getPositionOfOpponentRobots()
{
  vector<GaussianPositionDistribution> positionsOfDetectedRobots; // vector for the final opponent robot positions
  Vector2<> mean;
  Matrix2x2<> covariance;
  Matrix2x2<> kalmanGain;

  for(std::vector<Cluster>::const_iterator i = allCluster.begin(); i != allCluster.end(); ++i) // calculates the merged position for each cluster
  {
    bool detectedByMe = i->detectedRobots[0]->detectedByMe;
    mean = i->detectedRobots[0]->meanAndCovariance.robotPosition; // initializes mean with the mean of the first entry in a cluster
    covariance = i->detectedRobots[0]->meanAndCovariance.covariance; // initializes covariance with the covariance of the first entry in a cluster

    for(unsigned int j = 1; j != i->detectedRobots.size(); j++)
    {
      kalmanGain = covariance * (covariance + i->detectedRobots[j]->meanAndCovariance.covariance).invert(); // calculates the Kalman Gain
      mean += kalmanGain * (i->detectedRobots[j]->meanAndCovariance.robotPosition - mean); // calculates the new mean
      covariance -= kalmanGain * covariance; // calculates the new covariance
      if(i->detectedRobots[j]->detectedByMe)
        detectedByMe = true;
    }
    if(closeRobotsNeedLocalDetection)
    {
      if(detectedByMe || ((mean - theRobotPose.translation).abs() > closeRobotDetectionDistance))
        positionsOfDetectedRobots.push_back(GaussianPositionDistribution(mean, covariance));
    }
    else
    {
      positionsOfDetectedRobots.push_back(GaussianPositionDistribution(mean, covariance));
    }
  }

  // goal posts are added as obstacles
  positionsOfDetectedRobots.push_back(GaussianPositionDistribution(Vector2<>(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal), Matrix2x2<>(0.0f, 0.0f, 0.0f, 0.0f)));
  positionsOfDetectedRobots.push_back(GaussianPositionDistribution(Vector2<>(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal), Matrix2x2<>(0.0f, 0.0f, 0.0f, 0.0f)));
  positionsOfDetectedRobots.push_back(GaussianPositionDistribution(Vector2<>(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosLeftGoal), Matrix2x2<>(0.0f, 0.0f, 0.0f, 0.0f)));
  positionsOfDetectedRobots.push_back(GaussianPositionDistribution(Vector2<>(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosRightGoal), Matrix2x2<>(0.0f, 0.0f, 0.0f, 0.0f)));

  return positionsOfDetectedRobots;
}

void CombinedWorldModelProvider::recursiveClustering(DetectedRobot& currentRobot, const int clusterId)
{
  currentRobot.clusterId = clusterId; // sets clusterId for the current robot

  for(unsigned int j = 0; j < currentRobot.measurementsSmallerThanDistance.size(); j++) // adds same clusterId for all neighbors of this robot
  {
    DetectedRobot* nextRobot = currentRobot.measurementsSmallerThanDistance[j];
    if(nextRobot->clusterId == -1) // if neighbored robot is not assigned to another cluster yet.
    {
      recursiveClustering(*nextRobot, clusterId); // recursive calculation of the clusterId for the neighbors of the current selected robot
    }
  }
}

bool CombinedWorldModelProvider::ownTeammatesAreMeasured(const Vector2<>& positionOfMeasurement, const vector<Pose2D>& ownTeam, const Vector2<>& ownPosition)
{
  bool measurementIsOwnRobot = false;
  for(std::vector<Pose2D>::const_iterator j = ownTeam.begin(); j != ownTeam.end(); ++j)
  {
    if((positionOfMeasurement - j->translation).abs() <= distanceToTeammate && j->translation != ownPosition) // if distance between measurement and teammates position is smaller than given distance and the teammate is not the robot itself
      measurementIsOwnRobot = true;
  }
  return measurementIsOwnRobot; // returns true if a teammates position is near the measurement
}
