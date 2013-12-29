/**
* @file CombinedWorldModelProvider.h
* Declares a class that provides a combined world model
* @author Katharina Gillmann
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Modeling/CombinedWorldModel.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Modeling/RobotsModel.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Modeling/ObstacleClusters.h"
#include "Tools/RingBuffer.h"

MODULE(CombinedWorldModelProvider)
  REQUIRES(RobotPose)
  REQUIRES(RobotInfo)
  REQUIRES(RobotsModel)
  REQUIRES(BallModel)
  REQUIRES(TeamMateData)
  REQUIRES(OwnTeamInfo)
  REQUIRES(FieldDimensions)
  REQUIRES(FrameInfo)
  REQUIRES(FallDownState)
  REQUIRES(GroundContactState)
  REQUIRES(ObstacleModel)
  REQUIRES(CameraMatrix)
  REQUIRES(ObstacleClusters)
  USES(SideConfidence)
  PROVIDES_WITH_MODIFY_AND_DRAW(CombinedWorldModel)
  LOADS_PARAMETER(float, movementFactorBallDisappeared)   /**< factor for the movement of the sigmoid function for the ball disappeared weight */
  LOADS_PARAMETER(float, movementFactorBallSinceLastSeen) /**< factor for the movement of the sigmoid function for the ball time since last seen weight */
  LOADS_PARAMETER(float, scalingFactorBallDisappeared)    /**< factor for the scaling of the sigmoid function for the ball disappeared weight */
  LOADS_PARAMETER(float, scalingFactorBallSinceLastSeen)  /**< factor for the scaling of the sigmoid function for the ball time since last seen weight */
  LOADS_PARAMETER(float, clusteringDistance)              /**< The distance between obstacles which are added to the same cluster */
  LOADS_PARAMETER(float, distanceToTeamMate)              /**< distance of an obstacle to an own teammate */
  LOADS_PARAMETER(int, ballModelAge)                      /**< minimum age of the old used ballModel>*/
  LOADS_PARAMETER(int, ballModelOthersTimeOut)            /**< maximum age of a ball model that can be integrated into the ballStateOthers */
  LOADS_PARAMETER(bool, closeRobotsNeedLocalDetection)    /**< activates additional constraint to avoid close false positives resulting from self-localization errors of teammates */
  LOADS_PARAMETER(float, closeRobotDetectionDistance)     /**< distance up to which a local detection is require, if closeRobotsNeedLocalDetection is true */
END_MODULE

/**
 * @class CombinedWorldModelProvider
 * A combined world model
 */
class CombinedWorldModelProvider : public CombinedWorldModelProviderBase
{
  class DetectedRobot
  {
  public:
    GaussianPositionDistribution meanAndCovariance; // includes the position and the uncertainty of the detected robot
    int clusterId; // Id of the Cluster the robot was added to
    bool detectedByMe; // indicates that this detection has not been communicated but been observed locally
    std::vector<DetectedRobot*> measurementsSmallerThanDistance; // pointer to all robots which are closer to the own robot than the declared distance

    DetectedRobot() {} // Constructor
    DetectedRobot(const Pose2D& observer, const Vector2<>& robotPosition, const Matrix2x2<>& covariance, bool detectedByMe = false) : // initially clusterId is set to -1, which means that it was added to no cluster yet
      meanAndCovariance(observer * robotPosition, rotate(covariance, observer.rotation)), clusterId(-1), detectedByMe(detectedByMe) {}

    bool operator<(const DetectedRobot& other) const  // sorts the detected robots by descending size (>).
    {
      return measurementsSmallerThanDistance.size() > other.measurementsSmallerThanDistance.size();
    }

  private:
    static Matrix2x2<> rotate(const Matrix2x2<>& matrix, const float angle)
    {
      const float cosine = std::cos(angle);
      const float sine = std::sin(angle);
      const Matrix2x2<> rotationMatrix(cosine, -sine, sine, cosine);
      return (rotationMatrix * matrix) * rotationMatrix.transpose();
    }
  };

  class Cluster
  {
  public:
    std::vector<DetectedRobot*> detectedRobots; // pointer to all robots which belong to the same cluster
  };

  /**
  * Provides the combined world model representation
  */
  void update(CombinedWorldModel& combinedWorldModel);


  /**
  * Provides the combined ball model representation by teammates only
  */
  void updateOthers(CombinedWorldModel& combinedWorldModel);

  class ExtendedBallModel : public BallModel
  {
  public:
    float cameraHeight;
    ExtendedBallModel() {}
    ExtendedBallModel(const BallModel& ballModel, float cameraHeight) : BallModel(ballModel), cameraHeight(cameraHeight) {}
  };

  RingBuffer<ExtendedBallModel, 20> ballModelsAllPlayers[TeamMateData::numOfPlayers]; // last x BallModels of each player
  RingBuffer<ExtendedBallModel, 20> ballModelsAllOtherPlayers[TeamMateData::numOfPlayers]; // last x BallModels of each other player

  ExtendedBallModel lastValidBallModel[TeamMateData::numOfPlayers]; // last valid BallModels
  ExtendedBallModel lastValidOthersBallModel[TeamMateData::numOfPlayers]; // last valid BallModels from other robots

  bool oldBallModelUsed[TeamMateData::numOfPlayers]; // if old BallModel shall be used
  bool oldOthersBallModelUsed[TeamMateData::numOfPlayers]; // if old BallModel shall be used

  std::vector<DetectedRobot> allDetectedRobots; // all detected robots with cluster informations
  std::vector<Cluster> allCluster; // all found clusters

  BallState getCombinedBallPosition(bool& ballIsValid); // calculates the global ball position
  BallState getCombinedBallPositionOthers(bool& ballIsValid, float& maxSideConfidence); // calculates the global ball position
  float computeWeights(const ExtendedBallModel& ballModel, const RobotPose& robotPose, unsigned timeWhenBallDisappeared) const; // computes weight for the global ball position
  void clusterAllDetectedRobots(); // clusters all detected robots
  std::vector<GaussianPositionDistribution> getPositionOfOpponentRobots(); // calculates the positions and covariance of the opponent robots by using the clusters. All positions inside one cluster are merged by using the last Kalmafilter step.
  void recursiveClustering(DetectedRobot& currentRobot, const int clusterId); // adds robots recursive to a cluster
  bool ownTeamMatesAreMeasured(const Vector2<>& positionOfMeasurment, const std::vector<Pose2D>& ownTeam, const Vector2<>& ownPosition); // checks if an own team mate is measured with an ultrasonic measurement
};
