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
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Modeling/ObstacleClusters.h"
#include "Representations/Modeling/TeammateReliability.h"
#include "Tools/RingBuffer.h"

MODULE(CombinedWorldModelProvider,
{,
  REQUIRES(RobotPose),
  REQUIRES(RobotInfo),
  REQUIRES(BallModel),
  REQUIRES(TeammateData),
  REQUIRES(OwnTeamInfo),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(FallDownState),
  REQUIRES(GroundContactState),
  REQUIRES(ObstacleModel),
  REQUIRES(CameraMatrix),
  REQUIRES(ObstacleClusters),
  REQUIRES(TeammateReliability),
  USES(SideConfidence),
  PROVIDES_WITH_MODIFY_AND_DRAW(CombinedWorldModel),
  LOADS_PARAMETERS(
  {,
    (float) movementFactorBallDisappeared,   /**< factor for the movement of the sigmoid function for the ball disappeared weight */
    (float) movementFactorBallSinceLastSeen, /**< factor for the movement of the sigmoid function for the ball time since last seen weight */
    (float) scalingFactorBallDisappeared,    /**< factor for the scaling of the sigmoid function for the ball disappeared weight */
    (float) scalingFactorBallSinceLastSeen,  /**< factor for the scaling of the sigmoid function for the ball time since last seen weight */
    (float) clusteringDistance,              /**< The distance between obstacles which are added to the same cluster */
    (float) distanceToTeammate,              /**< distance of an obstacle to an own teammate */
    (int) ballModelAge,                      /**< minimum age of the old used ballModel>*/
    (bool) closeRobotsNeedLocalDetection,    /**< activates additional constraint to avoid close false positives resulting from self-localization errors of teammates */
    (float) closeRobotDetectionDistance,     /**< distance up to which a local detection is require, if closeRobotsNeedLocalDetection is true */
  }),
});

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

    DetectedRobot() = default; // Constructor
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

  class ExtendedBallModel : public BallModel
  {
  public:
    RobotPose robotPose;
    float cameraHeight;
    ExtendedBallModel() : cameraHeight(0) {}
    ExtendedBallModel(const BallModel& ballModel, const RobotPose& robotPose, float cameraHeight) : BallModel(ballModel), robotPose(robotPose), cameraHeight(cameraHeight) {}
  };

  RingBuffer<ExtendedBallModel, 20> ballModelsAllPlayers[TeammateData::numOfPlayers]; // last x BallModels of each player

  ExtendedBallModel lastValidBallModel[TeammateData::numOfPlayers]; // last valid BallModels

  bool oldBallModelUsed[TeammateData::numOfPlayers]; // if old BallModel shall be used

  std::vector<DetectedRobot> allDetectedRobots; // all detected robots with cluster informations
  std::vector<Cluster> allCluster; // all found clusters

  BallState getCombinedBallPosition(bool& ballIsValid); // calculates the global ball position
  float computeWeights(const ExtendedBallModel& ballModel, const RobotPose& robotPose, unsigned timeWhenBallDisappeared, TeammateReliability::ReliabilityState state) const; // computes weight for the global ball position
  void clusterAllDetectedRobots(); // clusters all detected robots
  std::vector<GaussianPositionDistribution> getPositionOfOpponentRobots(); // calculates the positions and covariance of the opponent robots by using the clusters. All positions inside one cluster are merged by using the last Kalmafilter step.
  void recursiveClustering(DetectedRobot& currentRobot, const int clusterId); // adds robots recursive to a cluster
  bool ownTeammatesAreMeasured(const Vector2<>& positionOfMeasurment, const std::vector<Pose2D>& ownTeam, const Vector2<>& ownPosition); // checks if an own team mate is measured with an ultrasonic measurement
};
