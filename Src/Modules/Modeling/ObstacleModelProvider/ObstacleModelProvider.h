/**
 * @file ObstacleModelProvider.h
 *
 * Declaration of a module that combines arm contacts,
 * foot bumper contacts and players percepts into one obstacle model.
 *
 * @author Florian Maaß
 * @author Jan Fiedler & Nicole Schrader
 */
#pragma once

#include "ObstacleHypothesis.h"

#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/ExtendedGameInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/FieldBoundary.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesFieldPercept.h"
#include "Representations/Sensing/ArmContactModel.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/FootBumperState.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Tools/Module/Module.h"

MODULE(ObstacleModelProvider,
{,
  REQUIRES(ArmContactModel),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(ExtendedGameInfo),
  REQUIRES(FallDownState),
  REQUIRES(FieldBoundary),
  REQUIRES(FieldDimensions),
  REQUIRES(FootBumperState),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(MotionInfo),
  REQUIRES(ObstaclesFieldPercept),
  REQUIRES(Odometer),
  REQUIRES(RobotInfo),
  REQUIRES(RobotModel),
  REQUIRES(RobotPose),
  REQUIRES(TeamData),
  REQUIRES(TorsoMatrix),
  PROVIDES(ObstacleModel),
  LOADS_PARAMETERS(
  {,
    (unsigned) maxDistance,                    /**< Maximal distance to an obstacle. */
    (unsigned) minPercepts,                    /**< Minimal amount of percepts to become an obstacle. */
    (bool) debug,                              /**< Flag for some debug stuff. */
    // deleteObstacles()
    (unsigned) notSeenThreshold,               /**< How many times an obstacle should be seen. If that threshold is reached an obstacle will be removed. */
    (int) deleteAfter,                         /**< Delete obstacles that are not seen for deleteAfter amount of milliseconds. */
    // dynamic()
    (float) pNp,                               /**< Process noise for position vector. */
    (Vector2f) odoDeviation,                   /**< Percentage inaccuracy of the odometry. */
    // addArmContacts()
    (bool) useArmContactModel,
    (Matrix2f) armCov,                         /**< 50mm standard deviation. */
    (int) maxContactTime,                      /**< Maximum time of a body contact. */
    // addFootContacts()
    (bool) useFootBumperState,
    (Matrix2f) feetCov,                        /**< 30mm standard deviation and 80mm standard deviation. */
    (unsigned) distJointToToe,                 /**< The distance from the joint to the toe. */
    (unsigned) distToeToBumper,                /**< The distance between the bumper and the toe. */
    // addPlayerPercepts()
    (Vector2f) pRobotRotationDeviationInStand, /**< Deviation of the rotation of the robot's torso while standing. */
    (Vector2f) pRobotRotationDeviation,        /**< Deviation of the rotation of the robot's torso. */
    // considerTeammates()
    (bool) useTeammatePositionForClassification,
    (unsigned) maxTeammateRadius,              /**< Maximal radius of an obstacle to be considered as nearby. */
    // tryToMerge()
    (float) weightedSum,                       /**< Factor for weighted sum for calculation of width of an obstacle. */
    (unsigned) maxMergeRadius,                 /**< Maximal radius of an obstacle to be considered as nearby. */
    // mergeOverlapping()
    (unsigned) mergeOverlapTimeDiff,           /**< Merge overlapping models if they are measured. Avoid merging of oscillating obstacles. */
    (float) minMahalanobisDistance,            /**< The minimum Mahalanobis distance to merge obstacles. */
    // shouldBeSeen()
    (float) cameraAngleFactor,                 /**< Factor for opening angle of the camera. */
    (int) recentlySeenTime,                    /**< Obstacle was seen in the last 300ms. */
    // calculateMergeRadius()
    (float) mergeDistance,                     /**< Distance to merge obstacles. */
    (float) goalMergeDistance,                 /**< Distance to merge goalposts. */
    (unsigned) minMergeDistance,               /**< Distance of an obstacle until it has a constant radius. */
    // calculateVelocity
    (int) velocitySampleTime,                  /**< Min time between two samples for velocity calculation in ms. */
    (unsigned) minSamplesForCalculation,       /**< The minimum samples for velocity calculation. */
    (int) minVelocityForMotionDetection,       /**< Minimum valid speed of a robot in mm/s. */
    (int) maxVelocityForMotionDetection,       /**< Maximum valid speed of a robot in mm/s. */
    // ObstacleHypothesis::considerType
    (int) teamThreshold,                       /**< Only switch team if this threshold is reached. */
    (int) uprightThreshold,                    /**< Only switch upright/fallen if this threshold is reached. */
  }),
});

/*
 * @class ObstacleModelProvider
 *
 * Combines arm contacts, foot bumper contacts and players percepts into one obstacle model.
 */
class ObstacleModelProvider : public ObstacleModelProviderBase
{
  static_assert(Obstacle::Type::fallenTeammate > Obstacle::Type::teammate, "Assumption broken");
  static_assert(Obstacle::Type::fallenOpponent > Obstacle::Type::opponent, "Assumption broken");
  static_assert(Obstacle::Type::fallenSomeRobot > Obstacle::Type::someRobot, "Assumption broken");
  static_assert(Obstacle::Type::unknown < Obstacle::Type::someRobot, "Assumption broken");
  static_assert(Obstacle::Type::goalpost < Obstacle::Type::unknown, "Assumption broken");

  // Used for writing annotations only once per contact.
  bool armContact[Arms::numOfArms] = { false, false }, footContact[Legs::numOfLegs] = { false, false };

  std::vector<ObstacleHypothesis, Eigen::aligned_allocator<ObstacleHypothesis>> obstacleHypotheses; /**< List of obstacles. */
  std::vector<bool> merged; /**< This is to merge obstacles once for every "percept" per frame. */

  /** The function is called when the representation provided needs to be updated. */
  void update(ObstacleModel& obstacleModel) override;

  /**
   * The function decides whether the obstacle model should be calculated or not
   * and deletes the model and hypotheses if it should not be calculated.
   *
   * @param obstacleModel The representation to be updated.
   * @return Whether the obstacle model should be calculated.
   */
  bool clearAndFinish(ObstacleModel& obstacleModel);

  /** The function deletes hypotheses that are no longer valid. */
  void deleteObstacles();

  /** The function apply dynamic step from extended kalman filter on all hypotheses. */
  void dynamic();
  /** The function add hypotheses measured by arm contact. */
  void addArmContacts();
  /** The function add hypotheses measured by foot contact. */
  void addFootContacts();
  /** The function add players percepts. */
  void addPlayerPercepts();

  /**
   * The function tries to merge the measurement with an existing hypothesis.
   * @param measurement The measurement to merge.
   */
  void tryToMerge(const ObstacleHypothesis& measurement);

  /**< The function fits team and position of obstacles located exclusively near a team member. */
  void considerTeamData();
  /** The function will merge overlapping hypotheses to one hypotheses. */
  void mergeOverlapping();
  /** The function increases the attribute notSeenButShouldSeenCount of obstacles that should be seen but wasn't seen recently. */
  void shouldBeSeen();

  /**
   * The function checks if any other obstacle is in the shadow of the obstacle closer.
   * @param closer The obstacle that may shadow other obstacles.
   * @param i The index of the obstacle closer in the list obstacleHypotheses.
   * @param cameraAngleLeft The left camera angle.
   * @param cameraAngleRight The right camera angle.
   */
  bool isAnyObstacleInShadow(ObstacleHypothesis* closer, const std::size_t i, const float cameraAngleLeft, const float cameraAngleRight);

  float calculateMergeRadius(const Vector2f center, const Obstacle::Type type, const unsigned maxRadius) const
  {
    const float measurementDistance = center.norm() - minMergeDistance;
    return (type == Obstacle::goalpost ? goalMergeDistance : mergeDistance)
           + (measurementDistance < 0 ? 0.f : measurementDistance * maxRadius / maxDistance);
  }

  inline bool isObstacle(const ObstacleHypothesis& obstacle)
  {
    return obstacle.seenCount >= minPercepts || debug;
  }

  /** Calculates the velocity for every obstacles. */
  void calculateVelocity();
};
