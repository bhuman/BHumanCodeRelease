/**
 * @file GlobalOpponentsTracker.h
 *
 * Declaration of a module that aims to track all opponent robots on the pitch
 *
 * @author Florian Maaß
 * @author Jan Fiedler & Nicole Schrader
 * @author Tim Laue
 * @author Michelle Gusev
 */

#pragma once

#include "GlobalOpponentsHypothesis.h"

#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/GlobalOpponentsModel.h"
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
#include "Math/BHMath.h"
#include "Math/Geometry.h"

MODULE(GlobalOpponentsTracker,
{,
  REQUIRES(ArmContactModel),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(ExtendedGameState),
  REQUIRES(FallDownState),
  REQUIRES(FieldBoundary),
  REQUIRES(FieldDimensions),
  REQUIRES(FootBumperState),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(MotionInfo),
  REQUIRES(ObstaclesFieldPercept),
  REQUIRES(Odometer),
  REQUIRES(RobotModel),
  REQUIRES(RobotPose),
  REQUIRES(TorsoMatrix),
  PROVIDES(GlobalOpponentsModel),
  LOADS_PARAMETERS(
  {,
    (float) maxOpponentDistance,               /**< Maximal distance to tracked objects (in mm). Everything that is farther away becomes deleted. */
    (unsigned) minPercepts,                    /**< Minimal amount of percepts to become an obstacle. */
    // deleteObstacles()
    (unsigned) notSeenThreshold,               /**< How many times an obstacle should be seen. If that threshold is reached an obstacle will be removed. */
    (int)deleteAfter,                         /**< Delete obstacles that are not seen for deleteAfter amount of milliseconds. */
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
    (float) distJointToToe,                    /**< The distance from the joint to the toe. */
    (float) distToeToBumper,                   /**< The distance between the bumper and the toe. */
    // addPlayerPercepts()
    // tryToMerge()
    (float) modelWidthWeighting,               /**< Factor for weighted sum for calculating the width of an obstacle. Currently modeled width is weighted by this factor, measured width is weighted with 1. */
    (float) maxMergeRadius,                    /**< Maximal radius of an obstacle to be considered as nearby. */
    // mergeOverlapping()
    (unsigned) mergeOverlapTimeDiff,           /**< Merge overlapping models if they are measured. Avoid merging of oscillating obstacles. */
    (float) minMahalanobisDistance,            /**< The minimum Mahalanobis distance to merge obstacles. */
    // shouldBeSeen()
    (float) cameraAngleFactor,                 /**< Factor for opening angle of the camera. */
    (int) recentlySeenTime,                    /**< Obstacle was seen in the last 300ms. */
    // calculateMergeRadius()
    (float) defaultMergeRadius,                /**< Default distance between two obstacles to merge them (in mm). */
    (float) rangeForDefaultMergeRadius,        /**< Up to this distance (in mm), the defaultMergeRadius applies. For farther objects, a larger radius might be applied. */
    // GlobalOpponentsHypothesis::determineAndSetType
    (int) teamThreshold,                       /**< Only switch team if this threshold is reached. */
    (int) uprightThreshold,                    /**< Only switch upright/fallen if this threshold is reached. */
    (float) goalAreaIgnoreTolerance,           /**< Tolerance around the goal area in mm for ignoring obstacles while the goalkeeper walks in at the beginning of the half. 0 turns it off. */
  }),
});

/**
 * @class GlobalOpponentsTracker
 *
 * An implementation that aims to keep track of the opponent robots that are currently on the pitch.
 */
class GlobalOpponentsTracker : public GlobalOpponentsTrackerBase
{
public:
  /** Constructor */
  GlobalOpponentsTracker();
  std::vector<GlobalOpponentsHypothesis, Eigen::aligned_allocator<GlobalOpponentsHypothesis>> obstacleHypotheses; /**< List of obstacles. */
  std::vector<bool> merged; /**< This is to merge obstacles once for every "percept" per frame. */
  // Used for writing annotations only once per contact.
  bool armContact[Arms::numOfArms] = { false, false }, footContact[Legs::numOfLegs] = { false, false };


private:
  int numberOfUnpenalizedOpponents;          /**< The number of opponent robots that are currently in play (== not penalized) */
  int numberOfPenalizedOpponents;            /**< The number of opponent robots that are currently penalized and thus assumed to be standing outside the actual playing area */

  std::vector<Geometry::Rect> penalizedRobotZonesOpponentTeam;     /**< The areas in which penalized robots of the opponent team are placed during the penalty */
  std::vector<Geometry::Rect> returnFromPenaltyZonesOpponentTeam;  /**< The areas in which unpenalized robots of the opponent team are placed to return to the game */

  /**
   * Fills the representation provided by this module. Some computations are done by
   * the other update method, which provides the GlobalTeammatesModel first.
   * @param globalOpponentsModel The representation
   */
  void update(GlobalOpponentsModel& globalOpponentsModel) override;

  /**
  * Actually fills the representation provided by this module
  * @param globalOpponentsModel The representation
  */
  void fillModel(GlobalOpponentsModel& globalOpponentsModel);

  /** Updates internal representations based on GameController information */
  void updateGameInfo();

  /** Draws internal data */
  void draw();


  /**
   * The function decides whether the obstacle model should be calculated or not
   * and deletes the model and hypotheses if it should not be calculated.
   *
   * @param obstacleModel The representation to be updated.
   * @return Whether the obstacle model should be calculated.
   */
  bool clearAndFinish(GlobalOpponentsModel& globalOpponentsModel);

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
  void tryToMerge(const GlobalOpponentsHypothesis& measurement);

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
  bool isAnyObstacleInShadow(GlobalOpponentsHypothesis* closer, const std::size_t i, const float cameraAngleLeft, const float cameraAngleRight);

  /**
   * Computes a merge radius for a given measurement. The farther the measurement, the higher the radius.
   * @param measurement A position in 2D that denotes a mearuement of an opponent
   * @return A radius (in mm)
   */
  float calculateMergeRadius(const Vector2f& measurement) const
  {
    const float distance = measurement.norm();
    // If the measurement is not too far away, the default merging distance is applied:
    if(distance < rangeForDefaultMergeRadius)
      return defaultMergeRadius;
    // Otherwise, the radius grows linearly to up to maxMergeRadius
    else
      return mapToRange(distance, rangeForDefaultMergeRadius, maxOpponentDistance, defaultMergeRadius, maxMergeRadius);
  }

  /**
   * Checks whether this obstacle hypothesis should be ignored.
   * Currently checks for obstacles close to the goal area, while the
   * goal keeper is walking in at the beginning of the half.
   */
  bool shouldIgnore(const GlobalOpponentsHypothesis& obstacle) const;
};
