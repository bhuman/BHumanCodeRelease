/**
 * @file GlobalRobotsTracker.h
 *
 * Declaration of a module the aims to track all robots on the pitch
 * by fusing local estimates with information sent by teammates and by the GameController.
 *
 * @author Tim Laue
 */

#pragma once

#include "Modules/Modeling/SelfLocator/SelfLocator.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/GlobalOpponentsModel.h"
#include "Representations/Modeling/GlobalTeammatesModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesFieldPercept.h"
#include "Representations/Sensing/FallDownState.h"
#include "Math/Geometry.h"

MODULE(GlobalRobotsTracker,
{,
  REQUIRES(CameraMatrix),
  REQUIRES(FallDownState),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(MotionInfo),
  REQUIRES(ObstaclesFieldPercept),
  REQUIRES(ObstacleModel),
  REQUIRES(RobotPose),
  REQUIRES(TeamData),
  REQUIRES(GlobalTeammatesModel),
  PROVIDES(GlobalOpponentsModel),
  PROVIDES(GlobalTeammatesModel),
  DEFINES_PARAMETERS(
  {,
    (int)(3000) maximumAgeOfObservationSet,          /**< A set of observations cannot be older that this value. */
    (float)(153.f) borderThreshold,                  /**< Observations that are this close to the field border become ignored. */
    (float)(154.) teammateExclusionDistThreshold,    /**< Any observation that is closer than this to a teammate is excluded as it can be assumed that it is a wrong observation of a teammate. */
    (float)(702.f) reinitThreshold,                  /**< Threshold for resetting an existing estimate to a recently communicated one, if it is farther away than this threshold */
    (float)(1337.f*1337.f) squaredDistanceThreshold, /**< When matching percepts with estimates, anything farther away than this cannot be integrated. */
  }),
});

/**
 * @class GlobalRobotsTracker
 * A combined world model
 */
class GlobalRobotsTracker : public GlobalRobotsTrackerBase
{
public:
  /** Constructor */
  GlobalRobotsTracker();

private:
  /**
  * An internal representation for a single observation of an opponent
  * currently tracked by one robot (of our team)
  */
  class OpponentObservation
  {
  public:
    Vector2f position;         /**< The position of a tracked robot (in global field coordinates) */
    Pose2f observerPose;       /**< The pose of the observing robot */
    float distanceOfObserver;  /**< The distance to the observing robot */
    unsigned lastSeen;         /**< The point of time when the robot was seen the last time */
  };

  /** Status of a teammate */
  ENUM(TeammateState,
  {,
    invalid,                 /**< Never received any data from this robot. Does not seem to exist at all. */
    penalized,               /**< Currently penalized */
    waitingAfterUnpenalize,  /**< Teammate has returned from penalty and is now entering the game from the sideline. We did not receive any data after the penalty yet.*/
    playing,                 /**< Teammate is actively playing */
  });

  /**
  * An internal representation for a teammate
  */
  class TeammateInformation
  {
  public:
    /** Default Constructor */
    TeammateInformation() : state(invalid) {}

    /** Updates the position estimate given a measurement of the position
     *  Performs simple 2D Kalman filter update.
     * @param mP The measurement of the position (in global field coordinates)
     * @param mC The covariance of the measurement.
     */
    void measurementUpdate(const Vector2f& mP, const Matrix2f& mC)
    {
      Vector2f& x = estimatedPosition;
      Matrix2f& C = covariance;
      Matrix2f covPlusSensorCov = C;
      covPlusSensorCov += mC;
      const Matrix2f K = C * covPlusSensorCov.inverse();
      const Vector2f innovation = mP - x;
      const Vector2f correction = K * innovation;
      x += correction;
      C -= K * C;
      Covariance::fixCovariance(covariance);
    }

    Vector2f estimatedPosition;             /**< The estimated teammate position given its communicated target and local perceptions of robots */
    float lastCommunicatedRotation;         /**< The last communicated rotation (maybe useless but nice for drawing ;-)) */
    Matrix2f covariance;                    /**< The pose covariance (implementation might change soon) */
    Vector2f extrapolatedPositionLastFrame; /**< The teammate position in the last frame, given its communicated target only. Use for computing motion offsets */
    unsigned lastCommunicationUpdate;       /**< The point of time when the last data was received from the teammate */
    TeammateState state;                    /**< Current state of my teammate */
    bool perceivedInThisFrame;              /**< Flag that indicates an integrated perception */
    Vector2f walkTargetOnField;             /**< The target position the robot is walking to (in absolute field coordinates). Based on communicated BehaviorStatus. */
    float speed;                            /**< The absolute speed in mm/s. Copied from communicated BehaviorStatus. */
  };

  /**
  * An internal representation for a perceived teammate
  */
  class TeammatePercept
  {
  public:
    Vector2f pos;      /**< The position of the perceived teammate in global field coordinates */
    Matrix2f cov;      /**< The measurement covariance of the perceived position in global field coordinates */
  };

  /**
   * An internal representation for a set of opponents that are
   * currently tracked by one robot (of our team)
   */
  class ObservedSetOfOpponents
  {
  public:
    /** Constructor */
    ObservedSetOfOpponents(){ lastUpdate = 0; }

    /** Draws the opponents with the given color
     * @param color The color
     */
    void draw(const ColorRGBA& color) const;

    std::vector<OpponentObservation, Eigen::aligned_allocator<OpponentObservation>> observations;  /**< A list of currently tracked opponents */
    unsigned lastUpdate;                            /**< The last point of time at which I received information from this robot */
  };

  std::vector<TeammatePercept> teammatePercepts;                    /**< A list of all perceptions of teammates in the current frame */
  std::vector<ObservedSetOfOpponents> opponentsObservedByTeammates; /**< A list of opponent sets. For each teammate (that is currently communicating, one entry is in this list. There are also unused entries for the robot itself as well as for the substitute. */
  ObservedSetOfOpponents opponentsObservedByMe;                     /**< Obstacle information that has ben computed locally. It is stored separately for some reasons. */
  std::vector<TeammateInformation> ownTeam;                           /**< The current information about all teammates (including me) */
  std::vector<OpponentObservation> pooledObservations;              /**< A list of observations of my team. Outliers have already been removed. This list is the input for the clustering. */

  int numberOfUnpenalizedOpponents;          /**< The number of opponent robots that are currently in play (== not penalized) */
  int numberOfPenalizedOpponents;            /**< The number of opponent robots that are currently penalized and thus assumed to be standing outside the actual playing area */
  int numberOfPenalizedTeammates;            /**< The number of teammates that are currently penalized and thus assumed to be standing outside the actual playing area */
  int numberOfTeammatesReturningFromPenalty; /**< The number of teammates that are currently returning from a penalty and thus assumed to be standing on the sideline */

  std::vector<Geometry::Rect> penalizedRobotZonesOpponentTeam;     /**< The areas in which penalized robots of the opponent team are placed during the penalty */
  std::vector<Geometry::Rect> returnFromPenaltyZonesOpponentTeam;  /**< The areas in which unpenalized robots of the opponent team are placed to return to the game */

  std::vector<Geometry::Rect> penalizedRobotZonesOwnTeam;          /**< The areas in which penalized robots of the own team are placed during the penalty */
  std::vector<Geometry::Rect> returnFromPenaltyZonesOwnTeam;       /**< The areas in which unpenalized robots of the own team are placed to return to the game */

  SelfLocator::Parameters selfLocatorParameters;       /**< Access self locator parameters to avoid duplicated configuration elements */

  /**
   * Fills the representation provided by this module. Some computations are done by
   * the other update method, which provides the GlobalTeammatesModel first.
   * @param globalOpponentsModel The representation
   */
  void update(GlobalOpponentsModel& globalOpponentsModel) override;

  /**
   * Fills the representation provided by this module and
   * calls all internal functions to compute the informatik.
   * @param globalOpponentsModel The representation
   */
  void update(GlobalTeammatesModel& globalTeammatesModel) override;

  /**
  * Actually fills the representation provided by this module
  * @param globalOpponentsModel The representation
  */
  void fillModel(GlobalOpponentsModel& globalOpponentsModel);

  /**
  * Actually fills the representation provided by this module
  * @param globalTeammatesModel The representation
  */
  void fillModel(GlobalTeammatesModel& globalTeammatesModel);

  /** Updates internal representations based on GameController and teammate information */
  void updateGameAndTeammateInfo();

  /** Converts local obstacle information to the internal representation for opponents */
  void updateLocalOpponentObservations();

  /** Converts the obstacle information sent by the teammates to the internal representations for opponents */
  void updateTeamOpponentObservations();

  /**
   * Tool function used to convert a list of obstacle objects to the internal opponent set representation
   * Only opponent obstacles are considered, all other elements are ignored.
   * @param obstacles A list of obstacles
   * @param robotPose The pose of the observing robot
   * @param setOfOpponents A reference to the internal structure for storing information about opponents
   */
  void obstacleModelToSetofOpponents(const std::vector<Obstacle>& obstacles, const Pose2f& robotPose,
                                     ObservedSetOfOpponents& setOfOpponents);

  /**
   * Checks all currently available observations and adds the valid ones to a list that
   * will be used for the clustering.
   */
  void computePoolOfObservations();

  /**
   * Perform some checks, if a single observation should be considered for the final assignment step.
   * Observations could be included, if they are at the field border or at the position of a penalized robot, for instance.
   * return true, if the observation seems to be usable.
   */
  bool observationCanBeUsed(const OpponentObservation& observation);

  /**
   * Perform some checks, if a percept should be considered for merging with an existing hypothesis.
   * Perceptions are excluded, if they are outside the carpet or at the position of a penalized robot, for instance.
   * return true, if the observation seems to be usable.
   */
  bool teammatePerceptCanBeUsed(const Vector2f& p);

  /** The core of this module: Takes all the information that is currently available
   *  and tries to solve the chaos :-)
   */
  void computeInternalModelOfOpponents();

  /** Searches the teammates in TeamData for the entry of a robot that has the given playerNumber.
   * @param playerNumber The number of the robot
   * @return A pointer do the entry or nullptr, if there is currently no teammate that has the given number.
   */
  const Teammate* findTeammateByNumber(int playerNumber) const;

  /** Reset all values of the internal teammate representation based on communicated data.
   *  Happens at the beginning of a half or after a penalty.
   *  @param tmi The internal representation
   *  @param teammateData Communicated information
   */
  void initializeTeammateInformation(TeammateInformation& tmi, const Teammate& teammateData);

  /** Update the internal teammate representation based on communicated data.
   *  Affects pose and covariance.
   *  @param tmi The internal representation
   *  @param teammateData Communicated information
   */
  void updateTeammateInformation(TeammateInformation& tmi, const Teammate& teammateData);

  /** Matches robot perceptions to current estimates and updates the position estimates accordingly.*/
  void integratePerceivedTeammates();

  /** Implements a distance metric for comparing a teammate estimate and a perception of a teammate
   *  @param tmi The teammate estimate
   *  @param tp The teammate percept
   *  @return squaredDistanceThreshold, if both are too far way from each other, the Mahalanobis distance betwenn both normal distributions otherwise.
   */
  float distanceTeammateAndPerception(const TeammateInformation& tmi, const TeammatePercept& tp);

  /** Draws internal data */
  void draw();
};
