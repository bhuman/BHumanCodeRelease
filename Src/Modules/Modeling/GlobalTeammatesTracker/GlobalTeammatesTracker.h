/**
 * @file GlobalTeammatesTracker.h
 *
 * Declaration of a module the aims to track all teammates on the pitch
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
#include "Representations/Modeling/GlobalTeammatesModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesFieldPercept.h"
#include "Math/Geometry.h"

MODULE(GlobalTeammatesTracker,
{,
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(ObstaclesFieldPercept),
  REQUIRES(RobotPose),
  REQUIRES(TeamData),
  REQUIRES(GlobalTeammatesModel),
  PROVIDES(GlobalTeammatesModel),
  DEFINES_PARAMETERS(
  {,
    (float)(153.f) borderThreshold,                    /**< Observations that are this close to the field border become ignored. */
    (float)(702.f) reinitThreshold,                    /**< Threshold for resetting an existing estimate to a recently communicated one, if it is farther away than this threshold */
    (float)(1337.f * 1337.f) squaredDistanceThreshold, /**< When matching percepts with estimates, anything farther away than this cannot be integrated. */
  }),
});

/**
 * @class GlobalTeammatesTracker
 * A combined world model
 */
class GlobalTeammatesTracker : public GlobalTeammatesTrackerBase
{
public:
  /** Constructor */
  GlobalTeammatesTracker();

private:
  /** Status of a teammate */
  ENUM(TeammateState,
  {,
    invalid,                 /**< Never received any data from this robot. Does not seem to exist at all. */
    penalized,               /**< Currently penalized */
    waitingAfterUnpenalize,  /**< Teammate has returned from penalty and is now entering the game from the touchline. We did not receive any data after the penalty yet.*/
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
      Covariance::fixCovariance<2>(covariance);
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
    bool isGoalkeeper;                      /**< Guess, in which situation this flag might be true. */
  };

  /**
   * An internal representation for a perceived teammate
   */
  class TeammatePercept
  {
  public:
    Vector2f pos;      /**< The position of the perceived teammate in global field coordinates */
    Matrix2f cov;      /**< The measurement covariance of the perceived position in global field coordinates */
    bool isGoalkeeper; /**< Guess, in which situation this flag might be true. */
  };

  std::vector<TeammatePercept> teammatePercepts;              /**< A list of all perceptions of teammates in the current frame */
  std::vector<TeammateInformation> ownTeam;                   /**< The current information about all teammates (including me) */
  int numberOfPenalizedTeammates;                             /**< The number of teammates that are currently penalized and thus assumed to be standing outside the actual playing area */
  int numberOfTeammatesReturningFromPenalty;                  /**< The number of teammates that are currently returning from a penalty and thus assumed to be standing on the touchline */
  std::vector<Geometry::Rect> penalizedRobotZonesOwnTeam;     /**< The areas in which penalized robots of the own team are placed during the penalty */
  std::vector<Geometry::Rect> returnFromPenaltyZonesOwnTeam;  /**< The areas in which unpenalized robots of the own team are placed to return to the game */
  SelfLocator::Parameters selfLocatorParameters;              /**< Access self locator parameters to avoid duplicated configuration elements */

  /**
   * Fills the representation provided by this module and
   * calls all internal functions to compute the informatik.
   * @param globalOpponentsModel The representation
   */
  void update(GlobalTeammatesModel& globalTeammatesModel) override;

  /**
   * Actually fills the representation provided by this module
   * @param globalTeammatesModel The representation
   */
  void fillModel(GlobalTeammatesModel& globalTeammatesModel);

  /** Updates internal representations based on GameController and teammate information */
  void updateGameAndTeammateInfo();

  /**
   * Perform some checks, if a percept should be considered for merging with an existing hypothesis.
   * Perceptions are excluded, if they are outside the carpet or at the position of a penalized robot, for instance.
   * return true, if the observation seems to be usable.
   */
  bool teammatePerceptCanBeUsed(const Vector2f& p);

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
   *  @return squaredDistanceThreshold, if both are too far way from each other, the Mahalanobis distance between both normal distributions otherwise.
   */
  float distanceTeammateAndPerception(const TeammateInformation& tmi, const TeammatePercept& tp);

  /** Draws internal data */
  void draw();
};
