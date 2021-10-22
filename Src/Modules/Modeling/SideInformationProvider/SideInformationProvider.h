/**
 * @file SideInformationProvider.h
 *
 * Declaration of the module SideInformationProvider.
 * The SPL field is symmetric and perceiving the field elements alone does not
 * allow to infer the correct playing direction.
 * This module computes some information
 * that might help to solve this problem in some situations.
 * One part is to compute, if a robot MUST be inside its ow half given the walked distance and the current rules of the game.
 * The other part is a flip detection based on a comparison of own and team observations of the ball.
 *
 * @author Tim Laue
 */

#pragma once

#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/SetupPoses.h"
#include "Representations/Configuration/StaticInitialPose.h"
#include "Representations/Infrastructure/ExtendedGameInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/SideInformation.h"
#include "Representations/Modeling/WorldModelPrediction.h"
#include "Representations/Sensing/FallDownState.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBuffer.h"

MODULE(SideInformationProvider,
{,
  USES(RobotPose),
  REQUIRES(BallModel),
  REQUIRES(FieldDimensions),
  REQUIRES(ExtendedGameInfo),
  REQUIRES(FallDownState),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(Odometer),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RobotInfo),
  REQUIRES(SetupPoses),
  REQUIRES(StaticInitialPose),
  REQUIRES(WorldModelPrediction),
  PROVIDES(SideInformation),
  REQUIRES(TeamData),
  LOADS_PARAMETERS(
  {,
    (float) centerBanZoneRadius,           /**< All balls closer to the field's center than this are ignored */
    (float) fieldLineBanDistance,          /**< All balls that are too close to a line or a penalty mark become ignored */
    (float) maxBallVelocity,               /**< Maximum velocity of balls that are considered for side confidence computation */
    (int)   maxBallAge,                    /**< Maximum age (in milliseconds) of a ball model that can be integrated */
    (int)   agreementTimeout,              /**< Time after which an agreement is removed from the list, if it has not been updated */
    (float) maxBallAgreementDistance,      /**< Between balls that are considered to be "the same" should not be a distance larger than this */
    (bool)  deactivateGoalieFlipping,      /**< If true, goalie does not set the mirror flag */
    (bool)  goalieIsAlwaysRight,           /**< If true, (dis)agreement of the goalie is always used to decide the confidence */
    (int)   minAgreementCount,             /**< Minimum number of required agreements for considering an agreemate */
    (int)   jumpRemovalTimeout,            /**< If a teammate has jumped recently (or I have jumped), agreements become deleted */
    (float) distanceUncertaintyOffset,     /**< Estimated odometry and localization base error (mm). */
    (float) distanceUncertaintyFactor,     /**< Estimated odometry error as a factor of the distance walked. */
    (float) awayFromLineDistance,          /**< The distance the robot has to be before or behind a certain line (mm). */
    (bool)  forceOwnHalf,                  /**< Robot is always in the own half (due do some special circumstances, like a demo or a technical challenge) */
    (bool)  forceOpponentHalf,             /**< Robot is always in the opponent half (due do some special circumstances, like a demo or a technical challenge) */
  }),
});

/**
* @class SideInformationProvider
*
* Computation of information that might help to overcome some field symmetry problems.
*/
class SideInformationProvider : public SideInformationProviderBase
{
public:
  /** Constructor */
  SideInformationProvider();

private:
  /** Description of the status of a mutual ball observation */
  ENUM(TeammateBallCompatibility,
  {,
    Agree,    /**< Both robots see the ball at the same position */
    Mirror,   /**< The observations are at positions point-symmetric to the field's center */
    Unknown,  /**< The observations do not match at all. That is not cool, but might happen */
  });

  /** Structure for describing the mutual ball observation with a teammate */
  struct Agreemate
  {
    int number;                                   /**< The player number of the teammate */
    bool isGoalkeeper;                            /**< My teammate is our goalkeeper, information used to privilige its opinion */
    unsigned timeOfLastAgreement;                 /**< Last time at which my teammate and I saw the ball */
    TeammateBallCompatibility ballCompatibility;  /**< Status of our mutual observation */
    int agreementCount;                           /**< Number of times (in a row) that we both observed the ball and had the same status */
    Vector2f lastGlobalBallPosition;              /**< Field position at which my teammate observed the ball (for visualization only) */
  };

  std::vector<Agreemate> agreemates;                    /**< Internal list that keeps the status of the recent mutual ball observations of me and my teammates */
  unsigned timeWhenLastBallReplacingSetPlayStarted = 0; /**< The last timestamp when a set play started in which a ball is replaced. */
  float distanceWalkedAtKnownPosition;                  /**< The robot walked this far at its last known position. */
  float largestXPossibleAtKnownPosition;                /**< The largest x coordinate possible at its last known position. */

  /** Main function of this module, calls all other functions
   *  @param sideInformation The representation provided by this module
   */
  void update(SideInformation& sideInformation) override;

  /** Extra function to determine, if the robot's side appears to have changed
   *  based on information from teammates about the ball
   *  @param sideInformation The representation provided by this module
   */
  void setMirrorFlagOfRepresentation(SideInformation& sideInformation);

  /** Check all teammates, if they recently saw the ball at the same position or at the mirrored position */
  void findAgreemates();

  /** Clear data that is too old or might be invalid */
  void removeOldAndJumpedAgreemates();

  /** When a match with a teammate was found, this function adds the mutual
   *  observation to the internal list.
   *  @param teammate The teammate that also observed the ball
   *  @param ballCompatibility Information about whether both robots agree on the same ball position or somehow disagree
   */
  void addToAgreemateList(const Teammate& teammate, TeammateBallCompatibility ballCompatibility);

  /** Compares the own obervation of the ball with an observation of a teammate
  *  @param globalBallPosition The position (in global field coordinates) at which I saw the ball
  *  @param globalBallPositionMirrored Same as globalBallPosition but turned around the field's center by 180 degrees (precompute to save some computing time)
  *  @param teammateGlobalBallPosition The position (in global field coordinates) at which my teammate saw the ball
  *  @return Information about the status of the mutual observation (agree / mirror / unknown)
  */
  TeammateBallCompatibility compareBallWithTeammateBall(const Vector2f& globalBallPosition,
                                                        const Vector2f& globalBallPositionMirrored,
                                                        const Vector2f& teammateGlobalBallPosition) const;

  /** Performs a set of basic checks to exclude false positives
   *  @param ballModel The most recent estimate of a ball
   *  @param robotPose The most recent estimate of a robot's pose
   *  @return true, if all checks are negative
   */
  bool ballModelCanBeUsed(const BallModel& ballModel, const Pose2f& robotPose) const;

  /** What the name says, eh. Used to exclude ball observations at places
   *  at which false positives are more likely
   *  @param b The ball position that is to be checked
   *  @return true, if the given position is close to a penalty mark or a line
   */
  bool ballIsCloseToPenaltyMarkOrLine(const Vector2f& b) const;

  /** Checks, how far into the opponent half the robot could have walked after it entered the game.
   *  Furthermore, it is checked, if the robot is probably still inside the own half of the field.
   *  Includes the code of the former "OwnSideModelProvider", i.e. computes the stuff unrelated to the teammates.
   *  @param SideInformation The representation of which some fields are filled by this function
   */
  void computeBasicOwnSideInformation(SideInformation& SideInformation);

  /** Visualization of some internal data */
  void draw();
};
