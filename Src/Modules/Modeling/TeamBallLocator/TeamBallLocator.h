/**
 * @file TeamBallLocator.h
 *
 * Declares a class that provides a ball model that incorporates information
 * from my teammates.
 *
 * @author Tim Laue
 */

#pragma once

#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CognitionStateChanges.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBuffer.h"

const size_t BALL_BUFFER_LENGTH = 10;

MODULE(TeamBallLocator,
{,
  REQUIRES(BallSpecification),
  REQUIRES(CognitionStateChanges),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(BallModel),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  REQUIRES(TeamData),
  PROVIDES(TeamBallModel),
  LOADS_PARAMETERS(
  {,
    (int) inactivityInvalidationTimeSpan,     /**< Minimum time to go back in ball buffer (in case of a problem) */
    (int) ballLastSeenTimeout,                /**< After this amount of time (in ms), a ball is not considered anymore */
  }),
});

/**
 * @class TeamBallLocator
 * The module that computes the TeamBallModel
 */
class TeamBallLocator : public TeamBallLocatorBase
{
public:
  /** Information about a ball observed by a member of my team*/
  struct BufferedBall
  {
    Pose2f robotPose;                  /**< The pose of the observing robot */
    float poseValidity;                /**< Validity of the pose, as estimated by self-localization */
    Vector2f pos = Vector2f::Zero();   /**< Position of the ball (relative to the observer) */
    Vector2f vel = Vector2f::Zero();   /**< Velocity of the ball (relative to the observer) */
    unsigned time;                     /**< Point of time (in ms) of the observation */
    bool valid;                        /**< This observation can be considered */
    bool seenByMixedTeamPartner;       /**< True, if ball was not seen by B-Human software, only relevant for Mixed Team */
  };

  /** A ball that is a candidate for becoming (a part of the) team ball */
  struct ActiveBall
  {
    Vector2f position;                            /**< The ball position in field coordinates */
    Vector2f velocity;                            /**< The ball velocity in field coordinates */
    unsigned char playerNumber;                   /**< The player number of the observer */
    float weighting;                              /**< Weighting (somehow represents quality) used for merging */
    unsigned time;                                /**< Point of time (in ms) of the observation */
    std::vector<unsigned> compatibleBallsIndices; /**< Indices of other balls that can be clustered with this one */
  };

private:
  std::vector<RingBuffer<BufferedBall, BALL_BUFFER_LENGTH>> balls; /**< A buffer for all observations made by my teammates */
  std::vector<ActiveBall> ballsAvailableForTeamBall;               /**< List of all balls that are suitable for computing a team ball */

  /** Main method that triggers the model computation */
  void update(TeamBallModel& teamBallModel) override;

  /** Adds new information to the ball buffers */
  void updateInternalBallBuffers();

  /** Updates the internal list of currently available balls
   * @param considerMixedTeammates Use balls from non-B-Human robots in Mixed Team Competition, if set to true
   */
  void findAvailableBalls(bool considerMixedTeammates);

  /** Check, which ball is compatible to which other ball, fill the compatibleBallsIndices lists */
  void clusterBalls();

  /** Uses the list of available balls to determine the current team ball
   * @param teamBallModel A reference to the representation that is filled
   */
  void computeModel(TeamBallModel& teamBallModel);

  /** Computes the weighting for a given ball observation
   * @param ball The ball observation
   * @return A weighting that says how "good" the observation is
   */
  float computeWeighting(const BufferedBall& ball) const;

  /** In some situatione (game is not in PLAY or ball was out), the ball
   *  becomes replaced by humans. Thus, we should reset all stored information then.
   *  @return true, if all buffers have been reset.
   */
  bool checkForResetByGameSituation();
};
