/**
 * @file TeamBallLocator.h
 *
 * Declares a class that provides a ball model that incorporates information
 * from my teammates.
 *
 * @author Tim Laue
 */

#pragma once

#include "Representations/Communication/TeamData.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CognitionStateChanges.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Tools/RingBuffer.h"
#include "Tools/Module/Module.h"

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
  struct Ball
  {
    Pose2f robotPose;                  /**< The pose of the observing robot */
    float poseValidity;                /**< Validity of the pose, as estimated by self-localization */
    Vector2f pos = Vector2f::Zero();   /**< Position of the ball (relative to the observer) */
    Vector2f vel = Vector2f::Zero();   /**< Velocity of the ball (relative to the observer) */
    unsigned time;                     /**< Point of time (in ms) of the observation */
    bool velocityIsValid;              /**< Whether the velocity information of this ball is valid */
    bool valid;                        /**< This observation can be considered */
  };

private:
  std::vector<RingBuffer<Ball, BALL_BUFFER_LENGTH>> balls;                /**< A buffer for all observations made by my teammates */
  int numberOfBallsByMe;                                                  /**< Keep track of who has contributed to the current team ball */
  int numberOfBallsByOthers;                                              /**< Keep track of who has contributed to the current team ball */
  std::vector<TeamBallModel::ConsideredBall> ballsConsideredForTeamBall;  /**< List of all balls that are merged to a team ball */
  std::vector<float> weightingsOfBallsConsideredForTeamBall;              /**< Additional information, entries correspond to ballsConsideredForTeamBall */

  /** Main method that triggers the model computation */
  void update(TeamBallModel& teamBallModel) override;

  /** Adds new information to the ball buffers */
  void updateInternalBallBuffers();

  /** Computes the model based on information in the buffers
   * @param pos A reference to a position that is set by this method
   * @param vel A reference to a velocity that is set by this method
   * @return The point of time when the most recent integrated ball was seen. 0, if no ball was considered.
   */
  unsigned computeTeamBallModel(Vector2f& pos, Vector2f& vel);

  /** Computes the weighting for a given ball observation
   * @param ball The ball observation
   * @return A weighting that says how "good" the observation is
   */
  float computeWeighting(const Ball& ball) const;
  
  /** In some situatione (game is not in PLAY or ball was out), the ball
   *  becomes replaced by humans. Thus, we should reset all stored information then.
   *  @return true, if all buffers have been reset.
   */
  bool checkForResetByGameSituation();
};
