/**
* @file TeamBallLocator.h
*
* Declares a class that provides a ball model that incorporates information
* from my teammates.
*
* @author Tim Laue
*/

#pragma once

#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/TeammateReliability.h"
#include "Tools/RingBuffer.h"
#include "Tools/Module/Module.h"

const size_t BALL_BUFFER_LENGTH = 10;

MODULE(TeamBallLocator,
{,
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(BallModel),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  REQUIRES(TeammateReliability),
  REQUIRES(TeammateData),
  PROVIDES(TeamBallModel),
  LOADS_PARAMETERS(
  {,
    (int) inactivityInvalidationTimeSpan,     /**< minimum time to go back in ball buffer (in case of a problem)*/
    (int) ballLastSeenTimeout,                /**< After this amount of time (in ms), a ball is not considered anymore*/
    (int) ballDisappearedTimeout,             /**< After this amount of time (in ms), a ball is not considered anymore*/
    (float) scalingFactorBallSinceLastSeen,   /**< Parameter for sigmoid function */
    (float) scalingFactorBallDisappeared,     /**< Parameter for sigmoid function */
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
    Pose2f robotPose;             /**< The pose of the observing robot */
    Vector2f pos = Vector2f::Zero();                /**< Position of the ball (relative to the observer) */
    Vector2f vel = Vector2f::Zero();                /**< Velocity of the ball (relative to the observer) */
    unsigned time;                /**< Point of time (in ms) of the observation */
    unsigned timeWhenDisappeared; /**< Point of time (in ms) of a theoretically possible observation that did not occur*/
    bool valid;                   /**< This observation can be considered */
  };

private:
  std::vector<RingBuffer<Ball, BALL_BUFFER_LENGTH>> balls; /** A buffer for all observations made by my teammates*/

  /** Main method that triggers the model computation */
  void update(TeamBallModel& teamBallModel);

  /** Adds new information to the ball buffer */
  void updateBalls();

  /** Computes the model based on information in the buffers
   * @param pos A reference to a position that is set by this method
   * @param vel A reference to a velocity that is set by this method
   * @return true, if position and velocity have been computed. false, otherwise.
   */
  bool computeTeamBallModel(Vector2f& pos, Vector2f& vel) const;

  /** Computes the weighting for a given ball observation
   * @param ball The ball observation
   * @return A weighting that says how "good" the observation is
   */
  float computeWeighting(const Ball& ball) const;
};
