/**
 * @file TeamBallModelProvider.h
 *
 * Declares a class that provides a ball model that fuses information
 * from my team and my observations.
 *
 * @author Tim Laue
 * @contributor Liam Hurwitz
 */

#pragma once

#include "Representations/Communication/ReceivedTeamMessages.h"
#include "Representations/Communication/SentTeamMessage.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/WorldModelPrediction.h"
#include "Framework/Module.h"

MODULE(TeamBallModelProvider,
{,
  REQUIRES(BallSpecification),
  REQUIRES(ExtendedGameState),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(ReceivedTeamMessages),
  REQUIRES(SentTeamMessage),
  REQUIRES(WorldModelPrediction),
  PROVIDES(TeamBallModel),
  LOADS_PARAMETERS(
  {,
    (int) ballLastSeenTimeout,                /**< After this amount of time (in ms), a ball is not considered anymore */
    (int) ballDisappearedTimeout,             /**< After this amount of time (in ms), a disappeared ball is not valid anymore */
    (int) timestampTolerance,                 /**< Maximum difference (in ms) between two timestamps to still considered to be the same */
  }),
});

/**
 * @class TeamBallModelProvider
 * The module that computes the TeamBallModel
 */
class TeamBallModelProvider : public TeamBallModelProviderBase
{
public:
  /** Information about a ball observed by a member of my team*/
  struct BufferedBall
  {
    Pose2f robotPose;                  /**< The pose of the observing robot */
    float poseQualityModifier = 0.f;   /**< Quality of the pose, robot pose, used for computing the weight of an observation */
    Vector2f pos = Vector2f::Zero();   /**< Position of the ball (relative to the observer) */
    Vector2f vel = Vector2f::Zero();   /**< Velocity of the ball (relative to the observer) */
    unsigned time = 0;                 /**< Point of time (in ms) of the observation */
    bool valid = false;                /**< This observation can be considered */
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
  std::vector<BufferedBall> balls;                         /**< A buffer for all observations made by my team */
  std::vector<ActiveBall> ballsAvailableForTeamBall;       /**< List of all balls that are suitable for computing a team ball */
  unsigned int timeLastBall;                               /**< Timestamp of last sent ball, 0 means that no position has been sent */

  /** Main method that triggers the model computation
   * @param teamBallModel A reference to the representation that is filled
   */
  void update(TeamBallModel& teamBallModel) override;

  /**
   * Updates the internal ball buffers with new information from team messages.
   *
   * This function iterates over all received team messages and updates the internal ball buffers accordingly.
   * It also updates the ball buffer for the last sent ball position.
   */
  void updateInternalBallBuffers();

  /** Updates the internal list of currently available balls */
  void findAvailableBalls();

  /** Check, which ball is compatible to which other ball, fill the compatibleBallsIndices lists */
  void clusterBalls();

  /** Uses the list of available balls to determine the current team ball
   * @param teamBallModel A reference to the representation that is filled
   */
  void computeModel(TeamBallModel& teamBallModel);

  /** Maps current localization quality to a floating point number.
   *  This function determines, how strong the self-localization should influence a ball weighting.
   * @param quality The assumed precision/trustworthiness of a robot pose estimate
   * @return A weighting in the range [0,..,1]
   */
  float localizationQualityToModifier(const RobotPose::LocalizationQuality quality) const;

  /** Computes the weighting for a given ball observation
   * @param ball The ball observation
   * @return A weighting that says how "good" the observation is
   */
  float computeWeighting(const BufferedBall& ball) const;

  /** In some situations (game is not in PLAY or ball was out), the ball
   *  becomes replaced by humans. Thus, we should reset all stored information then.
   *  @return true, if all buffers have been reset.
   */
  bool checkForResetByGameSituation();

  /**
   * Checks, if a message is relevant for computing the location of the ball position.
   *
   * @param ballModel The ball model to check.
   * @param robotStatus The robot status to check.
   * @param frameInfo The frame info to check.
   * @param ballIndex The index of the ball. This function expects the caller to deduct Settings::lowestValidPlayerNumber from the player number.
   *
   * @return True, if the message is relevant, false otherwise.
   */
  bool isMessageRelevantForBallPosition(const BallModel& ballModel, const RobotStatus& robotStatus, const FrameInfo& frameInfo, int ballIndex);
};
