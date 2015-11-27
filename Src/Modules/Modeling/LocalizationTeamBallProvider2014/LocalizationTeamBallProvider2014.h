/**
* @file LocalizationTeamBallProvider2014.h
*
* Declaration of a module that computes a combined ball model based on
* the reliability of teammates and the consistency of their obervations.
*
* @author <A href="mailto:tlaue@uni-bremen.de">Tim Laue</A>
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Modeling/LocalizationTeamBall.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Tools/RingBuffer.h"

MODULE(LocalizationTeamBallProvider2014,
{,
  REQUIRES(TeammateData),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  PROVIDES(LocalizationTeamBall),
  LOADS_PARAMETERS(
  {,
    (int)   timeout,              /**< Maximum age (in milliseconds) of a ball model that can be integrated */
    (float) maxBallVelocity,      /**< Threshold for excluding rolling balls (in mm/s) */
    (float) clusterThreshold,     /**< Maximum distance for joining to hypotheses (needs smarter solution in the future */
    (bool)  preferGoalieClusters, /**< If true, only clusters that contain the goalie are considered (in case the goalie saw the ball) */
  }),
});

/**
 * @class LocalizationTeamBallProvider2014
 *
 * Computes a combined and consistent ball model
 */
class LocalizationTeamBallProvider2014 : public LocalizationTeamBallProvider2014Base
{

  /** All necessary information about a seen ball*/
  struct BallObservation
  {
    int robotNumber;                /**< The observer's player number */
    bool isGoalkeeper;              /**< Yes, exactly what you think */
    Pose2f robotPose;               /**< The observer's pose on the field */
    Vector2f ballPositionAbsolute;  /**< The ball's position on the field in absolute coordinates */
    unsigned int timeOfObservation; /**< The The point of time when the ball was seen */
    float sideConfidence;           /**< The observer's side confidence */
  };

  /** A cluster of seen balls */
  struct TeamBallHypothesis
  {
    std::vector<int> observationIndizes;       /** The indizes of the observations in the "balls" vector */
  };

  std::vector<BallObservation> balls;          /** The list of recent observations */
  std::vector<TeamBallHypothesis> hypotheses;  /** The list of hypotheses (aka clusters) */
  bool goalieSawTheBall;                       /** True, if the goalie is among the recent observers */

  /**
  * Provides the combined world model representation
  */
  void update(LocalizationTeamBall& localizationTeamBall);

  /** Maintains the list of currently available ball observations of team mates */
  void updateObservations();

  /** Updates a list entry or adds a new entry */
  void addObservationToList(const BallObservation& ball);

  /** Clusters the recent observation and creates hypotheses */
  void clusterObservations();

  /** Compares two obvervations and indicates if both seem to refer to the same ball */
  bool observationsAreCompatible(const BallObservation& a, const BallObservation& b);

  /** Converts cluster information to the LocalizationTeamBall class */
  void computeTeamBallFromBestCluster(LocalizationTeamBall& localizationTeamBall);
};
