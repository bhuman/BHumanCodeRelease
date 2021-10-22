/**
 * @file KickoffStateProvider.h
 *
 * Module for encapsulating the functions and states for managing kickoff situations.
 *
 * @author Tim Laue
 * @author Andreas Stolpmann
 */

#include "Representations/BehaviorControl/KickoffState.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/ExtendedGameInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/GyroState.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBuffer.h"

MODULE(KickoffStateProvider,
{,
  REQUIRES(BallModel),
  REQUIRES(BallSpecification),
  REQUIRES(ExtendedGameInfo),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(MotionInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RobotPose),
  REQUIRES(TeamBallModel),
  REQUIRES(GyroState),
  PROVIDES(KickoffState),
  LOADS_PARAMETERS(
  {,
    (int) ballOutOfCenterCircleCounterThreshold,   /**< The number of times the ball needs to be seen at a certain position */
    (float) ballOutOfCenterCircleTolerance,        /**< Distance added to center circle radius for determining, if the ball is out of the center circle */
    (float) robotOutOfCenterCircleTolerance,       /**< Distance added to center circle radius for determining, if the robot is close enough to decide whether the ball is out of the center circle */
    (int) opponentKickoffMaxTime,                  /**< Maximum time (in ms) for opponent team to perform kickoff and for own team to wait outside the center circle */
    (int) ballSaveInterval,                        /**< Time between two ball measurements that are added to the buffer */
    (float) ballHasMovedTolerance,                 /**< Distance away from kick-off point the ball needs to be seen to be considered as moved */
    (float) ballHasMovedCloseToRobotThreshold,     /**< If the ball is less than a center circle radius minus this threshold away it is assumed to have been moved by the opponent. */
    (bool) alwaysAllowToScore,
    (float) gyroThreshold,                         /**< maximum Value of the deviation of the y-gyro to assume that the robot stands still. */
  }),
});

class KickoffStateProvider : public KickoffStateProviderBase
{
private:
  int ballOutOfCenterCircleCounter;                /**< Number of ball sightings out of center circle in playing state */
  RingBuffer<Vector2f, 30> ballPositions;          /**< Buffering the ball positions during opponent's kickoff for computing a possible ball motion */
  unsigned int lastBallAddedToBuffer;              /**< Point of time when the ballPositions buffer has been changed (by adding) the last time */
  bool ballHasMoved;                               /**< Are we allowed to enter the center circle? */
  bool ballWasOutOfCenterCircle;                   /**< Are we allowed to score a goal? */

  /** Reset the state eg. set ballHasMoved to false... */
  void reset();

  /** Returns the vector with the median length */
  const Vector2f& medianOfThree(const Vector2f& a, const Vector2f& b, const Vector2f& c) const;

  bool allowedToScore();

  bool allowedToEnterCenterCircle();

  void update(KickoffState& kickoffState) override;
public:
  /** Constructor for initializing all members*/
  KickoffStateProvider();
};
