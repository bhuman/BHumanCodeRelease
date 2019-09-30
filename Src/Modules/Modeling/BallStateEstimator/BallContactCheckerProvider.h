/**
 * @file BallContactCheckerProvider.h
 *
 * In ball tracking, the collisions between ball and robot feet need to be handled properly
 * for computing correct velocities and ball positions.
 *
 * The module declared in this file is provides a representation that contains
 * a function for checking a contact between a single ball hypothesis and a robot foot.
 *
 * @author Tim Laue
 */

#pragma once

#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Modeling/BallContactChecker.h"
#include "Representations/Modeling/WorldModelPrediction.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Tools/Module/Module.h"

MODULE(BallContactCheckerProvider,
{,
  REQUIRES(BallSpecification),
  REQUIRES(FrameInfo),
  REQUIRES(JointAngles),
  REQUIRES(MotionInfo),
  REQUIRES(RobotModel),
  REQUIRES(TorsoMatrix),
  REQUIRES(WorldModelPrediction),
  PROVIDES(BallContactChecker),
  LOADS_PARAMETERS(
  {,
    (float) footRadius,            /**< The radius of the approximated foot shape */
    (float) footMass,              /**< An assumed mass for each foot (in kg) */
    (float) ballMass,              /**< The mass of the ball (in kg) */
    (Vector3f) leftFootOffset,     /**< Offset from left foot sole point to a "center" of the foot used for approximating the foot shape with a circle */
    (Vector2f) kickDeviation,      /**< The percentage inaccuracy of passed velocities */
    (int) timeout,                 /**< If the ball was not seen for this number of milliseconds, do not perform any collision detection */
  }),
});

class BallContactCheckerProvider : public BallContactCheckerProviderBase
{
public:
  /** Constructor */
  BallContactCheckerProvider();

private:
  Vector2f lastLeftFootCenter;         /**< The foot's center at the last execution of this module */
  Vector2f lastRightFootCenter;        /**< The foot's center at the last execution of this module */
  Vector2f leftFootCenter;
  Vector2f rightFootCenter;
  float deltaTime;                     /**< Time difference in seconds to previous frame */
  unsigned int lastMotionFrameTime;    /**< The point of time at the last execution of this module */

  /**
   * Add some comment here
   */
  void update(BallContactChecker& ballContactChecker);

  bool handleCollisionWithFeet(const Vector2f& leftFootCenter, const Vector2f& rightFootCenter,
                               const Vector2f& position, const Vector2f& velocity, const Vector2f& lastPosition,
                               BallContactInformation& contactInfo);
  bool collisionForFootCalculation(Vector2f& assumedLastBallPosition, Vector2f& assumedBallOffset, const Vector2f& ballPosition, const Vector2f& lastBallPosition, const Vector2f& footCenter, const Vector2f& lastFootCenter, const float radius, float& collisionFactor);
};
