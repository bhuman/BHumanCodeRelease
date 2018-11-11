/**
 * @file BallContactDetector.h
 *
 * In ball tracking, the collisions between ball and robot feet need to be handled properly
 * for computing correct velocities and ball positions.
 *
 * The module declared in this file is based on the implementation that has been used by
 * B-Human inside the BallLocator module in recent years.
 *
 * @author Tim Laue
 */

#pragma once

#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Modeling/BallContactWithRobot.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/WorldModelPrediction.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Tools/Module/Module.h"

MODULE(BallContactDetector,
{,
  REQUIRES(BallSpecification),
  REQUIRES(FrameInfo),
  REQUIRES(JointAngles),
  REQUIRES(MotionInfo),
  REQUIRES(RobotModel),
  REQUIRES(TorsoMatrix),
  REQUIRES(WorldModelPrediction),
  USES(BallModel),
  PROVIDES(BallContactWithRobot),
  DEFINES_PARAMETERS(
  {,
    (float)(45.f) footRadius,                  /**< The radius of the approximated foot shape */
    (float)(0.1f) footMass,                    /**< An assumed mass for each foot (in kg) */
    (float)(0.05f) ballMass,                   /**< The mass of the ball (in kg) */
    (Vector3f)(65.f, 0.f, 0.f) leftFootOffset, /**< Offset from left foot sole point to a "center" of the foot used for approximating the foot shape with a circle */
    (Vector2f)(1.f, 1.f) kickDeviation,        /**< The percentage inaccuracy of passed velocities */
    (int)(2000) timeout,                       /**< If the ball was not seen for this number of milliseconds, do not perform any collision detection */
  }),
});

/**
 * @class BallContactDetector
 *
 * Some basic techniques to get rid off false positive BallPercepts.
 */
class BallContactDetector : public BallContactDetectorBase
{
public:
  /** Constructor */
  BallContactDetector();

private:
  Vector2f lastLeftFootCenter;         /**< The foot's center at the last execution of this module */
  Vector2f lastRightFootCenter;        /**< The foot's center at the last execution of this module */
  float deltaTime;                     /**< Time difference in seconds to previous frame */
  unsigned int lastMotionFrameTime;    /**< The point of time at the last execution of this module */
  bool footCollisionInLastMotionFrame; /**< Whether the foot collided with the ball in the last frame that had a motion update */

  /**
   * Performs collision detection and updates the representation
   * @param filteredBallPercepts The data structure that is filled
   */
  void update(BallContactWithRobot& ballContactWithRobot) override;

  bool collisionForFootCalculation(Vector2f& assumedLastBallPosition, Vector2f& assumedBallOffset, const Vector2f& ballPosition, const Vector2f& lastBallPosition, const Vector2f& footCenter, const Vector2f& lastFootCenter, const float radius, float& collisionFactor);
  void handleCollisionWithFeet(const Vector2f& leftFootCenter, const Vector2f& rightFootCenter, BallContactWithRobot& ballContactWithRobot);

};
