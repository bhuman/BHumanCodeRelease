/**
* Request for the arm motion engine.
* @author <a href="mailto:simont@tzi.de>Simon Taddiken</a>
*/
#pragma once
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Enum.h"

/**
* Class that represents the possible arm motions that can be requested from
* the robot.
*/
STREAMABLE(ArmMotionRequest,
{
public:
  ENUM(Arm,
    left,
    right
  );

  /** Existing arm motions. Ordering must match ordering in armMotionEngine.cfg */
  ENUM(ArmMotionId,
    useDefault,         /**< No explicit arm motion, so WalkingEngine's arm angles will be used */
    back,               /**< Move arm to the back */
    falling,            /**< Emergency motion to save arm when falling */
    keeperStand         /**< Arm position for the keeper when guarding the goal */
  ),

  (ArmMotionId[numOfArms]) motion,   /**< Motion to execute */
  (bool[numOfArms]) fast,            /**< Whether states should not be interpolated */
  (bool[numOfArms]) autoReverse,     /**< Whether arms should be moved back to default position after certain amount of time */
  (int[numOfArms]) autoReverseTime,  /**< Time (in motion frames) after which the arm is moved back to default position if autoReverse is true. */

  motion[left] = useDefault;
  motion[right] = useDefault;
  fast[left] = false;
  fast[right] = false;
  autoReverse[left] = false;
  autoReverse[right] = false;
  autoReverseTime[left] = 0;
  autoReverseTime[right] = 0;
});