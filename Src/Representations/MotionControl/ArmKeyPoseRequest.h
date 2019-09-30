/**
 * Request for the key pose engine.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */
#pragma once
#include "Tools/Streams/EnumIndexedArray.h"

/**
 * Class that represents the possible arm pose-motions that can be requested from
 * the robot.
 */
STREAMABLE(ArmKeyPoseRequest,
{
  /** Existing arm motions. Ordering must match the ordering in armPoseEngine.cfg */
  ENUM(ArmKeyPoseMotionId,
  {,
    nullPose, /**< Basic position */
    back,     /**< Move arm to the back */
    front,    /**< Move arm to the front */
    topBack,  /**< Move arm to the top back */
    topFront, /**< Move arm to the top front */
    outerWavePos,
    innerWavePos,
    numOfArmKeyPoseIds,
    wave = numOfArmKeyPoseIds,
  });

  STREAMABLE(Arm,
  {,
    ((ArmKeyPoseRequest) ArmKeyPoseMotionId)(nullPose) motion, /**< Motion to execute */
    (float)(1) speed, /**< The speed of the motion */
  }),

  (ENUM_INDEXED_ARRAY(Arm, Arms::Arm)) arms,
});
