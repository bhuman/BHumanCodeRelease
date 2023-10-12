/**
 * @file JointAnglePred.h
 *
 * The file declares a struct that contains a prediction for the leg joint angles.
 */

#pragma once

#include "Debugging/Plot.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/Enum.h"

STREAMABLE(JointAnglePred,
{
  ENUM(Joint, // TODO: This order is from pybh and should be corrected later.
  {,
    rAnkleRoll,
    rAnklePitch,
    rKneePitch,
    rHipPitch,
    rHipRoll,

    lKneePitch,
    lAnkleRoll,
    lAnklePitch,
    lHipPitch,
    lHipRoll,

    hipYawPitch,
  });

  void draw() const
  {
    PLOT("representation:JointAnglesPred:hipYawPitch", angles[JointAnglePred::hipYawPitch].toDegrees());
    PLOT("representation:JointAnglesPred:lHipRoll", angles[JointAnglePred::lHipRoll].toDegrees());
    PLOT("representation:JointAnglesPred:lHipPitch", angles[JointAnglePred::lHipPitch].toDegrees());
    PLOT("representation:JointAnglesPred:lKneePitch", angles[JointAnglePred::lKneePitch].toDegrees());
    PLOT("representation:JointAnglesPred:lAnklePitch", angles[JointAnglePred::lAnklePitch].toDegrees());
    PLOT("representation:JointAnglesPred:lAnkleRoll", angles[JointAnglePred::lAnkleRoll].toDegrees());
    PLOT("representation:JointAnglesPred:rHipRoll", angles[JointAnglePred::rHipRoll].toDegrees());
    PLOT("representation:JointAnglesPred:rHipPitch", angles[JointAnglePred::rHipPitch].toDegrees());
    PLOT("representation:JointAnglesPred:rKneePitch", angles[JointAnglePred::rKneePitch].toDegrees());
    PLOT("representation:JointAnglesPred:rAnklePitch", angles[JointAnglePred::rAnklePitch].toDegrees());
    PLOT("representation:JointAnglesPred:rAnkleRoll", angles[JointAnglePred::rAnkleRoll].toDegrees());
  },

  (std::string) modelName,  /**< The file name from the currently loaded model.*/
  (ENUM_INDEXED_ARRAY(Angle, JointAnglePred::Joint)) angles, /**< The angles of all joints. */
});
