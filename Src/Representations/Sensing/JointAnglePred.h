/**
 * @file JointAnglePred.h
 *
 * The file declares a struct that contains a prediction for the leg joint angles.
 *
 * @author Jan Fiedler
 */

#pragma once

#include "Debugging/Plot.h"
#include "RobotParts/Joints.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/Enum.h"

STREAMABLE(JointAnglePred,
{
  void draw() const
  {
    PLOT("representation:JointAnglePred:hipYawPitch", angles[Joints::hipYawPitch].toDegrees());
    PLOT("representation:JointAnglePred:lHipRoll", angles[Joints::lHipRoll].toDegrees());
    PLOT("representation:JointAnglePred:lHipPitch", angles[Joints::lHipPitch].toDegrees());
    PLOT("representation:JointAnglePred:lKneePitch", angles[Joints::lKneePitch].toDegrees());
    PLOT("representation:JointAnglePred:lAnklePitch", angles[Joints::lAnklePitch].toDegrees());
    PLOT("representation:JointAnglePred:lAnkleRoll", angles[Joints::lAnkleRoll].toDegrees());
    PLOT("representation:JointAnglePred:rHipRoll", angles[Joints::rHipRoll].toDegrees());
    PLOT("representation:JointAnglePred:rHipPitch", angles[Joints::rHipPitch].toDegrees());
    PLOT("representation:JointAnglePred:rKneePitch", angles[Joints::rKneePitch].toDegrees());
    PLOT("representation:JointAnglePred:rAnklePitch", angles[Joints::rAnklePitch].toDegrees());
    PLOT("representation:JointAnglePred:rAnkleRoll", angles[Joints::rAnkleRoll].toDegrees());
  },

  (std::string) modelName,  /**< The file name from the currently loaded model.*/
  (ENUM_INDEXED_ARRAY(Angle, Joints::Joint)) angles, /**< The angles of all joints if the model output is valid. */
  (bool)(false) isValid, /**< Whether the current output of the model is valid. The data should not be used if this is not the case, as the results are undefined. */
});
