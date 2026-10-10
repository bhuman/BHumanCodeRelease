#pragma once

#include "Math/Eigen.h"
#include "RobotParts/Joints.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/EnumIndexedArray.h"
#include "Representations/MotionControl/JointStiffnessMap.h"

STREAMABLE(MotorParameters,
{
  STREAMABLE(ControlParameter,
  {,
    (float) p,         /**< p factor for a joint controller. */
    (float) d,         /**< d factor for a joint controller. */
    (float) maxTorque, /**< If torque is set, it is clipped to this maximum value. */
  }),

  (ENUM_INDEXED_ARRAY(ENUM_INDEXED_ARRAY(ControlParameter, Joints::Joint), JointStiffnessMapping::Type)) controlParameters, /**< The controller factors for the different joints. */
  (ENUM_INDEXED_ARRAY(float, Joints::Joint)) dDamping,
});
