/**
 * @file JointDynamics.h
 *
 * This file declares a representation of the dynamic state of the joints.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Math/Angle.h"
#include "RobotParts/Joints.h"
#include "Streaming/EnumIndexedArray.h"

STREAMABLE(JointDynamics,
{,
  (ENUM_INDEXED_ARRAY(Angle, Joints::Joint)) angles, /**< The joint angles (in rad). */
  (ENUM_INDEXED_ARRAY(Angle, Joints::Joint)) velocities, /**< The joint velocities (in rad/s). */
  (ENUM_INDEXED_ARRAY(Angle, Joints::Joint)) accelerations, /**< The joint accelerations (in rad/s^2). */
  (ENUM_INDEXED_ARRAY(float, Joints::Joint)) torques, /**< The predicted torques acting on the joints, given a stationary torso and no external forces besides gravity (in uNmm). */
});
