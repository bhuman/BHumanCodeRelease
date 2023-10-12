#pragma once

#include "JointAngles.h"
#include "StiffnessData.h"
#include "Streaming/AutoStreamable.h"
#include "Debugging/Debugging.h"
#include <cmath>

STREAMABLE_WITH_BASE(JointRequest, JointAngles,
{
  JointRequest();

  void draw();

  /** Initializes this instance with the mirrored data of other. */
  void mirror(const JointRequest& other);

  /** Returns the mirrored angle of joint. */
  Angle mirror(Joints::Joint joint) const;

  /** Checks if the JointRequest is valid. */
  bool isValid(bool allowUseDefault = true) const,

  (StiffnessData) stiffnessData, /**< the stiffness for all joints */
});
