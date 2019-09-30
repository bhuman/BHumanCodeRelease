#pragma once

#include "JointAngles.h"
#include "StiffnessData.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Debugging/Debugging.h"
#include <cmath>

STREAMABLE_WITH_BASE(JointRequest, JointAngles,
{
  JointRequest();

  void draw();

  /** Initializes this instance with the mirrored data of other. */
  void mirror(const JointRequest& other);

  /** Returns the mirrored angle of joint. */
  Angle mirror(Joints::Joint joint) const;

  /** Checkes if the JointRequest is valide. */
  bool isValid(bool allowUseDefault = true) const,

  (StiffnessData) stiffnessData, /**< the stiffness for all joints */
});

STREAMABLE_WITH_BASE(HeadJointRequest, JointRequest,
{,
});

STREAMABLE_WITH_BASE(ArmJointRequest, JointRequest,
{,
});

STREAMABLE_WITH_BASE(LegJointRequest, JointRequest,
{
  LegJointRequest()
  {
    angles[0] = JointAngles::ignore;
    angles[1] = JointAngles::ignore;
  },
});

STREAMABLE_WITH_BASE(StandArmRequest, ArmJointRequest,
{,
});

STREAMABLE_WITH_BASE(StandLegRequest, LegJointRequest,
{,
});

STREAMABLE_WITH_BASE(NonArmeMotionEngineOutput, JointRequest,
{,
});
