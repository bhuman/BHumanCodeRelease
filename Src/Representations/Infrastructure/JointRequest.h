#pragma once

#include "JointAngles.h"
#include "StiffnessData.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Debugging/Debugging.h"
#include <cmath>

STREAMABLE_WITH_BASE(JointRequest, JointAngles,
{
  JointRequest();

  /** Initializes this instance with the mirrored data of other. */
  void mirror(const JointRequest& other);

  /** Returns the mirrored angle of joint. */
  Angle mirror(const Joints::Joint joint) const;

  /** Checkes if the JointRequest is valide. */
  bool isValid() const,

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

inline JointRequest::JointRequest()
{
  angles.fill(off);
}

inline void JointRequest::mirror(const JointRequest& other)
{
  JointAngles::mirror(other);
  stiffnessData.mirror(other.stiffnessData);
}

inline Angle JointRequest::mirror(const Joints::Joint joint) const
{
  return JointAngles::mirror(joint);
}

inline bool JointRequest::isValid() const
{
  bool isValid = true;
  for(unsigned i = 0; i < Joints::numOfJoints; i++)
    if(!std::isfinite(angles[i]))
    {
      OUTPUT_ERROR("Joint " << TypeRegistry::getEnumName(Joints::Joint(i)) << " is invalid");
      isValid = false;
    }
  return stiffnessData.isValid() && isValid;
}
