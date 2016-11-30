#pragma once

#include "JointAngles.h"
#include "StiffnessData.h"
#include "Tools/Streams/AutoStreamable.h"
#include <cmath>

STREAMABLE_WITH_BASE(JointRequest, JointAngles,
{
  JointRequest();

  /** Initializes this instance with the mirrored data of other. */
  void mirror(const JointRequest& other);

  /** Returns the mirrored angle of joint. */
  Angle mirror(const Joints::Joint joint) const;

  /** Checkes if the JointRequest is valide. */
  bool isValid() const;
  ,
  (StiffnessData) stiffnessData, /**< the stiffness for all joints */
});

struct ArmJointRequest : public JointRequest {};
struct LegJointRequest : public JointRequest
{
  LegJointRequest() : JointRequest()
  {
    angles[0] = JointAngles::ignore;
    angles[1] = JointAngles::ignore;
  };
};

struct StandArmRequest : public ArmJointRequest {};
struct StandLegRequest : public LegJointRequest {};

struct WalkArmRequest : public ArmJointRequest {};
struct WalkLegRequest : public LegJointRequest {};

struct NonArmeMotionEngineOutput : public JointRequest {};

inline JointRequest::JointRequest() :
  JointAngles()
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
  for(const Angle& angle : angles)
    if(!std::isfinite(angle))
      return false;
  return stiffnessData.isValid();
}
