#pragma once

#include "Tools/Joints.h"
#include "Tools/SensorData.h"
#include "Tools/Math/Angle.h"
#include "Tools/Streams/AutoStreamable.h"

#include <array>

STREAMABLE(JointAngles,
{
public:
  enum {off = SensorData::off}; /**< Special value that indicates that the joint is turned off. */ // TODO replace with constexpr
  enum {ignore = 20000}; /**< Special angle for not overwriting the previous setting. */ // TODO replace with constexpr

  JointAngles();

  /**
   * The method returns the angle of the mirror (left/right) of the given joint.
   * @param joint The joint the mirror of which is returned.
   * @return The angle of the mirrored joint.
   */
  Angle mirror(Joints::Joint joint) const;

  /** Initializes this instance with the mirrored angles of other. */
  void mirror(const JointAngles & other);

private:
  /**
   * Negate joint angle if it is a legal one.
   * @param angle The angle to negate. Can also be "off" or "ignore".
   * @return The mirrored joint angle.
   */
  static float mirror(float angle);

public:
  ,
  (std::array<Angle, Joints::numOfJoints>) angles, /**< The angles of all joints. */
  (unsigned)(0) timestamp, /**< The time when the jointangles were received*/
});

inline JointAngles::JointAngles()
{
  angles.fill(0.f);
}

inline Angle JointAngles::mirror(Joints::Joint joint) const
{
  switch(joint)
  {
    case Joints::headYaw:
      return mirror(angles[Joints::headYaw]);
    case Joints::lShoulderPitch:
    case Joints::lHand:
      return angles[joint - Joints::lShoulderPitch + Joints::rShoulderPitch];
    case Joints::lElbowRoll:
    case Joints::lShoulderRoll:
    case Joints::lElbowYaw:
    case Joints::lWristYaw:
      return mirror(angles[joint - Joints::lShoulderPitch + Joints::rShoulderPitch]);
    case Joints::rShoulderPitch:
    case Joints::rHand:
      return angles[joint - Joints::rShoulderPitch + Joints::lShoulderPitch];
    case Joints::rElbowRoll:
    case Joints::rShoulderRoll:
    case Joints::rElbowYaw:
    case Joints::rWristYaw:
      return mirror(angles[joint - Joints::rShoulderPitch + Joints::lShoulderPitch]);
    case Joints::lHipYawPitch:
    case Joints::lHipPitch:
    case Joints::lKneePitch:
    case Joints::lAnklePitch:
      return angles[joint - Joints::lHipYawPitch + Joints::rHipYawPitch];
    case Joints::lHipRoll:
    case Joints::lAnkleRoll:
      return mirror(angles[joint - Joints::lHipYawPitch + Joints::rHipYawPitch]);
    case Joints::rHipYawPitch:
    case Joints::rHipPitch:
    case Joints::rKneePitch:
    case Joints::rAnklePitch:
      return angles[joint - Joints::rHipYawPitch + Joints::lHipYawPitch];
    case Joints::rHipRoll:
    case Joints::rAnkleRoll:
      return mirror(angles[joint - Joints::rHipYawPitch + Joints::lHipYawPitch]);
    default:
      return angles[joint];
  }
}

inline void JointAngles::mirror(const JointAngles& other)
{
  for(int i = 0; i < Joints::numOfJoints; ++i)
    angles[i] = other.mirror(static_cast<Joints::Joint>(i));
  timestamp = other.timestamp;
}

inline float JointAngles::mirror(float angle)
{
  return (angle == off || angle == ignore) ? angle : -angle;
}