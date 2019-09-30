#pragma once

#include "Tools/RobotParts/Joints.h"
#include "Tools/Streams/EnumIndexedArray.h"
#include "Tools/Debugging/Debugging.h"

STREAMABLE(StiffnessData,
{
  enum {useDefault = -1};

  /** The constructor resets all data to its default value. */
  StiffnessData();

  /**
   * The method returns the stiffness of the mirror (left/right) of the given joint.
   * @param joint The joint the mirror of which is returned.
   * @return The output stiffness of the mirrored joint.
   */
  int mirror(const Joints::Joint& joint) const;

  /** Initializes this instance with the mirrored values of other  */
  void mirror(const StiffnessData& other);

  /** This function resets the stiffness for all joints to the default value. */
  void resetToDefault();

  /** Checks wheather all stiffnesses are in the rangel [0, 100] or have the value useDefault. */
  bool isValid(bool allowUseDefault = true) const,

  (ENUM_INDEXED_ARRAY(int, Joints::Joint)) stiffnesses, /**< The custom stiffnesses for each joint (in %). Range: [0, 100]. */
});

STREAMABLE_WITH_BASE(StiffnessSettings, StiffnessData,
{,
});

inline StiffnessData::StiffnessData()
{
  resetToDefault();
}

inline int StiffnessData::mirror(const Joints::Joint& joint) const
{
  switch(joint)
  {
    case Joints::lShoulderPitch:
    case Joints::lShoulderRoll:
    case Joints::lElbowYaw:
    case Joints::lElbowRoll:
    case Joints::lWristYaw:
    case Joints::lHand:
      return stiffnesses[joint - Joints::lShoulderPitch + Joints::rShoulderPitch];
    case Joints::rShoulderPitch:
    case Joints::rShoulderRoll:
    case Joints::rElbowYaw:
    case Joints::rElbowRoll:
    case Joints::rWristYaw:
    case Joints::rHand:
      return stiffnesses[joint - Joints::rShoulderPitch + Joints::lShoulderPitch];
    case Joints::lHipYawPitch:
    case Joints::lHipRoll:
    case Joints::lHipPitch:
    case Joints::lKneePitch:
    case Joints::lAnklePitch:
    case Joints::lAnkleRoll:
      return stiffnesses[joint - Joints::lHipYawPitch + Joints::rHipYawPitch];
    case Joints::rHipYawPitch:
    case Joints::rHipRoll:
    case Joints::rHipPitch:
    case Joints::rKneePitch:
    case Joints::rAnklePitch:
    case Joints::rAnkleRoll:
      return stiffnesses[joint - Joints::rHipYawPitch + Joints::lHipYawPitch];
    default:
      return stiffnesses[joint];
  }
}

inline void StiffnessData::mirror(const StiffnessData& other)
{
  for(int i = 0; i < Joints::numOfJoints; ++i)
    stiffnesses[i] = other.mirror(static_cast<Joints::Joint>(i));
}

inline void StiffnessData::resetToDefault()
{
  stiffnesses.fill(useDefault);
};

inline bool StiffnessData::isValid(bool allowUseDefault) const
{
  bool isValid = true;
  for(unsigned i = 0; i < Joints::numOfJoints; i++)
    if(stiffnesses[i] > 100 || (stiffnesses[i] < 0 && ((stiffnesses[i] != useDefault) || !allowUseDefault)))
    {
      OUTPUT_ERROR("Stiffness from Joint " << TypeRegistry::getEnumName(Joints::Joint(i)) << " is invalid");
      isValid = false;
    }
  return isValid;
}
