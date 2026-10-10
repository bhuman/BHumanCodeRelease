#pragma once

#include "RobotParts/Joints.h"
#include "Streaming/EnumIndexedArray.h"
#include "Debugging/Debugging.h"

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

  /** Checks whether all stiffnesses are in the range [0, 100] or have the value useDefault. */
  bool isValid(bool allowUseDefault = true) const,

  (ENUM_INDEXED_ARRAY(int, Joints::Joint)) stiffnesses, /**< The custom stiffnesses for each joint (in %). Range: [0, 100]. */
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
    case Joints::lElbowPitch:
    case Joints::lElbowYaw:
      return stiffnesses[joint - Joints::firstLeftArmJoint + Joints::firstRightArmJoint];
    case Joints::rShoulderPitch:
    case Joints::rShoulderRoll:
    case Joints::rElbowPitch:
    case Joints::rElbowYaw:
      return stiffnesses[joint - Joints::firstRightArmJoint + Joints::firstLeftArmJoint];
    case Joints::lHipPitch:
    case Joints::lHipRoll:
    case Joints::lHipYaw:
    case Joints::lKneePitch:
    case Joints::lAnklePitch:
    case Joints::lAnkleRoll:
      return stiffnesses[joint - Joints::firstLeftLegJoint + Joints::firstRightLegJoint];
    case Joints::rHipPitch:
    case Joints::rHipRoll:
    case Joints::rHipYaw:
    case Joints::rKneePitch:
    case Joints::rAnklePitch:
    case Joints::rAnkleRoll:
      return stiffnesses[joint - Joints::firstRightLegJoint + Joints::firstLeftLegJoint];
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
