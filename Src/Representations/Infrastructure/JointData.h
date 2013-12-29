/**
* @file Representations/Infrastructure/JointData.h
*
* This file declares a classes to represent the joint angles.
*
* @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
*/

#pragma once

#include "Tools/Math/Common.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Enum.h"

/**
* @class JointData
* A class to represent the joint angles sent to the robot.
*/
STREAMABLE(JointData,
{
public:
  ENUM(Joint,
    HeadYaw,
    HeadPitch,
    LShoulderPitch,
    LShoulderRoll,
    LElbowYaw,
    LElbowRoll,
    RShoulderPitch,
    RShoulderRoll,
    RElbowYaw,
    RElbowRoll,
    LHipYawPitch,
    LHipRoll,
    LHipPitch,
    LKneePitch,
    LAnklePitch,
    LAnkleRoll,
    RHipYawPitch,
    RHipRoll,
    RHipPitch,
    RKneePitch,
    RAnklePitch,
    RAnkleRoll
  );

  // If you change those values be sure to change them in MofCompiler.cpp too. (Line ~280)
  enum {off = 1000}; /**< Special angle for switching off a joint. */
  enum {ignore = 2000}; /**< Special angle for not overwriting the previous setting. */

  /**
  * The method returns the angle of the mirror (left/right) of the given joint.
  * @param joint The joint the mirror of which is returned.
  * @return The angle of the mirrored joint.
  */
  float mirror(Joint joint) const
  {
    switch(joint)
    {
      // don't mirror an invalid joint value (!)
    case HeadYaw:
      return angles[HeadYaw] == off || angles[HeadYaw] == ignore ? angles[HeadYaw] : -angles[HeadYaw];
    case LShoulderPitch:
      return angles[RShoulderPitch];
    case LShoulderRoll:
      return angles[RShoulderRoll];
    case LElbowYaw:
      return angles[RElbowYaw];
    case LElbowRoll:
      return angles[RElbowRoll];
    case RShoulderPitch:
      return angles[LShoulderPitch];
    case RShoulderRoll:
      return angles[LShoulderRoll];
    case RElbowYaw:
      return angles[LElbowYaw];
    case RElbowRoll:
      return angles[LElbowRoll];
    case LHipYawPitch:
      return angles[RHipYawPitch];
    case LHipRoll:
      return angles[RHipRoll];
    case LHipPitch:
      return angles[RHipPitch];
    case LKneePitch:
      return angles[RKneePitch];
    case LAnklePitch:
      return angles[RAnklePitch];
    case LAnkleRoll:
      return angles[RAnkleRoll];
    case RHipYawPitch:
      return angles[LHipYawPitch];
    case RHipRoll:
      return angles[LHipRoll];
    case RHipPitch:
      return angles[LHipPitch];
    case RKneePitch:
      return angles[LKneePitch];
    case RAnklePitch:
      return angles[LAnklePitch];
    case RAnkleRoll:
      return angles[LAnkleRoll];
    default:
      return angles[joint];
    }
  }

  /**
  * The method initializes the joint angles as a mirror of a set of other joint angles.
  * @param other The set of joint angles that are mirrored.
  */
  void mirror(const JointData& other)
  {
    for(int i = 0; i < numOfJoints; ++i)
      angles[i] = other.mirror((Joint) i);
    timeStamp = other.timeStamp;
  },

  (float[numOfJoints]) angles, /**< The angles of all joints. */
  (unsigned)(0) timeStamp, /**< The time when these angles were received. */

  // Initialization
  for(int i = 0; i < numOfJoints; ++i)
    angles[i] = off;
});


/**
 * @class HardnessData
 * This class represents the joint hardness in a jointRequest.
 * It loads the default hardness values from hardnessSettings.cfg.
 */
STREAMABLE(HardnessData,
{
public:
  enum {useDefault = -1};

  /**
  * The method returns the hardness of the mirror (left/right) of the given joint.
  * @param joint The joint the mirror of which is returned.
  * @return The output hardness of the mirrored joint.
  */
  int mirror(const JointData::Joint joint) const
  {
    switch(joint)
    {
    case JointData::HeadYaw:
      return hardness[JointData::HeadYaw];
    case JointData::LShoulderPitch:
      return hardness[JointData::RShoulderPitch];
    case JointData::LShoulderRoll:
      return hardness[JointData::RShoulderRoll];
    case JointData::LElbowYaw:
      return hardness[JointData::RElbowYaw];
    case JointData::LElbowRoll:
      return hardness[JointData::RElbowRoll];
    case JointData::RShoulderPitch:
      return hardness[JointData::LShoulderPitch];
    case JointData::RShoulderRoll:
      return hardness[JointData::LShoulderRoll];
    case JointData::RElbowYaw:
      return hardness[JointData::LElbowYaw];
    case JointData::RElbowRoll:
      return hardness[JointData::LElbowRoll];
    case JointData::LHipYawPitch:
      return hardness[JointData::RHipYawPitch];
    case JointData::LHipRoll:
      return hardness[JointData::RHipRoll];
    case JointData::LHipPitch:
      return hardness[JointData::RHipPitch];
    case JointData::LKneePitch:
      return hardness[JointData::RKneePitch];
    case JointData::LAnklePitch:
      return hardness[JointData::RAnklePitch];
    case JointData::LAnkleRoll:
      return hardness[JointData::RAnkleRoll];
    case JointData::RHipYawPitch:
      return hardness[JointData::LHipYawPitch];
    case JointData::RHipRoll:
      return hardness[JointData::LHipRoll];
    case JointData::RHipPitch:
      return hardness[JointData::LHipPitch];
    case JointData::RKneePitch:
      return hardness[JointData::LKneePitch];
    case JointData::RAnklePitch:
      return hardness[JointData::LAnklePitch];
    case JointData::RAnkleRoll:
      return hardness[JointData::LAnkleRoll];
    default:
      return hardness[joint];
    }
  }

  /**
   * initializes this instance with the mirrored values of other
   * @param other the HardnessData to be mirrored
   */
  void mirror(const HardnessData& other)
  {
    for(int i = 0; i < JointData::numOfJoints; ++i)
      hardness[i] = other.mirror((JointData::Joint)i);
  }

  /**
   * This function resets the hardness for all joints to the default value.
   */
  inline void resetToDefault()
  {
    for(int i = 0; i < JointData::numOfJoints; ++i)
      hardness[i] = useDefault;
  },

  (int[JointData::numOfJoints]) hardness, /**< the custom hardness for each joint */

  // Initialization
  resetToDefault();
});

class HardnessSettings : public HardnessData {};

/**
 * @class JointRequest
 */
STREAMABLE_WITH_BASE(JointRequest, JointData,
{
public:
  /**
   * Initializes this instance with the mirrored data from a other JointRequest
   * @param other the JointRequest to be mirrored
   */
  void mirror(const JointRequest& other)
  {
    JointData::mirror(other);
    jointHardness.mirror(other.jointHardness);
  }

  /**
   * Returns the mirrored angle of joint
   * @param joint the joint to be mirrored
   */
  float mirror(const JointData::Joint joint)
  {
    return JointData::mirror(joint);
  }

  bool isValid() const
  {
    for(int i = 0; i < numOfJoints; ++i)
      if(isnan(angles[i]) || jointHardness.hardness[i] < 0 || jointHardness.hardness[i] > 100)
        return false;
    return true;
  },

  (HardnessData) jointHardness, /**< the hardness for all joints */
});

class FilteredJointData : public JointData {};
