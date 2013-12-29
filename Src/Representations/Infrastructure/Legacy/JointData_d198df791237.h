/**
* @file JointData_d198df791237.h
*
* This file declares a class to represent the joint angles sent to the robot.
*
* @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
*/

#pragma once

#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Common.h"
#include "Tools/Debugging/Debugging.h"
#include "Platform/BHAssert.h"
#include "Tools/Enum.h"
/**
* @class JointData_d198df791237
* A class to represent the joint angles sent to the robot.
*/
class JointData_d198df791237 : public Streamable
{
protected:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(angles);
    STREAM(timeStamp);
    STREAM(cycleTime);
    STREAM_REGISTER_FINISH;
  }

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
  enum
  {
    off    = 1000, /**< Special angle for switching off a joint. */
    ignore = 2000  /**< Special angle for not overwriting the previous setting. */
  };
  float angles[numOfJoints]; /**< The angles of all joints. */
  unsigned timeStamp; /**< The time when these angles were received. */
  float cycleTime;

  /**
  * Default constructor.
  * Switches off all joints.
  */
  JointData_d198df791237() : timeStamp(0)
  {
    for(int i = 0; i < numOfJoints; ++i)
      angles[i] = off;
  }

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
  virtual void mirror(const JointData_d198df791237& other)
  {
    for(int i = 0; i < numOfJoints; ++i)
      angles[i] = other.mirror((Joint) i);
    timeStamp = other.timeStamp;
  }
};

/**
* @class JointData_d198df791237Deg
* A class that wraps joint data to be transmitted in degrees.
*/
class JointData_d198df791237Deg : public JointData_d198df791237
{
private:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    if(jointData)
    {
      ASSERT(out);
      for(int i = 0; i < JointData_d198df791237::numOfJoints; ++i)
        angles[i] = jointData->angles[i] == JointData_d198df791237::off ? JointData_d198df791237::off
                    : std::floor(toDegrees(jointData->angles[i]) * 10.0f + 0.5f) / 10.0f;
      timeStamp = jointData->timeStamp;

      STREAM(angles);
      STREAM(timeStamp);
    }
    else
    {
      STREAM_BASE(JointData_d198df791237);
    }
    STREAM_REGISTER_FINISH;
  }

  JointData_d198df791237* jointData; /**< The joint data that is wrapped. */

public:
  /**
  * Default constructor.
  */
  JointData_d198df791237Deg() : jointData(0) {}

  /**
  * Constructor.
  * @param jointData The joint data that is wrapped.
  */
  JointData_d198df791237Deg(JointData_d198df791237& jointData) : jointData(&jointData) {}

  /**
  * Assignment operator.
  */
  JointData_d198df791237Deg& operator=(const JointData_d198df791237Deg& other)
  {
    if(jointData)
      for(int i = 0; i < JointData_d198df791237::numOfJoints; ++i)
        jointData->angles[i] = other.angles[i] == JointData_d198df791237::off ? JointData_d198df791237::off
                               : fromDegrees(other.angles[i]);
    else
      *((JointData_d198df791237*) this) = other;
    return *this;
  }
};

/**
 * @class HardnessData_d198df791237
 * This class represents the joint hardness in a jointRequest.
 * It loads the default hardness values from hardnessSettings.cfg.
 */
class HardnessData_d198df791237 : public Streamable
{
public:
  enum Hardness
  {
    useDefault = -1,
  };

  int hardness[JointData_d198df791237::numOfJoints]; /**< the custom hardness for each joint */

  /**
   * Default Constructor
   */
  HardnessData_d198df791237()
  {
    resetToDefault();
  }

  /**
  * The method returns the hardness of the mirror (left/right) of the given joint.
  * @param joint The joint the mirror of which is returned.
  * @return The output hardness of the mirrored joint.
  */
  int mirror(const JointData_d198df791237::Joint joint) const
  {
    switch(joint)
    {
    case JointData_d198df791237::HeadYaw:
      return hardness[JointData_d198df791237::HeadYaw];
    case JointData_d198df791237::LShoulderPitch:
      return hardness[JointData_d198df791237::RShoulderPitch];
    case JointData_d198df791237::LShoulderRoll:
      return hardness[JointData_d198df791237::RShoulderRoll];
    case JointData_d198df791237::LElbowYaw:
      return hardness[JointData_d198df791237::RElbowYaw];
    case JointData_d198df791237::LElbowRoll:
      return hardness[JointData_d198df791237::RElbowRoll];
    case JointData_d198df791237::RShoulderPitch:
      return hardness[JointData_d198df791237::LShoulderPitch];
    case JointData_d198df791237::RShoulderRoll:
      return hardness[JointData_d198df791237::LShoulderRoll];
    case JointData_d198df791237::RElbowYaw:
      return hardness[JointData_d198df791237::LElbowYaw];
    case JointData_d198df791237::RElbowRoll:
      return hardness[JointData_d198df791237::LElbowRoll];
    case JointData_d198df791237::LHipYawPitch:
      return hardness[JointData_d198df791237::RHipYawPitch];
    case JointData_d198df791237::LHipRoll:
      return hardness[JointData_d198df791237::RHipRoll];
    case JointData_d198df791237::LHipPitch:
      return hardness[JointData_d198df791237::RHipPitch];
    case JointData_d198df791237::LKneePitch:
      return hardness[JointData_d198df791237::RKneePitch];
    case JointData_d198df791237::LAnklePitch:
      return hardness[JointData_d198df791237::RAnklePitch];
    case JointData_d198df791237::LAnkleRoll:
      return hardness[JointData_d198df791237::RAnkleRoll];
    case JointData_d198df791237::RHipYawPitch:
      return hardness[JointData_d198df791237::LHipYawPitch];
    case JointData_d198df791237::RHipRoll:
      return hardness[JointData_d198df791237::LHipRoll];
    case JointData_d198df791237::RHipPitch:
      return hardness[JointData_d198df791237::LHipPitch];
    case JointData_d198df791237::RKneePitch:
      return hardness[JointData_d198df791237::LKneePitch];
    case JointData_d198df791237::RAnklePitch:
      return hardness[JointData_d198df791237::LAnklePitch];
    case JointData_d198df791237::RAnkleRoll:
      return hardness[JointData_d198df791237::LAnkleRoll];
    default:
      return hardness[joint];
    }
  }

  /**
   * initializes this instance with the mirrored values of other
   * @param other the HardnessData_d198df791237 to be mirrored
   */
  void mirror(const HardnessData_d198df791237& other)
  {
    for(int i = 0; i < JointData_d198df791237::numOfJoints; ++i)
      hardness[i] = other.mirror((JointData_d198df791237::Joint)i);
  }

  /**
   * This function resets the hardness for all joints to the default value.
   */
  inline void resetToDefault()
  {
    for(int i = 0; i < JointData_d198df791237::numOfJoints; ++i)
      hardness[i] = useDefault;
  }

private:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(hardness);
    STREAM_REGISTER_FINISH;
  }
};

class HardnessSettings_d198df791237 : public HardnessData_d198df791237 {};

/**
 * @class JointRequest_d198df791237
 */
class JointRequest_d198df791237 : public JointData_d198df791237
{
public:
  HardnessData_d198df791237 jointHardness; /**< the hardness for all joints*/

  /**
   * Initializes this instance with the mirrored data from a other JointRequest_d198df791237
   * @param other the JointRequest_d198df791237 to be mirrored
   */
  void mirror(const JointRequest_d198df791237& other)
  {
    JointData_d198df791237::mirror(other);
    jointHardness.mirror(other.jointHardness);
  }

  /**
   * Returns the mirrored angle of joint
   * @param joint the joint to be mirrored
   */
  float mirror(const JointData_d198df791237::Joint joint)
  {
    return JointData_d198df791237::mirror(joint);
  }

protected:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM_BASE(JointData_d198df791237);
    STREAM(jointHardness);
    STREAM_REGISTER_FINISH;
  }
};

class FilteredJointData_d198df791237 : public JointData_d198df791237 {};
class FilteredJointData_d198df791237Prev : public FilteredJointData_d198df791237 {};
class UnstableJointRequest_d198df791237 : public JointRequest_d198df791237 {};
