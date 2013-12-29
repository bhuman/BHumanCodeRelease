/**
* @file Representations/Infrastructure/JointDataDeg.h
* This file declares a class to represent the joint angles in degrees.
*/

#pragma once

#include "JointData.h"
#include "Tools/Math/Common.h"
#include "Platform/BHAssert.h"

/**
* @class JointDataDeg
* A class that wraps joint data to be transmitted in degrees.
*/
class JointDataDeg : public Streamable
{
private:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    float angles[JointData::numOfJoints];
    unsigned& timeStamp = jointData.timeStamp;
    if(out)
      for(int i = 0; i < JointData::numOfJoints; ++i)
        angles[i] = jointData.angles[i] == JointData::off ? JointData::off
                    : std::floor(toDegrees(jointData.angles[i]) * 10.0f + 0.5f) / 10.0f;
    STREAM(angles);
    STREAM(timeStamp);
    if(in)
      for(int i = 0; i < JointData::numOfJoints; ++i)
        jointData.angles[i] = angles[i] == JointData::off ? JointData::off : fromDegrees(angles[i]);
    STREAM_REGISTER_FINISH;
  }

  JointData& jointData; /**< The joint data that is wrapped. */

public:
  /**
  * Constructor.
  * @param jointData The joint data that is wrapped.
  */
  JointDataDeg(JointData& jointData) : jointData(jointData) {}
};
