/**
 * @file JointCalibration.h
 * Declaration of a struct for representing the calibration values of joints.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Tools/Joints.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Streams/AutoStreamable.h"

#include <array>

struct JointCalibration : public Streamable
{
public:
  STREAMABLE(JointInfo,
  {
    /**
     * Default constructor.
     */
    JointInfo();

    bool isMissing() const,

    (Angle) offset, /**< An offset added to the angle. */
    (Angle) minAngle, /** the minmal angle */
    (Angle) maxAngle, /** the maximal angle */
  });

  std::array<JointInfo, Joints::numOfJoints> joints; /**< Information on the calibration of all joints. */

private:
  virtual void serialize(In* in, Out* out);
};

inline JointCalibration::JointInfo::JointInfo() :
  offset(0), minAngle(150_deg), maxAngle(150_deg)
{}

inline bool JointCalibration::JointInfo::isMissing() const
{
  return minAngle >= maxAngle;
}

inline void JointCalibration::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN
  for(int i = 0; i < Joints::numOfJoints; ++i)
    Streaming::streamIt(in, out, Joints::getName(static_cast<Joints::Joint>(i)), joints[i], nullptr);
  STREAM_REGISTER_FINISH
}
