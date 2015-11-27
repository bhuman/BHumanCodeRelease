/**
 * @file MassCalibration.h
 * Declaration of a struct for representing the relative positions and masses of mass points.
 * @author <a href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</a>
 */

#pragma once

#include "Tools/Enum.h"
#include "Tools/Limbs.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

struct MassCalibration : public Streamable
{
  /**
   * Information on the mass distribution of a limb of the robot.
   */
  STREAMABLE(MassInfo,
  {,
    (float)(0) mass, /**< The mass of this limb. */
    (Vector3f) offset, /**< The offset of the center of mass of this limb relative to its hinge. */
  });

  std::array<MassInfo, Limbs::numOfLimbs> masses; /**< Information on the mass distribution of all joints. */

private:
  virtual void serialize(In* in, Out* out);
};

inline void MassCalibration::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN
  for(int i = 0; i < Limbs::numOfLimbs; ++i)
    Streaming::streamIt(in, out, Limbs::getName(static_cast<Limbs::Limb>(i)), masses[i], nullptr);
  STREAM_REGISTER_FINISH
}