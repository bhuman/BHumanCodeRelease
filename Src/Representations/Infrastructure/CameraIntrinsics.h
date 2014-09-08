/**
* @file CameraIntrinsics.h
* Declaration of a class for representing the intrinsic parameters of the cameras.
* @author Alexis Tsogias
*/

#pragma once

#include "Tools/Math/Vector2.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Module/Next.h"

/**
 * This class stores the opening angles and optical centers of both cameras.
 * The opening angles are stored in radiant but are streamed out in degrees.
 * The optical center is stored in relative coordinates with a range from 0 to 1.
 * Restoring the optical center in pixels is a matter of multiplying the width / height of the cameras.
 * Thus, only resolutions with the same aspect ratios should be used.
 */
class CameraIntrinsics : public Streamable
{
public:
  float upperOpeningAngleWidth;
  float upperOpeningAngleHeight;
  Vector2<float> upperOpticalCenter;
  float lowerOpeningAngleWidth;
  float lowerOpeningAngleHeight;
  Vector2<float> lowerOpticalCenter;

private:
  virtual void serialize(In* in, Out* out);
};

using CameraIntrinsicsNext = Next<CameraIntrinsics>;
