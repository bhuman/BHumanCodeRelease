/**
* @file MassCalibration.h
* Declaration of a class for representing the relative positions and masses of mass points.
* @author <a href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</a>
*/

#pragma once

#include "Tools/Math/SpatialInertia.h"
#include "Tools/Math/Vector3.h"
#include "Tools/Enum.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(MassCalibration,
{
public:
  ENUM(Limb,
    neck,
    head,
    shoulderLeft,
    bicepsLeft,
    elbowLeft,
    foreArmLeft,
    shoulderRight,
    bicepsRight,
    elbowRight,
    foreArmRight,
    pelvisLeft,
    hipLeft,
    thighLeft,
    tibiaLeft,
    ankleLeft,
    footLeft,
    pelvisRight,
    hipRight,
    thighRight,
    tibiaRight,
    ankleRight,
    footRight,
    torso
  );

  /**
  * Information on the mass distribution of a limb of the robot.
  */
  class MassInfo : public Streamable
  {
  private:
    virtual void serialize(In* in, Out* out)
    {
      float inertiaMatrixLT[6];
      if(out)
        inertiaToLowerTriangle(inertiaMatrixLT);
      STREAM_REGISTER_BEGIN;
      STREAM(mass);
      STREAM(offset);
      STREAM(inertiaMatrixLT);
      STREAM_REGISTER_FINISH;
      if(in)
        inertiaFromLowerTriangle(inertiaMatrixLT);
    }

  public:
    float mass; /**< The mass of this limb. */
    Vector3<> offset; /**< The offset of the center of mass of this limb relative to its hinge. */
    Matrix3x3<> inertiaMatrix; /**< The moment of inertia matrix of this limb. */

    /**
    * Default constructor.
    */
    MassInfo() : mass(0) {}

    void inertiaFromLowerTriangle(const float lowerTriangle[6])
    {
      inertiaMatrix.c0[0] = lowerTriangle[0];
      inertiaMatrix.c1[1] = lowerTriangle[1];
      inertiaMatrix.c2[2] = lowerTriangle[2];
      inertiaMatrix.c0[1] = lowerTriangle[3];
      inertiaMatrix.c1[0] = lowerTriangle[3];
      inertiaMatrix.c1[2] = lowerTriangle[4];
      inertiaMatrix.c2[1] = lowerTriangle[4];
      inertiaMatrix.c0[2] = lowerTriangle[5];
      inertiaMatrix.c2[0] = lowerTriangle[5];
    }

    void inertiaToLowerTriangle(float lowerTriangle[6])
    {
      lowerTriangle[0] = inertiaMatrix.c0[0];
      lowerTriangle[3] = inertiaMatrix.c0[1];
      lowerTriangle[5] = inertiaMatrix.c0[2];
      lowerTriangle[1] = inertiaMatrix.c1[1];
      lowerTriangle[4] = inertiaMatrix.c1[2];
      lowerTriangle[2] = inertiaMatrix.c2[2];
    }
  },

  (MassInfo[numOfLimbs]) masses, /**< Information on the mass distribution of all joints. */
});
