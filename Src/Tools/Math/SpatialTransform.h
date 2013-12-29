/**
 * @file SpatialTransform.h
 * Contains class SpatialTransform to transform spatial vectors.
 *
 * @author Felix Wenk
 */

#pragma once

#include "Platform/BHAssert.h"
#include "SpatialVector.h"
#include "SpatialInertia.h"
#include "Pose3D.h"

class SpatialTransform : public Streamable
{
public:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(rot);
    STREAM(pos);
    STREAM_REGISTER_FINISH;
  }

  SpatialTransform() {}

  SpatialTransform(const SpatialTransform& other) : rot(other.rot), pos(other.pos) {}

  /**
   * Constructs a spatial transform from a rotation matrix and a position vector.
   * The position vector must be relative to the same coordiante system as the rotation matrix!
   */
  SpatialTransform(const RotationMatrix& rot, const Vector3<>& pos, bool posInverted) : rot(rot)
  {
    // Convert the position vector from 'B' in 'A' coordinates to 'A' in 'B' coordinates.
    // This saves some matrix multiplications in the other operations, but looks a little counterintuitive.
    this->pos = posInverted ? pos : rot.invert() * (-pos);
  }

  SpatialTransform& operator=(const SpatialTransform& other) {rot = other.rot; pos = other.pos; return *this;}
  bool operator==(const SpatialTransform& other) const { return rot == other.rot && pos == other.pos; }
  bool operator!=(const SpatialTransform& other) const { return !this->operator==(other); }

  void invert()
  {
    pos = rot * (-pos);
    rot = rot.invert();
  }

  SpatialTransform inverse() const
  {
    SpatialTransform inv(*this);
    inv.invert();
    return inv;
  }

  SpatialTransform& operator*=(const SpatialTransform& other)
  {
    rot *= other.rot;
    // If rot is B in A coordinates, pos is A in B coordinates. That's why it doesn't look like Pose3D!
    pos = other.rot.invert() * pos + other.pos;
    return *this;
  }

  SpatialTransform operator*(const SpatialTransform& other) const
  {
    return SpatialTransform(*this) *= other;
  }

  SpatialVector<> operator*(const SpatialVector<>& vector) const
  {
    if(vector.motion)
      return SpatialVector<>(rot * vector.top, rot * (vector.bottom - (pos ^ vector.top)), true);
    else
      return SpatialVector<>(rot * (vector.top - (pos ^ vector.bottom)), rot * vector.bottom, false);
  }

  /**
   * Given this spatial transform which transforms motion vectors from coordinate system A to B (A2Bm),
   * this method does the following calculation:
   * B2Af * inertia * A2Bm = B2Am.invert().transpose() * inertia * A2Bm = A2Bm.tranpose() * inertia * A2Bm
   *
   * I.e. if 'inertia' is relative to coordinate system B the result is the inertia relative to coordinate system A.
   */
  SpatialInertia transform(const SpatialInertia& inertia) const
  {
    const RotationMatrix rotT = rot.invert();
    const Vector3<> rotTc = rotT * inertia.centerOfMass;
    const Matrix3x3<> posCross = SpatialVector<>::crossProductMatrix(pos);
    const Matrix3x3<> rotTcCross = SpatialVector<>::crossProductMatrix(rotTc);
    const Vector3<> comNew = rotTc + pos;
    const Matrix3x3<> comNewCross = SpatialVector<>::crossProductMatrix(comNew);

    return SpatialInertia(rotT * inertia.momentOfInertia * rot - (posCross * rotTcCross + comNewCross * posCross) * inertia.mass,
                          comNew,
                          inertia.mass);
  }

  /**
   * This method performs a transformation on spatial inertia derivatives analogous to the transform
   * on spatial inertias. I.e. if this is a spatial transformation for vectors from coordinate system A to B,
   * then the spatial inertia derivative relative to coordinate system B is transformed to
   * coordinate system A.
   */
  SpatialInertiaDerivative transform(const SpatialInertiaDerivative& inertiaDerivative) const
  {
    const RotationMatrix rotT = rot.invert();

    // Compute top right block of transformed inertia derivative
    Matrix3x3<> topRight = rotT * inertiaDerivative.topRight * rot;
    // Compute top left block of transformed inertia derivative
    Matrix3x3<> rx = SpatialVector<>::crossProductMatrix(pos);
    Matrix3x3<> topLeft = rotT * inertiaDerivative.topLeft * rot;
    topLeft -= topRight * rx;
    topLeft -= rx * topRight;

    return SpatialInertiaDerivative(topLeft, topRight);
  }

  /**
   * Create a Pose3D corresponding to this spatial transform.
   */
  Pose3D toPose3D() const
  {
    const Vector3<> translation = rot * (-pos);
    return Pose3D(rot, translation);
  }

  bool isFinite() const
  {
#ifndef WIN32
    for(int c = 0; c < 3; ++c)
    {
      ASSERT(std::isfinite(pos[c]));
      for(int r = 0; r < 3; ++r)
        ASSERT(std::isfinite(rot[c][r]));
    }
#endif
    return true;
  }

  RotationMatrix rot;
  Vector3<> pos;
};
