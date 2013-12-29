/**
 * @file SpatialInertia.h
 * Contains class SpatialInertia.
 *
 * @author Felix Wenk
 */

#pragma once

#include "Platform/BHAssert.h"
#include "SpatialVector.h"
#include "SpatialInertiaDerivative.h"
#include "Pose3D.h"

/**
 * The spatial inertia represents both mass and moment of inertia of a body.
 */
class SpatialInertia : public Streamable
{
public:
  virtual void serialize(In* in, Out* out)
  {
    float momentOfInertiaLT[6];
    if(out)
      toLT(momentOfInertiaLT);
    STREAM_REGISTER_BEGIN;
    STREAM(momentOfInertiaLT);
    STREAM(centerOfMass);
    STREAM(mass);
    STREAM_REGISTER_FINISH;
    if(in)
      fromLT(momentOfInertiaLT);
  }

  SpatialInertia() : mass(0.0f) {}
  SpatialInertia(const SpatialInertia& other)
    : momentOfInertia(other.momentOfInertia),
      centerOfMass(other.centerOfMass), mass(other.mass)
  {}

  SpatialInertia(const Matrix3x3<>& momentOfInertia, const Vector3<>& centerOfMass, float mass)
    : momentOfInertia(momentOfInertia), centerOfMass(centerOfMass), mass(mass)
  {}

  SpatialInertia& operator=(const SpatialInertia& other)
  {
    momentOfInertia = other.momentOfInertia;
    centerOfMass = other.centerOfMass;
    mass = other.mass;
    return *this;
  }

  SpatialInertia& operator+=(const SpatialInertia& other)
  {
    momentOfInertia += other.momentOfInertia;
    centerOfMass = centerOfMass * mass + other.centerOfMass * other.mass;
    mass += other.mass;
    centerOfMass /= mass;
    return *this;
  }

  SpatialInertia operator+(const SpatialInertia& other) const
  {
    return SpatialInertia(*this) += other;
  }

  SpatialVector<> operator*(const SpatialVector<>& vector) const
  {
    ASSERT(vector.motion);
    Vector3<> massMoment = centerOfMass * mass;
    return SpatialVector<>((momentOfInertia * vector.top) + (massMoment ^ vector.bottom),
                           (vector.bottom * mass) - (massMoment ^ vector.top), false);
  }

  bool operator==(const SpatialInertia& other) const
  {
    return mass == other.mass && momentOfInertia == other.momentOfInertia
        && centerOfMass == other.centerOfMass;
  }

  bool operator!=(const SpatialInertia& other) const
  {
    return !(*this == other);
  }

  // If vector = Iv, this calculates v = I^{-1} * vector
  SpatialVector<> inverseApply(const SpatialVector<>& vector) const
  {
    ASSERT(!vector.motion);
    // Calculate inverse of moment of inertia at the center of mass.
    Matrix3x3<> cx = SpatialVector<>::crossProductMatrix(centerOfMass);
    Matrix3x3<> cxT = cx.transpose(); // To consider: Replace with -cx?
    Matrix3x3<> moiComInv = momentOfInertia - cx * cxT * mass;
    moiComInv = moiComInv.invert();

    // n -> vector.top, f -> vector.bottom. To consider: Replace multiplications with cx with cross products?
    Vector3<> ncxTf = vector.top + cxT * vector.bottom;
    return SpatialVector<>(moiComInv * ncxTf,
                           vector.bottom / mass + cx * (moiComInv * ncxTf));
  }

  /**
   * Compute the derivative of this SpatialInertia using the motion vector
   * 'vector': vx*I - Ivx
   */
  SpatialInertiaDerivative derive(const SpatialVector<>& vector) const
  {
    ASSERT(vector.motion);

    // Calculate top right block of SpatialInertiaDerivative
    const Vector3<> wxc =  vector.top ^ centerOfMass;
    Matrix3x3<> topRight = SpatialVector<>::crossProductMatrix(wxc + vector.bottom);
    topRight *= mass;

    // Calculate top left block of SpatialInertiaDerivative
    const Matrix3x3<> wx = SpatialVector<>::crossProductMatrix(vector.top);
    const Matrix3x3<> vx = SpatialVector<>::crossProductMatrix(vector.bottom);
    const Matrix3x3<> cx = SpatialVector<>::crossProductMatrix(centerOfMass);
    const Matrix3x3<> topLeft = wx * momentOfInertia - momentOfInertia * wx - (vx * cx + cx * vx) * mass;

    return SpatialInertiaDerivative(topLeft, topRight);
  }

  bool isFinite() const
  {
#ifdef WIN32
    return true;
#else
    ASSERT(std::isfinite(mass));
    for(int c = 0; c < 3; ++c)
    {
      ASSERT(std::isfinite(centerOfMass[c]));
      for(int r = 0; r < 3; ++r)
        ASSERT(std::isfinite(momentOfInertia[c][r]));
    }
    return true;
#endif
  }

  Matrix3x3<> momentOfInertia;
  Vector3<> centerOfMass;
  float mass;

private:
  /*
   * Methods create the moment of inertia matrix from it's lower triangle.
   * The elements of the lower triangle are stored in the follwing order
   * (i.e. the main diagonal and then the diagonals on the lower left):
   * lt[0] -> I00; lt[1] -> I11; lt[2] -> I22; lt[3] -> I10; lt[4] -> I21; lt[5] -> I20
   */
  void fromLT(const float lt[6])
  {
    momentOfInertia[0][0] = lt[0];
    momentOfInertia[1][1] = lt[1];
    momentOfInertia[2][2] = lt[2];
    momentOfInertia[0][1] = lt[3];
    momentOfInertia[1][0] = lt[3];
    momentOfInertia[1][2] = lt[4];
    momentOfInertia[2][1] = lt[4];
    momentOfInertia[0][2] = lt[5];
    momentOfInertia[2][0] = lt[5];
  }

  void toLT(float lt[6])
  {
    lt[0] = momentOfInertia.c0[0];
    lt[3] = momentOfInertia.c0[1];
    lt[5] = momentOfInertia.c0[2];
    lt[1] = momentOfInertia.c1[1];
    lt[4] = momentOfInertia.c1[2];
    lt[2] = momentOfInertia.c2[2];
  }
};
