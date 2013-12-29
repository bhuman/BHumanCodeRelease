/**
* \file RotationMatrix.h
* Delcaration of class RotationMatrix
* \author <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
* \author <a href="mailto:thomas.kindler@gmx.de">Thomas Kindler</a>
* \author Max Risler
*/

#pragma once

#include "Matrix3x3.h"

/**
 * Representation for 3x3 RotationMatrices
 */
class RotationMatrix : public Matrix3x3<>
{
protected:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM_BASE(Matrix3x3<>);
    STREAM_REGISTER_FINISH;
  }

public:
  /**
   * Default constructor.
   */
  RotationMatrix() {}

  /**
   * Constructor.
   *
   * \param  c0  the first column of the matrix.
   * \param  c1  the second column of the matrix.
   * \param  c2  the third column of the matrix.
   */
  RotationMatrix(
    const Vector3<>& c0,
    const Vector3<>& c1,
    const Vector3<>& c2) : Matrix3x3<>(c0, c1, c2) {}

  /**
   * Copy constructor.
   *
   * \param  other  The other matrix that is copied to this one
   */
  explicit RotationMatrix(const Matrix3x3<>& other) : Matrix3x3<>(other) {}

  /**
   * Assignment operator.
   *
   * \param  other  The other matrix that is assigned to this one
   * \return        A reference to this object after the assignment.
   */
  RotationMatrix& operator=(const Matrix3x3<>& other)
  {
    c0 = other.c0;
    c1 = other.c1;
    c2 = other.c2;
    return *this;
  }

  /**
   * RotationMatrix from rotation around any axis.
  * \param axis The axis.
   * \param angle The angle to rotate around the axis.
   */
  RotationMatrix(const Vector3<>& axis, float angle);

  /**
   * RotationMatrix from rotation around any axis with an angle given as the length of the axis.
   * \param axis The axis.
   */
  RotationMatrix(const Vector3<>& axis);

  /**
   * Invert the matrix.
   *
   * \note: Inverted rotation matrix is transposed matrix.
   */
  inline RotationMatrix invert() const
  {
    return RotationMatrix(
             Vector3<>(c0.x, c1.x, c2.x),
             Vector3<>(c0.y, c1.y, c2.y),
             Vector3<>(c0.z, c1.z, c2.z)
           );
  }

  /**
   * Multiplication of this rotation matrix by another matrix.
   * This function is reimplemented here to avoid hidden Matrix3x3<> to RotationMatrix (copy) casts.
   * \param  other  The other matrix this one is multiplied by
   * \return        A new matrix containing the result
   *                of the calculation.
  */
  RotationMatrix operator*(const Matrix3x3<>& other) const
  {
    RotationMatrix result;
    result.c0.x = c0.x * other.c0.x + c1.x * other.c0.y + c2.x * other.c0.z;
    result.c0.y = c0.y * other.c0.x + c1.y * other.c0.y + c2.y * other.c0.z;
    result.c0.z = c0.z * other.c0.x + c1.z * other.c0.y + c2.z * other.c0.z;
    result.c1.x = c0.x * other.c1.x + c1.x * other.c1.y + c2.x * other.c1.z;
    result.c1.y = c0.y * other.c1.x + c1.y * other.c1.y + c2.y * other.c1.z;
    result.c1.z = c0.z * other.c1.x + c1.z * other.c1.y + c2.z * other.c1.z;
    result.c2.x = c0.x * other.c2.x + c1.x * other.c2.y + c2.x * other.c2.z;
    result.c2.y = c0.y * other.c2.x + c1.y * other.c2.y + c2.y * other.c2.z;
    result.c2.z = c0.z * other.c2.x + c1.z * other.c2.y + c2.z * other.c2.z;
    return result;
  }

  /**
  * Multiplication of this matrix by vector.
  * This function is reimpletened here since the RotationMatrix * Matrix3x3<> reimplementation makes Matrix3x3<> * Vector3<> inaccessible
  * \param  vector  The vector this one is multiplied by
  * \return         A new vector containing the result
  *                 of the calculation.
  */
  Vector3<> operator*(const Vector3<>& vector) const
  {
    return Vector3<>(
             c0.x * vector.x + c1.x * vector.y + c2.x * vector.z,
             c0.y * vector.x + c1.y * vector.y + c2.y * vector.z,
             c0.z * vector.x + c1.z * vector.y + c2.z * vector.z);
  }

  /**
   * Rotation around the x-axis.
   *
   * \param   angle  The angle this pose will be rotated by
   * \return  A reference to this object after the calculation.
   */
  RotationMatrix& rotateX(const float angle);

  /**
   * Rotation around the y-axis.
   *
   * \param   angle  The angle this pose will be rotated by
   * \return  A reference to this object after the calculation.
   */
  RotationMatrix& rotateY(const float angle);

  /**
   * Rotation around the z-axis.
   *
   * \param   angle  The angle this pose will be rotated by
   * \return  A reference to this object after the calculation.
   */
  RotationMatrix& rotateZ(const float angle);

  /**
   * Get the x-angle of a RotationMatrix.
   *
   * \return  The angle around the x-axis between the original
   *          and the rotated z-axis projected on the y-z-plane
   */
  float getXAngle() const;

  /**
   * Get the y-angle of a RotationMatrix.
   *
   * \return  The angle around the y-axis between the original
   *          and the rotated x-axis projected on the x-z-plane
   */
  float getYAngle() const;

  /**
   * Get the z-angle of a RotationMatrix.
   *
   * \return  The angle around the z-axis between the original
   *          and the rotated x-axis projected on the x-y-plane
   */
  float getZAngle() const;

  /**
   * Create and return a RotationMatrix, rotated around x-axis
   *
   * \param   angle
   * \return  rotated RotationMatrix
   */
  static RotationMatrix fromRotationX(const float angle);

  /**
   * Create and return a RotationMatrix, rotated around y-axis
   *
   * \param   angle
   * \return  rotated RotationMatrix
   */
  static RotationMatrix fromRotationY(const float angle);

  /**
   * Create and return a RotationMatrix, rotated around z-axis
   *
   * \param   angle
   * \return  rotated RotationMatrix
   */
  static RotationMatrix fromRotationZ(const float angle);

  /**
  * Converts the rotation matrix into the single vector format.
  * @return The rotation matrix as angleAxis.
  */
  Vector3<> getAngleAxis() const;
};
