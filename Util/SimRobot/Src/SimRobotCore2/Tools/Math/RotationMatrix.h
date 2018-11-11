/**
 * @file Tools/Math/RotationMatrix.h
 * Delcaration of class RotationMatrix
 * @author <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
 * @author <a href="mailto:thomas.kindler@gmx.de">Thomas Kindler</a>
 * @author Max Risler
 * @author <a href="mailto:alexists@tzi.de">Alexis Tsogias</a>
 */

#pragma once

#include "Eigen.h"

/**
 * Representation for 3x3 RotationMatrices
 */
class RotationMatrix : public Matrix3f
{
public:
  RotationMatrix() : Matrix3f(Matrix3f::Identity()) {}
  RotationMatrix(const Matrix3f& other) : Matrix3f(other) {}
  RotationMatrix(const AngleAxisf& angleAxis) : Matrix3f(angleAxis.toRotationMatrix()) {}
  RotationMatrix(const Quaternionf& quat) : Matrix3f(quat.toRotationMatrix()) {}

  RotationMatrix& operator=(const Matrix3f& other)
  {
    Matrix3f::operator=(other);
    return *this;
  }

  RotationMatrix& operator=(const AngleAxisf& angleAxis)
  {
    Matrix3f::operator=(angleAxis.toRotationMatrix());
    return *this;
  }

  RotationMatrix& operator=(const Quaternionf& quat)
  {
    Matrix3f::operator=(quat.toRotationMatrix());
    return *this;
  }

  /**
   * Multiplication of this matrix by vector.
   * @param  vector  The vector this one is multiplied by
   * @return         A new vector containing the result
   */
  Vector3f operator*(const Vector3f& vector) const
  {
    return Matrix3f::operator*(vector);
  }

  /**
   * Multiplication of this rotation matrix by another rotation matrix.
   * @param  other  The other matrix this one is multiplied by
   * @return        A new matrix containing the result
   *                of the calculation.
   */
  RotationMatrix operator*(const RotationMatrix& other) const
  {
    return RotationMatrix(Base::operator*(other));
  }

  RotationMatrix& operator*=(const AngleAxisf& rot)
  {
    Matrix3f::operator*=(rot.toRotationMatrix());
    return *this;
  }

  RotationMatrix& operator*=(const Quaternionf& rot)
  {
    Matrix3f::operator*=(rot.toRotationMatrix());
    return *this;
  }

  RotationMatrix& operator*=(const RotationMatrix& rot)
  {
    Matrix3f::operator*=(rot);
    return *this;
  }

  /**
   * Invert the matrix.
   *
   * @note: Inverted rotation matrix is transposed matrix.
   */
  RotationMatrix& invert()
  {
    transposeInPlace();
    return *this;
  }

  RotationMatrix inverse() const
  {
    return RotationMatrix(transpose());
  }

  void normalize()
  {
    *this = Quaternionf(*this).normalized();
  }

  RotationMatrix normalized() const
  {
    return Quaternionf(*this).normalized();
  }

  /**
   * Converts the rotation matrix into an angleAxis.
   * @return The rotation matrix as angleAxis.
   */
  AngleAxisf getAngleAxis() const;

  /**
   * Converts the rotation matrix into an angleAxis in single vector format.
   * @return The rotation matrix as angleAxis.
   */
  Vector3f getPackedAngleAxis() const;

  /**
   * Rotation around the x-axis.
   *
   * @param   angle  The angle this pose will be rotated by
   * @return  A reference to this object after the calculation.
   */
  RotationMatrix& rotateX(const float angle);

  /**
   * Rotation around the y-axis.
   *
   * @param   angle  The angle this pose will be rotated by
   * @return  A reference to this object after the calculation.
   */
  RotationMatrix& rotateY(const float angle);

  /**
   * Rotation around the z-axis.
   *
   * @param   angle  The angle this pose will be rotated by
   * @return  A reference to this object after the calculation.
   */
  RotationMatrix& rotateZ(const float angle);

  /**
   * Create and return a RotationMatrix, rotated around x-axis
   *
   * @param   angle
   * @return  rotated RotationMatrix
   */
  static RotationMatrix aroundX(const float angle);

  /**
   * Create and return a RotationMatrix, rotated around y-axis
   *
   * @param   angle
   * @return  rotated RotationMatrix
   */
  static RotationMatrix aroundY(const float angle);

  /**
   * Create and return a RotationMatrix, rotated around z-axis
   *
   * @param   angle
   * @return  rotated RotationMatrix
   */
  static RotationMatrix aroundZ(const float angle);

  /**
   * Creates a RotationMatrix rotated around z, y and x (in this order!).
   * Equivalent to fromRotationZ(z).rotateY(y).rotateX(x);
   */
  static RotationMatrix fromEulerAngles(const float x, const float y, const float z);

  /**
   * Creates a RotationMatrix rotatied around the z, y and x components of the Vector3 (in this order!).
   * Equivalent to fromRotationZ(rotation.z).rotateY(rotation.y).rotateX(rotation.x);
   */
  static RotationMatrix fromEulerAngles(const Vector3f rotation);
};
