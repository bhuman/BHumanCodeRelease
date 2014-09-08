/**
* @file Pose3.h
* Contains class Pose3
* @author <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
* @author Max Risler
*/

#pragma once

#include "Matrix3x3.h"

/** representation for 3D Transformation (Location + Orientation)*/
template <class V = float> class Pose3
{
public:

  /** rotation as a Matrix3x3*/
  Matrix3x3<V> rotation;

  /** translation as a Vector3*/
  Vector3<V> translation;

  /** constructor*/
  Pose3<V>() = default;

  /** constructor from rotation and translation
   * \param rot Rotation
   * \param trans Translation
   */
  Pose3<V>(const Matrix3x3<V>& rot, const Vector3<V>& trans) : rotation(rot), translation(trans) {}

  /** constructor from rotation
   * \param rot Rotation
   */
  Pose3<V>(const Matrix3x3<V>& rot): rotation(rot) {}

  /** constructor from translation
   * \param trans Translation
   */
  Pose3<V>(const Vector3<V>& trans): translation(trans) {}

  /** constructor from three translation values
   * \param x translation x component
   * \param y translation y component
   * \param z translation z component
   */
  Pose3<V>(const V& x, const V& y, const V& z) : translation(x, y, z) {}

  /** Copy constructor
  *\param other The other vector that is copied to this one
  */
  Pose3<V>(const Pose3<V>& other) : rotation(other.rotation), translation(other.translation) {}

  /** Assignment operator
  *\param other The other Pose3<V> that is assigned to this one
  *\return A reference to this object after the assignment.
  */
  Pose3<V>& operator=(const Pose3<V>& other)
  {
    rotation = other.rotation;
    translation = other.translation;
    return *this;
  }

  /** Multiplication with Point
  *\param point (Vector3&lt;float&gt;)
  */
  Vector3<V> operator*(const Vector3<V>& point) const
  {
    return rotation * point + translation;
  }

  /** Comparison of another vector with this one.
  *\param other The other vector that will be compared to this one
  *\return Whether the two vectors are equal.
  */
  bool operator==(const Pose3<V>& other) const
  {
    return translation == other.translation && rotation == other.rotation;
  }

  /** Comparison of another vector with this one.
  *\param other The other vector that will be compared to this one
  *\return Whether the two vectors are unequal.
  */
  bool operator!=(const Pose3<V>& other) const
  {
    return translation != other.translation || rotation != other.rotation;
  }

  /**Concatenation of this pose with another pose
  *\param other The other pose that will be concatenated to this one.
  *\return A reference to this pose after concatenation
  */
  Pose3<V>& conc(const Pose3<V>& other)
  {
    translation = *this * other.translation;
    rotation *= other.rotation;
    return *this;
  }

  /** Calculates the inverse transformation from the current pose
  * @return The inverse transformation pose.
  */
  Pose3<V> invert() const
  {
    Pose3<V> result;
    result.rotation = rotation.transpose();
    result.translation = result.rotation * (-translation);
    return result;
  }

  /**Translate this pose by a translation vector
  *\param trans Vector to translate with
  *\return A reference to this pose after translation
  */
  Pose3<V>& translate(const Vector3<V>& trans)
  {
    translation = *this * trans;
    return *this;
  }

  /**Translate this pose by a translation vector
  *\param x x component of vector to translate with
  *\param y y component of vector to translate with
  *\param z z component of vector to translate with
  *\return A reference to this pose after translation
  */
  Pose3<V>& translate(const V& x, const V& y, const V& z)
  {
    translation = *this * Vector3<V>(x, y, z);
    return *this;
  }

  /**Rotate this pose by a rotation
  *\param rot Rotationmatrix to rotate with
  *\return A reference to this pose after rotation
  */
  Pose3<V>& rotate(const Matrix3x3<V>& rot)
  {
    rotation *= rot;
    return *this;
  }
};
