/**
* @file Pose3D.h
* Contains class Pose3D
* @author <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
* @author Max Risler
*/

#pragma once

#include "RotationMatrix.h"

/** representation for 3D Transformation (Location + Orientation)*/
class Pose3D : public Streamable
{
protected:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(rotation);
    STREAM(translation);
    STREAM_REGISTER_FINISH;
  }

public:

  /** rotation as a RotationMatrix*/
  RotationMatrix rotation;

  /** translation as a Vector3*/
  Vector3<> translation;

  /** constructor*/
  inline Pose3D() {}

  /** constructor from rotation and translation
   * \param rot Rotation
   * \param trans Translation
   */
  inline Pose3D(const RotationMatrix& rot, const Vector3<>& trans) : rotation(rot), translation(trans) {}

  /** constructor from rotation
   * \param rot Rotation
   */
  inline Pose3D(const RotationMatrix& rot) : rotation(rot) {}

  /** constructor from translation
   * \param trans Translation
   */
  inline Pose3D(const Vector3<>& trans) : translation(trans) {}

  /** constructor from three translation values
   * \param x translation x component
   * \param y translation y component
   * \param z translation z component
   */
  inline Pose3D(const float x, const float y, const float z) : translation(x, y, z) {}

  /** Copy constructor
  *\param other The other vector that is copied to this one
  */
  inline Pose3D(const Pose3D& other) : rotation(other.rotation), translation(other.translation) {}

  /** Assignment operator
  *\param other The other Pose3D that is assigned to this one
  *\return A reference to this object after the assignment.
  */
  inline Pose3D& operator=(const Pose3D& other)
  {
    rotation = other.rotation;
    translation = other.translation;
    return *this;
  }

  /** Multiplication with Point
  *\param point (Vector3&lt;float&gt;)
  */
  inline Vector3<> operator*(const Vector3<>& point) const
  {
    return rotation * point + translation;
  }

  /** Comparison of another vector with this one.
  *\param other The other vector that will be compared to this one
  *\return Whether the two vectors are equal.
  */
  inline bool operator==(const Pose3D& other) const
  {
    return translation == other.translation && rotation == other.rotation;
  }

  /** Comparison of another vector with this one.
  *\param other The other vector that will be compared to this one
  *\return Whether the two vectors are unequal.
  */
  inline bool operator!=(const Pose3D& other) const
  {
    return translation != other.translation || rotation != other.rotation;
  }

  /**Concatenation of this pose with another pose
  *\param other The other pose that will be concatenated to this one.
  *\return A reference to this pose after concatenation
  */
  inline Pose3D& conc(const Pose3D& other)
  {
    translation = *this * other.translation;
    rotation *= other.rotation;
    return *this;
  }

  /** Calculates the inverse transformation from the current pose
  * @return The inverse transformation pose.
  */
  inline Pose3D invert() const
  {
    Pose3D result;
    result.rotation = rotation.invert();
    result.translation = result.rotation * (-translation);
    return result;
  }

  /**Translate this pose by a translation vector
  *\param trans Vector to translate with
  *\return A reference to this pose after translation
  */
  inline Pose3D& translate(const Vector3<>& trans)
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
  inline Pose3D& translate(const float x, const float y, const float z)
  {
    translation = *this * Vector3<>(x, y, z);
    return *this;
  }

  /**Rotate this pose by a rotation
  *\param rot Rotationmatrix to rotate with
  *\return A reference to this pose after rotation
  */
  inline Pose3D& rotate(const RotationMatrix& rot)
  {
    rotation *= rot;
    return *this;
  }

  /**Rotate this pose around its x-axis
  *\param angle angle to rotate with
  *\return A reference to this pose after rotation
  */
  inline Pose3D& rotateX(const float angle)
  {
    rotation.rotateX(angle);
    return *this;
  }

  /**Rotate this pose around its y-axis
  *\param angle angle to rotate with
  *\return A reference to this pose after rotation
  */
  inline Pose3D& rotateY(const float angle)
  {
    rotation.rotateY(angle);
    return *this;
  }

  /**Rotate this pose around its z-axis
  *\param angle angle to rotate with
  *\return A reference to this pose after rotation
  */
  inline Pose3D& rotateZ(const float angle)
  {
    rotation.rotateZ(angle);
    return *this;
  }
};
