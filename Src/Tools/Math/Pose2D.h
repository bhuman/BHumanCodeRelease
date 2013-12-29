/**
 * @file Pose2D.h
 * Contains class Pose2D
 *
 * @author <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
 * @author Max Risler
 */

#pragma once

#include "Vector2.h"

template <class T> class Range;

/** representation for 2D Transformation and Position (Location + Orientation)*/
class Pose2D : public Streamable
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

  /** Rotation as an angle*/
  float rotation;

  /** translation as an vector2*/
  Vector2<> translation;

  /** noargs-constructor*/
  Pose2D(): rotation(0), translation(0, 0) {}

  /** constructor from rotation and translation
   * \param rotation rotation (float)
   * \param translation translation (Vector2)
   */
  Pose2D(const float rotation, const Vector2<>& translation): rotation(rotation), translation(translation) {}

  /** constructor from rotation and translation
   * \param rot rotation (float)
   * \param x translation.x (float)
   * \param y translation.y (float)
   */
  Pose2D(const float rot, const float x, const float y): rotation(rot), translation(x, y)
  {}

  /** constructor from rotation
   * \param rotation rotation (float)
   */
  Pose2D(const float rotation): rotation(rotation), translation(0, 0) {}

  /** constructor from translation
   * \param translation translation (Vector2)
   */
  Pose2D(const Vector2<>& translation): rotation(0), translation(translation) {}

  /** constructor from translation
   * \param translation translation (Vector2)
   */
  Pose2D(const Vector2<int>& translation): rotation(0), translation((float) translation.x, (float) translation.y) {}

  /** constructor from two translation values
   * \param x translation x component
   * \param y translation y component
   */
  Pose2D(const float x, const float y): rotation(0), translation(x, y) {}

  /** Assignment operator
  *\param other The other Pose2D that is assigned to this one
  *\return A reference to this object after the assignment.
  */
  Pose2D& operator=(const Pose2D& other)
  {
    rotation = other.rotation;
    translation = other.translation;
    return *this;
  }

  /** Copy constructor
  *\param other The other vector that is copied to this one
  */
  Pose2D(const Pose2D& other) {*this = other;}

  /** Multiplication of a Vector2 with this Pose2D.
   *
   * Same as (point.rotate(Pose2D.rotation) + Pose2D.translation)
   *
  *\param point The Vector2 that will be multiplicated with this Pose2D
  *\return The resulting Vector2
  */
  Vector2<> operator*(const Vector2<>& point) const
  {
    float s = std::sin(rotation);
    float c = std::cos(rotation);
    return (Vector2<>(point.x * c - point.y * s , point.x * s + point.y * c) + translation);
  }

  /** Comparison of another pose with this one.
  *\param other The other pose that will be compared to this one
  *\return Whether the two poses are equal.
  */
  bool operator==(const Pose2D& other) const
  {
    return ((translation == other.translation) && (rotation == other.rotation));
  }

  /** Comparison of another pose with this one.
  *\param other The other pose that will be compared to this one
  *\return Whether the two poses are unequal.
  */
  bool operator!=(const Pose2D& other) const
  {return !(*this == other);}

  /**Concatenation of this pose with another pose.
  *\param other The other pose that will be concatenated to this one.
  *\return A reference to this pose after concatenation.
  */
  Pose2D& operator+=(const Pose2D& other)
  {
    translation = *this * other.translation;
    rotation += other.rotation;
    rotation = normalize(rotation);
    return *this;
  }

  /**A concatenation of this pose and another pose.
  *\param other The other pose that will be concatenated to this one.
  *\return The resulting pose.
  */
  Pose2D operator+(const Pose2D& other) const
  {return Pose2D(*this) += other;}

  /**Difference of this pose relative to another pose. So if A+B=C is the addition/concatenation, this calculates C-A=B.
  *\param other The other pose that will be used as origin for the new pose.
  *\return A reference to this pose after calculating the difference.
  */
  Pose2D& operator-=(const Pose2D& other)
  {
    translation -= other.translation;
    Pose2D p(-other.rotation);
    return *this = p + *this;
  }

  /**Difference of this pose relative to another pose.
  *\param other The other pose that will be used as origin for the new pose.
  *\return The resulting pose.
  */
  Pose2D operator-(const Pose2D& other) const
  {return Pose2D(*this) -= other;}

  /**Concatenation of this pose with another pose
  *\param other The other pose that will be concatenated to this one.
  *\return A reference to this pose after concatenation
  */
  Pose2D& conc(const Pose2D& other)
  {return *this += other;}

  /**Translate this pose by a translation vector
  *\param trans Vector to translate with
  *\return A reference to this pose after translation
  */
  Pose2D& translate(const Vector2<>& trans)
  {
    translation = *this * trans;
    return *this;
  }

  /**Translate this pose by a translation vector
  *\param x x component of vector to translate with
  *\param y y component of vector to translate with
  *\return A reference to this pose after translation
  */
  Pose2D& translate(const float x, const float y)
  {
    translation = *this * Vector2<>(x, y);
    return *this;
  }


  /**Rotate this pose by a rotation
  *\param angle Angle to rotate.
  *\return A reference to this pose after rotation
  */
  Pose2D& rotate(const float angle)
  {
    rotation += angle;
    return *this;
  }


  /** Calculates the inverse transformation from the current pose
  * @return The inverse transformation pose.
  */
  Pose2D invert() const
  {
    const float& invRotation = -rotation;
    return Pose2D(invRotation, (Vector2<>() - translation).rotate(invRotation));
  }

  /**
  * The function creates a random pose.
  * @param x The range for x-values of the pose.
  * @param y The range for y-values of the pose.
  * @param angle The range for the rotation of the pose.
  */
  static Pose2D random(const Range<float>& x, const Range<float>& y, const Range<float>& angle);
};
