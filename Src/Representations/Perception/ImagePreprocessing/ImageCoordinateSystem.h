/**
 * @file ImageCoordinateSystem.h
 * Declaration of a struct that provides transformations on image coordinates.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 * @author <a href="mailto:oberlies@sim.tu-darmstadt.de">Tobias Oberlies</a>
 */

#pragma once

#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"

/**
 * @struct ImageCoordinateSystem
 * A struct that provides transformations on image coordinates.
 */
STREAMABLE(ImageCoordinateSystem,  // TODO: "Conditional jump or move depends on uninitialised value(s)"
{
private:
  CameraInfo cameraInfo;
  bool cameraInfoInitialized = false;
  int* xTable = nullptr;
  int* yTable = nullptr;
  int table[6144];

public:
  ~ImageCoordinateSystem();

  void setCameraInfo(const CameraInfo& cameraInfo);

  /**
   * Converts image coordintates into coordinates in the horizon-aligned coordinate system.
   * @param imageCoords The point in image coordinates.
   * @return The point in horizon-aligned coordinates.
   */
  //Vector2f toHorizonAligned(const Vector2i& imageCoords) const
  //{
  //  return invRotation * imageCoords.cast<float>();
  //}

  /**
   * Converts image coordintates into coordinates in the horizon-aligned coordinate system.
   * @param imageCoords The point in image coordinates.
   * @return The point in horizon-aligned coordinates.
   */
  Vector2f toHorizonAligned(const Vector2f& imageCoords) const
  {
    return invRotation * imageCoords;
  }

  /**
   * Converts coordinates in the horizon-aligned coordinate system into image coordinates.
   * No clipping is done.
   * @param horizonAlignedCoords The point in horizon-aligned coordinates.
   * @return The point in image coordinates.
   */
  Vector2i fromHorizonAligned(const Vector2f& horizonAlignedCoords) const
  {
    const Vector2f result = rotation * horizonAlignedCoords;
    return result.cast<int>();
  }

  /**
   * Converts image coordintates into coordinates in the horizon-based coordinate system,
   * i.e. a system of coordinates, in which the origin of the horizon is mapped to (0, 0).
   * @param imageCoords The point in image coordinates.
   * @return The point in horizon-based coordinates.
   */
  //Vector2f toHorizonBased(const Vector2i& imageCoords) const
  //{
  //  return invRotation * (imageCoords.cast<float>() -origin);
  //}

  /**
   * Converts image coordintates into coordinates in the horizon-based coordinate system,
   * i.e. a system of coordinates, in which the origin of the horizon is mapped to (0, 0).
   * @param imageCoords The point in image coordinates.
   * @return The point in horizon-based coordinates.
   */
  Vector2f toHorizonBased(const Vector2f& imageCoords) const
  {
    return invRotation * (imageCoords - origin);
  }

  /**
   * Converts coordinates in the horizon-based coordinate system into image coordinates.
   * No clipping is done.
   * @param horizonBasedCoords The point in horizon-based coordinates.
   * @return The point in image coordinates.
   */
  Vector2i fromHorizonBased(const Vector2f& horizonBasedCoords) const
  {
    const Vector2f result = rotation * horizonBasedCoords;
    return (result + origin).cast<int>();
  }

  /**
   * Corrects image coordinates so that the distortion resulting from the rolling
   * shutter is compensated.
   * No clipping is done.
   * @param imageCoords The point in image coordinates.
   * @return The corrected point.
   */
  Vector2f toCorrected(const Vector2f& imageCoords) const
  {
    const float factor = a + imageCoords.y() * b;
    return Vector2f(float(cameraInfo.opticalCenter.x() - tan(atan((cameraInfo.opticalCenter.x() - imageCoords.x()) / cameraInfo.focalLength) - factor * offset.x()) * cameraInfo.focalLength),
                    float(cameraInfo.opticalCenter.y() + tan(atan((imageCoords.y() - cameraInfo.opticalCenter.y()) / cameraInfo.focalLength) - factor * offset.y()) * cameraInfo.focalLength));
  }

  /**
   * This is simply the inverse version of the function "toCorrected". 
   *  
   * Also just approximated too, if imageCoords.y() != result.y(), because "factor" has to be calculated with result.y() for exact result.
   *
   * Used by UnionPlayersPerceptor.
   *
   * @param imageCoords The point in image coordinates.
   * @return The corrected point (inverse).
   */

  Vector2f toCorrectedInverse(const Vector2f imageCoords) const
  {
    const float factor = a + imageCoords.y() * b;
    return Vector2f(float(-(tan(-atan((imageCoords.x() - cameraInfo.opticalCenter.x()) / cameraInfo.focalLength) + factor * offset.x()) * cameraInfo.focalLength - cameraInfo.opticalCenter.x())),
                    float(tan(atan((imageCoords.y() - cameraInfo.opticalCenter.y()) / cameraInfo.focalLength) + factor * offset.y()) * cameraInfo.focalLength + cameraInfo.opticalCenter.y()));

  }
  /**
   * Corrects image coordinates so that the distortion resulting from the rolling
   * shutter is compensated.
   * No clipping is done.
   * @param imageCoords The point in image coordinates.
   * @return The corrected point.
   */
  Vector2f toCorrected(const Vector2i& imageCoords) const
  {
    return toCorrected(Vector2f(imageCoords.cast<float>()));
  }

  /**
   * "Incorrects" image coordinates so that the result contains an approximation of distortion from the rolling shutter.
   * @param coords The point in corrected image coordinates.
   * @return The incorrected point.
   */
  Vector2f fromCorrectedApprox(const Vector2f& coords) const
  {
    // This function may be called in BodyContourProvider:add() before theImageCordinateSystem has been
    // initialized properly. If that happens, we simply keep the original vector, which appears to be less
    // destructive than dropping all calculations and shuts up valgrind.
    // TODO: Perform initialization earlier
    if(!cameraInfoInitialized)
      return coords;

    float factor = a + cameraInfo.height / 2 * b;
    Vector2f v(factor * offset.x() - std::atan((coords.x() - cameraInfo.opticalCenter.x()) / cameraInfo.focalLength),
               factor * offset.y() + std::atan((coords.y() - cameraInfo.opticalCenter.y()) / cameraInfo.focalLength));
    const float EPSILON = 0.1f;
    if(v.x() < -pi_2 + EPSILON)
      v.x() = -pi_2 + EPSILON;
    else if(v.x() > pi_2 - EPSILON)
      v.x() = pi_2 - EPSILON;

    if(v.y() < -pi_2 + EPSILON)
      v.y() = -pi_2 + EPSILON;
    else if(v.y() > pi_2 - EPSILON)
      v.y() = pi_2 - EPSILON;

    return Vector2f(cameraInfo.opticalCenter.x() - std::tan(v.x()) * cameraInfo.focalLength,
                    cameraInfo.opticalCenter.y() + std::tan(v.y()) * cameraInfo.focalLength);
  }

  /**
   * "Incorrects" image coordinates so that the result contains an approximation of distortion from the rolling shutter.
   * @param coords The point in corrected image coordinates.
   * @return The incorrected point.
   */
  Vector2f fromCorrectedApprox(const Vector2i& coords) const
  {
    return fromCorrectedApprox(Vector2f(coords.cast<float>()));
  }

  /**
   * "Incorrects" image coordinates using a linearized version of the motion distortion model.
   * @param coords The point in corrected image coordinates.
   * @return The incorrected point.
   */
  Vector2f fromCorrectedLinearized(const Vector2f& coords) const
  {
    const float temp1 = cameraInfo.focalLength * offset.y();
    const float temp2 = 1.0f / (1.0f - temp1 * b);
    return Vector2f(coords.x() - cameraInfo.focalLength * offset.x() * (a + coords.y() * b) * temp2, (coords.y() + temp1 * a) * temp2);
  }

  /**
   * "Incorrects" image coordinates using a linearized version of the motion distortion model.
   * @param coords The point in corrected image coordinates.
   * @return The incorrected point.
   */
  Vector2f fromCorrectedLinearized(const Vector2i& coords) const
  {
    return fromCorrectedLinearized(Vector2f(coords.cast<float>()));
  }

  /**
   * Corrects image coordinates using a linearized version of the motion distortion model.
   * @param coords The point in corrected image coordinates.
   * @return The incorrected point.
   */
  Vector2f toCorrectedLinearized(const Vector2f& coords) const
  {
    const float temp = cameraInfo.focalLength * (a + coords.y() * b);
    return Vector2f(coords.x() + temp * offset.x(), coords.y() - temp * offset.y());
  }

  /**
   * Corrects image coordinates using a linearized version of the motion distortion model.
   * @param coords The point in corrected image coordinates.
   * @return The incorrected point.
   */
  //Vector2f toCorrectedLinearized(const Vector2i& coords) const
  //{
  //  return toCorrectedLinearized(Vector2f(coords.cast<float>()));
  //}

  /**
   * Corrects image coordinates so that the distortion resulting from the rolling
   * shutter is compensated.
   * No clipping is done.
   * @param x The x coordinate of the point in image coordinates.
   * @param y The y coordinate of the point in image coordinates.
   * @return The corrected point relative to the image center with negated signs.
   */
  Vector2i toCorrectedCenteredNeg(int x, int y) const
  {
    const float factor = a + y * b;
    x = xTable[x * Image::maxResolutionWidth / cameraInfo.width] - int(factor * offset.x());
    y = yTable[y * Image::maxResolutionHeight / cameraInfo.height] - int(factor * offset.y());
    if(x < -3072)
      x = -3072;
    else if(x > 3071)
      x = 3071;
    if(y < -3072)
      y = -3072;
    else if(y > 3071)
      y = 3071;
    return Vector2i(table[x + 3072], -table[y + 3072]) * cameraInfo.width / Image::maxResolutionWidth;
  }

  /**
   * Some coordinate system debug drawings.
   */
  void draw() const,

  /**
   * The rotation from a horizon-aligned coordinate system to the image coordinate
   * system. The horizon-aligned coordinate system is defined as follows:
   *  - the x-coordinate points parallel to the horizon to the right
   *  - the y-coordinate points perpendicular to the horizon downwards
   *  - the origin is the top left corner of the image, i.e. the same as the origin
   *    of the image coordinate system. Thus the transformation from horizon-aligned to
   *    image coordinates only requires the rotation matrix.
   * The direction of the horizon is c[0], the direction downwards is c[1].
   */
  (Matrix2f) rotation,
  (Matrix2f) invRotation, /**< The rotation from the image coordinates to the horizon-aligned coordinates. */
  (Vector2f) origin, /**< The origin of the horizon in image coordinates. */
  (Vector2f) offset, /**< The offset of the previous image to the current one. */
  (float)(0) a, /**< Constant part of equation to motion distortion. */
  (float)(0) b, /**< Linear part of equation to motion distortion. */
});
