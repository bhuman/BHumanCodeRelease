/**
 * @file ImageCoordinateSystem.h
 * Declaration of a struct that provides transformations on image coordinates.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 * @author <a href="mailto:oberlies@sim.tu-darmstadt.de">Tobias Oberlies</a>
 */

#pragma once

#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Tools/ImageProcessing/PixelTypes.h"
#include "Tools/ImageProcessing/TImage.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"

/**
 * @struct ImageCoordinateSystem
 * A struct that provides transformations on image coordinates.
 */
STREAMABLE(ImageCoordinateSystem,
{
public:
  CameraInfo cameraInfo; /**< A copy of the camera information that is required for the methods to work. Isn't logged. */

  /**
   * Converts image coordinates into coordinates in the horizon-aligned coordinate system.
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
    return Vector2f(cameraInfo.opticalCenter.x() - std::tan(std::atan((cameraInfo.opticalCenter.x() - imageCoords.x()) / cameraInfo.focalLength) - factor * offset.x()) * cameraInfo.focalLength,
                    cameraInfo.opticalCenter.y() + std::tan(std::atan((imageCoords.y() - cameraInfo.opticalCenter.y()) / cameraInfo.focalLengthHeight) - factor * offset.y()) * cameraInfo.focalLengthHeight);
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
   * Inverse of toCorrected.
   *
   * @param correctedCoords The corrected point in image coordinates.
   * @return The original point.
   */
  Vector2f fromCorrected(const Vector2f& correctedCoords) const;

  /**
   * Inverse of toCorrected.
   *
   * @param correctedCoords The corrected point in image coordinates.
   * @return The original point.
   */
  Vector2f fromCorrected(const Vector2i& correctedCoords) const
  {
    return fromCorrected(Vector2f(correctedCoords.cast<float>()));
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
