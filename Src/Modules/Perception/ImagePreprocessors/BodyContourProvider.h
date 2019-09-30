/**
 * @file BodyContourProvider.h
 * This file declares a module that provides the contour of the robot's body in the image.
 * The contour can be used to exclude the robot's body from image processing.
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/Math/Eigen.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Perception/ImagePreprocessing/BodyContour.h"

MODULE(BodyContourProvider,
{,
  REQUIRES(CameraInfo),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(RobotCameraMatrix),
  REQUIRES(RobotModel),
  PROVIDES(BodyContour),
  LOADS_PARAMETERS(
  {,
    (std::vector<Vector3f>) torso, /**< The contour of the torso. */
    (std::vector<Vector3f>) shoulder, /**< The contour of the left inner shoulder. */
    (std::vector<Vector3f>) upperArm, /**< The contour of the left upper arm. */
    (std::vector<Vector3f>) lowerArm, /**< The contour of the left lower arm. */
    (std::vector<Vector3f>) lowerArm2, /**< The contour of the left lower arm. */
    (std::vector<Vector3f>) upperLeg1, /**< The contour of the left upper leg (part 1). */
    (std::vector<Vector3f>) upperLeg2, /**< The contour of the left upper leg (part 2). */
    (std::vector<Vector3f>) foot, /**< The contour of the left foot. */
  }),
});

/**
 * @class BodyContourProvider
 * A module that provides the contour of the robot's body in the image.
 * The contour can be used to exclude the robot's body from image processing.
 */
class BodyContourProvider: public BodyContourProviderBase
{
private:
  Pose3f robotCameraMatrixInverted; /**< The inverse of the current robotCameraMatrix. */

  void update(BodyContour& bodyContour) override;

  /**
   * The method projects a point in world coordinates into the image using the precomputed
   * inverse of the robot camera matrix.
   *
   * @param pointInWorld The point to be projected into the image in robot relative coordinates.
   * @param pointInImage Guess what?!
   * @return Whether the point lies in front of the image plane.
   */
  bool calculatePointInImage(const Vector3f& pointInWorld, Vector2i& pointInImage) const;

  /**
   * The method add a 3-D contour to the 2-D image contour.
   * @param origin The origin the 3-D coordinates will be interpreted relatively to.
   * @param c The 3-D contour.
   * @param sign The sign for y coordinates. Passing -1 allows to use contours of left
   *             body parts for right body parts.
   * @param bodyContour The 2-D contour in image coordinates the 3-D contour is added to.
   */
  void add(const Pose3f& origin, const std::vector<Vector3f>& c, float sign, BodyContour& bodyContour);
};
