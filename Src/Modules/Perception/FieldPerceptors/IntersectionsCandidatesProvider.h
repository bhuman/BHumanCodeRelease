/**
 * @file IntersectionsCandidatesProvider.h
 *
 * This file implements a module that detects and prepares intersections candidates for classifying.
 *
 * @author Arne Böckmann
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author Felix Thielke
 * @author Laurens Schiefelbein
 * @author Kevin Dehmlow
 * @author Roman Sablotny
 *
 */

#pragma once

#include "Framework/Module.h"
#include "ImageProcessing/PatchUtilities.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Perception/MeasurementCovariance.h"
#include "Representations/Perception/FieldPercepts/IntersectionCandidates.h"
#include "Representations/Perception/FieldPercepts/LinesPercept.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include <CompiledNN/CompiledNN.h>
#include <CompiledNN/Model.h>

MODULE(IntersectionsCandidatesProvider,
{,
    REQUIRES(CameraInfo),
    REQUIRES(CameraMatrix),
    REQUIRES(ECImage),
    REQUIRES(FrameInfo),
    REQUIRES(LinesPercept),
    REQUIRES(MeasurementCovariance),
    REQUIRES(RobotPose),
    PROVIDES(IntersectionCandidates),
    LOADS_PARAMETERS(
    {,
      (unsigned) patchSize, /**< size of the patch */
      (float) maxAllowedAngleDifference, /**< max difference between two lines to declare as intersection*/
      (float) maxIntersectionGap, /**< max gap between lines to declare as intersection */
      (float) normFactor, /**< factor to normalize the distance to the intersection from the robot */
      // Parameters for saving Patches to use as training data for the neural network model
      (bool) savePatches, /**< save patch of every found intersection candidate */
      /** absolute path or relative path from current working directory where to store the patches (ReplayRobot works in Config/Scenes) */
      (std::string) baseDirectory,
      (std::string) logFileName, /**< name of the replayed logfile for naming the stored patches */
    }),
});

class IntersectionsCandidatesProvider : public IntersectionsCandidatesProviderBase
{
public:
  /** Constructor */
  IntersectionsCandidatesProvider();

private:
  void update(IntersectionCandidates& intersectionCandidates) override;

  /**
   * Stores a patch from the given image coordinates as center of the patch to the given reference.
   * @param interImg image coordinates of the intersection
   * @param interField field coordinates of the intersection
   * @param[out] patch, reference where to store the patch
   * @return True, when a patch has been correctly stored in the patch reference
   */
  bool generatePatch(const Vector2f interImg, const Vector2f& interField, Image<PixelTypes::GrayscaledPixel>& patch) const;

  /**
   * Returns the distance of the closer point to target.
   * @param[out] closer the point closer to the target
   * @param[out] further the point further away from the target
   */
  template<typename T>
  float getCloserPoint(const Eigen::Matrix<T, 2, 1>& a, const Eigen::Matrix<T, 2, 1>& b, const Eigen::Matrix<T, 2, 1>& target, Eigen::Matrix<T, 2, 1>& closer, Eigen::Matrix<T, 2, 1>& further) const;

  /**
   * Checks if the center of the patch is within the camera bounds.
   * @param intersectionPoint Image coordinate of the intersection
   * @return True if the point is within the camera bounds. False otherwise.
   */
  bool isWithinBounds(const Vector2f& intersectionPoint) const;

  /**
   * Save the given patch in the Directory specified in Parameter baseDirectory named with the prefix from logFileName as a binary file
   * Binary file will be saved in cwd/baseDirectory/type/logFileName-timestamp-camera-distance.bin
   * @param patch to save on the disk
   * @param type intersection type, patch will be saved in the directory baseDirectory/type
   * @param distance the distance from the robot to the intersection
   */
  void savePatchOnDisk(const Image<PixelTypes::GrayscaledPixel>& patch, const IntersectionsPercept::Intersection::IntersectionType type, const int distance) const;
  void savePatchOnDisk(const Image<PixelTypes::GrayscaledPixel>& patch, const std::string& type, const int distance) const;

  /**
   * Determines a possible none intersection by calculating a point on the image around the given image coordinates.
   * Stores on the given reference the patch and the distance to the point
   * @param intersectionOnImage image coordinates of an intersection
   * @param[out] patch reference, where to store the patch
   * @param[out] robotToNoneIntersectionDistance reference, where to store the distance from the robot to the calculated point (from field coordinates)
   * @return True, when a possible none intersection point has been calculated
   */
  bool calculateNoneIntersection(const Vector2f& intersectionOnImage, Image<PixelTypes::GrayscaledPixel>& patch, int& robotToNoneIntersectionDistance) const;

  /** Determines whether the point is in the line segment or not */
  bool isPointInSegment(const LinesPercept::Line& line, const Vector2f& point) const;
};
