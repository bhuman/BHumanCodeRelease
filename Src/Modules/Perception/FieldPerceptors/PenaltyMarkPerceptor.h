/**
 * @file PenaltyMarkPerceptor.h
 *
 * This file declares a module that searches a number of regions for a penalty mark.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/Configuration/CameraIntrinsics.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/FieldPercepts/PenaltyMarkPercept.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/CNSImage.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Perception/ImagePreprocessing/ImageRegions.h"
#include "Tools/ImageProcessing/CNS/ObjectCNSStereoDetector.h"
#include "Tools/Module/Module.h"

MODULE(PenaltyMarkPerceptor,
{,
  REQUIRES(CameraInfo),
  REQUIRES(CameraIntrinsics),
  REQUIRES(CameraMatrix),
  REQUIRES(CNSImage),
  REQUIRES(ECImage),
  REQUIRES(FieldDimensions),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(PenaltyMarkRegions),
  PROVIDES(PenaltyMarkPercept),
  LOADS_PARAMETERS(
  {,
    (float) maxTableRadius, /**< The maximum distance from the robot covered by the precomputed penalty mark contour table. */
    (float) minTableHeight, /**< The minimum height of the camera covered by the precomputed penalty mark contour table. */
    (float) maxTableHeight, /**< The maximum height of the camera covered by the precomputed penalty mark contour table. */
    (float) spacing, /**< The spatial discretization of the precomputed penalty mark contour table. */
    (int) numOfRotations, /**< The number of rotations searched over a 90° range. */
    (int) refineIterations, /**< The number of refinements performed after the global search. */
    (float) refineStepSize, /**< The step size during refinement (in pixels). */
    (float) minResponse, /**< The minimum response returned by the contour detector required for a penalty mark candidate. */
    (float) maxNonWhiteRatio, /**< The maximum ratio of non-white pixels allowed in a penalty mark. */
    (std::vector<float>) greenCheckRadiusRatios, /**< The radii ratios of half of the penalty mark's height and width to check for green. */
    (int) numberOfGreenChecks, /**< The number of pixels checked per scan around the penalty mark. */
    (float) minAroundGreenRatio, /**< The minimum ratio of pixels that must be green when scanning around the penalty mark. */
  }),
});

class PenaltyMarkPerceptor : public PenaltyMarkPerceptorBase
{
  ObjectCNSStereoDetector detector; /**< The detector. */
  SearchSpecification spec; /**< The current search specification. */
  std::vector<Vector3d> samplePoints; /**< Points around mark center to search for white. */

  void update(PenaltyMarkPercept& thePenaltyMarkPercept) override;

  /**
   * The method updates the search space relative to the pose of the camera.
   */
  void updateSearchSpace();

  /**
   * The method counts how much the ball is surrounded by enough green.
   * @param center The assumed center of the ball candidate.
   * @param radius The expected ball radius at that point in the image.
   * @param yRatio The ratio of the height relative to the width.
   * @return The number of green pixels found.
   */
  int countGreen(const Vector2i& center, float radius, float yRatio) const;

  /**
   * The method counts how much the penalty mark is surrounded by green
   * for a single radius.
   * @param center The assumed center of the penalty mark candidate.
   * @param radius The radius that is scanned around the center.
   * @param yRatio The ratio of the height relative to the width.
   * @return The number of green pixels found.
   */
  int countGreenAtRadius(const Vector2i& center, float radius, float yRatio) const;

  // Drawing methods for debugging
  void draw();
  void drawContourViaLutRasterizer(const LutRasterizer& lr, const Eigen::Isometry3d& object2World, const CameraModelOpenCV& camera, const ColorRGBA& color) const;
  void drawRasteredContour(const Contour& contour, const ColorRGBA& color) const;
  void drawCylinderRing(const CylinderRing& cylinderRing, const CameraModelOpenCV& camera) const;

public:
  PenaltyMarkPerceptor();
};
