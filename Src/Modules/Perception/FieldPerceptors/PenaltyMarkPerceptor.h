/**
 * @file PenaltyMarkPerceptor.h
 *
 * This file declares a module that searches a number of regions for a penalty mark.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/Communication/GameInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Perception/FieldPercepts/LinesPercept.h"
#include "Representations/Perception/FieldPercepts/PenaltyMarkPercept.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/CNSImage.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Perception/ImagePreprocessing/ImageRegions.h"
#include "Representations/Perception/ImagePreprocessing/RelativeFieldColors.h"
#include "Tools/ImageProcessing/CNS/ObjectCNSStereoDetector.h"
#include "Tools/Module/Module.h"

MODULE(PenaltyMarkPerceptor,
{,
  REQUIRES(BallModel),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(CNSImage),
  REQUIRES(ECImage),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(LinesPercept),
  REQUIRES(PenaltyMarkRegions),
  REQUIRES(RelativeFieldColors),
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
    (float) minDistanceToLine, /**< The minimum distance between a penalty mark and a line in order to be kept. */
  }),
});

class PenaltyMarkPerceptor : public PenaltyMarkPerceptorBase
{
  using IsGreen = std::function<bool(int, int)>;
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
   * @param isGreen A function which checks whether a certain pixel is green.
   * @return The number of green pixels found.
   */
  int countGreen(const Vector2i& center, float radius, float yRatio, const IsGreen& isGreen) const;

  /**
   * The method counts how much the penalty mark is surrounded by green
   * for a single radius.
   * @param center The assumed center of the penalty mark candidate.
   * @param radius The radius that is scanned around the center.
   * @param yRatio The ratio of the height relative to the width.
   * @param isGreen A function which checks whether a certain pixel is green.
   * @return The number of green pixels found.
   */
  int countGreenAtRadius(const Vector2i& center, float radius, float yRatio, const IsGreen& isGreen) const;

  /**
   * This method checks if a point is near a percepted line.
   * @param point A point in field coordinates.
   * @return Whether the point is near a percepted line.
   */
  bool isPointNearLine(const Vector2f& point) const;

  // Drawing methods for debugging
  void draw();
  void drawContourViaLutRasterizer(const LutRasterizer& lr, const Eigen::Isometry3d& object2World, const CameraModelOpenCV& camera, const ColorRGBA& color) const;
  void drawRasteredContour(const Contour& contour, const ColorRGBA& color) const;
  void drawCylinderRing(const CylinderRing& cylinderRing, const CameraModelOpenCV& camera) const;

public:
  PenaltyMarkPerceptor();
};
