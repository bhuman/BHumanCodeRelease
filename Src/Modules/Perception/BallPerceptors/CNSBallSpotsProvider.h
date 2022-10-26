/**
 * @file CNSBallSpotsProvider.h
 * This file declares a module that detects balls in CNS images.
 * @author Thomas Röfer
 * @author Udo Frese
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/WorldModelPrediction.h"
#include "Representations/Perception/BallPercepts/BallSpots.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/CNSImage.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Perception/ImagePreprocessing/ImageRegions.h"
#include "ImageProcessing/CNS/ObjectCNSStereoDetector.h"
#include "Math/Eigen.h"
#include "Framework/Module.h"

MODULE(CNSBallSpotsProvider,
{,
  REQUIRES(BallRegions),
  REQUIRES(BallSpecification),
  REQUIRES(BallSpots),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(CNSImage),
  REQUIRES(FrameInfo),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(WorldModelPrediction),
  PROVIDES(BallSpots),
  LOADS_PARAMETERS(
  {,
    (int) contourSubDivisions, /**< The number of times triangles of a cube are split to form a sphere as ball model for contour computation. */
    (float) maxTableRadius, /**< The maximum distance from the robot covered by the precomputed ball contour table. */
    (float) minTableHeight, /**< The minimum height of the camera covered by the precomputed ball contour table. */
    (float) maxTableHeight, /**< The maximum height of the camera covered by the precomputed ball contour table. */
    (float) spacing, /**< The spatial discretization of the precomputed ball contour table. */
    (bool) usePrediction, /**< Include the ball prediction as a starting point for the search? */
    (int) refineIterations, /**< The number of refinements performed after the global search. */
    (float) refineStepSize, /**< The step size during refinement (in pixels). */
    (float) minResponse, /**< The minimum response returned by the contour detector required for a ball candidate. */
  }),
});

class CNSBallSpotsProvider : public CNSBallSpotsProviderBase
{
private:
  ObjectCNSStereoDetector detector; /**< The detector. */
  ObjectCNSStereoDetector::IsometryWithResponses objects; /**< The poses of the detected objects in the coordinate system of the camera and the responses. */
  SearchSpecification spec; /**< The current search specification. */

  /**
   * Searches the image for potential balls that need a final validation.
   * @param ballSpots The ball candidates to be calculated.
   */
  void update(BallSpots& ballSpots) override;

  /**
   * The method updates the search space relative to the pose of the camera.
   */
  void updateSearchSpace();

  // Drawing methods for debugging
  void draw();
  void drawRasteredContour(const Contour& contour, const ColorRGBA& color) const;
  void drawCylinderRing(const CylinderRing& cylinderRing, const CameraModelOpenCV& camera) const;

public:
  CNSBallSpotsProvider();
};
