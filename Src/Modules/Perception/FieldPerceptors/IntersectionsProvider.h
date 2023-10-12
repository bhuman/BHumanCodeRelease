/**
 * @file IntersectionsProvider.h
 *
 * This file declares a module that detects and classifies intersections of field lines.
 *
 * @author Arne Böckmann
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author Felix Thielke
 * @author Laurens Schiefelbein
 */

#pragma once

#include "Framework/Module.h"
#include "ImageProcessing/PatchUtilities.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/WorldModelPrediction.h"
#include "Representations/Perception/MeasurementCovariance.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Perception/FieldPercepts/LinesPercept.h"
#include "Representations/Perception/FieldPercepts/IntersectionsPercept.h"
#include <CompiledNN/CompiledNN.h>
#include <CompiledNN/Model.h>

MODULE(IntersectionsProvider,
{,
  REQUIRES(BallSpecification),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(ECImage),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(LinesPercept),
  REQUIRES(MeasurementCovariance),
  REQUIRES(WorldModelPrediction),
  PROVIDES(IntersectionsPercept),
  LOADS_PARAMETERS(
  {,
    (float) maxAllowedIntersectionAngleDifference,  /**<The angle between two intersecting lines should not differ more from 90° than this number (in rad) */
    (float) maxLengthUnrecognizedProportionRelaxed,  /**< the length of the recognized line multiplied by this value could maximal imagine */
    (float) maxOverheadToDecleareAsEndRelaxed,  /**< the max of pixel an end can be farther away to declear as end*/
    (float) minimumBallExclusionCheckDistance,  /**< When a ball is at least this far away, it is used to exclude intersections*/
    (float) ballRadiusInImageScale,  /**< Enlarge ball radius in image by this factor */
    (float) maxLengthUnrecognizedProportionStrict, /**< same as the other but stricter */
    (float) maxIntersectionGapStrict, /**< same as the other but stricter*/
    (float) maxOverheadToDecleareAsEndStrict, /**< same as the other but stricter*/
    (float) threshold,  /**< threshold value for the confidence value of the neural net. If 0, neural net is not used. */
    (int) patchSize, /**< Size of the patch used for classification. */
    (int) minClassificationDistance, /**< Only intersections from this distance will be classified by the network.*/
  }),
});

class IntersectionsProvider : public IntersectionsProviderBase
{
public:
  /** Constructor. */
  IntersectionsProvider();

private:
  void update(IntersectionsPercept& intersectionsPercept) override;

  /**
   * Returns the distance of the closer point to target.
   * @param[out] closer the point closer to the target
   * @param[out] further the point further away from the target
   */
  float getCloserPoint(const Vector2f& a, const Vector2f& b, const Vector2f target, Vector2f& closer, Vector2f& further) const;

  /**Determines whether the point is in the line segment or not*/
  bool isPointInSegment(const LinesPercept::Line& line, const Vector2f& point) const;

  /**
   * Validtes the intersection type and emplaces the newly found intersection into the IntersectionsPercept.
   * @param intersectionsPercept the IntersectionsPercept the intersection is added to.
   * @param type the type of the intersection. X, T or L.
   * @param intersection the position of the intersection candidate, in field coordinates
   * @param dir1 the first direction of the lines intersected, in field coordinates
   * @param dir2 the second direction of the lines intersected, in field coordinates
   * @param line1 the first line of the intersection
   * @param line2 the second line of the intersection
   */
  void addIntersection(IntersectionsPercept& intersectionsPercept, IntersectionsPercept::Intersection::IntersectionType type, Vector2f& intersection, const Vector2f& dir1, const Vector2f& dir2, unsigned line1, unsigned line2);

  /**enforces that horizontal is +90° of vertical*/
  void enforceTIntersectionDirections(const Vector2f& vertical, Vector2f& horizontal) const;

  /** Classfies the intersection candidate with a neural net and returns the predicted type.
   * @param type the type of the intersection candidate that was predicted by the provider
   * @param intersection the position of the intersection candidate in field coordinates
   * @param pInImg the position of the intersection candidate in the image
   * @return False if the neural net predicted the given candidate not to be an intersection. True otherwise.
   */
  bool classifyIntersection(IntersectionsPercept::Intersection::IntersectionType& type, const Vector2f& intersection, const Vector2f& pInImg);

  /** Checks if the center of the patch is within the camera bounds.
   * @param intersectionPoint Image coordinate of the intersection
   * @return True if the point is within the camera bounds. False otherwise.
   */
  bool isWithinBounds(const Vector2f& intersectionPoint);

  bool ballIsInImageAndCanBeUsed;           // True, if the center of the ball is currently inside the image
  Vector2f ballPositionInFieldCoordinates;  // Ball position in global coordinates
  Vector2f ballPositionInImage;             // Ball position in image coordinates
  float ballRadiusInImageScaled;            // Ball radius in image, scaled by ballRadiusInImageScale

  NeuralNetwork::CompiledNN network;
  std::unique_ptr<NeuralNetwork::Model> model;
};
