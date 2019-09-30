/**
 * @author Arne Böckmann
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author Felix Thielke
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/WorldModelPrediction.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/FieldPercepts/CirclePercept.h"
#include "Representations/Perception/FieldPercepts/LinesPercept.h"
#include "Representations/Perception/FieldPercepts/IntersectionsPercept.h"

MODULE(IntersectionsProvider,
{,
  REQUIRES(BallSpecification),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(CirclePercept), // Just to make sure that lines on the circle are marked as such
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(LinesPercept),
  REQUIRES(WorldModelPrediction),
  PROVIDES(IntersectionsPercept),
  LOADS_PARAMETERS(
  {,
    (float)(0.15f) maxAllowedIntersectionAngleDifference, /**<The angle between two intersecting lines should not differ more from 90° than this number (in rad) */
    (float)(0.8f) maxLengthUnrecognizedProportion,  /**< the length of the recognized line multiplied by this value could maximal imagine */
    (float)(800.f) maxIntersectionGap,  /**< the maximum distance between the intersection and one end of the line (if the intersection is not on the line) */
    (float)(10.f) maxOverheadToDecleareAsEnd,  /**< the max of pixel an end can be farther away to declear as end*/
    (float)(1500.f) minimumBallExclusionCheckDistance, /**< When a ball is at least this far away, it is used to exclude intersections*/
    (float)(1.5f) ballRadiusInImageScale, /**< Enlarge ball radius in image by this factor */
  }),
});

class IntersectionsProvider : public IntersectionsProviderBase
{
private:
  bool ballIsInImageAndCanBeUsed;           // True, if the center of the ball is currently inside the image
  Vector2f ballPositionInFieldCoordinates;  // Ball position in global coordinates
  Vector2f ballPositionInImage;             // Ball position in image coordinates
  float ballRadiusInImageScaled;            // Ball radius in image, scaled by ballRadiusInImageScale

public:
  void update(IntersectionsPercept& intersectionsPercept) override;

  /**
   * Returns the distance of the closer point to target.
   * @param[out] closer the point closer to the target
   * @param[out] further the point further away from the target
   */
  float getCloserPoint(const Vector2f& a, const Vector2f& b, const Vector2f target, Vector2f& closer, Vector2f& further) const;

  /**Determines whether the point is in the line segment or not*/
  bool isPointInSegment(const LinesPercept::Line& line, const Vector2f& point) const;

  void addIntersection(IntersectionsPercept& intersectionsPercept, IntersectionsPercept::Intersection::IntersectionType type,
                       const Vector2f& intersection, const Vector2f& dir1, const Vector2f& dir2, unsigned line1, unsigned line2) const;

  /**enforces that horizontal is +90° of vertical*/
  void enforceTIntersectionDirections(const Vector2f& vertical, Vector2f& horizontal) const;
};
