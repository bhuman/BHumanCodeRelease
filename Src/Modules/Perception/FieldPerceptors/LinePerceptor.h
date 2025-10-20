/**
 * @file LinePerceptor.h
 *
 * Declares a module which detects lines and the center circle based on ColorScanLineRegions.
 *
 * @author Felix Thielke
 * @author Lukas Monnerjahn
 */

#pragma once

#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Perception/MeasurementCovariance.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Perception/FieldPercepts/CirclePercept.h"
#include "Representations/Perception/FieldPercepts/LinesPercept.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ColorScanLineRegions.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ImagePreprocessing/FieldBoundary.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Perception/ImagePreprocessing/RelativeFieldColors.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesImagePercept.h"
#include "Math/Eigen.h"
#include "Math/LeastSquares.h"
#include "Framework/Module.h"

#include <limits>
#include <vector>

MODULE(LinePerceptor,
{,
  REQUIRES(BallPercept),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(ColorScanLineRegionsVerticalClipped),
  REQUIRES(ColorScanLineRegionsHorizontal),
  REQUIRES(ECImage),
  REQUIRES(FieldBoundary),
  REQUIRES(FieldDimensions),
  REQUIRES(GameState),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(MeasurementCovariance),
  REQUIRES(ObstaclesImagePercept),
  REQUIRES(RelativeFieldColors),
  PROVIDES(LinesPercept),
  REQUIRES(LinesPercept),
  PROVIDES(CirclePercept),
  LOADS_PARAMETERS(
  {,
    (int) maxLineWidthDeviationPx,           /**< maximum deviation of line width in the image to the expected width at that position in px */
    (int) maxSkipWidth,                      /**< regions with a size of up to this many pixels can be skipped next to lines. */
    (int) maxSkipNumber,                     /**< The maximum number of neighboring regions to skip. */
    (float) greenAroundLineRatio,            /**< minimum green next to the line required as factor of line width. */
    (float) greenAroundLineRatioCalibration,
    (float) maxDistantHorizontalLength,      /**< maximum length of distant horizontal lines in mm */
    (float) maxLineFittingError,             /**< maximum error of fitted lines through spots on the field in mm */
    (unsigned) minSpotsPerLine,              /**< minimum number of spots per line */
    (unsigned) minSpotsPerLineCalibration,
    (unsigned) whiteCheckStepSize,           /**< step size in px when checking if lines are white */
    (float) minWhiteRatio,                   /**< minimum ratio of white pixels in lines */
    (float) minSquaredLineLength,            /**< minimum squared length of found lines in mm */
    (float) minSquaredSpotToCircleCenterDistance, /**< minimum distance in mm between a fieldSpot and the estimated circle center for a spot to be kept */
    (float) minCircleDeviation,              /**< minimum deviation in mm allowed for the spots to be considered as a potential circle */
    (float) maxCircleFittingError,           /**< maximum error of fitted circles through spots on the field in mm */
    (float) maxCircleRadiusDeviation,        /**< maximum deviation in mm of the perceived center circle radius to the expected radius */
    (Angle) minCircleAngleBetweenSpots,      /**< minimum angular distance around the circle center that has to be spanned by a circle candidate */
    (unsigned int) minSpotsOnCircle,         /**< minimum number of spots on the center circle */
    (float) minCircleWhiteRatio,             /**< minimum ratio of white pixels in the center circle */
    (bool) trimLines,                        /**< whether lines extended by robots shall be trimmed */
    (bool) trimLinesCalibration,
    (int) maxWidthImage,                     /**< maximum width of a line in the image at a spot up to which the spot is considered a valid line spot without further checks*/
    (float) maxWidthImageSquared,            /**< maximum squared width of a line in the image at a spot up to which the spot is considered a valid line spot without further checks */
    (float) mFactor,                         /**< the calculated width of a line at a spot in mm must be below the expected width multiplied by this factor to consider the spot a valid line spot */
    (int) minConsecutiveSpots,               /**< number of consecutive valid line spots found at which trimming shall be stopped */

    (float) squaredWhiteCheckNearField,      /**< squared distance in mm from which on the whiteCheckDistance becomes increased due to possibly blurry images */
    (float) maxNormalAngleDiff,              /**< maximum angle difference between two normal vectors for two line segments possibly belong to the same line */
    (bool) relaxedGreenCheckAtImageBorder,   /**< accept line spots with too small neighboring green regions if they reach to the image border */
    (bool) perspectivelyCorrectWhiteCheck,   /**< true: transform image to field, compute test points on field, project back; false: compute test points in image */
    (bool) highResolutionScan,               /**< use all the vertical scan lines or only the low resolution subset */
    (float) ballCheckRadiusFactor,           /**< margin factor multiplied with the ball radius when checking of a line spot is inside a seen ball */
  }),
});

class LinePerceptor : public LinePerceptorBase
{
private:
  /**
   * Structure for a line spot.
   */
  struct Spot
  {
    Vector2f image;
    Vector2f field;
    unsigned int candidate;

    Spot(const float imgX, const float imgY) : image(imgX, imgY) {}
    Spot(const Vector2f& image, const Vector2f& field) : image(image), field(field) {}
  };

  /**
   * Structure for a line candidate.
   */
  struct Candidate
  {
    Vector2f n0;
    float d;
    std::vector<const Spot*> spots;

    Candidate(const Spot* anchor) : spots()
    {
      spots.emplace_back(anchor);
    }

    /**
     * Calculates the distance of the given point to this line candidate.
     *
     * @param point point to calculate the distance to
     */
    float getDistance(const Vector2f& point) const
    {
      return std::abs(n0.dot(point) - d);
    }

    /**
     * Recalculates n0 and d.
     */
    void fitLine()
    {
      LeastSquares::LineFitter fitter;
      for(const Spot* spot : spots)
        fitter.add(spot->field);

      VERIFY(fitter.fit(n0, d));
    }
  };

  /**
   * Structure for a center circle candidate.
   */
  struct CircleCandidate
  {
    Vector2f center;
    float radius;
    std::vector<Vector2f> fieldSpots;
    LeastSquares::CircleFitter fitter;

    CircleCandidate(const Candidate& line, const Vector2f& spot)
    {
      for(const Spot* const lineSpot : line.spots)
        fieldSpots.emplace_back(lineSpot->field);
      fieldSpots.emplace_back(spot);
      fitter.add(fieldSpots);
      if(!fitter.fit(center, radius))
        radius = std::numeric_limits<float>::max();
    }

    /**
     * Adds the given field spot to the candidate and refits the circle.
     *
     * @param spot field spot to add
     */
    void addSpot(const Vector2f& spot)
    {
      fieldSpots.emplace_back(spot);
      fitter.add(spot);
      if(!fitter.fit(center, radius))
        radius = std::numeric_limits<float>::max();
    }

    /**
     * Remove field spots that are close to the estimated circle center.
     * @param minSquaredSpotToCircleCenterDistance
     */
    void removeSpotsCloseToCenter(float minSquaredSpotToCircleCenterDistance)
    {
      bool spotWasRemoved = false;
      auto spotCloseToCenter = [this, minSquaredSpotToCircleCenterDistance](Vector2f& spot) {
        return std::abs((center - spot).squaredNorm()) < minSquaredSpotToCircleCenterDistance;
      };
      do
      {
        spotWasRemoved = false;
        size_t removedSpots = std::erase_if(fieldSpots, spotCloseToCenter);
        if(removedSpots > 0)
        {
          spotWasRemoved = true;
          if(!fitter.fit(center, radius))
            radius = std::numeric_limits<float>::max();
        }
      } while(spotWasRemoved);
    }

    /**
     * Calculates the distance of the given point to this circle candidate.
     *
     * @param point point to calculate the distance to
     */
    float getDistance(const Vector2f& point) const
    {
      return std::abs((center - point).norm() - radius);
    }

    /**
     * Calculates the average error of this circle candidate.
     */
    float calculateError() const
    {
      float error = 0.f;
      for(const Vector2f& spot : fieldSpots)
        error += getDistance(spot);
      return error / static_cast<float>(fieldSpots.size());
    }

    /**
     * Calculates how much of a circle corresponding to this candidate lies between the outermost fieldSpots.
     * @return Portion of the candidate that is between the outermost fieldSpots expressed as an angle.
     */
    Angle circlePartInImage() const
    {
      Vector2f referenceVector = fieldSpots[0] - center;
      Angle low = 0_deg;
      Angle high = 0_deg;
      auto spot = fieldSpots.cbegin();
      ++spot;
      for(; spot != fieldSpots.cend(); ++spot)
      {
        const Vector2f spotAngleVector = *spot - center;
        ASSERT(spotAngleVector != Vector2f::Zero());
        const Angle spotToReference = spotAngleVector.angleTo(referenceVector);
        if(spotAngleVector.x()*referenceVector.y() - spotAngleVector.y()*referenceVector.x() < 0)
        {
          if(spotToReference > low)
            low = spotToReference;
        }
        else
        {
          if(spotToReference > high)
            high = spotToReference;
        }
      }
      return low + high;
    }

    /**
     * Computes the average distance of the robot to all field spots.
     * The distance will be used later for computing a covariance of the circle measurement.
     *
     * @param The average distance to all points that form the circle
     */
    float getAverageDistanceToFieldSpots()
    {
      float sqrDistSum = 0.f;
      for(const Vector2f& spot : fieldSpots)
        sqrDistSum += spot.squaredNorm();
      return std::sqrt(sqrDistSum / static_cast<float>(fieldSpots.size())); // Function will/should not be called, if there are no spots.
    }
  };

  std::vector<std::vector<Spot>> spotsH;
  std::vector<std::vector<Spot>> spotsV;
  std::vector<Candidate> candidates;
  std::vector<CircleCandidate> circleCandidates;

  /** distance in mm where the field next to the line is sampled during white checks */
  const float whiteCheckDistance = theFieldDimensions.fieldLinesWidth * 2;
  /** squared maximum line width in mm that can occur when fitting circle candidates to the image */
  const float circleCorrectionMaxLineWidthSquared = sqr(theFieldDimensions.fieldLinesWidth * 2);

  /**
   * Updates the LinesPercept for the current frame.
   *
   * @param linesPercept the LinesPercept to update
   */
  void update(LinesPercept& linesPercept) override;

  /**
   * Updates the CirclePercept for the current frame.
   *
   * @param circlePercept the CirclePercept to update
   */
  void update(CirclePercept& circlePercept) override;

  /**
   * Scans the ColorScanLineRegionsHorizontal for line candidates.
   *
   * @param linesPercept representation in which the found lines are stored
   */
  void scanHorizontalScanLines(LinesPercept& linesPercept);

  /**
   * Scans the ColorScanLineRegionsVerticalClipped for line candidates.
   *
   * @param linesPercept representation in which the found lines are stored
   */
  void scanVerticalScanLines(LinesPercept& linesPercept);

  /**
   * Extends all found lines by tracing white lines in the image starting at
   * the line ends.
   *
   * @param linesPercept representation in which the lines to extend are stored
   */
  void extendLines(LinesPercept& linesPercept) const;

  /**
   * Checks whether the given line is extended by robots.
   * If this is the case, the line is trimmed accordingly.
   *
   * @param line the line to trim
   */
  void trimLine(LinesPercept::Line& line) const;

  /**
   * Checks whether a given segment can be added to a line candidate
   * @param a line spot of the segments first point
   * @param b line spot of the segments last point
   * @param candidate the line candidate
   */
  bool isSegmentValid(const Spot& a, const Spot& b, const Candidate& candidate);

  /**
   * Checks whether the given points are connected by a white line.
   *
   * @param a line spot of the first point
   * @param b line spot of the second point
   * @param n0Field normal on field of the line segment or the candidate it shall be added to
   */
  bool isWhite(const Spot& a, const Spot& b, const Vector2f& n0Field) /*const*/;

  /**
   * Determines the width of a line in the image with normal n0 at the given
   * spot in field coordinates.
   *
   * @param spot the spot to check
   * @param n0 normal of the line in field coordinates
   * @return the line width in field coordinates
   */
  float getLineWidthAtSpot(const Spot& spot, const Vector2f& n0) const;

  /**
   * Corrects the given center circle candidate by projecting spots on the
   * circle back into the image, shifting them to actual white spots in the
   * image and calculating a new circle in field candidates from the results.
   *
   * @param circle circle candidate to correct
   * @return whether the candidate is valid before and after the correction
   */
  bool correctCircle(CircleCandidate& circle) const;

  /**
   * Sets a flag on all lines in the LinePercept that lie on the detected center
   * circle.
   *
   * @param center center of the detected circle
   */
  void markLinesOnCircle(const Vector2f& center);

  /**
   * Determines whether a given circle candidate is valid by verifying that
   * its points deviate sufficiently from being aligned in a straight line.
   *
   * This function calculates the maximum perpendicular distance of the points in
   * the circle candidate from the line passing through the first and last points.
   * If this maximum deviation is below a predefined threshold, the candidate is
   * considered invalid.
   *
   * @param circle The circle candidate to evaluate, containing field spots to analyze.
   * @return true if the circle candidate exhibits sufficient deviation from a straight line.
   * @return false if the circle candidate's points are too close to forming a straight line.
   */
  bool isPotentialCircle(const CircleCandidate& circle) const;

  /**
   * Checks if the model error of the circle is lower than the error if the spots
   * were fitted to a line.
   *
   * @param circle the circle candidate to check
   * @param lineError returns the model error of the fitted line
   * @return true if the circle error is lower than the line error
   */
  bool isCircleNotALine(const CircleCandidate& circle, float& lineError) const;

  /**
   * Checks whether the given center circle candidate is white when projected
   * into image coordinates.
   *
   * @param center center of the circle in field coordinates
   * @param radius radius of the circle in field coordinates
   * @return result
   */
  bool isCircleWhite(const Vector2f& center, const float radius) const;

  /**
   * Tests whether a given point qualifies as white based on reference points
   * on both sides of the line or circle it lies on.
   * @param pointOnField checked point in field coordinates
   * @param pointInImage checked point in image coordinates
   * @param n0 normal vector of the checked line point i.e. direction in which to expect field
   * @return true, if the point is considered white
   */
  bool isPointWhite(const Vector2f& pointOnField, const Vector2i& pointInImage, const Vector2f& n0) const;

  /**
   * Tests whether a given point qualifies as white based on reference points
   * on both sides of the line or circle it lies on.
   * @param pointInImage checked point in image coordinates
   * @param n points from the pointInImage to the reference points
   * @return true, if the point is considered white
   */
  bool isPointWhite(const Vector2f& pointInImage, const Vector2f& n) const;

  /**
   * Calculates the on field distance that a reference point for a white check should have to a given point
   * @param pointOnField the point
   * @return the points white check distance on field
   */
  float calcWhiteCheckDistance(const Vector2f& pointOnField) const;

  /**
   * Calculates the pixel distance a reference point for a white check should have to a given point.
   * @param pointOnField the Point in field coordinates. Used to estimate its distance
   * @return the points white check distance in pixels
   */
  float calcWhiteCheckDistanceInImage(const Vector2f& pointOnField) const;

  /**
   * Finds the vertical scan line straight left of the given spot.
   * This handles the possibly differing vertical scan line lengths
   * @param spot A spot on a vertical scan line
   * @param scanLineId id of the scanline the spot lies on
   * @return id of the scan line left of the spot
   */
  int previousVerticalScanLine(const Spot& spot, int scanLineId) const;

  /**
   * Checks whether the given region lies within an obstacle in the ObstaclesPercept.
   *
   * @param fromX left coordinate of the region
   * @param toX right coordinate of the region
   * @param fromY upper coordinate of the region
   * @param toY lower coordinate of the region
   * @return `true` if the region is inside an obstable, else `false`
   */
  bool isSpotInsideObstacle(const int fromX, const int toX, const int fromY, const int toY) const;

  /**
   * Checks if a line spot overlaps a seen ball.
   *
   * @param xPos in image X-coordinate of the region
   * @param yPos in image Y-coordinate of the region
   * @return `true`, if the region is inside the ball, else `false`
   */
  bool isSpotInsideBall(const float xPos, const float yPos) const;

  bool debugIsPointWhite = false;

#define GREEN_AROUND_LINE_RATIO (theGameState.playerState == GameState::calibration ? greenAroundLineRatioCalibration : greenAroundLineRatio)
#define MIN_SPOTS_PER_LINE (theGameState.playerState == GameState::calibration ? minSpotsPerLineCalibration : minSpotsPerLine)
#define TRIM_LINES (theGameState.playerState == GameState::calibration ? trimLinesCalibration : trimLines)

public:
  LinePerceptor()
  {
    spotsH.reserve(20);
    spotsV.reserve(20);
    candidates.reserve(50);
    circleCandidates.reserve(50);
  }
};
