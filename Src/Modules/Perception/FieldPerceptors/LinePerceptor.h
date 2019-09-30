/**
 * @file LinePerceptor.h
 *
 * Declares a module which detects lines and the center circle based on ColorScanLineRegions.
 *
 * @author Felix Thielke
 */

#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ImagePreprocessing/FieldBoundary.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Perception/ImagePreprocessing/ColorScanLineRegions.h"
#include "Representations/Perception/FieldPercepts/LinesPercept.h"
#include "Representations/Perception/FieldPercepts/CirclePercept.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesImagePercept.h"
#include "Tools/Math/LeastSquares.h"

#include <vector>
#include <limits>

MODULE(LinePerceptor,
{,
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(ECImage),
  REQUIRES(FieldBoundary),
  REQUIRES(FieldDimensions),
  REQUIRES(ColorScanLineRegionsVerticalClipped),
  REQUIRES(ColorScanLineRegionsHorizontal),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(ObstaclesImagePercept),
  PROVIDES(LinesPercept),
  REQUIRES(LinesPercept),
  PROVIDES(CirclePercept),
  DEFINES_PARAMETERS(
  {,
    (int)(20) maxLineWidthDeviation,            /**< maximum deviation of line width in the image to the expected width at that position in px */
    (int)(2) maxSkipWidth,                      /**< regions with a size of up to this many pixels can be skipped next to lines. */
    (int)(2) maxSkipNumber,                     /**< The maximum number of neighboring regions to skip. */
    (float)(4.f) greenAroundLineRatio,          /**< minimum green next to the line required as factor of line width. */
    (float)(600) maxDistantHorizontalLength,    /**< maximum length of distant horizontal lines in mm */
    (float)(50.f) maxLineFittingError,          /**< maximum error of fitted lines through spots on the field in mm */
    (unsigned int)(4) minSpotsPerLine,          /**< minimum number of spots per line */
    (unsigned int)(20) whiteCheckStepSize,      /**< step size in px when checking if lines are white */
    (float)(0.75f) minWhiteRatio,               /**< minimum ratio of white pixels in lines */
    (float)(22500.f) minSquaredLineLength,      /**< minimum squared length of found lines in mm */
    (float)(100.f) maxCircleFittingError,       /**< maximum error of fitted circles through spots on the field in mm */
    (float)(100.f) maxCircleRadiusDeviation,    /**< maximum deviation in mm of the perceived center circle radius to the expected radius */
    (unsigned int)(10) minSpotsOnCircle,        /**< minimum number of spots on the center circle */
    (float)(0.75f) minCircleWhiteRatio,         /**< minimum ratio of white pixels in the center circle */
    (float)(sqr(200.f)) sqrCircleClusterRadius, /**< squared radius for clustering center circle candidates */
    (unsigned int)(8) minCircleClusterSize,     /**< minimum size of a cluster to be considered a valid circle candidate */
    (bool)(false) doAdvancedWidthChecks,        /**< whether experimental advanced line width checks shall be done */
    (float)(2000.f) maxWidthCheckDistance,      /**< maximum distance of line spots in mm to be checked for the correct line width on forming candidates */
    (bool)(true) trimLines,                     /**< whether lines extended by robots shall be trimmed */
    (float)(sqr(20.f)) maxWidthImage,           /**< maximum squared width of a line in the image at a spot up to which the spot is considered a valid line spot without further checks */
    (float)(2.f) mFactor,                       /**< the calculated width of a line at a spot in mm must be below the expected width multiplied by this factor to consider the spot a valid line spot */
    (int)(5) minConsecutiveSpots,               /**< number of consecutive valid line spots found at which trimming shall be stopped */
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

    inline Spot(const float imgX, const float imgY) : image(imgX, imgY) {}
    inline Spot(const Vector2f& image, const Vector2f& field) : image(image), field(field) {}
  };

  /**
   * Structure for a line candidate.
   */
  struct Candidate
  {
    Vector2f n0;
    float d;
    std::vector<const Spot*> spots;

    inline Candidate(const Spot* anchor) : spots()
    {
      spots.emplace_back(anchor);
    }

    /**
     * Calculates the distance of the given point to this line candidate.
     *
     * @param point point to calculate the distance to
     */
    inline float getDistance(const Vector2f& point) const
    {
      return std::abs(n0.dot(point) - d);
    }

    /**
     * Recalculates n0 and d.
     */
    inline void fitLine()
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

    inline CircleCandidate(const Candidate& line, const Vector2f& spot)
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
    inline void addSpot(const Vector2f& spot)
    {
      fieldSpots.emplace_back(spot);
      fitter.add(spot);
      if(!fitter.fit(center, radius))
        radius = std::numeric_limits<float>::max();
    }

    /**
     * Calculates the distance of the given point to this circle candidate.
     *
     * @param point point to calculate the distance to
     */
    inline float getDistance(const Vector2f& point) const
    {
      return std::abs((center - point).norm() - radius);
    }

    /**
     * Calculates the average error of this circle candidate.
     */
    inline float calculateError() const
    {
      float error = 0.f;
      for(const Vector2f& spot : fieldSpots)
        error += getDistance(spot);
      return error / fieldSpots.size();
    }
  };

  /**
   * Structure for clustering center circle candidates.
   */
  struct CircleCluster
  {
    Vector2f center;
    std::vector<Vector2f> centers;

    inline CircleCluster(const Vector2f& center) : center(center) { centers.emplace_back(center); }
  };

  std::vector<std::vector<Spot>> spotsH;
  std::vector<std::vector<Spot>> spotsV;
  std::vector<Candidate> candidates;
  std::vector<CircleCandidate, Eigen::aligned_allocator<CircleCandidate>> circleCandidates;
  std::vector<CircleCluster> clusters;

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
  template<bool advancedWidthChecks> void scanHorizontalScanLines(LinesPercept& linesPercept);

  /**
   * Scans the ColorScanLineRegionsVerticalClipped for line candidates.
   *
   * @param linesPercept representation in which the found lines are stored
   */
  template<bool advancedWidthChecks> void scanVerticalScanLines(LinesPercept& linesPercept);

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
   * Checks whether the given points are connected by a white line.
   *
   * @param a coordinates of the first point in the image
   * @param b coordinates of the second point in the image
   */
  bool isWhite(const Vector2i& a, const Vector2i& b) const;

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
   * Adds the given potential circle center to a center circle cluster.
   *
   * @param center potential circle center coordinates
   */
  void clusterCircleCenter(const Vector2f& center);

  /**
   * Sets a flag on all lines in the LinePercept that lie on the detected center
   * circle.
   *
   * @param center center of the detected circle
   */
  void markLinesOnCircle(const Vector2f& center);

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
   * Checks whether the given region lies within an obstacle in the ObstaclesPercept.
   *
   * @param fromX left coordinate of the region
   * @param toX right coordinate of the region
   * @param fromY upper coordinate of the region
   * @param toY lower coordinate of the region
   */
  bool isSpotInsideObstacle(const int fromX, const int toX, const int fromY, const int toY) const;

public:
  LinePerceptor()
  {
    spotsH.reserve(20);
    spotsV.reserve(20);
    candidates.reserve(50);
    circleCandidates.reserve(50);
    clusters.reserve(20);
  }
};
