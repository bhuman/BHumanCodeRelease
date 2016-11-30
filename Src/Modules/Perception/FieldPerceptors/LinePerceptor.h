/**
 * @file LinePerceptor.h
 *
 * Declares a module which detects lines based on ScanlineRegions.
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
#include "Representations/Perception/ImagePreprocessing/ColorScanlineRegions.h"
#include "Representations/Perception/FieldPercepts/LinesPercept.h"
#include "Representations/Perception/FieldPercepts/CirclePercept.h"
#include "Representations/Perception/PlayersPercepts/PlayersPercept.h"

#include <vector>

MODULE(LinePerceptor,
{,
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(ECImage),
  REQUIRES(FieldBoundary),
  REQUIRES(FieldDimensions),
  REQUIRES(ColorScanlineRegionsVerticalClipped),
  REQUIRES(ColorScanlineRegionsHorizontal),
  REQUIRES(Image),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(PlayersPercept),
  PROVIDES(LinesPercept),
  REQUIRES(LinesPercept),
  PROVIDES(CirclePercept),
  DEFINES_PARAMETERS({,
    (int)(20) maxLineWidthDeviation,         /**< maximum deviation of line width in the image to the expected width at that position in px */
    (float)(600) maxDistantHorizontalLength, /**< maximum length of distant horizontal lines in mm */
    (float)(50.f) maxLineFittingError,       /**< maximum error of fitted lines through spots on the field in mm */
    (unsigned int)(4) minSpotsPerLine,       /**< minimum number of spots per line */
    (unsigned int)(20) whiteCheckStepSize,   /**< step size in px when checking if lines are white */
    (float)(0.75f) minWhiteRatio,            /**< minimum ratio of white pixels in lines */
    (float)(22500.f) minSquaredLineLength,   /**< minimum squared length of found lines in mm */
    (float)(100.f) maxCircleFittingError,     /**< maximum error of fitted circles through spots on the field in mm */
    (float)(100.f) maxCircleRadiusDeviation, /**< maximum deviation in mm of the perceived center circle radius to the expected radius */
    (unsigned int)(10) minSpotsOnCircle,     /**< minimum number of spots on the center circle */
    (float)(0.75f) minCircleWhiteRatio,      /**< minimum ratio of white pixels in the center circle */
    (float)(sqr(200.f)) sqrCircleClusterRadius,
    (unsigned int)(8) minCircleClusterSize,
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
    void fitLine();
  };

  struct CircleCandidate
  {
    Vector2f center;
    float radius;

    std::vector<const Spot*> spots;

    inline CircleCandidate(const Candidate& base, Spot* anchor) : spots(base.spots)
    {
      spots.emplace_back(anchor);
      fitCircle();
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
      for(const Spot* spot : spots)
        error += getDistance(spot->field);
      return error / spots.size();
    }

    /**
    * Recalculates center and radius.
    */
    void fitCircle();
  };

  struct CircleCluster
  {
    Vector2f center;
    std::vector<Vector2f> centers;

    inline CircleCluster(const Vector2f& center) : center(center) { centers.emplace_back(center); }
  };

  std::vector<std::vector<Spot>> spotsH;
  std::vector<std::vector<Spot>> spotsV;
  std::vector<Candidate> candidates;
  std::vector<CircleCandidate> circleCandidates;
  std::vector<CircleCluster> clusters;

  /**
  * Updates the LinesPercept for the current frame.
  *
  * @param linesPercept the LinesPercept to update
  */
  void update(LinesPercept& linesPercept);

  /**
  * Updates the CirclePercept for the current frame.
  *
  * @param circlePercept the CirclePercept to update
  */
  void update(CirclePercept& circlePercept);

  /**
   * Scans the HorizontalGrayscaleScanlineRegions for line candidates.
   *
   * @param linesPercept representation in which the found lines are stored
   */
  void scanHorizontalScanlines(LinesPercept& linesPercept);
  /**
  * Scans the GrayscaleScanlineRegions for line candidates.
  *
  * @param linesPercept representation in which the found lines are stored
  */
  void scanVerticalScanlines(LinesPercept& linesPercept);

  /**
   * Checks whether the given points are connected by a white line.
   *
   * @param a coordinates of the first point in the image
   * @param b coordinates of the second point in the image
   */
  bool isWhite(const Vector2i& a, const Vector2i& b) const;

  bool correctCircle(CircleCandidate& circle) const;

  void clusterCircleCenter(const Vector2f& center);

  bool isCircleWhite(const Vector2f& center, const float radius) const;

  /**
   * Checks whether the given region lies within a player in the PlayersPercept.
   *
   * @param fromX left coordinate of the region
   * @param toX right coordinate of the region
   * @param fromY upper coordinate of the region
   * @param toY lower coordinate of the region
   */
  bool isSpotInsidePlayer(const int fromX, const int toX, const int fromY, const int toY) const;

  /**
   * Checks whether the given region lies within the feet positions of a player in the PlayersPercept.
   *
   * @param fromX left coordinate of the region
   * @param toX right coordinate of the region
   * @param fromY upper coordinate of the region
   * @param toY lower coordinate of the region
   */
  bool isSpotInsidePlayerFeet(const int fromX, const int toX, const int fromY, const int toY) const;

public:
  LinePerceptor()
  {
    spotsH.reserve(20);
    spotsV.reserve(20);
    candidates.reserve(50);
    circleCandidates.reserve(50);
  }
};
