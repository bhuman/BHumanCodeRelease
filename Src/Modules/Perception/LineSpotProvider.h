/**
* @author Arne Böckmann
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Transformation.h"
#include "Representations/Configuration/ColorTable.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Perception/LineSpots.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/LinePercept.h"
#include "Representations/Perception/ScanlineRegions.h"
#include "Representations/Perception/PlayersPercept.h"
#include "Representations/Perception/PotentialLineSpots.h"
#include <vector>
#include <deque>

using Line = LineSpots::Line;

MODULE(LineSpotProvider,
{,
  REQUIRES(Image),
  REQUIRES(ColorTable),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraInfo),
  REQUIRES(FieldDimensions),
  REQUIRES(PlayersPercept),
  REQUIRES(PotentialLineSpots),
  PROVIDES(LineSpots),
  REQUIRES(LineSpots),//guarantees that linespots is updated before line percept
  PROVIDES(LinePercept), //Just for testing
  DEFINES_PARAMETERS(
  {,
    (unsigned)(12) maxNumLines, /**< maximum number of detected lines. The detection will stop as soon as this number of lines has been found */
    (unsigned)(4) refitLineAfter, /**< Update line parameters after n spots have been added to a line. */
    (float)(70.0f) maxDistToLine, /**<Maximum allowed distance between a spot and a line in mm */
          //FIXME white percent sollte von der erwarteten liniendicke abhängen. bei 1px dicken linien die schräg sind geht das sonst nicht
    (float)(0.4f) whitePercent, /**<Amount of pixels that should be white between two points to assume that they are connected */
    (int)(3) scanlineLookahead, /**<FIXME should be a distance in field coordiantes */
    (float)(150.0f) minLineLength, /**<Minimum length of a line. Shorter lines will be discarded. (in mm) */
    (unsigned)(4) minSpotsOnLine, /**<Minimum number of spots per line. Note: This should usually not be changed. 3 is the minimum number to be sure that this is a straight line. */
    (unsigned)(3) minCenterCircleEvidence, /**<Minimum number of normals that need to meet each other to assume that this is a center circle spot */
    (float)(180000.0f) maxCenterCircleClusterDist, /**<Two normals are clustered if they are less than this value apart  (square distance) */
    (float)(2000.0f) maxCenterCircleFittingError,
    (float)(200.0f) maxIntersectionDistance, /**< Size of the circle around an intersection that has to contain the end points of the intersecting lines */
    (float)(0.15f) maxAllowedIntersectionAngleDifference, /**<The angle between two intersecting lines should not differ more from 90° than this number (in rad) */
    (int)(100) maxAllowedLineHeightVariance, /**<FIXME */
  }),
});

using Region = ScanlineRegions::Region;
using RScanline = ScanlineRegions::Scanline;

/**TODO*/
class LineSpotProvider : public LineSpotProviderBase
{
private:
  struct ScanlineSpot
  {
    Vector2i spotInImg;
    Vector2f spotInField;
    bool usedAsStartingPoint; /**<Whether this point has already been used as a starting point for line fitting */
    int heightInImg;
    ScanlineSpot() = default;
    ScanlineSpot(int xImg, int yImg, bool used, Vector2f inField, const int height) :
      spotInImg(xImg, yImg), spotInField(inField), usedAsStartingPoint(used), heightInImg(height) {}
  };

  struct Cluster
  {
    std::vector<LineSpots::Line*> lines;
    std::vector<Vector2f> normalSpots;
    Vector2f center;
    Vector2f sum; //used to recalculate the center without iteration
    bool deleted = false;

    Cluster(LineSpots::Line* line, const Vector2f& initialPoint) :
      center(initialPoint), sum(initialPoint)
    {
      normalSpots.emplace_back(initialPoint);
      lines.emplace_back(line);
    }
  };

  void addToClusters(std::vector<Cluster>& clusters, const Vector2f& spot) const;

  using SpotIter = std::vector<ScanlineSpot>::iterator;
  using Scanline = std::vector<ScanlineSpot>;
  using ScanlineIter = std::vector<Scanline>::iterator;

  void update(LineSpots& lineSpots);
  void update(LinePercept& linePercept);

  /**
   * Fits lines to the points found by findPotentialLineSpots().
   * Fills lines and leftOverSpots
   */
  void findLines(LineSpots& linespots);

  /**
   * Tries to calculate the center circle position
   */
  void findCircle(LineSpots& lineSpots);

  float evaluateCircle(const Cluster& cluster, Geometry::Circle& circle);

  Vector2f mean(const std::vector<Vector2f>& vector);

  /**Tries to find intersections in the lines*/
  void findIntersections(LineSpots& lineSpots);

  void validateLines(LineSpots& lineSpots) const;
  /**
   * returns the distance of the closer point to target
   * @param[out] closer the point closer to the target
   * @param[out] further the point further away from the target
   */
  float getCloserPoint(const Vector2f& a, const Vector2f& b, const Vector2f target, Vector2f& closer, Vector2f& further) const;

  /**Determines whether the point is on the line or not*/
  bool isPointOnLine(const Line& line, const Vector2f& point) const;

  bool getInitialPoints(ScanlineIter& leftScanline, SpotIter& leftSpot,
                        ScanlineIter& rightScanline, SpotIter& rightSpot);

  bool getConnectedPair(const unsigned leftLineIndex, const unsigned rightLineIndex,
                        SpotIter& outSpotLeft, SpotIter& outSpotRight);

  /**determines whether there is a white line connecting a and b*/
  bool isWhite(Vector2i a, Vector2i b) const;


  void addIntersection(LineSpots& ls, LineSpots::Intersection::IntersectionType type,
                       const Vector2f& intersection, const Vector2f& dir1, const Vector2f& dir2, const Line& line1, const Line& line2) const;

  bool getNextLeft(ScanlineIter& currentLine, SpotIter& currentSpot, const Line& line) const;
  bool getNextRight(ScanlineIter& currentLine, SpotIter& currentSpot, const Line& line) const;

  void updateLine(Line& line, std::deque<std::pair<ScanlineIter, SpotIter>>& spots) const;

  /**moves the specified spots from scanlinesVert to the specified line*/
  void moveSpotsToLine(std::deque<std::pair<ScanlineIter, SpotIter>>& spots, Line& line);

  /**sorts spot pairs according to their distance*/
  static bool compare(const std::pair<float, SpotIter>& a, const std::pair<float, SpotIter>& b);

  /**comperator used to find the min/max elements in the spot lists*/
  static bool compareSpots(std::pair<ScanlineIter, SpotIter>& i, std::pair<ScanlineIter, SpotIter>& j);


  static bool compareCluster(const std::tuple<const Cluster*, const Cluster*, float>& a, const std::tuple<const Cluster*, const Cluster*, float>& b);

  void removeSpotsInsidePlayers();

  bool isSpotInsidePlayer(const int x, const int y) const;

  /**enforces that horizontal is +90° of vertical*/
  void enforceTIntersectionDirections(const Vector2f& vertical, Vector2f& horizontal) const;

  template<class T>
  static bool isPointInsideRectangle(const Eigen::Matrix< T, 2, 1>& rect1,
                                     const Eigen::Matrix< T, 2, 1>& rect2,
                                     const Eigen::Matrix< T, 2, 1>& point)
  {
    const T leftX = std::min(rect1.x(), rect2.x());
    const T rightX = std::max(rect1.x(), rect2.x());
    const T bottomY = std::min(rect1.y(), rect2.y());
    const T topY = std::max(rect1.y(), rect2.y());
    return point.x() >= leftX && point.x() <= rightX &&
           point.y() >= bottomY && point.y() <= topY;
  }

  /**Contains the edges that have been detected on each scanline*/
  std::vector<Scanline> scanlinesVert;
};
