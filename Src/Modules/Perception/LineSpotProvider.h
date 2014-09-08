/**
* @author Arne Böckmann
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/Math/Vector2.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Debugging/DebugImages.h"
#include "Representations/Configuration/ColorTable.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Perception/FieldBoundary.h"
#include "Representations/Perception/LineSpots.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Perception/BodyContour.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/LinePercept.h"
#include "Representations/Perception/BallSpot.h"
#include "Representations/Perception/BallSpots.h"
#include "Representations/Perception/ScanlineRegions.h"
#include "Representations/Perception/RobotPercept.h"
#include <vector>
#include <deque>

using Line = LineSpots::Line;

MODULE(LineSpotProvider,
{,
  REQUIRES(Image),
  REQUIRES(ScanlineRegionsClipped),
  REQUIRES(ColorTable),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(BodyContour),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraInfo),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPercept),
  PROVIDES_WITH_MODIFY_AND_DRAW(LineSpots),
  REQUIRES(LineSpots),//guarantees that linespots is updated before line percept
  PROVIDES_WITH_MODIFY_AND_DRAW(LinePercept), //Just for testing
  DEFINES_PARAMETERS(
  {,
    (float)(0.5f) maxError, /**< maximum error between expected line width and measured line width [FIXME range???]*/
    (unsigned)(12) maxNumLines, /**< maximum number of detected lines. The detection will stop as soon as this number of lines has been found */
    (unsigned)(4) refitLineAfter, /**< Update line parameters after n spots have been added to a line. */
    (float)(70.0f) maxDistToLine, /**<Maximum allowed distance between a spot and a line in mm */
          //FIXME white percent sollte von der erwarteten liniendicke abhängen. bei 1px dicken linien die schräg sind geht das sonst nicht
    (float)(0.4f) whitePercent, /**<Amount of pixels that should be white between two points to assume that they are connected */
    (int)(3) scanlineLookahead, /**<FIXME should be a distance in field coordiantes */
    (float)(150.0f) minLineLength, /**<Minimum length of a line. Shorter lines will be discarded. (in mm) */
    (unsigned)(4) minSpotsOnLine, /**<Minimum number of spots per line. Note: This should usually not be changed. 3 is the minimum number to be sure that this is a straight line. */
    (int)(4) maxEdgeNoisePixels, /**<Maximum number of noise pixels at the edges of field lines */
    (unsigned)(3) minCenterCircleEvidence, /**<Minimum number of normals that need to meet each other to assume that this is a center circle spot */
    (float)(180000.0f) maxCenterCircleClusterDist, /**<Two normals are clustered if they are less than this value apart  (square distance) */
    (float)(2000.0f) maxCenterCircleFittingError,
    (float)(200.0f) maxIntersectionDistance, /**< Size of the circle around an intersection that has to contain the end points of the intersecting lines */
    (float)(0.15f) maxAllowedIntersectionAngleDifference, /**<The angle between two intersecting lines should not differ more from 90° than this number (in rad) */
    (int)(100) maxAllowedLineHeightVariance, /**<FIXME */
    (float)(3.0f) maxAllowedLineHeightFactor, /**<FIXME */
  }),
});

using Region = ScanlineRegions::Region;
using RScanline = ScanlineRegions::Scanline;
using RIterator = ScanlineRegions::RegionIterator;

/**TODO*/
class LineSpotProvider : public LineSpotProviderBase
{
private:
  ENUM(EdgeType, //edge type is read from bottom to top
    GreenToWhite,
    WhiteToGreen
  );
  
  struct Edge
  {
    EdgeType type;
    int y;

    Edge(EdgeType t, int y) :
      type(t), y(y)
    {}
  };

  struct ScanlineSpot
  {
    Vector2<int> spotInImg;
    Vector2<> spotInField;
    bool usedAsStartingPoint; /**<Whether this point has already been used as a starting point for line fitting */
    int heightInImg;

    ScanlineSpot(int xImg, int yImg, bool used, const CameraMatrix& theCameraMatrix,
                 const CameraInfo& theCameraInfo, const int height) :
      spotInImg(xImg, yImg), usedAsStartingPoint(used), heightInImg(height)
    {
      Transformation::imageToRobot(xImg, yImg, theCameraMatrix, theCameraInfo, spotInField);
    }
  };

  struct Cluster
  {
    std::vector<LineSpots::Line*> lines;
    std::vector<Vector2<>> normalSpots;
    Vector2<> center;
    Vector2<> sum; //used to recalculate the center without iteration
    bool deleted = false;

    Cluster(LineSpots::Line* line, const Vector2<>& initialPoint) :
      center(initialPoint), sum(initialPoint)
    {
      normalSpots.emplace_back(initialPoint);
      lines.emplace_back(line);
    }
  };

  void addToClusters(std::vector<Cluster>& clusters, const Vector2<>& spot) const;

  using SpotIter = std::vector<ScanlineSpot>::iterator;
  using Scanline = std::vector<ScanlineSpot>;
  using ScanlineIter = std::vector<Scanline>::iterator;

  void update(LineSpots& lineSpots);
  void update(LinePercept& linePercept);

  /*searchs one scanline for edges, fills the scanlines attribute*/
  void runVerticalScanline(const RScanline& line);

  /**calculate sobel vector on y channel for edge pixel*/
  Vector2<> calculateEdgeDirection(const int y, const int x) const;

  /**
   * Finds potential line spots by searching for green-white edges
   * Fills scanlinesVert.
   */
  void findPotentialLineSpots();
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

  Vector2<> mean(const std::vector<Vector2<>>& vector);

  /**Tries to find intersections in the lines*/
  void findIntersections(LineSpots& lineSpots);

  void validateLines(LineSpots& lineSpots) const;
  /**
   * returns the distance of the closer point to target
   * and out becomes the closer point
   */
  float getCloserPoint(const Vector2<>& a, const Vector2<>& b, const Vector2<> target, Vector2<>& out) const;

  /**Determines whether the point is on the line or not*/
  bool isPointOnLine(const Line& line, const Vector2<>& point) const;


  /**check if distance and direction of two edges is correct*/
  bool validateEdges(const Edge& gToW, const Edge& wToG, const int x) const;

  bool getInitialPoints(ScanlineIter& leftScanline, SpotIter& leftSpot,
                        ScanlineIter& rightScanline, SpotIter& rightSpot);

  bool getConnectedPair(const unsigned leftLineIndex, const unsigned rightLineIndex,
                        SpotIter& outSpotLeft, SpotIter& outSpotRight);

  /**determines whether there is a white line connecting a and b*/
  bool isWhite(Vector2<int> a, Vector2<int> b) const;


  void addIntersection(LineSpots& ls, LineSpots::Intersection::IntersectionType type,
                       const Vector2<>& intersection) const;

  /**Same as Geometry::calculateLineSize but without bugs.
   * @param p in image
   */
  template <class T>
  T calculateLineSize(const Vector2<T>& p) const
  {
    Vector2<> pOnField;
    Vector2<> pInImage;
    Transformation::imageToRobot(p.x, p.y, theCameraMatrix, theCameraInfo, pOnField);
    pOnField.x += theFieldDimensions.fieldLinesWidth;
    Transformation::robotToImage(pOnField, theCameraMatrix, theCameraInfo, pInImage);
    return (p - pInImage).abs();
  }

  float calculateExpectedLineWidth(const int x, const int y) const;

  bool getNextLeft(ScanlineIter& currentLine, SpotIter& currentSpot, const Line& line) const;
  bool getNextRight(ScanlineIter& currentLine, SpotIter& currentSpot, const Line& line) const;

  void updateLine(Line& line, std::deque<std::pair<ScanlineIter, SpotIter>>& spots) const;

  /**moves the specified spots from scanlinesVert to the specified line*/
  void moveSpotsToLine(std::deque<std::pair<ScanlineIter, SpotIter>>& spots, Line& line);

  /**sorts spot pairs according to their distance*/
  static bool compare(const std::pair<float, SpotIter>& a, const std::pair<float, SpotIter>& b);

  template<class T>
  static bool compareX(const Vector2<T>& a, const Vector2<T>& b)
  {
    return a.x < b.x;
  }
  
  /**comperator used to find the min/max elements in the spot lists*/
  static bool compareSpots(std::pair<ScanlineIter, SpotIter>& i, std::pair<ScanlineIter, SpotIter>& j);
  

  static bool compareCluster(const std::tuple<const Cluster*, const Cluster*, float>& a, const std::tuple<const Cluster*, const Cluster*, float>& b);

  bool isEdgeTowards(const RScanline& line, const int currentIndex, int& outNewIndex,
                     int& outEdgeY, ColorClasses::Color color);

  void removeSpotsInsideRobots();
  
  bool isSpotInsideRobot(const int x, const int y) const;
  
  
  template<class T>
  static bool isPointInsideRectangle(const Vector2<T> rect1,
                                     const Vector2<T> rect2,
                                     const Vector2<T>& point)
  {
    const T leftX = std::min(rect1.x, rect2.x);
    const T rightX = std::max(rect1.x, rect2.x);
    const T bottomY = std::min(rect1.y, rect2.y);
    const T topY = std::max(rect1.y, rect2.y);
    return point.x >= leftX && point.x <= rightX &&
           point.y >= bottomY && point.y <= topY;
  }

  /**Contains the edges that have been detected on each scanline*/
  std::vector<Scanline> scanlinesVert;
};
