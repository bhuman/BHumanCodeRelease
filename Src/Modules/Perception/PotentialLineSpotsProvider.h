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
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Perception/ScanlineRegions.h"
#include "Representations/Perception/PlayersPercept.h"
#include "Representations/Perception/PotentialLineSpots.h"
#include <vector>
#include <deque>

using Line = LineSpots::Line;

MODULE(PotentialLineSpotsProvider,
{,
  REQUIRES(Image),
  REQUIRES(ScanlineRegionsClipped),
  REQUIRES(ColorTable),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraInfo),
  REQUIRES(FieldDimensions),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(PlayersPercept),
  PROVIDES(PotentialLineSpots),
  DEFINES_PARAMETERS(
  {,
    (int) (4) maxEdgeNoisePixels, /**<Maximum number of noise pixels at the edges of field lines */
    (float) (3.0f) maxAllowedLineHeightFactor, /**<FIXME */
  }),
});

using Region = ScanlineRegions::Region;
using RScanline = ScanlineRegions::Scanline;

/**TODO*/
class PotentialLineSpotsProvider : public PotentialLineSpotsProviderBase
{
private:
  ENUM(EdgeType,
  {, //edge type is read from bottom to top
    GreenToWhite,
    WhiteToGreen,
  });

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
    Vector2i spotInImg;
    Vector2f spotInField;
    bool usedAsStartingPoint; /**<Whether this point has already been used as a starting point for line fitting */
    int heightInImg;

    ScanlineSpot(int xImg, int yImg, bool used, Vector2f inField, const int height) :
      spotInImg(xImg, yImg), spotInField(inField), usedAsStartingPoint(used), heightInImg(height) {}
  };

  using SpotIter = std::vector<ScanlineSpot>::iterator;
  using Scanline = std::vector < ScanlineSpot > ;
  using ScanlineIter = std::vector<Scanline>::iterator;

  void update(PotentialLineSpots& potentialLineSpots);

  /*searchs one scanline for edges, fills the scanlines attribute*/
  void runVerticalScanline(unsigned int lineIdx, PotentialLineSpots& potentialLineSpots);

  /**calculate sobel vector on y channel for edge pixel*/
  Vector2f calculateEdgeDirection(const int y, const int x) const;

  /**
   * Finds potential line spots by searching for green-white edges
   * Fills scanlinesVert.
   */
  void findPotentialLineSpots(PotentialLineSpots& potentialLineSpots);
  /**check if distance and direction of two edges is correct*/
  bool validateEdges(const Edge& gToW, const Edge& wToG, const int x) const;

  /**Same as Geometry::calculateLineSize but without bugs.
   * returns -1 when image robot conversion failed
   * @param p in image
   */
  float calculateLineSize(const Vector2f& p) const
  {
    Vector2f pOnField;
    Vector2f pInImage;
    if(Transformation::imageToRobot(p, theCameraMatrix, theCameraInfo, pOnField))
    {
      pOnField.x() += theFieldDimensions.fieldLinesWidth;
      if(Transformation::robotToImage(pOnField, theCameraMatrix, theCameraInfo, pInImage))
      {
        return (p - pInImage).norm();
      }
    }
    return -1;
  }

  float calculateExpectedLineWidth(const int x, const int y) const;

  bool isEdgeTowards(const RScanline& line, const int currentIndex, int& outNewIndex,
                     int& outEdgeY, ColorClasses::Color color) const;


  /**Contains the edges that have been detected on each scanline*/
  std::vector<Scanline> scanlinesVert;
};
