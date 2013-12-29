/*
 * LineSpotProviderExp.h
 *
 *  Created on: Aug 21, 2012
 *      Author: Arne Böckmann (arneboe@tzi.de)
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Perception/BodyContour.h"
#include "Representations/Perception/PossibleObstacleSpots.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Perception/BodyContour.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Perception/ColorReference.h"
#include "Representations/Perception/FieldBoundary.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Tools/Math/Geometry.h"


using namespace std;
MODULE(PossibleObstacleSpotProvider)
  REQUIRES(CameraMatrix)
  REQUIRES(CameraInfo)
  REQUIRES(FieldDimensions)
  REQUIRES(BodyContour)
  REQUIRES(Image)
  REQUIRES(ImageCoordinateSystem)
  REQUIRES(ColorReference)
  REQUIRES(FieldBoundary)
  REQUIRES(MotionInfo)
  PROVIDES_WITH_DRAW(PossibleObstacleSpots)
  //PARAMETERS
  DEFINES_PARAMETER(int, minGridJump, 5) /**< The minimum number of pixels between two points in vertical direction */
  DEFINES_PARAMETER(int, minCrossSize, 2) /**< Minimum size of the cross descriptor */
  DEFINES_PARAMETER(float, minNonGreen, 0.93f) /**< Minimum amount of non green in percent that a possible obstacle spot needs */
  DEFINES_PARAMETER(float, scanLineDist, 2.f) /**< distance between scanlines in percent [0..100] of the image width */
  DEFINES_PARAMETER(unsigned, minNumOfSpotsInARow, 3) /**< minimum number of spots that have to be in a row */
  DEFINES_PARAMETER(float, lowerCameraMaxAngle, 1.15f)/**< disable obstacle detection in lower camera if abs(head angle) is bigge than this value */
END_MODULE

class PossibleObstacleSpotProvider : public PossibleObstacleSpotProviderBase
{
public:
  PossibleObstacleSpotProvider();
  void update(PossibleObstacleSpots& obs);
private:
  /**Calculate if the cross at (x,gridy[gridYIndex]) is not green enough */
  bool isNotGreenEnough(const int x, const int gridYIndex, const int width) const;

  /**Fills gridY and minNonGreenCounts*/
  void calculateGrid();
  /**calculates the sum image of non green pixels for each row specified in gridY */
  void calculateNonGreenRowCount();

  /**calculates the y coordinate of the horizon with boundary check*/
  int calcHorizon() const;
  /**calculates the distance between two scanlines in pixel based in parameters*/
  int calculateScanlineDistance(int size, float percent) const;

  /**Determines the first index above the body cuntoure for the specified x. Returns false if an error occured */
  bool getFirstIndexAboveBodyConture(const int x, unsigned& outIndex) const;

  vector<int> gridY;/**< y coordinates of the grid points from bottom to top. Not clipped */
  vector<int> crossSizes; /**< Contains the sizes of the cross that should be used. Index is the same as for gridY */
  vector<vector<int>> nonGreenCount; /** Sum over the non green pixels for each grid row. Index is the same as for gridY */
  vector<int> minNonGreenCounts; /**< The minimum number of non green pixels that have to be inside a cross. index is the same as for gridY*/
};

