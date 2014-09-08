/**
* @file RobotPerceptor.h
* @ author Michel Bartsch
*/

#pragma once

#include "Representations/Configuration/ColorTable.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Perception/RobotPercept.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Perception/BodyContour.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/FieldBoundary.h"
#include "Tools/Debugging/DebugImages.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Module/Module.h"
#include <cstring>

MODULE(RobotPerceptor,
{,
  REQUIRES(CameraMatrix),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(BodyContour),
  REQUIRES(CameraInfo),
  REQUIRES(Image),
  REQUIRES(FieldDimensions),
  REQUIRES(FieldBoundary),
  REQUIRES(FrameInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(ColorTable),
  PROVIDES_WITH_MODIFY_AND_DRAW(RobotPercept),
  LOADS_PARAMETERS(
  {,
    (int) xStep, // step size between vertical scans
    (int) minFarthestMillis, // min distance of the farthest visible point in millis
    (int) yOffset, // vertical scans start that much pixel below the FieldBoundary
    (int) downRange, // negative max of longRange to indicate there is NO obstacle, set it lower to be more tolerant to lying obstacles
    (int) minHeight, // min height of an obstacle in scan points, not pixel
    (int) greenMinus, // unlikelyness of an obstacle when there was green above it in the vertical scan line
    (int) lastNongreen, // how many vertical scan points in a row needs to be not green
    (float) minConnected, // how many potential obstacle points within an obstacle do not have gaps, set it higher against noise
    (float) yStepBaseUpper, // starting vertical step size at the farthest distance for the upper camera
    (float) growBaseUpper, // vertical step growing (to increase step size near the robot) before distance factors for the upper camera
    (int) yGapMillisUpper, // max vertical gap between potential obstacle points within an obstacle in millis for the upper camera
    (int) xGapMillisUpper, // max horizontal gap between potential obstacle points within an obstacle in millis for the upper camera
    (int) minWidthMillisUpper, // min width of an obstacle in millis for the upper camera
    (float) yStepBaseLower, // same as yStepBaseUpper for lower camera
    (float) growBaseLower, // same as growBaseUpper for lower camera
    (int) yGapMillisLower, // same as yGapMillisUpper for lower camera
    (int) xGapMillisLower,  // same as xGapMillisUpper for lower camera
    (int) minWidthMillisLower,  // same as minWidthMillisUpper for lower camera
    (float) addedWidth, // how far to expand a RobotBox to both sides to get the arms
    (float) onGroundMajority, // how many of the vertical scan lines need to indicate a lying obstacle to decide the obstacle lay on the ground
    (float) jerseyYFirst, // first height within a RobotBox to scan for jersey colors
    (float) jerseyYSecond, // second height within a RobotBox to scan for jersey colors
    (int) jerseyScanConcentration, // max number of scan points to find jersey colors
    (float) jerseyNeeded, // how many of the scanned points need to have jersey color to be an obstacle with the certain jersey
  }),
});

/**
 * @class RobotPerceptor
 * This class finds indicators for robots on the image.
 */
class RobotPerceptor: public RobotPerceptorBase
{
private:
  /** Real world sizes in pixels. */
  int horizon; // horizon's height
  int farthestXGap; // max horizontal gab size within a robot at the farthest visible point
  int farthestYGap; // max vertical gab size within a robot at the farthest visible point
  int farthestMinWidth;  // min width of a robot at the farthest visible point
  int nearestXGap; // same as farthestXGap for the nearest visible point
  int nearestYGap; // same as farthestYGap for the nearest visible point
  int nearestMinWidth; // same as farthestMinWidth for the nearest visible point

  /** Updates the RobotPercept. */
  void update(RobotPercept& robotPercept);

  bool calcRealWorldSizes(); // calculates the real world sizes in pixel; returns false, if it is impossible
  void scanVerticalLine(int index, int x); // performs a vertical scan at given x coordinate and writes result in verticalLines array
  bool combineVerticalLines(RobotPercept& robotPercept); // one trie to combine vertical scan lines to complete obstacles; returns false, if more tries would not help
  bool scanJersey(RobotPercept::RobotBox& obstacle, float height); // scans for jersey at given height in RobotBox and writes result into it; returns true, if a jersey was detected

  /** Result of one vertical scan line. */
  struct scanLine
  {
    short y; // y coordinate at the bottom of the potential obstacle or zero if nothing significant was found
    bool onGround; // true, if this scan line indicates that the potential obstacle is lying on the ground
    bool noFeet; // true, if the bottom of the potential obstacle reached into the body contour or out of the image
  };
  scanLine verticalLines[maxResolutionWidth];
};
