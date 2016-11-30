/**
 * @file PlayersPerceptor.h
 * @author Michel Bartsch
 * @author Vitali Gutsch
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Perception/PlayersPercepts/PlayersPercept.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Perception/ImagePreprocessing/BodyContour.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/FieldBoundary.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Tools/Module/Module.h"

STREAMABLE(HSI,
{
  HSI() = default;
  HSI(unsigned char h, unsigned char s, unsigned char i);

  operator Image::Pixel() const
  {
    Image::Pixel pixel;
    pixel.h = h;
    pixel.s = s;
    pixel.i = i;
    return pixel;
  },

  (unsigned char) h,
  (unsigned char) s,
  (unsigned char) i,
});

inline HSI::HSI(unsigned char h, unsigned char s, unsigned char i) : h(h), s(s), i(i) {}

MODULE(PlayersPerceptor,
{,
  REQUIRES(CameraMatrix),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(Image),
  REQUIRES(BodyContour),
  REQUIRES(CameraInfo),
  REQUIRES(ECImage),
  REQUIRES(FieldBoundary),
  REQUIRES(OwnTeamInfo),
  REQUIRES(OpponentTeamInfo),

  PROVIDES(PlayersPercept),
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
    (float) addedWidth, // how far to expand a player to both sides to get the arms
    (float) onGroundMajority, // how many of the vertical scan lines need to indicate a lying obstacle to decide the obstacle lay on the ground
    (float) jerseyYFirst, // first height within a player to scan for jersey colors
    (float) jerseyYSecond, // second height within a player to scan for jersey colors
    (float) jerseyYThird,  // third height within a player to scan for jersey colors
    (float) refereeY, // height within a player to scan if it's a referee
    (int) jerseyScanConcentration, // max number of scan points to find jersey colors
    (float) jerseyNeeded, // how many of the scanned points need to have jersey color to be an obstacle with the certain jersey
    (float) robotDepth, /**< Unit: mm. How far is the center of a robot behind the position seen in the image */
    (int) minGreenH, // Colors with Hue between min and maxGreenH and with Intensity above minGreenI looking like the field
    (int) maxGreenH, //
    (int) minGreenI, //
    (int) maxWhiteS, // Colors with a Intensity above maxWhiteI and a Saturation below maxWhiteS looking like white/gray
    (int) minWhiteI, //
    (int) maxBlackI, // Colors with a Intensity below the value looking like black
    (int) maxDarkI,  // Colors with a Intensity below the value are darker
    (int) maxGrayS,  // Colors with a Saturation below the value looing like gray
    (int) maxRangeH, // specifies how big the difference between the y-channel of the found pixel and the searched color can be to count the pixel
    (int) maxRangeS, // specifies how big the difference between the cb-channel of the found pixel and the searched color can be to count the pixel
    (int) maxRangeI, // specifies how big the difference between the cr-channel of the found pixel and the searched color can be to count the pixel
    (std::vector<HSI>) colors, // Prototypical HSI color values for all team colors
  }),
});

/**
 * @class PlayersPerceptor
 * This class finds indicators for players in the image.
 */
class PlayersPerceptor: public PlayersPerceptorBase
{
private:
  /** Real world sizes in pixels. */
  int horizon; // horizon's height
  int farthestXGap; // max horizontal gab size within a player at the farthest visible point
  int farthestYGap; // max vertical gab size within a player at the farthest visible point
  int farthestMinWidth;  // min width of a player at the farthest visible point
  int nearestXGap; // same as farthestXGap for the nearest visible point
  int nearestYGap; // same as farthestYGap for the nearest visible point
  int nearestMinWidth; // same as farthestMinWidth for the nearest visible point

  /** Updates the PlayersPercept. */
  void update(PlayersPercept& PlayersPercept);

  bool calcRealWorldSizes(); // calculates the real world sizes in pixel; returns false, if it is impossible
  void scanVerticalLine(int index, int x); // performs a vertical scan at given x coordinate and writes result in verticalLines array
  bool combineVerticalLines(PlayersPercept& PlayersPercept); // one try to combine vertical scan lines to complete obstacles; returns false, if more tries would not help
  bool scanJersey(PlayersPercept::Player& obstacle, float height, bool searchForPlayer); // scans for jersey at given height in Player and writes result into it; returns true, if a jersey was detected
  void analyzePixel(int x, int y, const Image::Pixel& current, const Image::Pixel& color, unsigned char blackI,
                    bool searchForPlayer, int& black, int& nonBlack) const;
  Vector2f computeVanishingPointZ() const;

  /** Result of one vertical scan line. */
  struct scanLine
  {
    short y; // y coordinate at the bottom of the potential obstacle or zero if nothing significant was found
    bool onGround; // true, if this scan line indicates that the potential obstacle is lying on the ground
    bool noFeet; // true, if the bottom of the potential obstacle reached into the body contour or out of the image
  };
  scanLine verticalLines[Image::maxResolutionWidth];
};
