/**
 * @file BallSpotsProvider.h
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Perception/BallPercepts/BallSpots.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesImagePercept.h"
#include "Representations/Perception/ImagePreprocessing/BodyContour.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ImagePreprocessing/FieldBoundary.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Perception/ImagePreprocessing/ColorScanLineRegions.h"
#include "Representations/Perception/ImagePreprocessing/RelativeFieldColors.h"
#include "Representations/Modeling/WorldModelPrediction.h"

MODULE(BallSpotsProvider,
{,
  REQUIRES(BallSpecification),
  REQUIRES(BodyContour),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(ColorScanLineRegionsVerticalClipped),
  REQUIRES(ECImage),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(ObstaclesImagePercept),
  REQUIRES(RelativeFieldColors),
  REQUIRES(WorldModelPrediction),
  PROVIDES(BallSpots),
  LOADS_PARAMETERS(
  {,
    (float) minRadiusOfWantedRegion, //< if a regions radius is smaller than this, the green check must be successful
    (Angle) greenEdge, //< The greenEdge for IISC::calcPossibleVisibleBallByLowestPoint(..)
    (float) ballSpotDistUsage, //< How much of the lower visible part of the ball has to be found
    (float) scanLengthRadiusFactor, //< The factor to determine the maximum scan length
    (unsigned) maxNumberOfSkippablePixel, //< The maximum number of pixels that are allowed to be skipped while scanBallSpotOneDirection(..)
    (float) minAllowedDistanceRadiusRelation, //< The factor to determine the minimum distance between two ball spots
    (float) minFoundDiameterPercentage, //< The minimum ratio of the messured width compared to the calculated width
    (float) noiseThreshold, //< The maximum ratio of non-good pixels
    (float) minGoodNeutralRatio, //< The minimum ratio of good pixels compared to non-bad pixels

    (int) additionalRadiusForGreenCheck, //< the distance between the ball spot and the green scan areas
    (float) greenPercent, //< the minimum ratio of detected green compared to all considered pixels in the green check

    (bool) allowScanLineTopSpotFitting, // Is it allowed to find a spot on top of a scanLine?
    (bool) lessStrictChecks, // Allow more Ballspots?
  }),
});

/**
 * @class BallSpotsProvider
 * A module that provides spots that might be inside balls
 */
class BallSpotsProvider : public BallSpotsProviderBase
{
  /**
   * The main method of this module.
   * @param ballSpots The percept that is filled by this module.
   */
  void update(BallSpots& ballSpots) override;

  /**
   * The method searches with the help of ColorScanLineRegionsVerticalClipped
   * for initial ball spot and adds them to the list (ballSpot.spots) if no
   * check fails.
   *
   * @param ballSpots The percept that is filled by this module.
   */
  void searchScanLines(BallSpots& ballSpots) const;

  /**
   * The method scans in y-direction to adjust the initial spot guess.
   *
   * @param initialPoint The initial ball spot guess
   * @param circle The guessed ball in image
   * @param luminanceRef The luminance of a white reference Pixel
   * @param saturationRef The saturation of a white reference Pixel
   * @return Is the scanned width not entirely smaller then the calculated one?
   *         And are the pixels right colored in the majority?
   */
  bool correctWithScanLeftAndRight(Vector2i& initialPoint, const Geometry::Circle& circle, unsigned char luminanceRef, unsigned char saturationRef) const;

  /**
   * The method performs a scan in one direction.
   *
   * @param spot The current ball spot to work with
   * @param currentLength A variable that counts the scanned pixels
   * @param maxLength The maximum scan length
   * @param goodPixelCounter A variable that counts the accepted pixels
   * @param getX(Vector2i,int) A pointer of a function that calculates the x-Value of the next element
   * @param getY(Vector2i,int) A pointer of a function that calculates the y-Value of the next element
   * @param luminanceRef The luminance of a white reference Pixel
   * @param saturationRef The saturation of a white reference Pixel
   */
  void scanBallSpotOneDirection(const Vector2i& spot, int& currentLength, const int& maxLength,
                                unsigned& goodPixelCounter,
                                int(*getX)(const Vector2i& spot, const int currentLength),
                                int(*getY)(const Vector2i& spot, const int currentLength),
                                unsigned char luminanceRef, unsigned char saturationRef) const;

  /**
   * The method checks a pixel.
   *
   * @param pixel The color pixel
   * @param goodPixelCounter A variable that counts the accepted pixels
   * @param currentSkipped A variable that counts the consecutive skipped pixels
   * @param luminanceRef The luminance of a white reference Pixel
   * @param saturationRef The saturation of a white reference Pixel
   * @return Is the consecutive skipped pixel (or green pixel) count to high?
   */
  bool checkPixel(unsigned char pixelLuminance, unsigned char pixelSaturation,
                  unsigned& goodPixelCounter, unsigned& currentSkipped, unsigned char luminanceRef, unsigned char saturationRef) const;

  /**
   * The method checks if the last spot is duplicative.
   *
   * @param ballSpots The representation with all previously found ball spots
   * @param minAllowedDistance The minimum allowed distance (in pixel) to an other spot
   * @return Is the spot duplicative?
   */
  bool isLastDuplicative(const BallSpots& ballSpots, const int minAllowedDistance) const;

  /**
   * The method checks if the spot is clearly inside a visually detected robot.
   * A spot is not clearly inside a robot if it is somewhere inside the feet
   *  or not above them
   *
   * @param spot The spot to check
   * @param estimatedRadius The calculate radius (in pixel) for a ball at this image position
   * @return Is the spot clearly inside a robot?
   */
  bool isSpotClearlyInsideARobot(const Vector2i& spot, const float estimatedRadius) const;

  /**
   * The method checks if the spot surrounded by green pixels
   *
   * @param spot The spot to check
   * @param radius The calculate radius (in pixel) for a ball at this image position
   * @return Is the spot surrounded by enough green pixel?
   */
  bool checkGreenAround(const Vector2i& spot, const float radius, unsigned char luminanceRef, unsigned char saturationRef) const;

  /**
   * The method calculates how a ball would look like if the ball visibility starts at
   * (x,y) and which part of it is probably recognizable.
   *
   * @param x The x-coordinate to calculate with
   * @param y The y-coordinate to calculate with
   * @param circle The circle which describes the possible ball
   * @return The amount of pixel that must be recognizable above this y-coordinate
   *           if it is a ball.
   */
  float getNeededLengthFor(const int x, const int y, Geometry::Circle& circle) const;
};
