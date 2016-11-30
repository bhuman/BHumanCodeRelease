#pragma once

/**
 * Provides an easy calculation to detect PenaltyMarks
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/BallPrediction.h"
#include "Representations/Perception/FieldPercepts/PenaltyMarkPercept.h"
#include "Representations/Perception/ImagePreprocessing/BodyContour.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ColorScanlineRegions.h"
#include "Tools/Module/Module.h"
#include <vector>

MODULE(PenaltyMarkPerceptor3000,
{,
  REQUIRES(BodyContour),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraInfo),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(ECImage),
  REQUIRES(BallPrediction),
  REQUIRES(ColorScanlineRegionsVerticalClipped),
  PROVIDES(PenaltyMarkPercept),
  DEFINES_PARAMETERS(
  {,
    (float)(0.3f) allowedLengthDisplacement, //< the messured lenght is allowed to be displaced (compared to the calculated length) by a factor of this value, e.g. 0.3 means a maximum offset of 30% is allowed
    (int)(6) additionalRadiusForGreenCheck, //< the distance between the potential penalty mark and the green scan areas
    (float)(0.99f) greenPercent, //< the minimum ratio of detected green compare to all considered pixels in the green check
    (float)(1.3f) scanLengthRadiusFactor, //< describes the maximum length of an scanSpotOneDirection
    (unsigned)(5) maxNumberOfSkippablePixel, //< the maximum number of pixel that are alowed to skipp while scanSpotOneDirection
    (unsigned)(2) maxAllowedBlackPixel, //< the maximum number of pixel that are allowed to be black inside the a penalty mark
    (float)(2.f) ballRadiusMultiplier, //< a factor of the ball radius to describe a the distance that a penalty mark must be away from the ball (in image)
  }),
});

class PenaltyMarkPerceptor3000 : public PenaltyMarkPerceptor3000Base
{
public:
  PenaltyMarkPerceptor3000();
private:
  std::vector<Vector2i> centerSpots; //< a list of all spots (in image) to be evaluated if they are penalty marks.

  /**
  * The main method of this module.
  * @param penaltyMark The percept that is filled by this module.
  */
  void update(PenaltyMarkPercept& penaltyMark);

  /**
   * This method uses the ColorScanlineRegionsVerticalClipped to generates potential spots of a penalty mark.
   * The center of the generated spots are saved in centerSpots.
   */
  void searchScanLines();
  
  /**
  * This method iterates over all spots and checks for each of them if it is an penalty mark.
  * If a penalty mark is found the penaltyMark will be filled by the help of fillPenaltyMark(..)
  *
  * @param penaltyMark The percept that is filled by this module.
  */
  void checkAndSelect(PenaltyMarkPercept& penaltyMark);

  /**
  * This method fills the penalty mark percept correctly according to a given spot
  *
  * @param penaltyMark The percept that is filled by this module.
  * @param spot The spot that is the penalty mark
  */
  void fillPenaltyMark(PenaltyMarkPercept& penaltyMark, const Vector2i& spot) const;

  /**
  * The method checks if a given spot is duplicative.
  *
  * @param ptr The current spot (out of centerSpots) to check
  * @param estimatedRadius The estimated in image width radius of the current spot
  * @return Is the spot duplicative?
  */
  bool isDuplicative(std::vector<Vector2i>::reverse_iterator ptr, const float estimatedRadius) const;

  /**
  * The method corrects a spot in y-direction and checks if the messured in-image-width of the spot is
  *   approximately correct.
  *
  * @param initailPoint The spot to work with
  * @param radiusLength The calculated with/2 of an penalty mark at the in-image-position of spot
  * @return Is the spot in-image-width approximately correct?
  */
  bool correctWithScanSidewards(Vector2i& initialPoint, const int radiusLength) const;

  /**
  * The method scans in one direction and performes in each step checkPixel while it returns true.
  *
  * @param spot The start of the scan
  * @param currentLength A variable that counts the scanned pixels
  * @param maxLength The maximum scan length
  * @param goodPixelCounter A variable that counts the accepted pixels
  * @param getX(Vector2i,int) A pointer of a function that calculates the x-Value of the next element
  * @param getY(Vector2i,int) A pointer of a function that calculates the y-Value of the next element
  */
  void scanSpotOneDirection(const Vector2i& spot, int& currentLength, const int& maxLength,
                            unsigned& goodPixelCounter,
                            int(*getX)(const Vector2i& spot, const int currentLength),
                            int(*getY)(const Vector2i& spot, const int currentLength)) const;

  /**
  * The method checks a pixel.
  *
  * @param pixel The color pixel
  * @param goodPixelCounter A variable that counts the accepted pixels
  * @param currentSkipped A variable that counts the consecutive skipped pixels
  * @return Is the consecutive skipped pixel count to high?
  */
  bool checkPixel(const PixelTypes::ColoredPixel& pixel, unsigned& goodPixelCounter, unsigned& currentSkipped) const;

  /**
  * The method checks if the penalty mark spot is surrounded by green pixel
  *
  * @param spot The spot to check
  * @param radiusHeight The calculated in-image-height / 2 for this spot
  * @param lengthRadius The calculated in-image-length / 2 for this spot
  * @return Is the spot surrounded by anouth green pixel?
  */
  bool checkGreenAround(const Vector2i& spot, const int radiusHeight, const int lengthRadius) const;

  /**
  * The method checks if the black count to high inside the possible penalty mark 
  *
  * @param spot The spot to check
  * @param radiusHeight The calculated in-image-height / 2 for this spot
  * @param lengthRadius The calculated in-image-length / 2 for this spot
  * @return Is the black count to high inside the possible penalty mark?
  */
  bool checkBlackInside(const Vector2i& spot, const int radiusHeight, const int lengthRadius) const;

  /**
  * The method checks if the spot is inside the ball (image wise).
  *
  * @param spot The spot to check
  * @return Is the spot inside the ball?
  */
  bool isInBall(const Vector2i& spot) const;
};
