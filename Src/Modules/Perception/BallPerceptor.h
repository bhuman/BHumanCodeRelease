/**
 * @file BallPerceptor.h
 * This file declares a module that provides the ball percept.
 * If possible, it uses the full YCbCr422 image.
 * @author Colin Graf
 * @author marcel
 * @author Florian Maaß
 * @author Thomas Röfer
 */

#pragma once

#ifdef TARGET_ROBOT
#define IS_FULL_SIZE true
#else
#define IS_FULL_SIZE theImage.isFullSize
#endif

#include "Tools/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/ColorTable.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Perception/BallSpots.h"

MODULE(BallPerceptor,
{,
  REQUIRES(FieldDimensions),
  REQUIRES(Image),
  REQUIRES(CameraMatrix),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(CameraInfo),
  REQUIRES(ColorTable),
  REQUIRES(Odometer),
  REQUIRES(BallSpots),
  USES(RobotPose),
  PROVIDES(BallPercept),
  DEFINES_PARAMETERS(
  {,
    (float)(2.f) clippingApproxRadiusScale,
    (float)(2.5f) clippingApproxRadiusPixelBonus,
    (unsigned)(30) scanMaxColorDistance,
    (unsigned)(2) scanPixelTolerance,
    (unsigned)(2) refineMaxPixelCount,
    (unsigned)(40) refineMaxColorDistance,
    (float)(1.3f) checkMaxRadiusDifference,
    (float)(0.9f) checkMinRadiusDifference,
    (float)(2.f) checkMaxRadiusPixelBonus,
    (float)(6.f) checkMinRadiusPixelBonus,
    (float)(1.1f) checkOutlineRadiusScale,
    (float)(2.f) checkOutlineRadiusPixelBonus,
    (unsigned)(1) orangeSkipping,
    (unsigned)(4) minBallSpotSize,
    (float)(0.1f) percentRedNoise, // percent [0, 1]
  }),
});

/**
 * The class scales the input and output data if full size images
 * are avalable.
 */
class BallPerceptorScaler : public BallPerceptorBase
{
private:
  using BallPerceptorBase::theImage; // prevent direct access to image

protected:
  CameraInfo theCameraInfo;
  BallSpots theBallSpots;
  ImageCoordinateSystem theImageCoordinateSystem;

  /**
   * The only access to image pixels.
   * @param y The y coordinate of the pixel in the range defined in theCameraInfo.
   * @param y The y coordinate of the pixel in the range defined in theCameraInfo.
   * @param The pixel as a temporary object. Do not use yCbCrPadding of that pixel.
   */
  const Image::Pixel getPixel(int y, int x) const
  {
    if(theCameraInfo.camera == CameraInfo::upper && IS_FULL_SIZE)
      return theImage.getFullSizePixel(y, x);
    else
      return theImage[y][x];
  }

  /**
   * Update the copies of input representations that contain data
   * that might have to be scaled up.
   * Must be called in each cycle before any computations are performed.
   */
  void scaleInput();

  /**
   * Scale down the output if required.
   * @param ballPercept The ball percept the fields of which might be scaled.
   */
  void scaleOutput(BallPercept& ballPercept) const;

  /**
   * Scale down a single value if required.
   * This is only supposed to be used in debug drawings.
   * @param value The value that might be scaled.
   * @return The scaled value.
   */
  template<typename T> T scale(T value) const
  {
    if(theCameraInfo.camera == CameraInfo::upper && IS_FULL_SIZE)
      return value / (T) 2;
    else
      return value;
  }
};

/**
 * The actual ball perceptor.
 * It is basically identical to the module "BallPerceptor".
 */
class BallPerceptor : public BallPerceptorScaler
{
private:
  struct BallPoint
  {
    Vector2i step;
    Vector2i start;
    Vector2i point;
    Vector2f pointf;
    bool atBorder = false;
    bool isValid = false;
  };

  float sqrMaxBallDistance; /**< The square of the maximal allowed ball distance. */

  void update(BallPercept& ballPercept);

  void fromBallSpots(BallPercept&);

  /** ########## begin: analyzing chain for possible balls. ########## */

  // limits of the image where a ball should be found
  static const int left = 2;
  int right; //right, horizon and height are set in update()
  int horizon;
  int height;

  BallPercept::Status analyzeBallSpot(const BallSpot& ballspot, BallPercept& ballPercept);
  bool checkNoNoise(const BallSpot& ballSpot) const;
  bool checkBallSpot(const BallSpot& ballSpot);

  float approxRadius1; /**< Bearing based approximation of the radius. */

  bool searchBallPoints(const BallSpot& ballSpot);

  Image::Pixel startPixel; /**< The ball spot pixel. */
  BallPoint ballPoints[8]; /**< Points on the outer edge of the ball. */
  Vector2i approxCenter2;
  int totalPixelCount;
  int totalCbSum;
  int totalCrSum;

  bool searchBallPoint(const Vector2i& start, Image::Pixel startPixel,
                         const Vector2i& step, float maxLength, BallPoint& ballPoint);
  void drawBall(const Vector2i& pos, float approxDiameter, unsigned char opacity) const;
  bool checkBallPoints();
  bool getBallFromBallPoints(Vector2f& center, float& radius) const;

  unsigned int validBallPoints; /**< Count of usable points on the outer edge of the ball. */

  bool calculateBallInImage(BallPercept& ballPercept) const;
  bool checkBallInImage(BallPercept& ballPercept) const;
  bool calculateBallOnField(BallPercept& ballPercept) const;
  bool checkBallOnField(BallPercept& ballPercept) const;

  /** Returns false if there is too much red color inside the ball. */
  bool checkJersey() const;

  /** ########## end: analyzing chain for possible balls. ########## */

public:
  BallPerceptor();
};
