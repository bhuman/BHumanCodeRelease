/**
* @file BallPerceptor.h
* This file declares a module that provides the ball percept.
* @author Colin Graf & marcel
* @author Florian Maaß
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/ColorReference.h"
#include "Representations/Perception/FieldBoundary.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Perception/BallSpots.h"

class BallSpot;
class RobotPose;
class Image;
class BallModel;

MODULE(BallPerceptor)
  REQUIRES(FieldDimensions)
  REQUIRES(Image)
  REQUIRES(CameraMatrix)
  REQUIRES(ImageCoordinateSystem)
  REQUIRES(CameraInfo)
  REQUIRES(ColorReference)
  REQUIRES(Odometer)
  REQUIRES(FieldBoundary)
  REQUIRES(BallSpots)
  USES(RobotPose)
  USES(BallModel)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(BallPercept)
  DEFINES_PARAMETER(float, clippingApproxRadiusScale, 2)
  DEFINES_PARAMETER(float, clippingApproxRadiusPixelBonus, 2.5f)
  DEFINES_PARAMETER(unsigned, scanMaxColorDistance, 40)
  DEFINES_PARAMETER(unsigned, scanPixelTolerance, 2)
  DEFINES_PARAMETER(unsigned, refineMaxPixelCount, 2)
  DEFINES_PARAMETER(unsigned, refineMaxColorDistance, 40)
  DEFINES_PARAMETER(float, checkMaxRadiusDifference, 1.3f)
  DEFINES_PARAMETER(float, checkMinRadiusDifference, 0.9f)
  DEFINES_PARAMETER(float, checkMinRadiusPixelBonus, 6)
  DEFINES_PARAMETER(float, checkOutlineRadiusScale, 1.1f)
  DEFINES_PARAMETER(float, checkOutlineRadiusPixelBonus, 2)
  DEFINES_PARAMETER(unsigned int, orangeSkipping, 1)
  DEFINES_PARAMETER(unsigned int, minBallSpotSize, 4)
  DEFINES_PARAMETER(float, percentRedNoise, 0.1f) // percent [0, 1]
  DEFINES_PARAMETER(bool, useFieldBoundary, true)
END_MODULE

class BallPerceptor : public BallPerceptorBase
{
private:
  class BallPoint
  {
  public:
    Vector2<int> step;
    Vector2<int> start;
    Vector2<int> point;
    Vector2<> pointf;
    bool atBorder;
    bool isValid;

    BallPoint() : atBorder(false), isValid(false) {}
  };

  float sqrMaxBallDistance; /**< The square of the maximal allowed ball distance. */

  void update(BallPercept& ballPercept);

  bool fromBallSpots(BallPercept&);

  /** ########## begin: analyzing chain for possible balls. ########## */
  /*limits of the image where a ball should be found*/
  static const int left = 2;
  int right; //right, horizon and height are set in update()
  int horizon;
  int height;

  bool analyzeBallSpot(const BallSpot& ballspot, BallPercept& ballPercept);

  bool checkNoNoise(const BallSpot& ballSpot);
  
  bool checkBallSpot(const BallSpot& ballSpot);
  float approxRadius1; /**< Bearing based approximation of the radius. */

  bool searchBallPoints(const BallSpot& ballSpot);
  Image::Pixel startPixel; /**< The ball spot pixel. */
  BallPoint ballPoints[8]; /**< Points on the outer edge of the ball. */
  Vector2<int> approxCenter2;
  int totalPixelCount;
  int totalCbSum;
  int totalCrSum;

  bool searchBallPoint(const Vector2<int>& start, Image::Pixel startPixel,
                         const Vector2<int>& step, float maxLength, BallPoint& ballPoint);
  void drawBall(const Vector2<int>& pos, float approxDiameter, unsigned char opacity) const;

  bool checkBallPoints();
  bool getBallFromBallPoints(Vector2<>& center, float& radius) const;
  unsigned int validBallPoints; /**< Count of usable points on the outer edge of the ball. */

  bool calculateBallInImage(BallPercept& ballPercept);

  bool checkBallInImage(BallPercept& ballPercept);

  bool calculateBallOnField(BallPercept& ballPercept);

  bool checkBallOnField(BallPercept& ballPercept);
  /**returns false if there is too much red color inside the ball*/
  bool checkJersey() const;

  /** ########## end: analyzing chain for possible balls. ########## */

public:
  BallPerceptor();
};
