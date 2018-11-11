/**
 * @file BallCorrector.h
 *
 * This file declares a module that determines the actual position of the
 * ball in the vicinity of a ball spot, given that it was already confirmed
 * that there is ball.
 *
 * @author Jesse Richter-Klug
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Perception/BallPercepts/ConfirmedBallSpot.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Tools/ImageProcessing/Sobel.h"
#include "Tools/Module/Module.h"

MODULE(BallCorrector,
{,
  REQUIRES(BallSpecification),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(ConfirmedBallSpot),
  REQUIRES(ECImage),
  REQUIRES(ImageCoordinateSystem),
  PROVIDES(BallPercept),
  DEFINES_PARAMETERS(
  {
    static constexpr int NUM_ANGLES = 36;
    static constexpr int MAXIMUM_RADIUS = 50,

    (bool)(true) useBallCorrection,
    (float)(8000.f) maxBallCorrectionDistance,
    (float)(3.5f) ballAreaFactor,
    (int)(sqr(30)) squaredEdgeThreshold,
    (float)(0.5f) centerBallCorrectionRadiusValue,
    (int)(40 / 2 / (360 / NUM_ANGLES)) halfSobelAngleAccuracy,
  }),
});

class BallCorrector : public BallCorrectorBase
{
  static Vector2i circleAngles[MAXIMUM_RADIUS][NUM_ANGLES];
  unsigned short parametersSpace[Image::maxResolutionHeight][Image::maxResolutionWidth];

  void createLookUpTable();
  void calcHough(const Sobel::SobelImage& sobelImage, const int radius, const int fromX, const int toX, const int fromY, const int toY);
  Vector2i findBallCenterInParameterSpace(const int maxXParameterSpace, const int maxYParameterSpace, const float estimateRadius) const;
  Vector2f correctBallCenterWithParamerSpace(const Vector2i& peak, const float estimateRadius) const;

  bool evaluateSobelPixel(const Sobel::SobelPixel* pixel) const
  {
    return pixel->x * pixel->x + pixel->y * pixel->y > squaredEdgeThreshold;
  }
  
  void resetHoughSpace(const int height, const int width);

  Vector2f correctCenter(const Vector2i& ballSpot, const CameraInfo& camerainfo, const CameraMatrix& cameramatrix, const BallSpecification& ballspecification, const TImage<PixelTypes::GrayscaledPixel>& src, const int patchSize = 32);

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theBallPercept The representation updated.
   */
  void update(BallPercept& theBallPercept);

public:
  BallCorrector();
};
