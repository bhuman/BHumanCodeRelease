/**
 * @file BallCorrector.cpp
 *
 * This file implements a module that determines the actual position of the
 * ball in the vicinity of a ball spot, given that it was already confirmed
 * that there is ball.
 *
 * @author Jesse Richter-Klug
 * @author Thomas Röfer
 */

#include "BallCorrector.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/ImageProcessing/InImageSizeCalculations.h"
#include "Tools/Math/Approx.h"
#include "Tools/Math/Transformation.h"
#include "Tools/NeuralNetwork/NNUtilities.h"

MAKE_MODULE(BallCorrector, perception)

Vector2i BallCorrector::circleAngles[MAXIMUM_RADIUS][NUM_ANGLES];

BallCorrector::BallCorrector()
{
  createLookUpTable();
}

void BallCorrector::update(BallPercept& theBallPercept)
{
  theBallPercept.status = BallPercept::notSeen;
  if(theConfirmedBallSpot.status != BallPercept::notSeen)
  {
    Vector2f ball(theConfirmedBallSpot.positionInImage.cast<float>());
    Vector2f correctedBall = theImageCoordinateSystem.toCorrected(ball);
    Vector2f ballOnField;

    if(Transformation::imageToRobotHorizontalPlane(correctedBall, theBallSpecification.radius, theCameraMatrix, theCameraInfo, ballOnField))
    {
      if(useBallCorrection && ballOnField.squaredNorm() <= sqr(maxBallCorrectionDistance))
      {
        ball = correctCenter(theConfirmedBallSpot.positionInImage, theCameraInfo, theCameraMatrix, theBallSpecification, theECImage.grayscaled);

        Vector2f correctedBall = theImageCoordinateSystem.toCorrected(ball);
        if(!Transformation::imageToRobotHorizontalPlane(correctedBall, theBallSpecification.radius, theCameraMatrix, theCameraInfo, ballOnField))
          return;
      }

      theBallPercept.positionInImage = ball;
      theBallPercept.positionOnField = ballOnField;
      theBallPercept.radiusInImage = IISC::getImageBallRadiusByCenter(theBallPercept.positionInImage, theCameraInfo, theCameraMatrix, theBallSpecification);
      theBallPercept.status = theConfirmedBallSpot.status;
    }
  }
}

void BallCorrector::createLookUpTable()
{
  const float angleDistance = pi2 / NUM_ANGLES;
  for(unsigned radius = 1; radius <= MAXIMUM_RADIUS; ++radius)
    for(int i = 0; i < NUM_ANGLES; ++i)
    {
      const Angle angle = i * angleDistance;
      circleAngles[radius - 1][i] = Vector2i(static_cast<int>(radius * std::cos(angle)), static_cast<int>(radius * std::sin(angle)));
    }
}

Vector2f BallCorrector::correctCenter(const Vector2i& ballSpot, const CameraInfo& camerainfo, const CameraMatrix& cameramatrix, const BallSpecification& ballspecification, const TImage<PixelTypes::GrayscaledPixel>& grayscaled, int patchSize)
{
  Sobel::Image1D grayImage(patchSize, patchSize, sizeof(Sobel::Image1D::PixelType));

  Vector2f imageSectionStartSpot = ballSpot.cast<float>();
  float radius = IISC::getImageBallRadiusByCenter(imageSectionStartSpot, theCameraInfo, theCameraMatrix, theBallSpecification);
  radius *= ballAreaFactor;
  
  imageSectionStartSpot.array() -= radius / 2.f;

  float yy = imageSectionStartSpot.y();
  float xx;
  for(int y = 0; y < patchSize; y++, yy += radius / patchSize)
  {
    xx = imageSectionStartSpot.x();
    const PixelTypes::GrayscaledPixel* pix2 = grayscaled[static_cast<unsigned>(yy)];
    for(int x = 0; x < patchSize; x++, xx += radius / patchSize)
    {
      if(xx < camerainfo.width && yy < camerainfo.height && xx >= 0 && yy >= 0)
        grayImage[y][x] = *(pix2 + static_cast<unsigned>(xx));
    }
  }

  DEBUG_DRAWING("module:BallCorrector:correctionArea", "drawingOnImage")
  {
    RECTANGLE("module:BallCorrector:correctionArea", imageSectionStartSpot.x(), imageSectionStartSpot.y(), static_cast<int>(xx), static_cast<int>(yy), 2, Drawings::PenStyle::solidPen, ColorRGBA::black);
  }

  float ratio = radius / patchSize;
  float estimatedRadius = patchSize / ballAreaFactor;

  Sobel::SobelImage sobelImage(grayImage.width, grayImage.height);
  STOPWATCH("module:BallCorrector:sobel")
  Sobel::sobelSSE(grayImage, sobelImage);

  resetHoughSpace(static_cast<int>(estimatedRadius) * 2 + sobelImage.height, static_cast<int>(estimatedRadius) * 2 + sobelImage.width);

  STOPWATCH("module:BallCorrector:hough")
  calcHough(sobelImage, static_cast<int>(estimatedRadius), 0, sobelImage.width, 0, sobelImage.height);
  Vector2i foundCenterSpot = findBallCenterInParameterSpace(static_cast<int>(estimatedRadius) * 2 + sobelImage.width, static_cast<int>(estimatedRadius) * 2 + sobelImage.height, estimatedRadius);

  DEBUG_DRAWING("module:BallCorrector:ballCenterPoint", "drawingOnImage")
  {
    DOT("module:BallCorrector:ballCenterPoint", ratio * (foundCenterSpot.x() - estimatedRadius) + imageSectionStartSpot.x(), ratio * (foundCenterSpot.y() - estimatedRadius) + imageSectionStartSpot.y(), ColorRGBA::red, ColorRGBA::red);
  }

  Vector2f correctedCenterSpot;
  STOPWATCH("module:BallCorrector:evaluateCenters")
  correctedCenterSpot = correctBallCenterWithParamerSpace(foundCenterSpot, estimatedRadius);

  DEBUG_DRAWING("module:BallCorrector:correctedBallCenterPoint", "drawingOnImage")
  {
    DOT("module:BallCorrector:correctedBallCenterPoint", ratio * (correctedCenterSpot.x() - estimatedRadius) + imageSectionStartSpot.x(), ratio * (correctedCenterSpot.y() - estimatedRadius) + imageSectionStartSpot.y(), ColorRGBA::red, ColorRGBA::green);
  }

  return (ratio * (correctedCenterSpot.array() - estimatedRadius) + Vector2f(imageSectionStartSpot.x(), imageSectionStartSpot.y()).array() + 1).matrix();
}

void BallCorrector::calcHough(const Sobel::SobelImage& sobelImage, const int estimateRadius, const int fromX, const int toX, const int fromY, const int toY)
{
  const Vector2i* circleAnglesByThisRadius = circleAngles[estimateRadius - 1];
  const float angeDist = 360.f / NUM_ANGLES * pi / 180.f;
  const int halfCircleIndex = NUM_ANGLES / 2;
  const int quarterCircleIndex = NUM_ANGLES / 4;
  for(int y = fromY; y < toY; y++)
  {
    for(int x = fromX; x < toX; x++)
    {
      if(evaluateSobelPixel(&sobelImage[y][x]))
      {
        const int angleIndex = static_cast<int>(std::fmod(Approx::atan2(static_cast<float>(sobelImage[y][x].y), static_cast<float>(sobelImage[y][x].x)), pi) / angeDist);
        ASSERT(angleIndex < halfCircleIndex);
        const int mirrorAngle = angleIndex + halfCircleIndex;
        const int viceAngle = angleIndex < quarterCircleIndex ? (angleIndex - halfCircleIndex) : (mirrorAngle + halfCircleIndex);
        for(int i = 0; i < NUM_ANGLES; i++)
        {
          if(!(std::abs(i - mirrorAngle) <= halfSobelAngleAccuracy
               || std::abs(i - angleIndex) <= halfSobelAngleAccuracy
               || std::abs(i - viceAngle) <= halfSobelAngleAccuracy))
            continue;
          int vecX = x + circleAnglesByThisRadius[i].x();
          int vecY = y + circleAnglesByThisRadius[i].y();
          ++parametersSpace[vecY + estimateRadius][vecX + estimateRadius];
        }
      }
    }
  }
}

Vector2i BallCorrector::findBallCenterInParameterSpace(const int maxXParameterSpace, const int maxYParameterSpace, const float estimateRadius) const
{
  int max = 0;
  int maxX = 0;
  int maxY = 0;
  for(int y = 0; y < maxYParameterSpace; y++)
  {
    for(int x = 0; x < maxXParameterSpace; x++)
    {
      if(parametersSpace[y][x] > max)
      {
        max = parametersSpace[y][x];
        maxX = x;
        maxY = y;
      }
    }
  }

  return Vector2i(maxX, maxY);
}

Vector2f BallCorrector::correctBallCenterWithParamerSpace(const Vector2i& peak, const float estimateRadius) const
{
  const int halfRadius = static_cast<int>(estimateRadius * centerBallCorrectionRadiusValue);
  const int toX = peak.x() + halfRadius;
  const int toY = peak.y() + halfRadius;

  long valuedSumX = 0;
  long valuedSumY = 0;
  long numOfPoints = 0;

  for(long y = std::max(0, peak.y() - halfRadius); y < toY; y++)
    for(long x = std::max(0, peak.x() - halfRadius); x < toX; x++)
    {
      const long value = sqr(static_cast<long>(parametersSpace[y][x]));
      valuedSumX += x * value;
      valuedSumY += y * value;
      numOfPoints += value;
    }

  const float newX = static_cast<float>(valuedSumX) / static_cast<float>(numOfPoints);
  const float newY = static_cast<float>(valuedSumY) / static_cast<float>(numOfPoints);

  return Vector2f(newX, newY);
}

void BallCorrector::resetHoughSpace(const int height, const int width)
{
  for(int y = 0; y < height; ++y)
    memset(parametersSpace[y], 0, sizeof(*parametersSpace[y]) * width);
}
