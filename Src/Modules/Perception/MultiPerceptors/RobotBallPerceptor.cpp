/**
 * @author Krishnen Ganeshakumar
 * @author Steffen Gebhard
 */

#include "RobotBallPerceptor.h"
#include "Debugging/DebugDrawings.h"
#include "Framework/Settings.h"
#include "ImageProcessing/ColorModelConversions.h"
#include "Platform/File.h"
#include "Tools/Math/Projection.h"
#include <algorithm>
#include <cstdint>
#include <limits>
#include <vector>
#include <memory>
#include <string>
#include <array>
#include <cmath>

MAKE_MODULE(RobotBallPerceptor);

Image<PixelTypes::GrayscaledPixel> greyScaledBoundingBox(1000, 1000);

RobotBallPerceptor::RobotBallPerceptor()
{
  // Allocate 'Network'
  model = std::make_unique<NeuralNetworkONNX::Model>(std::string(File::getBHDir())
                                                     + "/Config/NeuralNets/RobotBallPerceptor/ball_robot_"
                                                     + std::to_string(theCameraInfo.width)
                                                     + ".onnx");
  NeuralNetworkONNX::CompilationSettings settings;
  settings.useTensorRT = true;
  settings.fp16 = true;
  settings.useCoreML = true;
  compiled.compile(*model, settings);

  // Set the input size (it's squared)
  networkInputSize = compiled.input(0).dims(1);
}

void RobotBallPerceptor::update(BallPercept& theBallPercept)
{
  SEND_DEBUG_IMAGE("module:RobotBallPerceptor:subimage", greyScaledBoundingBox);
  DECLARE_DEBUG_DRAWING("module:RobotBallPerceptor:ballEstimations", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RobotBallPerceptor:field", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RobotBallPerceptor:box", "drawingOnImage");
  if(theCameraMatrix.isValid)
  {
    processFrameOnce();
    theBallPercept = cachedBallPercept;
  }
  else
  {
    theBallPercept.status = BallPercept::notSeen;
  }
}

void RobotBallPerceptor::update(RobotPercept& theRobotPercept)
{
  SEND_DEBUG_IMAGE("module:RobotBallPerceptor:subimage", greyScaledBoundingBox);
  DECLARE_DEBUG_DRAWING("module:RobotBallPerceptor:ballEstimations", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RobotBallPerceptor:field", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RobotBallPerceptor:box", "drawingOnImage");
  if(theCameraMatrix.isValid)
  {
    processFrameOnce();
    theRobotPercept = cachedRobotPercept;
  }
  else
  {
    theRobotPercept.robots.clear();
  }
}

void RobotBallPerceptor::preProcess()
{
  auto input = compiled.input(0);

  float* dest = input.data();
  const uint64_t nwHeight = input.dims(1);
  const uint64_t nwWidth  = input.dims(2);
  const uint64_t channels = input.dims(0);
  const uint32_t cameraWidth = theCameraImage.width;
  const uint32_t cameraHeight = theCameraImage.height;

  ASSERT(nwHeight == static_cast<uint64_t>(theCameraInfo.width));
  ASSERT(nwWidth == static_cast<uint64_t>(theCameraInfo.width));

  // Set all pixels to 114
  // TODO: We could save some computation by just greying out the lower part of the image
  for(uint32_t i = 0; i < nwHeight * nwWidth * channels; ++i)
    dest[i] = 114.f;

  // For Conversion from YUYV to RGB
  uint8_t R0;
  uint8_t G0;
  uint8_t B0;
  uint8_t R1;
  uint8_t G1;
  uint8_t B1;
  for(uint32_t y = 0; y < cameraHeight; y++)
  {
    const PixelTypes::YUYVPixel* src = theCameraImage[y];
    for(uint32_t x = 0; x < cameraWidth * 2; x += 2)
    {
      ColorModelConversions::fromYUYVToRGB(src->y0, src->u, src->y1, src->v, R0, G0, B0, R1, G1, B1);

      dest[0 * nwHeight * nwWidth + y * nwWidth + x]     = static_cast<float>(B0);
      dest[1 * nwHeight * nwWidth + y * nwWidth + x]     = static_cast<float>(G0);
      dest[2 * nwHeight * nwWidth + y * nwWidth + x]     = static_cast<float>(R0);

      dest[0 * nwHeight * nwWidth + y * nwWidth + x + 1] = static_cast<float>(B1);
      dest[1 * nwHeight * nwWidth + y * nwWidth + x + 1] = static_cast<float>(G1);
      dest[2 * nwHeight * nwWidth + y * nwWidth + x + 1] = static_cast<float>(R1);

      src++;
    }
  }
}

void RobotBallPerceptor::extractPredictions(const float confidenceThresh, const float IOUthresh)
{
  // Network data
  auto output = compiled.output(0);
  float* outputData = output.data();

  auto input = compiled.input(0);
  const uint64_t nwImgHeight = input.dims(1);
  const uint64_t nwImgWidth  = input.dims(2);

  // Relevant network dimensions
  const uint32_t boxCount = static_cast<uint32_t>(output.dims(0));
  const uint8_t boxDataLen = static_cast<uint8_t>(output.dims(1));
  const uint8_t classCount = boxDataLen - 5;

  // How many possible sizes are there
  constexpr uint32_t sizeCount = 3;

  // All the sizes
  const std::array<uint32_t, sizeCount> sizes = {8, 16, 32};

  // Determine number of pairs
  uint32_t pairCount = boxCount;

  // All pairs
  std::vector<std::pair<uint32_t, uint32_t>> allPairs;
  allPairs.reserve(pairCount);

  // All sizes
  std::vector<uint32_t> sizeMap;
  sizeMap.reserve(pairCount);

  // Valid predictions that lie in the image go here
  std::vector<Prediction> validPredicts;
  validPredicts.reserve(pairCount);

  // Fill the pairs and sizeMap
  for(uint32_t i = 0; i < sizeCount; ++i)
  {
    const uint64_t hSize = nwImgHeight / sizes[i];
    const uint64_t wSize = nwImgWidth / sizes[i];
    for(uint32_t j = 0; j < hSize; ++j)
    {
      for(uint32_t k = 0; k < wSize; ++k)
      {
        allPairs.push_back(std::make_pair(k, j));
        sizeMap.push_back(sizes[i]);
      }
    }
  }

  // transform the output
  for(uint32_t i = 0; i < pairCount; ++i)
  {
    float* outputRow = outputData + (i * 7);
    outputRow[0] = ((outputRow[0] + allPairs[i].first) * sizeMap[i]); // X
    outputRow[1] = ((outputRow[1] + allPairs[i].second) * sizeMap[i]); // Y
    outputRow[2] = (std::exp(outputRow[2]) * sizeMap[i]); // height
    outputRow[3] = (std::exp(outputRow[3]) * sizeMap[i]); // width

    // The class probabilities start at index 5
    const uint8_t classOffset = 5;

    float prob = std::numeric_limits<float>::min();
    ObjectType object;
    Prediction currentPredicion(Vector2f(outputRow[0], outputRow[1]),
                                outputRow[3], outputRow[2],
                                ObjectType::ball, 0);

    for(uint8_t j = 0; j < classCount; ++j)
    {
      const float thisProb = outputRow[4] * outputRow[classOffset + j];
      if(thisProb > confidenceThresh && thisProb > prob)
      {
        prob = thisProb;
        object = static_cast<ObjectType>(j);
      }
    }
    if(prob > std::numeric_limits<float>::min())
    {
      currentPredicion.probability = prob;
      currentPredicion.type = object;
      validPredicts.push_back(currentPredicion);
    }
  }

  // Sort for nms
  std::sort(validPredicts.begin(), validPredicts.end(),
            [](const Prediction& lhs, const Prediction& rhs)
  {
    return lhs.probability > rhs.probability;
  });

  std::vector<bool> erased(validPredicts.size(), false);
  for(uint32_t i = 0; i < validPredicts.size(); ++i)
  {
    if(erased[i])
      continue;
    for(uint32_t j = i + 1; j < validPredicts.size(); ++j)
    {
      if(validPredicts[i].type == validPredicts[j].type
         && validPredicts[i].IOU(validPredicts[j]) > IOUthresh)
      {
        erased[j] = true;
      }
    }
  }

  for(uint32_t i = 0; i < validPredicts.size(); ++i)
  {
    if(!erased[i])
    {
      usefulPredicts.push_back(validPredicts[i]);
    }
  }
}

void RobotBallPerceptor::filterPredictions()
{
  Vector2f relativePoint;
  Geometry::Circle ball;
  for(auto prediction = usefulPredicts.begin(); prediction != usefulPredicts.end();)
  {
    if(prediction->type == ObjectType::robot)
    {
      // Clamp bounding box to image borders
      int left   = static_cast<int>(prediction->topLeft.x());
      int right  = static_cast<int>(prediction->bottomRight.x());
      int top    = static_cast<int>(prediction->topLeft.y());
      int bottom = static_cast<int>(prediction->bottomRight.y());

      if(right <= left || bottom <= top)
      {
        prediction = usefulPredicts.erase(prediction);
        continue;
      }

      // If robot is not big enough in comparison to a ball at the same position, skip it
      if(!Transformation::imageToRobotHorizontalPlane(Vector2f((left + right) * 0.5, bottom), 0.f, theCameraMatrix, theCameraInfo, relativePoint)
         || !Projection::calculateBallInImage(relativePoint, theCameraMatrix, theCameraInfo, theBallSpecification.radius, ball)
         || (right - left) * (bottom - top) < sqr(ball.radius) * 4.f * minObstacleToBallRatio)
      {
        prediction = usefulPredicts.erase(prediction);
        continue;
      }

      // If an robot is too much inside the body contour, it is probably the body itself. Skip it.
      int pixelsAbove = 0;
      for(int i = 0; i < numOfBodyContourSamples; ++i)
      {
        const int x = left + (right - left) * i / (numOfBodyContourSamples - 1);
        int y = bottom;
        theBodyContour.clipBottom(x, y, theCameraInfo.height);
        if(y > top)
          pixelsAbove += y - top;
      }
      if(pixelsAbove < numOfBodyContourSamples * (bottom - top) * minRatioAboveBodyContour)
      {
        prediction = usefulPredicts.erase(prediction);
        continue;
      }
    }
    // keep prediction
    ++prediction;
  }
}

void RobotBallPerceptor::processFrameOnce()
{
  if(lastProcessedTimestamp == theCameraImage.timestamp)
  {
    return;
  }

  lastProcessedTimestamp = theCameraImage.timestamp;

  usefulPredicts.clear();

  STOPWATCH("module:RobotBallPerceptor:preProcess") preProcess();
  STOPWATCH("module:RobotBallPerceptor:apply") compiled.apply();
  STOPWATCH("module:RobotBallPerceptor:extractPredictions") extractPredictions(boxConfidenceThreshold, iouThreshold);

  COMPLEX_DRAWING("module:RobotBallPerceptor:box")
  {
    for(const Prediction& pred : usefulPredicts)
    {
      DRAW_TEXT("module:RobotBallPerceptor:box", pred.topLeft.x(), pred.topLeft.y(),
                7, ColorRGBA(100, 255, 100, 255),
                TypeRegistry::getEnumName(pred.type) << ": " << pred.probability);

      RECTANGLE("module:RobotBallPerceptor:box",
                pred.topLeft.x(), pred.topLeft.y(),
                pred.bottomRight.x(), pred.bottomRight.y(), 1,
                Drawings::PenStyle::solidPen, ColorRGBA(100, 255, 100, 255));
    }
  }

  STOPWATCH("module:RobotBallPerceptor:filterPredictions") filterPredictions();
  STOPWATCH("module:RobotBallPerceptor:writeBallPercept") writeBallPercept(cachedBallPercept);
  STOPWATCH("module:RobotBallPerceptor:writeRobotPercept") writeRobotPercept(cachedRobotPercept);
}

void RobotBallPerceptor::writeBallPercept(BallPercept& theBallPercept)
{
  // Set default values
  theBallPercept.status = BallPercept::notSeen;

  theBallPercept.positionInImage = Vector2f::Zero();
  theBallPercept.radiusInImage   = 0.f;

  theBallPercept.radiusOnField   = theBallSpecification.radius;
  theBallPercept.positionOnField = Vector2f::Zero();
  theBallPercept.covarianceOnField = Matrix2f::Zero();

  // Find best ball prediction
  const Prediction* bestBall = nullptr;

  for(const Prediction& prediction : usefulPredicts)
  {
    if(prediction.type == ObjectType::ball
       && std::isfinite(prediction.center.x()) && std::isfinite(prediction.bottomRight.y())
       && (!theFieldBoundary.isValid || theFieldBoundary.getBoundaryY(static_cast<int>(prediction.center.x())) <= prediction.bottomRight.y()))
    {
      bestBall = &prediction;
      break;
    }
  }

  if(bestBall == nullptr)
    return;

  // Camera image bounds
  const float cameraWidth = static_cast<float>(theCameraInfo.width);
  const float cameraHeight = static_cast<float>(theCameraInfo.height);

  // Bounding box center
  Vector2f center = bestBall->center;

  // Distance to optical center
  float ballDistToOpticalCenter = (theCameraInfo.opticalCenter - center).norm();
  static_cast<void>(ballDistToOpticalCenter);

  if(!std::isfinite(center.x()) || !std::isfinite(center.y()))
    return;

  if(center.x() < 0.f || center.y() < 0.f || center.x() >= cameraWidth || center.y() >= cameraHeight)
    return;

#define ESTIMATED_BALL 0
#if ESTIMATED_BALL == 1
  if(ballDistToOpticalCenter > distanceForBallCenterEstimation || true)
  {
    // Line from bounding box to optical center
    Geometry::Line ballToCenter(bestBall->center, theCameraInfo.opticalCenter - bestBall->center);

    // Used to determine the shape of the ball in the image
    struct Ellipse
    {
      Vector2f center;
      float minLen;
      float maxLen;
      Angle angle;
    };

    // Constructing ellipse
    const Angle ellipseRotation(ballToCenter.direction.angle());
    static Ellipse ballEllipse;
    const float doubleRotEllipse = std::cos(ellipseRotation * 2);
    if(doubleRotEllipse <= 0.1 && doubleRotEllipse >= -0.1)
    {
      ballEllipse.angle = ellipseRotation;
      ballEllipse.center = bestBall->center;
    }
    else
    {
      const float cosineRot = static_cast<float>(std::cos(ellipseRotation));
      const float sineRot = static_cast<float>(std::sin(ellipseRotation));
      const float cosSq = sqr(cosineRot);
      const float sinSq = sqr(sineRot);
      const float ellipseMaxY = static_cast<float>(bestBall->height / 2);
      const float ellipseMaxX = static_cast<float>(bestBall->width / 2);
      const float ellipseMaxYSq = sqr(ellipseMaxX);
      const float ellipseMaxXSq = sqr(ellipseMaxY);
      const float aSqEllipse =
        static_cast<float>((ellipseMaxXSq / sinSq - ellipseMaxYSq / cosSq) / ((cosSq / sinSq) - (sinSq / cosSq)));
      const float bSqEllipse = static_cast<float>(-sinSq * aSqEllipse / cosSq + (ellipseMaxYSq / cosSq));
      const float aEllipse = std::sqrt(aSqEllipse);
      const float bEllipse = std::sqrt(bSqEllipse);
      const float majAxis = aEllipse > bEllipse ? aEllipse : bEllipse;
      const float minAxis = aEllipse < bEllipse ? aEllipse : bEllipse;
      ballEllipse = {.center = bestBall->center, .minLen = minAxis, .maxLen = majAxis, .angle = ellipseRotation};
    }

    // Unit vector from ellipse center to optical center
    const Vector2f ballToCenterUnit = ballToCenter.direction.normalized();
    // The extrema of the ellipse
    Vector2f closerExtrema = ballEllipse.center + (ballToCenterUnit * ballEllipse.maxLen);
    Vector2f furtherExtrema = ballEllipse.center - (ballToCenterUnit * ballEllipse.maxLen);

    // Vectors from the extrema to the optical center in 2d
    const Vector2f closerExtremaToOpticalCenter2d = theCameraInfo.opticalCenter - closerExtrema;
    const Vector2f furtherExtremaToOpticalCenter2d = theCameraInfo.opticalCenter - furtherExtrema;

    // Vector from the image sensor to the optical center
    const Vector3f opticalCenterAxis(0, 0, theCameraInfo.focalLength);

    // Vectors from the extrema to the optical center in 3d
    const Vector3f closerExtremaToOpticalCenter3d(closerExtremaToOpticalCenter2d.x(), closerExtremaToOpticalCenter2d.y(), theCameraInfo.focalLength);
    const Vector3f furtherExtremaToOpticalCenter3d(
      furtherExtremaToOpticalCenter2d.x(),
      furtherExtremaToOpticalCenter2d.y(), theCameraInfo.focalLength);

    if(!improveBallExtrema(closerExtrema, furtherExtrema, ballToCenter, *bestBall))
    {
      // TODO: do something
    }

    // Opening angle of the cone
    const Angle coneAngle = closerExtremaToOpticalCenter3d.angleTo(furtherExtremaToOpticalCenter3d);
    const Angle coneAngleHalfed = coneAngle / 2.f;

    // Angle from the vector of the closer extremum to the axis of the optical center
    const Angle closerExtremaOpticalCenterAngle = closerExtremaToOpticalCenter3d.angleTo(opticalCenterAxis);

    // Distance of the Ball center from the optical center
    const float distToOpticalCenter = std::tan(closerExtremaOpticalCenterAngle + coneAngleHalfed) * theCameraInfo.focalLength;

    // Estimated ball center
    const Vector2f estimatedBallCenter = theCameraInfo.opticalCenter - (ballToCenterUnit * distToOpticalCenter);

    // Estimated distance from ball to pinhole of the camera (which totally exists)
    const float ballDistToPinhole = theBallSpecification.radius / std::tan(coneAngleHalfed);

    // Vector from pinhole of the camera to ball center
    const Vector3f pinholeToBallCenter = Vector3f(estimatedBallCenter.x() - theCameraInfo.opticalCenter.x(),
                                                  estimatedBallCenter.y() - theCameraInfo.opticalCenter.y(),
                                                  theCameraInfo.focalLength).normalized() * ballDistToPinhole;

    DRAW_TEXT("module:RobotBallPerceptor:ballEstimations", bestBall->topLeft.x(),
              bestBall->bottomRight.y() + 10, 10, ColorRGBA(100, 255, 100, 255),
              "Ball distance: " << ballDistToPinhole << "mm");

    CROSS("module:RobotBallPerceptor:ballEstimations", closerExtrema.x(), closerExtrema.y(), 3, 1, Drawings::PenStyle::solidPen,
          ColorRGBA(100, 255, 0, 150));
    CROSS("module:RobotBallPerceptor:ballEstimations", furtherExtrema.x(), furtherExtrema.y(), 3, 1, Drawings::PenStyle::solidPen,
          ColorRGBA(100, 255, 0, 150));

    CROSS("module:RobotBallPerceptor:ballEstimations", center.x(), center.y(), 3, 1, Drawings::PenStyle::solidPen,
          ColorRGBA::violet);
    CROSS("module:RobotBallPerceptor:ballEstimations", estimatedBallCenter.x(), estimatedBallCenter.y(), 3, 1, Drawings::PenStyle::solidPen,
          ColorRGBA::orange);
    center = estimatedBallCenter;

    /*
    DRAW_TEXT("module:RobotBallPerceptor:ballEstimations", bestBall->topLeft.x(),
              bestBall->bottomRight.y(), 10, ColorRGBA(100, 255, 100, 255),
              "coneAngle: " << coneAngle.toDegrees());

    DRAW_TEXT("module:RobotBallPerceptor:ballEstimations", bestBall->topLeft.x(),
              bestBall->bottomRight.y() + 20, 10, ColorRGBA(100, 255, 100, 255),
              "coneAngleHalfed: " << coneAngleHalfed.toDegrees());
               */
  }
#endif

  // Estimate radius from bounding box
  const float width = bestBall->width;
  const float height = bestBall->height;

  if(!(width > 0.f) || !(height > 0.f) || !std::isfinite(width) || !std::isfinite(height))
    return;

  float radiusPx = 0.5f * std::max(width, height);

  Vector2f relativePoint;
  Geometry::Circle ball;
  for(uint8_t i = 0; i < 3; ++i)
  {
    if(!(Transformation::imageToRobotHorizontalPlane(center, theBallSpecification.radius, theCameraMatrix, theCameraInfo, relativePoint)
         && Projection::calculateBallInImage(relativePoint, theCameraMatrix, theCameraInfo, theBallSpecification.radius, ball)))
    {
      return;
    }

    if(bestBall->bottomRight.x() >= theCameraInfo.width - 3)
    {
      center.x() = bestBall->topLeft.x() + ball.radius;
    }
    if(bestBall->bottomRight.y() >= theCameraInfo.height - 3)
    {
      center.y() = bestBall->topLeft.y() + ball.radius;
    }
    if(bestBall->topLeft.x() <= 3)
    {
      center.x() = bestBall->bottomRight.x() - ball.radius;
    }
    if(bestBall->topLeft.y() <= 3)
    {
      center.y() = bestBall->bottomRight.y() - ball.radius;
    }
  }

  const float allowedOffs = ball.radius * ballPixelRadiusFactor + ballPixelRadiusBaseOffset;
  if(radiusPx > ball.radius + allowedOffs || radiusPx < ball.radius - allowedOffs || !(radiusPx > 0.f) || !std::isfinite(radiusPx))
  {
    RECTANGLE("module:RobotBallPerceptor:box",
              bestBall->topLeft.x(), bestBall->topLeft.y(),
              bestBall->bottomRight.x(), bestBall->bottomRight.y(), 1,
              Drawings::PenStyle::solidPen, ColorRGBA(255, 100, 100, 255));
    return;
  }

  // Fill image percept
  theBallPercept.status = BallPercept::seen;
  theBallPercept.positionInImage = center;
  theBallPercept.radiusInImage   = radiusPx;

  Vector2f posOnField;
  Matrix2f covOnField;
  if(theMeasurementCovariance.transformPointWithCov(center,
                                                    theBallSpecification.radius,
                                                    posOnField,
                                                    covOnField))
  {
    theBallPercept.positionOnField   = posOnField;
    theBallPercept.covarianceOnField = covOnField;
  }
  else
  {
    // Projection failed
    theBallPercept.status = BallPercept::notSeen;
    theBallPercept.positionOnField = Vector2f::Zero();
    theBallPercept.covarianceOnField = Matrix2f::Zero();
  }
  DRAW_TEXT("module:RobotBallPerceptor:ballEstimations", bestBall->topLeft.x(),
            bestBall->bottomRight.y() + 20, 10, ColorRGBA(100, 255, 100, 255),
            "Other ball distance: " << sqrt(sqr(posOnField.norm()) + sqr(theCameraMatrix.translation.z() - theBallSpecification.radius)) << "mm");
}

void RobotBallPerceptor::writeRobotPercept(RobotPercept& theRobotPercept)
{
  // Clear previous robots
  theRobotPercept.robots.clear();

  // Camera image bounds
  const float cameraHeight = static_cast<float>(theCameraInfo.height);

  for(const Prediction& p : usefulPredicts)
  {
    if(p.type != ObjectType::robot)
      continue;

    // Clamp bounding box bottom to image borders
    const float leftX   = p.topLeft.x();
    const float rightX  = p.bottomRight.x();
    const float bottomY = p.bottomRight.y();
    const float topY    = p.topLeft.y();

    RECTANGLE("module:RobotBallPerceptor:field",
              p.topLeft.x(), p.topLeft.y(),
              p.bottomRight.x(), p.bottomRight.y(), 1,
              Drawings::PenStyle::solidPen, ColorRGBA(255, 100, 100, 255));

    if(!(bottomY < cameraHeight - 1.f))
      continue;

    // Points of robot in image coordinates
    const Vector2f bottomCenterImg(0.5f * (leftX + rightX), bottomY);
    Vector2f bottomLeftImg(leftX, bottomY);
    Vector2f bottomRightImg(rightX, bottomY);

    RobotPercept::Robot robot;
    robot.type = RobotPercept::Type::unknown;

    // This can often be wrong
    robot.fallen = (p.width > p.height);

    // Perform conversion from image to field coordinates,
    // including heuristic distortion correction based on assumed robot size.

    // Center of box (in image) is assumed to be the actual robot center:
    const float boxCenterX(0.5f * (leftX + rightX));
    const float boxCenterY(0.5f * (bottomY + topY));
    const Vector2f boxCenterImg = Vector2f(boxCenterX, boxCenterY);
    // Computed the assumed height over ground (in field coordinates) of the box center:
    const Settings::RobotType robotType = Global::getSettings().robotType;
    const float assumedRobotHeight = robotType == Settings::k1 ? 950.f : 1180.f;
    const float assumedShoulderHeight = robotType == Settings::k1 ? 950.f - 160.f : 1180.f - 220.f;
    const float assumedCenterHeight = topY == 0.f ? assumedShoulderHeight / 2.f : assumedRobotHeight / 2.f;
    // Project robot center to the ground (in relative field coordinates)...
    Vector2f centerOnGround;
    if(!Transformation::imageToRobotHorizontalPlane(boxCenterImg, assumedCenterHeight, theCameraMatrix, theCameraInfo, centerOnGround))
      continue;
    // ... and the center point back into the the image:
    Vector2f newBottomCenterImage;
    if(!Transformation::robotToImage(centerOnGround, theCameraMatrix, theCameraInfo, newBottomCenterImage))
      continue;
    // Finally compute center point (in relative field coordinates) based on new image x coordinate and
    // the bottom y coordinate of original bounding box, which has less trouble with outliers (but it is still not a really good solution):
    if(!theMeasurementCovariance.transformPointWithCov(Vector2f(newBottomCenterImage.x(), bottomCenterImg.y()), 0.f, robot.center, robot.covariance))
      continue;

    // Heuristic for adapting the outermost left and the right points of the robot (in image coordinates),
    // assuming that both should have the same distance to the center:
    if(!robot.fallen)
    {
      const float widthToLeft = std::abs(newBottomCenterImage.x() - bottomLeftImg.x());
      const float widthToRight = std::abs(newBottomCenterImage.x() - bottomRightImg.x());
      if(widthToRight < widthToLeft)
        bottomLeftImg.x() = newBottomCenterImage.x() - widthToRight;
      else
        bottomRightImg.x() = newBottomCenterImage.x() + widthToLeft;
    }
    // Project robot's left and right points to field coordinates
    if(!Transformation::imageToRobot(bottomLeftImg, theCameraMatrix, theCameraInfo, robot.left))
      continue;
    if(!Transformation::imageToRobot(bottomRightImg, theCameraMatrix, theCameraInfo, robot.right))
      continue;
    // This might happen and produces an unwanted result:
    if(robot.left == robot.right)
      continue;
    // Correction step to prevent super thin robots (might happen, if center point is very close to one side):
    auto adaptDistance = [](const Vector2f& base, Vector2f& side) -> void
    {
      const float minDist = 100.f;
      const Vector2f baseToSide(side - base);
      if(baseToSide.norm() < minDist)
        side = base + baseToSide.normalized(minDist);
    };
    adaptDistance(robot.center, robot.right);
    adaptDistance(robot.center, robot.left);

    // Current approach detects the robot points closest to the observer,
    // move these points a bit away towards the hypothetical robot center:
    robot.center.normalize(robot.center.norm() + theRobotDimensions.robotDepth);
    robot.left.normalize(robot.left.norm() + theRobotDimensions.robotDepth);
    robot.right.normalize(robot.right.norm() + theRobotDimensions.robotDepth);

    // Finally, classify the jersey color and add robot to the list
    if(robot.fallen)
      robot.type = RobotPercept::unknown;
    else
      theJerseyClassifier.detectJersey(robot);
    robot.topLeftInImage = Vector2i(static_cast<int>(leftX), static_cast<int>(topY));
    robot.bottomRightInImage = Vector2i(static_cast<int>(rightX), static_cast<int>(bottomY));
    theRobotPercept.robots.emplace_back(robot);
  }
}

template <typename Pixel>
static inline bool subImage(const Image<Pixel>& cameraImage, const Vector2f& topLeft,
                            const Vector2f& bottomRight, Image<Pixel>& subImage,
                            const uint32_t padding = 0)
{
  if(topLeft.x() >= bottomRight.x() || topLeft.y() >= bottomRight.y() ||
     topLeft.y() < 0 || topLeft.x() < 0 ||
     bottomRight.x() >= cameraImage.width || bottomRight.y() >= cameraImage.height)
  {
    return false;
  }
  // Height and width
  const uint32_t height = static_cast<uint32_t>(bottomRight.y() - topLeft.y());
  const uint32_t width = static_cast<uint32_t>(bottomRight.x() - topLeft.x());

  subImage.setResolution(width, height, padding);
  greyScaledBoundingBox.setResolution(width, height);
  for(uint32_t y = 0 + padding; y < height - padding; ++y)
  {
    for(uint32_t x = 0 + padding; x < width - padding; ++x)
    {
      PixelTypes::GrayscaledPixel pixel = cameraImage[static_cast<uint32_t>(topLeft.y()) + y][static_cast<uint32_t>(topLeft.x()) + x];
      subImage[y][x] = pixel;
      greyScaledBoundingBox[y][x] = subImage[y][x];
    }
  }
  return true;
}

bool RobotBallPerceptor::pointIsInBoundingBox(const Vector2f& point, const Vector2f& topLeft, const Vector2f& botRight) const
{
  return (point.x() >= topLeft.x() && point.y() >= topLeft.y()) || (point.x() < botRight.x() && point.y() < botRight.y());
}

bool RobotBallPerceptor::improveBallExtrema(Vector2f& closerToMiddle, Vector2f& furtherFromMiddle,
                                            const Geometry::Line& ballCenterToOpticalCenter, const Prediction& ballPrediction)
{
  // We have to make sure the dimensions of the smaller image will be correct
  Vector2f subImgBottomRight = ballPrediction.bottomRight;
  Vector2f subImgTopLeft = ballPrediction.topLeft;

  const uint32_t remainder16 = static_cast<uint32_t>(subImgBottomRight.x() - subImgTopLeft.x()) % 16;
  const uint32_t height = static_cast<uint32_t>(subImgBottomRight.y() - subImgTopLeft.y());

  // Width has to be divisible by 16
  if(remainder16 != 0)
  {
    subImgBottomRight.x() += 16 - remainder16;
  }

  // Height has to be greater or equal to 3
  if(height < 3)
  {
    subImgBottomRight.y() += 3 - height;
  }

  // Extract smaller sub image
  Image<PixelTypes::GrayscaledPixel> predictionSubImage;
  if(!subImage(theECImage.grayscaled, subImgTopLeft, subImgBottomRight, predictionSubImage, 0))
  {
    return false;
  }

  // Move extrema inside bounding box
  for(uint32_t i = 0; i < 50 && !pointIsInBoundingBox(closerToMiddle, ballPrediction.topLeft, ballPrediction.bottomRight); ++i)
  {
    closerToMiddle -= ballCenterToOpticalCenter.direction.normalized();
  }
  for(uint32_t i = 0; i < 50 && !pointIsInBoundingBox(furtherFromMiddle, ballPrediction.topLeft, ballPrediction.bottomRight); ++i)
  {
    furtherFromMiddle -= ballCenterToOpticalCenter.direction.normalized();
  }

  const Vector2f closerToMiddleInBox = closerToMiddle - subImgTopLeft;
  const Vector2f furtherFromMiddleInBox = furtherFromMiddle - subImgTopLeft;

  if(closerToMiddleInBox.y() < greyScaledBoundingBox.height && closerToMiddleInBox.x() < greyScaledBoundingBox.width &&
     furtherFromMiddleInBox.y() < greyScaledBoundingBox.height && furtherFromMiddleInBox.x() < greyScaledBoundingBox.width &&
     closerToMiddleInBox.y() >= 0 && closerToMiddleInBox.x() >= 0 && furtherFromMiddleInBox.y() >= 0 && furtherFromMiddleInBox.x() >= 0)
  {
    greyScaledBoundingBox[static_cast<uint32_t>(closerToMiddleInBox.y())]
    [static_cast<uint32_t>(closerToMiddleInBox.x())] = 0;
    greyScaledBoundingBox[static_cast<uint32_t>(furtherFromMiddleInBox.y())]
    [static_cast<uint32_t>(furtherFromMiddleInBox.x())] = 0;
  }

  static_cast<void>(closerToMiddle);
  static_cast<void>(furtherFromMiddle);
  static_cast<void>(ballCenterToOpticalCenter);
  static_cast<void>(ballPrediction);
  return true;
}
