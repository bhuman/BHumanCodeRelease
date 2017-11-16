/**
 * @file BallPerceptor.cpp
 * This file implements a module that detects balls in CNS images.
 * @author Thomas Röfer
 * @author Udo Frese
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "BallPerceptor.h"
#include "Platform/File.h"
#include "Platform/SystemCall.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/ImageProcessing/CNS/CNSTools.h"
#include "Tools/ImageProcessing/CNS/LutRasterizer.h"
#include "Tools/ImageProcessing/InImageSizeCalculations.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Settings.h"
#include "Tools/Streams/InStreams.h"

MAKE_MODULE(BallPerceptor, perception)

BallPerceptor::BallPerceptor()
{
  // Create or load contour table
  Eigen::AlignedBox3d viewpointRange(Vector3d(-maxTableRadius,
                                              -maxTableRadius,
                                              minTableHeight),
                                     Vector3d(maxTableRadius,
                                              maxTableRadius,
                                              maxTableHeight));
  const TriangleMesh& mesh = TriangleMesh::sphere(theBallSpecification.radius, contourSubDivisions);
  detector.create(mesh,
                  CameraModelOpenCV(),
                  viewpointRange,
                  spacing,
                  SearchSpecification(),
                  ParametricLaplacian(),
                  (std::string(File::getBHDir()) + "/Config/ballPerceptor.dat").c_str());

  // Select sample points for checking the ball's pattern
  for(const Eigen::Vector3d& v : TriangleMesh::sphere(theBallSpecification.radius, sampleSubDivisions).vertex)
    if(v.y() <= theBallSpecification.radius * sampleHeightRatio && v.z() <= -theBallSpecification.radius * sampleDepthRatio)
      samplePoints.push_back(v);

  createOrLoadPatternTable();
}

void BallPerceptor::update(BallPercept& ballPercept)
{
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:candidates", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:around", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:samplePoints", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:acceptedPattern", "drawingOnImage");
  tips.clear();

  detector.setCamera(CNS::toCameraModelOpenCV(theCameraInfo, theCameraIntrinsics));
  objects.clear();
  ballPercept.status = BallPercept::notSeen;

  if(theCameraMatrix.isValid)
  {
    updateSearchSpace();

    if(usePrediction && theWorldModelPrediction.ballIsValid)
    {
      Matrix4d cameraInImage;
      cameraInImage << 0, -1,  0, 0,
                       0,  0, -1, 0,
                       1,  0,  0, 0,
                       0,  0,  0, 1;
      Matrix4d cameraInRobot = CNS::toMatrix4d(theCameraMatrix);
      Matrix4d ballInRobot;
      ballInRobot << 1, 0, 0, theWorldModelPrediction.ballPosition.x(),
                     0, 1, 0, theWorldModelPrediction.ballPosition.y(),
                     0, 0, 1, theBallSpecification.radius,
                     0, 0, 0, 1;
      Matrix4d ballInImage = cameraInImage * cameraInRobot.inverse() * ballInRobot;
      objects.push_back(IsometryWithResponse(Eigen::Isometry3d(ballInImage), 0));
      if(spec.nRefineIterations > 0)
        detector.refine(theCNSImage, objects.front(), spec.nRefineIterations);
      if(objects.front().response < minResponse)
        objects.clear();
    }

    for(const Boundaryi& region : theBallRegions.regions)
    {
      spec.blockX = region.x.getSize();
      spec.blockY = region.y.getSize();
      detector.setSearchSpecification(spec);
      ObjectCNSStereoDetector::IsometryWithResponses newObjects;
      detector.searchBlockAllPoses(newObjects, theCNSImage, region.x.min, region.y.min);

      for(const IsometryWithResponse& object : newObjects)
        if(object.response >= minResponse)
          objects.emplace_back(object);

      if(spec.nRefineIterations > 0)
      {
        for(IsometryWithResponse& object : newObjects)
          detector.refine(theCNSImage, object, spec.nRefineIterations);

        for(const IsometryWithResponse& object : newObjects)
          if(object.response >= minResponse)
            objects.emplace_back(object);
      }
    }

    sort(objects.begin(), objects.end(), MoreOnResponse());

    std::vector<IsometryWithResponse> guesses;
    while(!objects.empty())
    {
      ballPercept.status = checkBall(objects.front());
      if(ballPercept.status == BallPercept::seen)
        break;
      else if(ballPercept.status == BallPercept::guessed)
        guesses.push_back(objects.front());

      objects.erase(objects.begin());
    }

    if(ballPercept.status != BallPercept::seen && !guesses.empty())
    {
      objects.push_back(guesses.front());
      ballPercept.status = BallPercept::guessed;
    }

    // Special ball handling for penalty goal keeper
    if(ballPercept.status == BallPercept::notSeen
       && theRole.role == Role::penaltyKeeper && theMotionInfo.motion == MotionRequest::specialAction)
    {
      std::vector<Vector2i> sortedBallSpots = theBallSpots.ballSpots;
      std::sort(sortedBallSpots.begin(), sortedBallSpots.end(), [&](const Vector2i a, const Vector2i b) {return a.y() > b.y(); });

      Vector2f inImageLowPoint;
      Vector2f inImageUpPoint;
      if(Transformation::robotToImage(theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea + 350, 0), theCameraMatrix, theCameraInfo, inImageLowPoint)
         && Transformation::robotToImage(theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyMark, 0), theCameraMatrix, theCameraInfo, inImageUpPoint))
      {
        const int lowerY = std::min(static_cast<int>(inImageLowPoint.y()), theCameraInfo.height);
        const int upperY = std::max(static_cast<int>(inImageUpPoint.y()), 0);

        for(const Vector2i& spot : sortedBallSpots)
          if(spot.y() < lowerY && spot.y() > upperY)
          {
            ballPercept.positionInImage = spot.cast<float>();
            if(Transformation::imageToRobot(ballPercept.positionInImage, theCameraMatrix, theCameraInfo, ballPercept.positionOnField))
              ballPercept.status = BallPercept::seen;

            ballPercept.radiusInImage = 30;

            return;
          }
      }
    }

    fillBallPercept(ballPercept);
  }

  draw();
}

void BallPerceptor::updateSearchSpace()
{
  Matrix4d cameraInImage;
  cameraInImage << 0, -1,  0, 0,
                   0,  0, -1, 0,
                   1,  0,  0, 0,
                   0,  0,  0, 1;
  Matrix4d cameraInRingCenter = CNS::toMatrix4d(theCameraMatrix);
  Matrix4d ringCenterInImage = cameraInImage * cameraInRingCenter.inverse();
  Pose3f p = CNS::toPose3f(ringCenterInImage);
  spec.positionSpace = CylinderRing(Eigen::Isometry3d(ringCenterInImage),
                                    0,
                                    maxTableRadius,
                                    theBallSpecification.radius - 1.f,
                                    theBallSpecification.radius + 1.f);
  spec.nRefineIterations = refineIterations;
  spec.stepInPixelDuringRefinement = refineStepSize;
  spec.object2WorldOrientation.clear();
  Vector3f up = theCameraMatrix.rotation.inverse() * Vector3f::UnitZ();
  spec.object2WorldOrientation.push_back(fromTo(Vector3d::UnitZ(), Vector3d(-up.y(), -up.z(), up.x())));
}

void BallPerceptor::fillBallPercept(BallPercept& ballPercept) const
{
  if(!objects.empty())
  {
    const IsometryWithResponse& object = objects.front();
    double x, y;
    detector.camera.camera2Image(Vector3d(object(0, 3), object(1, 3), object(2, 3)), x, y);
    ballPercept.positionInImage = Vector2f(static_cast<float>(x), static_cast<float>(y));

    CodedContour contour;
    detector.lr.rasterize(contour, object, detector.camera);
    float sum = 0;
    for(const CodedContourPoint& p : contour)
      sum += Vector2f(static_cast<float>(xOfCCP(p)), static_cast<float>(yOfCCP(p))).norm();
    ballPercept.radiusInImage = sum / static_cast<float>(contour.size());

    const Vector2f correctedCenter = theImageCoordinateSystem.toCorrected(ballPercept.positionInImage);
    Vector3f cameraToBall(theCameraInfo.focalLength, theCameraInfo.opticalCenter.x() - correctedCenter.x(), theCameraInfo.opticalCenter.y() - correctedCenter.y());
    cameraToBall.normalize(theBallSpecification.radius * theCameraInfo.focalLength / ballPercept.radiusInImage);
    Vector3f rotatedCameraToBall = theCameraMatrix.rotation * cameraToBall;
    const Vector3f sizeBasedCenterOnField = theCameraMatrix.translation + rotatedCameraToBall;
    const Vector3f bearingBasedCenterOnField = theCameraMatrix.translation - rotatedCameraToBall * ((theCameraMatrix.translation.z() - theBallSpecification.radius) / rotatedCameraToBall.z());

    if(rotatedCameraToBall.z() < 0)
    {
      ballPercept.positionOnField.x() = bearingBasedCenterOnField.x();
      ballPercept.positionOnField.y() = bearingBasedCenterOnField.y();
    }
    else
    {
      ballPercept.positionOnField.x() = sizeBasedCenterOnField.x();
      ballPercept.positionOnField.y() = sizeBasedCenterOnField.y();
    }
  }
}

BallPercept::Status BallPerceptor::checkBall(const IsometryWithResponse& object) const
{
  CodedContour contour;
  detector.lr.rasterize(contour, object, detector.camera);
  const Vector2i center(contour.referenceX, contour.referenceY);

  char buf[10000];
  bool createTips = recordTips;
  COMPLEX_DRAWING("module:BallPerceptor:candidates")
    createTips = true;
  if(createTips)
  {
    tip = new OutTextRawMemory(buf);
    if(recordTips)
      *tip << endl << "    center\t\t" << center.x() << "\t" << center.y();
    leq(minResponse, static_cast<float>(object.response), "min response");
  }

  // Compute dimensions in image.
  Boundaryi range(Rangei(std::numeric_limits<int>::max(), std::numeric_limits<int>::min()),
                  Rangei(std::numeric_limits<int>::max(), std::numeric_limits<int>::min()));
  for(const CodedContourPoint& p : contour)
    range.add(Vector2i(xOfCCP(p), yOfCCP(p)));
  range.add(Vector2i(range.x.max + 1, range.y.max + 1));
  const float radius = static_cast<float>(std::max(-range.x.min, range.x.max) + std::max(-range.y.min, range.y.max)) / 2.f;

  float expectedRadius = IISC::getImageBallRadiusByCenter(Vector2f(static_cast<float>(center.x()),
                         static_cast<float>(center.y())),
                         theCameraInfo,
                         theCameraMatrix,
                         theBallSpecification);
  float distance = static_cast<float>(object.translation().squaredNorm()) - sqr(theCameraMatrix.translation.z());
  if(distance > 0)
    distance = std::sqrt(distance);
  else
    distance = 0;

  int greenCount;
  int nonWhiteCount;
  const float response = static_cast<float>(object.response);
  std::vector<unsigned char> brightnesses;
  brightnesses.reserve(samplePoints.size());
  int threshold;
  BallPattern pattern = 0;

#if defined TARGET_ROBOT || defined NDEBUG
#define AND &&
#define OR ||
#else
#define AND &
#define OR |
#endif

  BallPercept::Status status = BallPercept::notSeen;
  if(leq(std::abs(radius - expectedRadius) / expectedRadius, maxRadiusDeviation, "delta radius")
     AND print(theBodyContour.isValidPoint(center), "outside body")
     AND (skipMostChecks
          OR (((countAround(center, expectedRadius, greenCount, nonWhiteCount),
               (leq(minAroundNonWhiteRatio, static_cast<float>(nonWhiteCount) / static_cast<float>(numberOfGreenChecks * greenCheckRadiusRatios.size()), "min around nonWhite")
                AND leq(minAroundGreenRatio, static_cast<float>(greenCount) / static_cast<float>(numberOfGreenChecks * greenCheckRadiusRatios.size()), "min around green")
                    ? status = BallPercept::guessed, true : false))
               AND leq(minResponseWithinGreen, response, "min response around")
               AND leq(minAroundDistance, distance, "min around distance")
               AND leq(distance, maxAroundDistance, "max around distance"))
              OR static_cast<bool>(leq(minResponse + (maxResponse - minResponse) * (1.f - static_cast<float>(greenCount) / static_cast<float>(numberOfGreenChecks * greenCheckRadiusRatios.size())), response, "min response green")
                                   AND print(checkSamplePoints(object, false, brightnesses, threshold, pattern),
                                             pattern ? ("pattern\t\t" + std::to_string(pattern)).c_str() : "pattern")
                                   AND checkContrast(brightnesses, threshold))))) // remove static_cast<bool> if VS2017 bug is fixed
  {
    status = BallPercept::seen;
    COMPLEX_DRAWING("module:BallPerceptor:acceptedPattern")
      checkSamplePoints(object, true, brightnesses, threshold, pattern);
  }

  COMPLEX_DRAWING("module:BallPerceptor:candidates")
  {
    static const ColorRGBA colors[] =
    {
      ColorRGBA::red,
      ColorRGBA::green,
      ColorRGBA::yellow
    };

    drawContourViaLutRasterizer(detector.lr, object, detector.camera, colors[status]);
  }

  if(tip)
  {
    if(tip->getLength() > 1)
    {
      ASSERT(tip->getLength() < 9999);
      reinterpret_cast<char*>(tip->getMemory())[tip->getLength()] = 0;
      TIP("module:BallPerceptor:candidates", center.x(), center.y(), radius, (reinterpret_cast<const char*>(tip->getMemory()) + 1));
      if(recordTips)
        tips.push_back(reinterpret_cast<const char*>(tip->getMemory()) + 1);
    }
    delete tip;
    tip = nullptr;
  }

  return status;
}

void BallPerceptor::countAround(const Vector2i& center, float radius, int& greenCount, int& nonWhiteCount) const
{
  greenCount = nonWhiteCount = 0;
  for(float radiusRatio : greenCheckRadiusRatios)
    countAtRadius(center, radius * radiusRatio, greenCount, nonWhiteCount);
}

void BallPerceptor::countAtRadius(const Vector2i& center, float radius, int& greenCount, int& nonWhiteCount) const
{
  std::function<bool(int, int)> isInside;
  if(center.x() - radius > 0 && center.x() + radius + 1 < theECImage.colored.width &&
     center.y() - radius > 0 && center.y() + radius + 1 < theECImage.colored.height)
    isInside = [&](int, int) -> bool {return true;};
  else
    isInside = [&](int x, int y) -> bool
    {
      return x >= 0 && x < theECImage.colored.width &&
             y >= 0 && y < theECImage.colored.height;
    };
  const float epsilon = pi2 / static_cast<float>(numberOfGreenChecks);
  float dx = -radius;
  float dy = 0.f;
  for(int i = 0; i < numberOfGreenChecks; ++i)
  {
    int x = center.x() + static_cast<int>(std::round(dx));
    int y = center.y() + static_cast<int>(std::round(dy));
    if(isInside(x, y))
    {
      const FieldColors::Color color = theECImage.colored[y][x];
      if(color == FieldColors::field)
      {
        ++greenCount;
        ++nonWhiteCount;
        DOT("module:BallPerceptor:around", x, y, ColorRGBA::green, ColorRGBA::green);
      }
      else if(color != FieldColors::white)
      {
        ++nonWhiteCount;
        DOT("module:BallPerceptor:around", x, y, ColorRGBA::red, ColorRGBA::red);
      }
      else
        DOT("module:BallPerceptor:around", x, y, ColorRGBA::yellow, ColorRGBA::yellow);
    }
    dx -= dy * epsilon;
    dy += dx * epsilon;
  }
}

bool BallPerceptor::leq(float op1, float op2, const char* label) const
{
  if(tip)
    *tip << endl << (op1 <= op2 ? "x" : "  ") << "  " << label << "\t"
         << std::round(op1 * 100.f) / 100.f << "\t" << std::round(op2 * 100.f) / 100.f;

  return op1 <= op2;
}

bool BallPerceptor::print(bool result, const char* label) const
{
  if(tip)
    *tip << endl << (result ? "x" : "  ") << "  " << label;

  return result;
}

bool BallPerceptor::checkSamplePoints(const IsometryWithResponse& object, bool accepted, std::vector<unsigned char>& brightnesses, int& threshold, BallPattern& pattern) const
{
  const double aroundX = std::atan2(object.translation().y(), object.translation().z());
  const double aroundY = std::atan2(object.translation().x(), object.translation().z());
  const Quaterniond rotation = Quaterniond(Eigen::AngleAxisd(-aroundX, Vector3d::UnitX())) *
                               Quaterniond(Eigen::AngleAxisd(aroundY, Vector3d::UnitY()));
  std::vector<Vector2i> points;
  points.reserve(samplePoints.size());
  brightnesses.clear();
  threshold = 0;
  Eigen::Isometry3d ballInWorld = spec.positionSpace.cylinder2World.inverse();
  ballInWorld(0, 3) = ballInWorld(1, 3) = ballInWorld(2, 3) = 0;

  for(const Eigen::Vector3d& samplePoint : samplePoints)
  {
    double x, y;
    const Vector3d visiblePoint = rotation * samplePoint;
    if(!detector.camera.world2ImageClipped(object.translation() + visiblePoint, x, y))
      return false;
    const Vector2i point(static_cast<int>(std::round(x)), static_cast<int>(std::round(y)));
    if(point.x() >= theCameraInfo.width || point.y() >= theCameraInfo.height)
      return false;
    points.push_back(point);
    const float z = static_cast<float>((ballInWorld * visiblePoint).z());
    const float factor = 1.f + (1.f - z / theBallSpecification.radius) * brightnessBonus;
    const int brightness = std::min(255, static_cast<int>(theECImage.grayscaled[point.y()][point.x()] * factor));
    brightnesses.push_back(static_cast<unsigned char>(brightness));
  }

  threshold = calcThreshold(brightnesses);

  pattern = 0;
  for(size_t i = 0; i < points.size(); ++i)
  {
    const Vector2i& point = points[i];
    bool dark = brightnesses[i] <= threshold;
    if(accepted)
      DOT("module:BallPerceptor:acceptedPattern", point.x(), point.y(), dark ? ColorRGBA::red : ColorRGBA::blue, dark ? ColorRGBA::red : ColorRGBA::blue);
    else
      DOT("module:BallPerceptor:samplePoints", point.x(), point.y(), dark ? ColorRGBA::red : ColorRGBA::blue, dark ? ColorRGBA::red : ColorRGBA::blue);
    pattern = pattern << 1 | (dark ? 0 : 1);
  }

  return ballPatterns.find(pattern) != ballPatterns.end();
}

int BallPerceptor::calcThreshold(const std::vector<unsigned char>& brightnesses)
{
  // Build histogram
  std::array<unsigned char, 256> histogram;
  histogram.fill(0);
  int minIndex = 255;
  int maxIndex = 0;
  for(unsigned char brightness : brightnesses)
  {
    ++histogram[brightness];
    minIndex = std::min(minIndex, static_cast<int>(brightness));
    maxIndex = std::max(maxIndex, static_cast<int>(brightness));
  }

  int sumCov2 = 0;
  int sumLow = 0; // Sum of coverage values <= the threshold
  int sumHigh = 0; // Sum of coverage values > the threshold
  int sumCovLow = 0; // Sum of coverage values <= the threshold weighted with its occurrence
  int sumCovHigh = 0; // Sum of coverage values > the threshold weighted with its occurrence

  // Initial threshold is 0.
  for(int i = minIndex; i <= maxIndex; ++i)
    if(histogram[i])
    {
      sumCov2 += histogram[i] * i * i;
      sumCovHigh += histogram[i] * i;
      sumHigh += histogram[i];
    }

  int bestThreshold = -1; // Best coverage threshold
  float bestBalance = static_cast<float>(sumCovHigh * sumCovHigh) / static_cast<float>(sumHigh);
  for(int i = minIndex; i <= maxIndex; ++i)
    if(histogram[i])
    {
      sumHigh -= histogram[i];
      sumLow += histogram[i];
      sumCovLow += histogram[i] * i;
      sumCovHigh -= histogram[i] * i;
      if(sumLow > 0 && sumHigh > 0)
      {
        float balance = static_cast<float>(sumCovLow * sumCovLow) / static_cast<float>(sumLow)
                        + static_cast<float>(sumCovHigh * sumCovHigh) / static_cast<float>(sumHigh);
        if(balance > bestBalance)
        {
          bestBalance = balance;
          bestThreshold = static_cast<int>(i);
        }
      }
    }

  return bestThreshold;
}

bool BallPerceptor::checkContrast(const std::vector<unsigned char>& brightnesses, int threshold) const
{
  unsigned sumBlack = 0;
  unsigned sumWhite = 0;
  unsigned countBlack = 0;
  for(unsigned char brightness : brightnesses)
    if(brightness <= threshold)
    {
      sumBlack += brightness;
      ++countBlack;
    }
    else
      sumWhite += brightness;

  if(countBlack == 0 || countBlack == brightnesses.size())
    return print(false, "contrast");
  else
  {
    unsigned meanBlack = sumBlack / countBlack;
    unsigned meanWhite = sumWhite / (static_cast<unsigned>(brightnesses.size()) - countBlack);
    return leq(minContrast, (static_cast<float>(meanWhite) - static_cast<float>(meanBlack)) / static_cast<float>(meanWhite), "minContrast");
  }
}

void BallPerceptor::createOrLoadPatternTable()
{
  const size_t bytesPerPattern = (samplePoints.size() + 7) / 8;
  ASSERT(sizeof(BallPattern) >= bytesPerPattern);

  InBinaryFile in("ballPatterns.dat");
  if(in.exists())
  {
    unsigned size;
    BallPattern ballPattern = 0;
    in >> size;
    ballPatterns.clear();
    ballPatterns.reserve(size);
    while(size-- > 0)
    {
      in.read(&ballPattern, bytesPerPattern);
      ballPatterns.insert(ballPattern);
    }
    return;
  }

  unsigned char ballTexture[ballTextureHeight][ballTextureWidth];
  InBinaryFile stream("ballTexture.dat");
  if(stream.exists())
    stream.read(ballTexture, sizeof(ballTexture));

  ballPatterns.clear();
  for(float x = -sampleRange; x < sampleRange; x += sampleStep)
  {
    printf("%f\n", x);
    for(float y = -sampleRange; y < sampleRange; y += sampleStep)
      for(float z = -sampleRange; z < sampleRange; z += sampleStep)
        ballPatterns.insert(getPattern(ballTexture, x, y, z));
  }

  OutBinaryFile out("ballPatterns.dat");
  out << static_cast<unsigned>(ballPatterns.size());
  for(BallPattern ballPattern : ballPatterns)
    out.write(&ballPattern, bytesPerPattern);
}

BallPerceptor::BallPattern BallPerceptor::getPattern(unsigned char ballTexture[ballTextureHeight][ballTextureWidth], float x, float y, float z) const
{
  Quaterniond rotation = Quaterniond(Eigen::AngleAxisd(x, Vector3d::UnitX())) *
                         Quaterniond(Eigen::AngleAxisd(y, Vector3d::UnitY())) *
                         Quaterniond(Eigen::AngleAxisd(z, Vector3d::UnitZ()));
  BallPattern bits = 0;
  for(const Eigen::Vector3d& v : samplePoints)
    bits = bits << 1 | (getBrightness(ballTexture, rotation * v) < 128 ? 0 : 1);
  return bits;
}

unsigned char BallPerceptor::getBrightness(unsigned char ballTexture[ballTextureHeight][ballTextureWidth], const Vector3d& v)
{
  Angle aroundY = static_cast<float>(std::atan2(v.x(), v.z()));
  const Vector3d v2 = Quaterniond(Eigen::AngleAxisd(-aroundY, Vector3d::UnitY())) * v;
  const Angle aroundX = static_cast<float>(std::atan2(v2.y(), v2.z())) + pi_2;
  aroundY += pi;
  const int x = static_cast<int>(aroundY * ballTextureWidth / pi2) % ballTextureWidth;
  const int y = static_cast<int>(aroundX * ballTextureHeight / pi) % ballTextureHeight;
  ASSERT(x >= 0 && x < ballTextureWidth);
  ASSERT(y >= 0 && y < ballTextureHeight);
  return ballTexture[y][x];
}

void BallPerceptor::draw()
{
  spec.blockX = 64;
  spec.blockY = 48;
  detector.setSearchSpecification(spec);
  DEBUG_DRAWING("module:BallPerceptor:ring", "drawingOnImage")
  {
    drawCylinderRing(detector.spec.positionSpace, detector.camera);
  }
  DEBUG_DRAWING("module:BallPerceptor:searchSpace", "drawingOnImage")
  {
    Contour ctAll;
    detector.renderSearchSpace(ctAll, detector.camera, 1);
    drawRasteredContour(ctAll, ColorRGBA::green);
  }
}

void BallPerceptor::drawContourViaLutRasterizer(const LutRasterizer& lr, const Eigen::Isometry3d& object2World, const CameraModelOpenCV& camera, const ColorRGBA& color) const
{
  CodedContour contour;
  lr.rasterize(contour, object2World, camera);
  for(unsigned i = 0; i < contour.size(); ++i)
  {
    int x = xOfCCP(contour[i]) + contour.referenceX,
        y = yOfCCP(contour[i]) + contour.referenceY;
    double alpha = angleOfCCP(contour[i]);
    double c = cos(alpha),
           s = sin(alpha);
    LINE("module:BallPerceptor:candidates", (int) round(x - 2 * s), (int) round(y + 2 * c),
         (int) round(x + 2 * s), (int) round(y - 2 * c), 1, Drawings::solidPen, color);
  }
}

void BallPerceptor::drawRasteredContour(const Contour& contour, const ColorRGBA& color) const
{
  for(unsigned i = 0; i < contour.size(); ++i)
  {
    int x = contour[i].x + contour.referenceX,
        y = contour[i].y + contour.referenceY;
    double alpha = contour[i].angle;
    double c = cos(alpha),
           s = sin(alpha);
    LINE("module:BallPerceptor:searchSpace", round(x - 2 * s), round(y + 2 * c),
         round(x + 2 * s), round(y - 2 * c), 1, Drawings::solidPen, color);
  }
}

void BallPerceptor::drawCylinderRing(const CylinderRing& cylinderRing, const CameraModelOpenCV& camera) const
{
  ColorRGBA colors[4] =
  {
    ColorRGBA(255, 255, 255),
    ColorRGBA(255, 0, 0),
    ColorRGBA(0, 255, 0),
    ColorRGBA(0, 0, 255)
  };
  for(int i = 0; i < 4; ++i)
  {
    ColorRGBA& color = colors[i];
    for(double alpha = 0; alpha < 2 * M_PI; alpha += 0.02)
    {
      double x, y;
      if(camera.world2ImageClipped(cylinderRing.sampleEdgePoint(alpha, i), x, y))
        DOT("module:BallPerceptor:ring", x, y, color, color);
    }
  }
}
