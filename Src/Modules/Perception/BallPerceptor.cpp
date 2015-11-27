/**
 * @file BallPerceptor.cpp
 * This file declares a module that provides a ball percept without using color tables.
 * If possible, it uses the full YCbCr422 image.
 * The ball center / radius calculation algorithm is based on the BallSpecialist in GT2005.
 * @author Colin Graf
 * @author marcel
 * @author Florian Maaß
 * @author Thomas Röfer
 */

#include "BallPerceptor.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Eigen.h"

#include <algorithm>

MAKE_MODULE(BallPerceptor, perception)

void BallPerceptorScaler::scaleInput()
{
  typedef BallPerceptorBase B;
  theCameraInfo = B::theCameraInfo;
  theBallSpots = B::theBallSpots;

  // do not copy "table"
  theImageCoordinateSystem.rotation = B::theImageCoordinateSystem.rotation;
  theImageCoordinateSystem.invRotation = B::theImageCoordinateSystem.invRotation;
  theImageCoordinateSystem.origin = B::theImageCoordinateSystem.origin;
  theImageCoordinateSystem.offset = B::theImageCoordinateSystem.offset;
  theImageCoordinateSystem.a = B::theImageCoordinateSystem.a;
  theImageCoordinateSystem.b = B::theImageCoordinateSystem.b;

  if(theCameraInfo.camera == CameraInfo::upper && IS_FULL_SIZE)
  {
    theCameraInfo.width *= 2;
    theCameraInfo.height *= 2;
    theCameraInfo.opticalCenter *= 2.f;
    theCameraInfo.focalLength *= 2.f;
    theCameraInfo.focalLengthInv /= 2.f;
    theCameraInfo.focalLenPow2 *= 4.f;

    theImageCoordinateSystem.origin *= 2.f;
    theImageCoordinateSystem.b *= 0.5f;

    for(BallSpot& b : theBallSpots.ballSpots)
    {
      b.position *= 2;
      ++b.position.x(); // original y channel was the second one
    }
  }
  theImageCoordinateSystem.setCameraInfo(theCameraInfo);
}

void BallPerceptorScaler::scaleOutput(BallPercept& ballPercept) const
{
  if(theCameraInfo.camera == CameraInfo::upper && IS_FULL_SIZE)
  {
    ballPercept.positionInImage *= 0.5f;
    ballPercept.radiusInImage *= 0.5f;
  }
}

// Alternate drawing macros that scale down if required
#define LINE2(n, x1, y1, x2, y2, w, s, c) LINE(n, scale(x1), scale(y1), scale(x2), scale(y2), w, s, c)
#define CROSS2(n, x, y, r, w, s, c) CROSS(n, scale(x), scale(y), scale(r), w, s, c)
#define CIRCLE2(n, x, y, r, w, s1, c1, s2, c2) CIRCLE(n, scale(x), scale(y), scale(r), w, s1, c1, s2, c2)
#define RECTANGLE3(n, x1, y1, x2, y2, w, s, c) RECTANGLE(n, scale(x1), scale(y1), scale(x2), scale(y2), w, s, c)
#define DOT2(n, x, y, c1, c2) DOT(n, scale(x), scale(y), c1, c2)

static bool ballSpotComparator(const BallSpot& b1, const BallSpot& b2)
{
  if(b1.position.y() != b2.position.y())
    return b1.position.y() > b2.position.y();
  else
    return b1.position.x() < b2.position.x();
}

BallPerceptor::BallPerceptor() : right(0), horizon(0), height(0)
{
  sqrMaxBallDistance = Vector2f(theFieldDimensions.xPosOpponentFieldBorder - theFieldDimensions.xPosOwnFieldBorder,
                                theFieldDimensions.yPosLeftFieldBorder - theFieldDimensions.yPosRightFieldBorder).squaredNorm();
  if(SystemCall::getMode() == SystemCall::simulatedRobot)
  {
    scanMaxColorDistance = 60;
    refineMaxColorDistance = 55;
  }
}

void BallPerceptor::update(BallPercept& ballPercept)
{
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:image", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:field", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:ballSpot", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:ballSpotScanLines", "drawingOnImage");
  DECLARE_PLOT("module:BallPerceptor:angle");

  scaleInput();

  // first of all we think, that no ball was found...
  ballPercept.status = BallPercept::notSeen;
  if(!theCameraMatrix.isValid)
  {
    return;
  }
  // calculate the image limits
  height = theCameraInfo.height - 3;
  horizon = std::max(2, (int) theImageCoordinateSystem.origin.y());
  horizon = std::min(horizon, height);
  right = theCameraInfo.width - 3;

  fromBallSpots(ballPercept);
  scaleOutput(ballPercept);

  // set ball radius
  ballPercept.radiusOnField = theFieldDimensions.ballRadius;

  PLOT("module:BallPerceptor:angle", toDegrees(ballPercept.relativePositionOnField.angle()));
}

void BallPerceptor::fromBallSpots(BallPercept& ballPercept)
{
  std::vector<BallSpot> ballSpots = theBallSpots.ballSpots;
  std::sort(ballSpots.begin(), ballSpots.end(), ballSpotComparator);

  ballPercept.status = BallPercept::notSeen;

  for(const BallSpot& ballSpot : ballSpots)
  {
    CROSS2("module:BallPerceptor:ballSpot",
           ballSpot.position.x(), ballSpot.position.y(),
           1, 1, Drawings::solidPen, ColorRGBA::blue);

    Vector2f prevPositionInImage = ballPercept.positionInImage;
    ballPercept.positionInImage = ballSpot.position.cast<float>();
    BallPercept::Status status = analyzeBallSpot(ballSpot, ballPercept);
    if(status == BallPercept::seen || status > ballPercept.status)
      ballPercept.status = status;
    else
      ballPercept.positionInImage = prevPositionInImage;
    if(status == BallPercept::seen)
    {
      CROSS2("module:BallPerceptor:ballSpot",
             ballSpot.position.x(), ballSpot.position.y(),
             1, 1, Drawings::solidPen, ColorRGBA::green);
      return;
    }
  }
}

BallPercept::Status BallPerceptor::analyzeBallSpot(const BallSpot& ballSpot, BallPercept& ballPercept)
{
  return !checkNoNoise(ballSpot) ? BallPercept::checkNoNoise :
         !checkBallSpot(ballSpot) ? BallPercept::checkBallSpot :
         !searchBallPoints(ballSpot) ? BallPercept::searchBallPoints :
         !checkBallPoints() ? BallPercept::checkBallPoints :
         !calculateBallInImage(ballPercept) ? BallPercept::calculateBallInImage :
         !checkBallInImage(ballPercept) ? BallPercept::checkBallInImage :
         !calculateBallOnField(ballPercept) ? BallPercept::calculateBallOnField :
         !checkBallOnField(ballPercept) ? BallPercept::checkBallOnField :
         !checkJersey() ? BallPercept::checkJersey :
         BallPercept::seen;
}

bool BallPerceptor::checkNoNoise(const BallSpot& ballSpot) const
{
  unsigned int skipped;
  int lower, upper;
  int left, right;

  // find upper/lower => middle vertical
  lower = upper = ballSpot.position.y();
  skipped = 0;
  while(lower <= height && skipped < orangeSkipping)
  {
    if(theColorTable[getPixel(lower, ballSpot.position.x())].is(ColorClasses::orange))
      skipped = 0;
    else
      skipped++;
    lower++;
  }
  lower -= skipped;

  skipped = 0;
  while(upper >= horizon && skipped < orangeSkipping)
  {
    if(theColorTable[getPixel(upper, ballSpot.position.x())].is(ColorClasses::orange))
      skipped = 0;
    else
      skipped++;
    upper--;
  }
  upper += skipped + 1;

  // find left/right => middle horizontal
  int redCounter = 0;
  left = right = ballSpot.position.x();
  skipped = 0;
  while(left >= this->left && skipped < orangeSkipping)
  {
    if(theColorTable[getPixel(ballSpot.position.y(), left)].is(ColorClasses::orange))
      skipped = 0;
    else
    {
      skipped++;

      if(theColorTable[getPixel(ballSpot.position.y(), left)].is(ColorClasses::red))
        redCounter++;
    }
    left--;
  }
  left += skipped + 1;
  skipped = 0;
  while(right <= this->right && skipped < orangeSkipping)
  {
    if(theColorTable[getPixel(ballSpot.position.y(), right)].is(ColorClasses::orange))
      skipped = 0;
    else
    {
      skipped++;

      if(theColorTable[getPixel(ballSpot.position.y(), right)].is(ColorClasses::red))
        redCounter++;
    }
    right++;
  }
  right -= skipped;

  // draw the scan lines
  LINE2("module:BallPerceptor:ballSpotScanLines",
        ballSpot.position.x(), lower,
        ballSpot.position.x(), upper,
        1, Drawings::solidPen, ColorRGBA::blue);
  LINE2("module:BallPerceptor:ballSpotScanLines",
        left, ballSpot.position.y(),
        right, ballSpot.position.y(),
        1, Drawings::solidPen, ColorRGBA::blue);

  const unsigned int width  = right - left;
  const unsigned int height = lower - upper;

  return width >= minBallSpotSize &&
         height >= minBallSpotSize;
}

bool BallPerceptor::checkBallSpot(const BallSpot& ballSpot)
{
  // calculate an approximation of the radius based on bearing distance of the ball spot
  const Vector2i& spot = ballSpot.position;
  Vector2f correctedStart = theImageCoordinateSystem.toCorrected(spot);
  Vector3f cameraToStart(theCameraInfo.focalLength, theCameraInfo.opticalCenter.x() - correctedStart.x(), theCameraInfo.opticalCenter.y() - correctedStart.y());
  Vector3f unscaledField = theCameraMatrix.rotation * cameraToStart;
  if(unscaledField.z() >= 0.f)
  {
    return false; // above horizon
  }
  const float scaleFactor = (theCameraMatrix.translation.z() - theFieldDimensions.ballRadius) / unscaledField.z();
  cameraToStart *= scaleFactor;
  unscaledField *= scaleFactor;
  if(unscaledField.topRows(2).squaredNorm() > sqrMaxBallDistance)
  {
    return false; // too far away
  }
  cameraToStart.y() += cameraToStart.y() > 0 ? -theFieldDimensions.ballRadius : theFieldDimensions.ballRadius;
  cameraToStart /= scaleFactor;
  approxRadius1 = std::abs(theCameraInfo.opticalCenter.x() - cameraToStart.y() - correctedStart.x());

  return true;
}

bool BallPerceptor::searchBallPoints(const BallSpot& ballSpot)
{
  Vector2i start = ballSpot.position;
  const float approxDiameter = approxRadius1 * clippingApproxRadiusScale + clippingApproxRadiusPixelBonus;
  int halfApproxRadius = int(approxRadius1 * 0.5f);
  COMPLEX_DRAWING("module:BallPerceptor:image") drawBall(start, approxDiameter, 0x6a);
  // try to improve the start point
  int maxColorDistance = scanMaxColorDistance;
  int resolutionWidth = theCameraInfo.width;
  int resolutionHeight = theCameraInfo.height;
  startPixel = getPixel(start.y(), start.x());
  Vector2i preScanPoints[4] =
  {
    start + Vector2i(halfApproxRadius, halfApproxRadius),
    start + Vector2i(-halfApproxRadius, -halfApproxRadius),
    start + Vector2i(halfApproxRadius, -halfApproxRadius),
    start + Vector2i(-halfApproxRadius, halfApproxRadius)
  };
  bool preScanResults[4];
  for(int i = 0; i < 4; ++i)
  {
    if(preScanPoints[i].x() < 0 || preScanPoints[i].x() >= resolutionWidth ||
       preScanPoints[i].y() < 0 || preScanPoints[i].y() >= resolutionHeight)
    {
      i -= i % 2;
      preScanResults[i++] = false;
      ASSERT(i < 4);
      preScanResults[i] = false;
    }
    else
    {
      const Image::Pixel pixel = getPixel(preScanPoints[i].y(), preScanPoints[i].x());
      preScanResults[i] = std::abs((int) startPixel.cb - pixel.cb) + std::abs((int) startPixel.cr - pixel.cr) < maxColorDistance;
    }
  }
  if(preScanResults[0] != preScanResults[1] && preScanResults[2] != preScanResults[3])
  {
    start = Vector2i::Zero();
    if(preScanResults[0])
    {
      start += preScanPoints[0];
    }
    else
    {
      start += preScanPoints[1];
    }
    if(preScanResults[2])
    {
      start += preScanPoints[2];
    }
    else
    {
      start += preScanPoints[3];
    }
    start /= 2;
  }
  else if(preScanResults[0] != preScanResults[1])
  {
    start = preScanResults[0] ? preScanPoints[0] : preScanPoints[1];
  }
  else if(preScanResults[2] != preScanResults[3])
  {
    start = preScanResults[2] ? preScanPoints[2] : preScanPoints[3];
  }
  COMPLEX_DRAWING("module:BallPerceptor:image") drawBall(start, approxDiameter, 0xff);
  // prepare scans
  totalPixelCount = 0;
  totalCbSum = 0;
  totalCrSum = 0;
  // vertical scan
  if(!searchBallPoint(start, startPixel, Vector2i(0, 1), approxDiameter, ballPoints[0]) ||
     !searchBallPoint(start, startPixel, Vector2i(0, -1), approxDiameter, ballPoints[4]))
  {
    return false;
  }
  if(ballPoints[0].atBorder && ballPoints[4].atBorder)
  {
    return false; // too large
  }
  else if(ballPoints[0].atBorder)
  {
    start.y() = ballPoints[4].point.y() + int(approxRadius1);
    if(start.y() > ballPoints[0].point.y() - 1)
    {
      start.y() = ballPoints[0].point.y() - 1;
    }
  }
  else if(ballPoints[4].atBorder)
  {
    start.y() = ballPoints[0].point.y() - int(approxRadius1);
    if(start.y() < ballPoints[4].point.y() + 1)
    {
      start.y() = ballPoints[4].point.y() + 1;
    }
  }
  else
  {
    start.y() = (ballPoints[0].point.y() + ballPoints[4].point.y()) / 2;
  }
  // horizontal scan
  if(!searchBallPoint(start, startPixel, Vector2i(1, 0), approxDiameter, ballPoints[2]) ||
     !searchBallPoint(start, startPixel, Vector2i(-1, 0), approxDiameter, ballPoints[6]))
  {
    return false;
  }
  if(ballPoints[2].atBorder && ballPoints[6].atBorder)
  {
    return false; // too large
  }
  else if(ballPoints[2].atBorder)
  {
    start.x() = ballPoints[6].point.x() + int(approxRadius1);
    if(start.x() > ballPoints[2].point.x() - 1)
    {
      start.x() = ballPoints[2].point.x() - 1;
    }
  }
  else if(ballPoints[6].atBorder)
  {
    start.x() = ballPoints[2].point.x() - int(approxRadius1);
    if(start.x() < ballPoints[6].point.x() + 1)
    {
      start.x() = ballPoints[6].point.x() + 1;
    }
  }
  else
  {
    start.x() = (ballPoints[2].point.x() + ballPoints[6].point.x()) / 2;
  }
  approxCenter2 = start;
  // maybe repeat vertical and horizontal scans
  int skipArea = std::min(ballPoints[0].point.y() - ballPoints[4].point.y(), ballPoints[2].point.x() - ballPoints[6].point.x()) / 4;
  float maxLength = approxDiameter - skipArea;
  if(abs(start.x() - ballPoints[0].start.x()) > halfApproxRadius)
  {
    if(!searchBallPoint(start + Vector2i(0, skipArea), startPixel, Vector2i(0, 1), maxLength, ballPoints[0]) ||
       !searchBallPoint(start + Vector2i(0, -skipArea), startPixel, Vector2i(0, -1), maxLength, ballPoints[4]))
    {
      return false;
    }
  }
  if(abs(start.y() - ballPoints[2].start.y()) > halfApproxRadius)
  {
    if(!searchBallPoint(start + Vector2i(skipArea, 0), startPixel, Vector2i(1, 0), maxLength, ballPoints[2]) ||
       !searchBallPoint(start + Vector2i(-skipArea, 0), startPixel, Vector2i(-1, 0), maxLength, ballPoints[6]))
    {
      return false;
    }
  }
  // diagonal scans
  skipArea = std::min(ballPoints[0].point.y() - ballPoints[4].point.y(), ballPoints[2].point.x() - ballPoints[6].point.x()) / 4;
  maxLength = approxDiameter - 1.41421356f * skipArea;
  if(!searchBallPoint(start + Vector2i(skipArea, skipArea), startPixel, Vector2i(1, 1), maxLength, ballPoints[1]) ||
     !searchBallPoint(start + Vector2i(-skipArea, -skipArea), startPixel, Vector2i(-1, -1), maxLength, ballPoints[5]) ||
     !searchBallPoint(start + Vector2i(skipArea, -skipArea), startPixel, Vector2i(1, -1), maxLength, ballPoints[3]) ||
     !searchBallPoint(start + Vector2i(-skipArea, skipArea), startPixel, Vector2i(-1, 1), maxLength, ballPoints[7]))
  {
    return false;
  }
  // improve ball points
  if(totalPixelCount == 0)
  {
    return false;
  }
  int cbAvg = totalCbSum / totalPixelCount;
  int crAvg = totalCrSum / totalPixelCount;
  for(unsigned j = 0; j < sizeof(ballPoints) / sizeof(*ballPoints); ++j)
  {
    BallPoint& ballPoint = ballPoints[j];
    const Vector2i& step = ballPoint.step;
    Vector2i pos = ballPoint.point;
    int i = 0;
    for(; i < (int) refineMaxPixelCount; ++i)
    {
      pos += step;
      if(pos.x() < 0 || pos.x() >= resolutionWidth ||
         pos.y() < 0 || pos.y() >= resolutionHeight)
      {
        break;
      }
      if(abs(getPixel(pos.y(), pos.x()).cb - cbAvg) + abs(getPixel(pos.y(), pos.x()).cr - crAvg) > (int) refineMaxColorDistance)
      {
        break;
      }
    }
    if(i)
    {
      ballPoint.point = pos - step;
    }
    ballPoint.pointf = ballPoint.point.cast<float>();
  }
  return true;
}

void BallPerceptor::drawBall(const Vector2i& pos, float approxDiameter, unsigned char opacity) const
{
  CROSS2("module:BallPerceptor:image", pos.x(), pos.y(), 2, 1, Drawings::solidPen, ColorRGBA(0xff, 0, 0, opacity));
  CIRCLE2("module:BallPerceptor:image", pos.x(), pos.y(), approxDiameter, 1, Drawings::solidPen, ColorRGBA(0xff, 0, 0, opacity), Drawings::noBrush, ColorRGBA());
}

bool BallPerceptor::searchBallPoint(const Vector2i& start, Image::Pixel startPixel, const Vector2i& step, float maxLength, BallPoint& ballPoint)
{
  ASSERT(step.x() == 0 || step.x() == 1 || step.x() == -1);
  ASSERT(step.y() == 0 || step.y() == 1 || step.y() == -1);
  Vector2i pos = start;
  Vector2i lastValidPos = pos;
  unsigned int overtime = 0;
  int cbSum = startPixel.cb;
  int crSum = startPixel.cr;
  int pixelCount = 1;
  int cb, cr;
  ballPoint.step = step;
  ballPoint.atBorder = false;
  ballPoint.start = start;
  int maxColorDistance = scanMaxColorDistance;
  unsigned int maxOvertime = scanPixelTolerance;
  int resolutionWidth = theCameraInfo.width;
  int resolutionHeight = theCameraInfo.height;
  int stepAbs1024 = (step.x() == 0 || step.y() == 0) ? 1024 : 1448; // 1448 = sqrt(2) * 1024
  int maxAbs1024 = int(maxLength * 1024.f); // + 1024;
  int length1024 = 0;
  for(;;)
  {
    pos += step;
    length1024 += stepAbs1024;
    if(length1024 > maxAbs1024)
    {
      if(overtime > 0)
      {
        break;
      }
      LINE2("module:BallPerceptor:image", start.x(), start.y(), pos.x(), pos.y(), 1, Drawings::solidPen, ColorRGBA(0, 0, 0xff, 0x70));
      return false;
    }

    if(pos.x() < 0 || pos.x() >= resolutionWidth ||
       pos.y() < 0 || pos.y() >= resolutionHeight)
    {
      ballPoint.atBorder = true;
      break;
    }

    cb = getPixel(pos.y(), pos.x()).cb;
    cr = getPixel(pos.y(), pos.x()).cr;
    if(abs(cb - cbSum / pixelCount) + abs(cr - crSum / pixelCount) > maxColorDistance)
    {
      if(overtime == 0)
      {
        lastValidPos = pos - step;
      }
      ++overtime;
      if(overtime <= maxOvertime)
      {
        continue;
      }
      break;
    }
    cbSum += cb;
    crSum += cr;
    ++pixelCount;
    overtime = 0;
  }

  ballPoint.point = lastValidPos;

  if(pixelCount > 0)
  {
    --pixelCount;
    cbSum -= startPixel.cb;
    crSum -= startPixel.cr;
    totalPixelCount += pixelCount;
    totalCbSum += cbSum;
    totalCrSum += crSum;
  }
  LINE2("module:BallPerceptor:image", ballPoint.start.x(), ballPoint.start.y(), ballPoint.point.x(), ballPoint.point.y(), 1, Drawings::solidPen, ColorRGBA(0, 0, 0xff));
  return true;
}

bool BallPerceptor::checkBallPoints()
{
  // find "valid" ball points
  validBallPoints = 0;
  static const int countOfBallPoints = sizeof(ballPoints) / sizeof(*ballPoints);
  for(int i = 0; i < countOfBallPoints; ++i)
  {
    BallPoint& ballPoint(ballPoints[i]);
    ballPoint.isValid = !ballPoint.atBorder && ballPoint.point != ballPoint.start;
    if(ballPoint.isValid)
    {
      ++validBallPoints;
      DOT2("module:BallPerceptor:image", ballPoint.point.x(), ballPoint.point.y(), ColorRGBA(0, 0xff, 0, 0x70), ColorRGBA(0, 0xff, 0, 0x70));
    }
  }
  if(validBallPoints < 4)
  {
    return false;
  }

  // find duplicated ball points (may occur in small balls)
  for(int i = 0; i < countOfBallPoints; ++i)
  {
    BallPoint& ballPoint(ballPoints[i]);
    if(ballPoint.isValid)
    {
      const BallPoint& nextBallPoint(ballPoints[(i + 1) % countOfBallPoints]);
      if(nextBallPoint.isValid && ballPoint.point == nextBallPoint.point)
      {
        ballPoint.isValid = false;
        --validBallPoints;
      }
    }
  }
  if(validBallPoints < 4)
    return false;

  // drop mismatching ball points
  while(validBallPoints > 4)
  {
    Vector2f preCenter;
    float preRadius;
    if(!getBallFromBallPoints(preCenter, preRadius))
    {
      return false;
    }

    float minDist = 0;
    BallPoint* minDistBallPoint = 0;
    for(int i = 0; i < countOfBallPoints; ++i)
    {
      BallPoint& ballPoint(ballPoints[i]);
      if(ballPoint.isValid)
      {
        float dist = (ballPoint.pointf - preCenter).squaredNorm();
        if(!minDistBallPoint || dist < minDist)
        {
          minDist = dist;
          minDistBallPoint = &ballPoint;
        }
      }
    }
    minDistBallPoint->isValid = false;
    --validBallPoints;

    if((preRadius - (sqrt(minDist) + 2.f)) / preRadius < 0.1f)
    {
      break;
    }
  }

  COMPLEX_DRAWING("module:BallPerceptor:image")
  {
    for(BallPoint* ballPoint = ballPoints, * end = ballPoints + sizeof(ballPoints) / sizeof(*ballPoints); ballPoint < end; ++ballPoint)
      if(ballPoint->isValid)
      {
        DOT2("module:BallPerceptor:image", ballPoint->point.x(), ballPoint->point.y(), ColorRGBA(0, 0xff, 0), ColorRGBA(0, 0, 0));
      }
  }

  return true;
}

bool BallPerceptor::getBallFromBallPoints(Vector2f& center, float& radius) const
{
  float Mx = 0, My = 0, Mxx = 0, Myy = 0, Mxy = 0, Mz = 0, Mxz = 0, Myz = 0;

  for(const BallPoint* ballPoint = ballPoints, * end = ballPoints + sizeof(ballPoints) / sizeof(*ballPoints); ballPoint < end; ++ballPoint)
    if(ballPoint->isValid)
    {
      float x = static_cast<float>(ballPoint->point.x());
      float y = static_cast<float>(ballPoint->point.y());
      float xx = x * x;
      float yy = y * y;
      float z = xx + yy;
      Mx += x;
      My += y;
      Mxx += xx;
      Myy += yy;
      Mxy += x * y;
      Mz += z;
      Mxz += x * z;
      Myz += y * z;
    }

  // Construct and solve matrix
  // Result will be center and radius of ball in theImage.
  Eigen::Matrix3d M;
  M << Mxx, Mxy, Mx,
  Mxy, Myy, My,
  Mx,  My,  validBallPoints;

  Eigen::Matrix3d Minv;
  bool invertible;
  M.computeInverseWithCheck(Minv, invertible);
  if(!invertible)
  {
    return false;
  }
  Eigen::Vector3d BCD = Minv * Eigen::Vector3d(-Mxz, -Myz, -Mz);

  center.x() = static_cast<float>(BCD.x() * -0.5);
  center.y() = static_cast<float>(BCD.y() * -0.5);
  float radicand = static_cast<float>(BCD.x() * BCD.x() / 4.0 + BCD.y() * BCD.y() / 4.0 - BCD.z());
  if(radicand <= 0.0f)
  {
    return false;
  }
  radius = std::sqrt(radicand);
  return true;
}

bool BallPerceptor::calculateBallInImage(BallPercept& ballPercept) const
{
  return getBallFromBallPoints(ballPercept.positionInImage, ballPercept.radiusInImage);
}

bool BallPerceptor::checkBallInImage(BallPercept& ballPercept) const
{
  const Vector2f center = ballPercept.positionInImage;
  const float radius = ballPercept.radiusInImage;

  // check if the ball covers approxCenter2
  if((Vector2i(int(center.x() + 0.5), int(center.y() + 0.5)) - approxCenter2).squaredNorm() > sqr(radius))
  {
    return false;
  }

  // check ball radius
  if(radius - checkMaxRadiusPixelBonus > approxRadius1 * checkMaxRadiusDifference || radius + checkMinRadiusPixelBonus < approxRadius1 * checkMinRadiusDifference)
  {
    return false;
  }

  float noBallRadius = radius * checkOutlineRadiusScale + checkOutlineRadiusPixelBonus;
  CIRCLE2("module:BallPerceptor:image", center.x(), center.y(), noBallRadius, 1, Drawings::solidPen, ColorRGBA(0xff, 0xff, 0xff), Drawings::noBrush, ColorRGBA());
  Vector2i center32(int(center.x() * 32.f), int(center.y() * 32.f));
  int noBallRadius32 = int(noBallRadius * 32.f);
  int noBallDRadius32 = int(noBallRadius * 32.f / 1.41421356f);
  Vector2i noBallPoints32[8 + 4 * 4] =
  {
    Vector2i(center32.x() + noBallRadius32, center32.y()),
    Vector2i(center32.x() - noBallRadius32, center32.y()),
    Vector2i(center32.x(), center32.y() + noBallRadius32),
    Vector2i(center32.x(), center32.y() - noBallRadius32),
    Vector2i(center32.x() + noBallDRadius32, center32.y() + noBallDRadius32),
    Vector2i(center32.x() - noBallDRadius32, center32.y() - noBallDRadius32),
    Vector2i(center32.x() + noBallDRadius32, center32.y() - noBallDRadius32),
    Vector2i(center32.x() - noBallDRadius32, center32.y() + noBallDRadius32),
  };
  int noBallPointCount = 8;
  int resolutionWidth = theCameraInfo.width;
  int resolutionHeight = theCameraInfo.height;
  int borderDists[2] = {0, 3};
  if(center32.x() + noBallRadius32 >= resolutionWidth * 32)
  {
    for(int i = 0; i < 2; ++i)
    {
      int x = resolutionWidth - 1 - borderDists[i];
      float d = std::sqrt(float(sqr(noBallRadius) - sqr(x - center.x())));
      noBallPoints32[noBallPointCount++] = Vector2i(x * 32, int((center.y() + d) * 32.f));
      noBallPoints32[noBallPointCount++] = Vector2i(x * 32, int((center.y() - d) * 32.f));
    }
  }
  if(center32.y() + noBallRadius32 >= resolutionHeight * 32)
  {
    for(int i = 0; i < 2; ++i)
    {
      int y = resolutionHeight - 1 - borderDists[i];
      float d = std::sqrt(float(sqr(noBallRadius) - sqr(y - center.y())));
      noBallPoints32[noBallPointCount++] = Vector2i(int((center.x() + d) * 32.f), y * 32);
      noBallPoints32[noBallPointCount++] = Vector2i(int((center.x() - d) * 32.f), y * 32);
    }
  }
  if(center32.x() - noBallRadius32 < 0)
  {
    for(int i = 0; i < 2; ++i)
    {
      int x = borderDists[i];
      float d = std::sqrt(float(sqr(noBallRadius) - sqr(x - center.x())));
      noBallPoints32[noBallPointCount++] = Vector2i(x * 32, int((center.y() + d) * 32.f));
      noBallPoints32[noBallPointCount++] = Vector2i(x * 32, int((center.y() - d) * 32.f));
    }
  }
  if(center32.y() - noBallRadius32 < 0)
  {
    for(int i = 0; i < 2; ++i)
    {
      int y = borderDists[i];
      float d = std::sqrt(float(sqr(noBallRadius) - sqr(y - center.y())));
      noBallPoints32[noBallPointCount++] = Vector2i(int((center.x() + d) * 32.f), y * 32);
      noBallPoints32[noBallPointCount++] = Vector2i(int((center.x() - d) * 32.f), y * 32);
    }
  }
  int maxColorDistance = scanMaxColorDistance;
  int cbStart = totalCbSum / totalPixelCount, crStart = totalCrSum / totalPixelCount;
  for(int i = 0; i < noBallPointCount; ++i)
  {
    Vector2i pos(noBallPoints32[i] / 32);
    if(pos.x() < 0 || pos.x() >= resolutionWidth ||
       pos.y() < 0 || pos.y() >= resolutionHeight)
    {
      continue;
    }
    DOT2("module:BallPerceptor:image", pos.x(), pos.y(), ColorRGBA(0xff, 0xff, 0xff), ColorRGBA(0, 0, 0));

    if(std::abs((int) getPixel(pos.y(), pos.x()).cb - cbStart) + std::abs((int) getPixel(pos.y(), pos.x()).cr - crStart) <= maxColorDistance)
    {
      return false;
    }
  }
  return true;
}

bool BallPerceptor::calculateBallOnField(BallPercept& ballPercept) const
{
  const Vector2f correctedCenter = theImageCoordinateSystem.toCorrected(ballPercept.positionInImage);
  Vector3f cameraToBall(theCameraInfo.focalLength, theCameraInfo.opticalCenter.x() - correctedCenter.x(), theCameraInfo.opticalCenter.y() - correctedCenter.y());
  cameraToBall.normalize(theFieldDimensions.ballRadius * theCameraInfo.focalLength / ballPercept.radiusInImage);
  Vector3f rotatedCameraToBall = theCameraMatrix.rotation * cameraToBall;
  const Vector3f sizeBasedCenterOnField = theCameraMatrix.translation + rotatedCameraToBall;
  const Vector3f bearingBasedCenterOnField =  theCameraMatrix.translation - rotatedCameraToBall * ((theCameraMatrix.translation.z() - theFieldDimensions.ballRadius) / rotatedCameraToBall.z());

  CIRCLE2("module:BallPerceptor:field", sizeBasedCenterOnField.x(), sizeBasedCenterOnField.y(), theFieldDimensions.ballRadius, 1, Drawings::solidPen, ColorRGBA(0, 0, 0xff), Drawings::noBrush, ColorRGBA());
  CIRCLE2("module:BallPerceptor:field", bearingBasedCenterOnField.x(), bearingBasedCenterOnField.y(), theFieldDimensions.ballRadius, 1, Drawings::solidPen, ColorRGBA(0xff, 0, 0), Drawings::noBrush, ColorRGBA());

  if(rotatedCameraToBall.z() < 0)
  {
    ballPercept.relativePositionOnField.x() = bearingBasedCenterOnField.x();
    ballPercept.relativePositionOnField.y() = bearingBasedCenterOnField.y();
  }
  else
  {
    ballPercept.relativePositionOnField.x() = sizeBasedCenterOnField.x();
    ballPercept.relativePositionOnField.y() = sizeBasedCenterOnField.y();
  }
  return true;
}

bool BallPerceptor::checkBallOnField(BallPercept& ballPercept) const
{
  // Not sure about self-localization => Do not use it to check ball position
  if(theRobotPose.validity < 1.f)
  {
    return true;
  }
  // Check, if the computed ball position is still on the carpet
  Pose2f currentRobotPose = theRobotPose + theOdometer.odometryOffset;
  Vector2f absoluteBallPosition = (currentRobotPose * ballPercept.relativePositionOnField);
  return ((fabs(absoluteBallPosition.x()) < theFieldDimensions.xPosOpponentFieldBorder + 300.f) &&
          (fabs(absoluteBallPosition.y()) < theFieldDimensions.yPosLeftFieldBorder + 300.f));
}

bool BallPerceptor::checkJersey() const
{
  Vector2i topLeft(Image::maxResolutionWidth, Image::maxResolutionHeight);
  Vector2i bottomRight(0, 0);
  for(const BallPoint& point : ballPoints)
  {
    bottomRight.x() = std::max(bottomRight.x(), point.point.x());
    topLeft.y() = std::min(topLeft.y(), point.point.y());
    topLeft.x() = std::min(topLeft.x(), point.point.x());
    bottomRight.y() = std::max(bottomRight.y(), point.point.y());
  }

  //clip at image border
  bottomRight.x() = std::min(bottomRight.x(), theCameraInfo.width - 1);
  bottomRight.y() = std::min(bottomRight.y(), theCameraInfo.height - 1);
  topLeft.x() = std::max(topLeft.x(), 0);
  topLeft.y() = std::max(topLeft.y(), 0);
  RECTANGLE3("module:BallPerceptor:ballRectangle", topLeft.x(), topLeft.y(), bottomRight.x(), bottomRight.y(), 1, Drawings::solidPen, ColorRGBA::white);
  unsigned int count = 0;
  //calculate red percentage inside rectangle
  for(int y = topLeft.y(); y <= bottomRight.y(); ++y)
  {
    for(int x = topLeft.x(); x <= bottomRight.x(); ++x)
    {
      if(theColorTable[getPixel(y, x)].is(ColorClasses::red))
      {
        ++count;
      }
    }
  }

  const int totalCount = (bottomRight.x() - topLeft.x() + 1) * (bottomRight.y() - topLeft.y() + 1);
  const float percent = (float) count / (float) totalCount;
  return percent <= percentRedNoise;
}
