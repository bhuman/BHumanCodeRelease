/**
 * @author Alexis Tsogias
 */

#include "FieldBoundaryProvider.h"
#include "Platform/BHAssert.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Transformation.h"

#include <algorithm>

using std::vector;

MAKE_MODULE(FieldBoundaryProvider, perception)

void FieldBoundaryProvider::update(FieldBoundary& fieldBoundary)
{
  DECLARE_DEBUG_DRAWING("module:FieldBoundaryProvider:lowerCamSpots", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:FieldBoundaryProvider:lowerCamSpotsInterpolated", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:FieldBoundaryProvider:greaterPenaltyPoint", "drawingOnImage");

  // Clear vectors.
  fieldBoundary.boundarySpots.clear();
  fieldBoundary.convexBoundary.clear();
  convexBoundaryCandidates.clear();
  fieldBoundary.boundaryInImage.clear();
  lowerCamSpotsInImage.clear();
  lowerCamSpotsInterpol.clear();

  if(theCameraMatrix.isValid)
  {
    if(theCameraInfo.camera == CameraInfo::Camera::upper)
    {
      fieldBoundary.boundaryOnField.clear();
      fieldBoundary.isValid = false;
    }
    else
    {
      lowerCamConvexHullOnField.clear();
      validLowerCamSpots = true;
    }

    if(theCameraInfo.camera == CameraInfo::Camera::upper && validLowerCamSpots)
    {
      handleLowerCamSpots();
    }
    findBoundarySpots(fieldBoundary);
    bool valid = cleanupBoundarySpots(fieldBoundary.boundarySpots);
    calcBoundaryCandidates(fieldBoundary.boundarySpots);
    findBestBoundary(convexBoundaryCandidates, fieldBoundary.boundarySpots, fieldBoundary.convexBoundary);

    if(theCameraInfo.camera == CameraInfo::Camera::upper)
    {
      fieldBoundary.isValid = valid;
    }
  }
  else
  {
    lowerCamConvexHullOnField.clear();

    invalidateBoundary(fieldBoundary);

    draw();

    return;
  }
  bool valid = true;
  if(theCameraInfo.camera == CameraInfo::Camera::upper)
  {
    // Project the convexBoundary to the field.
    for(const Vector2i& p : fieldBoundary.convexBoundary)
    {
      Vector2f pField;
      valid &= Transformation::imageToRobot(p.x(), p.y(), theCameraMatrix, theCameraInfo, pField);
      fieldBoundary.boundaryOnField.push_back(pField);
    }
    fieldBoundary.isValid = valid;
    fieldBoundary.boundaryInImage = fieldBoundary.convexBoundary;
  }
  else
  {
    // Project the convex Hull to the field.
    for(const Vector2i& p : fieldBoundary.convexBoundary)
    {
      Vector2f pField;
      valid &= Transformation::imageToRobot(p.x(), p.y(), theCameraMatrix, theCameraInfo, pField);
      lowerCamConvexHullOnField.push_back(pField);
    }

    // Update fieldboundary from last upper image so it is shown correctly on the lower image.
    InImage projectedPoints;
    projectedPoints.reserve(fieldBoundary.boundaryOnField.size());
    for(Vector2f& point : fieldBoundary.boundaryOnField)
    {
      point = theOdometer.odometryOffset.inverse() * point;

      Vector2f pImg;
      valid &= Transformation::robotToImage(point, theCameraMatrix, theCameraInfo, pImg);
      projectedPoints.emplace_back(static_cast<int>(pImg.x() + 0.5f), static_cast<int>(pImg.y() + 0.5f));
    }

    std::sort(projectedPoints.begin(), projectedPoints.end(), [](const Vector2i & a, const Vector2i & b)
    {
      return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
    });
    fieldBoundary.boundaryInImage.clear();
    getUpperConvexHull(projectedPoints, fieldBoundary.boundaryInImage);
  }

  if(!valid)
    invalidateBoundary(fieldBoundary);

  draw();
}

void FieldBoundaryProvider::handleLowerCamSpots()
{
  InImage tmpLowerCamSpotsInImage;

  for(Vector2f& p : lowerCamConvexHullOnField)
  {
    Vector2f pImg;
    p = Eigen::Rotation2Df(-theOdometer.odometryOffset.rotation) * p;
    p -= theOdometer.odometryOffset.translation;
    if(Transformation::robotToImage(p, theCameraMatrix, theCameraInfo, pImg))
      tmpLowerCamSpotsInImage.push_back(Vector2i(static_cast<int>(std::floor(pImg.x() + 0.5f)), static_cast<int>(std::floor(pImg.y() + 0.5f))));
  }

  std::sort(tmpLowerCamSpotsInImage.begin(), tmpLowerCamSpotsInImage.end(), [](Vector2i a, Vector2i b)
  {
    return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
  });

  // Remove lower spots on the same scanline...
  if(tmpLowerCamSpotsInImage.size() >= 2)
  {
    lowerCamSpotsInImage.push_back(tmpLowerCamSpotsInImage.front());
    const Vector2i* prev = &tmpLowerCamSpotsInImage.front();
    for(const Vector2i& current : tmpLowerCamSpotsInImage)
    {
      if(current.x() > prev->x())
      {
        lowerCamSpotsInImage.push_back(current);
        prev = &current;
      }
    }
  }

  // Draw Spots from last frame
  for(const Vector2i& p : lowerCamSpotsInImage)
  {
    DOT("module:FieldBoundaryProvider:lowerCamSpots", p.x(), p.y(), ColorRGBA::green, ColorRGBA::green);
  }
}

void FieldBoundaryProvider::findBoundarySpots(FieldBoundary& fieldBoundary)
{
  const unsigned height = theCameraInfo.height;

  Vector2f pointInImage;
  const float fieldDiagional = Vector2f(theFieldDimensions.boundary.x.getSize(), theFieldDimensions.boundary.y.getSize()).norm();
  if(!Transformation::robotWithCameraRotationToImage(Vector2f(fieldDiagional, 0), theCameraMatrix, theCameraInfo, pointInImage))
    return;

  const int highestPossiblePoint = static_cast<int>(pointInImage.y());

  // find y-coordinate where a point on the field is more than nonGreenPenaltyDistance meters away
  int yBound = findYInImageByDistance(nonGreenPenaltyDistance);
  const int bodyContourMargin = static_cast<int>(height * 0.05);
  int badSpots = 0;
  unsigned lowResStep = std::max(theColorScanlineRegionsVertical.lowResStep, 1u); // Workaround for old logs
  for(unsigned i = theColorScanlineRegionsVertical.lowResStart; i < theColorScanlineRegionsVertical.scanlines.size(); i += lowResStep)
  {
    const ColorScanlineRegionsVertical::Scanline& scanline = theColorScanlineRegionsVertical.scanlines[i];
    SpotAccumulator spot = {static_cast<int>(height), static_cast<int>(height), 0, 0};
    if(theCameraInfo.camera == CameraInfo::Camera::upper && lowerCamSpotsInImage.size() > 1 && validLowerCamSpots)
    {
      // TODO maybe also clip body contour
      spot.yStart = std::max(highestPossiblePoint, clipToBoundary(lowerCamSpotsInImage, scanline.x));
      Vector2i p(scanline.x, spot.yStart);
      lowerCamSpotsInterpol.push_back(p);
      DOT("module:FieldBoundaryProvider:lowerCamSpotsInterpolated", scanline.x, spot.yStart, ColorRGBA::black, ColorRGBA::black);
      spot.score = (spot.yStart > theCameraInfo.height) ? (height - spot.yStart) : 0; // FIXME score should be normalized by jumpsize
      spot.yMax = spot.yStart;
    }
    else
    {
      spot.yStart = height;
      theBodyContour.clipBottom(scanline.x, spot.yStart, height);
      if(spot.yStart < bodyContourMargin)
        ++badSpots;
      if(theColorScanlineRegionsVertical.scanlines.size() / lowResStep - badSpots < 3 && theCameraInfo.camera == CameraInfo::Camera::lower)
      {
        validLowerCamSpots = false;
        return;
      }
    }
    int penalty = nonGreenPenalty;
    int reward = 1;
    for(const ScanlineRegion& region : scanline.regions)
    {
      if(region.range.lower < yBound) // TODO review this
        penalty = nonGreenPenaltyGreater;
      int regionSize;
      if(region.range.upper > spot.yStart)
        continue;
      else if(region.range.upper <= spot.yStart && region.range.lower + 1 > spot.yStart)
        regionSize = (spot.yStart + 1) - region.range.upper;
      else
        regionSize = region.range.lower - region.range.upper;

      if(region.is(FieldColors::green))
        spot.score += reward * regionSize;
      else
        spot.score -= penalty * regionSize;

      if(spot.maxScore <= spot.score)
      {
        spot.maxScore = spot.score;
        spot.yMax = region.range.upper;
      }
    }
    if(theCameraInfo.camera == CameraInfo::Camera::lower)
    {
      int yEnd = spot.yMax + static_cast<int>(minGreenCount * 1.5);
      const PixelTypes::ColoredPixel* pImg = &theECImage.colored[spot.yMax][scanline.x];
      if(*pImg == FieldColors::green)
      {
        pImg += theECImage.colored.width;
        int greenCount = 1;
        for(int y = spot.yMax + 1; y < yEnd && y < theECImage.colored.height; ++y)
        {
          if(*pImg == FieldColors::green)
          {
            ++greenCount;
          }
          pImg += theECImage.colored.width;
        }
        if(greenCount >= minGreenCount) // line.yMax != line.yStart && TODO Add more stuff to prevent fuckups above shoulders
          fieldBoundary.boundarySpots.emplace_back(scanline.x, spot.yMax);
      }
    }
    else
      fieldBoundary.boundarySpots.emplace_back(scanline.x, spot.yMax);
  }
}

bool FieldBoundaryProvider::cleanupBoundarySpots(InImage& boundarySpots) const
{
  int height = theCameraInfo.height;

  // remove trailing point with y == height.
  while(boundarySpots.begin() != boundarySpots.end() && boundarySpots.back().y() == height)
    boundarySpots.erase(boundarySpots.end() - 1);

  // remove leading point with y == height.
  while(boundarySpots.begin() != boundarySpots.end() && boundarySpots.front().y() == height)
    boundarySpots.erase(boundarySpots.begin());

  // make sure that the boundary contains at least 2 points.
  if(boundarySpots.size() < 2)
  {
    boundarySpots.clear();
    boundarySpots.push_back(Vector2i(0, height));
    boundarySpots.push_back(Vector2i(theECImage.colored.width - 1, height));
    return false;
  }
  return true;
}

void FieldBoundaryProvider::calcBoundaryCandidates(InImage boundarySpots)
{
  const int maxIter = 20;

  for(int i = 0; i < maxIter && boundarySpots.size() >= 2; ++i)
  {
    convexBoundaryCandidates.emplace_back();
    InImage& boundaryCandidate = convexBoundaryCandidates.back();
    getUpperConvexHull(boundarySpots, boundaryCandidate);
    if(boundaryCandidate.size() >= 2)
    {
      int maxDist = 0;
      Vector2i* val;

      //Length of first Line segment
      int firstDist = std::abs((boundaryCandidate.begin() + 1)->y() - boundaryCandidate.front().y());
      //Length of last line segment
      int lastDist = std::abs((boundaryCandidate.end() - 2)->y() - boundaryCandidate.back().y());

      if(firstDist >= lastDist)
      {
        maxDist = firstDist;
        val = &boundaryCandidate.front();
      }
      else
      {
        maxDist = lastDist;
        val = &boundaryCandidate.back();
      }

      // erase the point with the maximum y-distance to its neighbors
      for(auto iter = boundaryCandidate.begin(); iter + 2 < boundaryCandidate.end(); ++iter)
      {
        int tmpDist = std::abs((iter + 1)->y() - iter->y()) + std::abs((iter + 2)->y() - (iter + 1)->y());
        if(tmpDist > maxDist)
        {
          maxDist = tmpDist;
          val = &*(iter + 1);
        }
      }
      for(auto iter = boundarySpots.begin(); iter < boundarySpots.end(); iter++)
      {
        if(*iter == *val)
        {
          boundarySpots.erase(iter);
          break;
        }
      }
    }
    else
      ASSERT(false);
  }
}

void FieldBoundaryProvider::findBestBoundary(const vector<InImage>& boundaryCandidates,
    const InImage& boundarySpots, InImage& boundary) const
{
  int spotsOnLine;
  int spotsNearLine;
  int score;
  int y;
  int maxScore = 0;
  const InImage* tmpBoundary = &boundaryCandidates.front();

  for(const InImage& boundarycandidate : boundaryCandidates)
  {
    spotsOnLine = 0;
    spotsNearLine = 0;
    for(const Vector2i& point : boundarySpots)
    {
      y = clipToBoundary(boundarycandidate, point.x());
      if(point.y() > y && point.y() - y < lowerBound)
        ++spotsNearLine;
      else if(point.y() < y && y - point.y() < upperBound)
        ++spotsNearLine;

      if(point.y() == y)
        ++spotsOnLine;
    }
    score = spotsOnLine + spotsNearLine;
    if(maxScore < score)
    {
      maxScore = score;
      tmpBoundary = &boundarycandidate;
    }
  }

  boundary = *tmpBoundary;
}

inline bool FieldBoundaryProvider::isLeftOf(const Vector2i& a, const Vector2i& b, const Vector2i& c) const
{
  return ((b.x() - a.x()) * (-c.y() + a.y()) - (c.x() - a.x()) * (-b.y() + a.y()) > 0);
}

void FieldBoundaryProvider::getUpperConvexHull(const InImage& boundary, InImage& hull) const
{
  ASSERT(boundary.size() > 1);

  //Andrew's Monotone Chain Algorithm to compute the upper hull
  const auto pmin = boundary.begin();
  const auto pmax = boundary.end() - 1;
  hull.push_back(*pmin);
  for(auto pi = pmin + 1; pi != pmax + 1; pi++)
  {
    if(!isLeftOf((*pmin), (*pmax), (*pi)) && pi != pmax)
      continue;

    while(hull.size() > 1)
    {
      const auto p1 = hull.end() - 1, p2 = hull.end() - 2;
      if(isLeftOf((*p1), (*p2), (*pi)))
        break;
      hull.pop_back();
    }
    hull.push_back(*pi);
  }
}

int FieldBoundaryProvider::clipToBoundary(const InImage& boundary, int x) const
{
  ASSERT(boundary.size() >= 2);

  const Vector2i* left = &boundary.front();
  const Vector2i* right = &boundary.back();

  if(x < left->x())
    right = &(*(boundary.begin() + 1));
  else if(x > right->x())
    left = &(*(boundary.end() - 2));
  else
  {
    for(const Vector2i& point : boundary)
    {
      if(point.x() == x)
        return point.y();
      else if(point.x() < x && point.x() > left->x())
        left = &point;
      else if(point.x() > x && point.x() < right->x())
        right = &point;
    }
  }

  float m = static_cast<float>(right->y() - left->y()) / static_cast<float>(right->x() - left->x());
  return static_cast<int>(static_cast<float>(x - left->x()) * m) + left->y();
}

int FieldBoundaryProvider::findYInImageByDistance(int distance) const
{
  const Vector2f pointOnField(static_cast<float>(nonGreenPenaltyDistance), 0);
  Vector2f pointInImage;
  int yBound;
  if(Transformation::robotWithCameraRotationToImage(pointOnField, theCameraMatrix, theCameraInfo, pointInImage))
    yBound = static_cast<int>(pointInImage.y() + 0.5f);
  else
    yBound = theCameraInfo.height;
  CROSS("module:FieldBoundaryProvider:greaterPenaltyPoint", theCameraInfo.width / 2, yBound,
        5, 5, Drawings::solidPen, ColorRGBA::black);
  return yBound;
}

void FieldBoundaryProvider::invalidateBoundary(FieldBoundary& fieldBoundary) const
{
  fieldBoundary.boundaryOnField.clear();

  fieldBoundary.convexBoundary.push_back(Vector2i(0, theCameraInfo.height));
  fieldBoundary.convexBoundary.push_back(Vector2i(theCameraInfo.width - 1, theCameraInfo.height));

  fieldBoundary.boundaryInImage = fieldBoundary.convexBoundary;

  fieldBoundary.boundaryOnField.push_back(Vector2f(0, 1));
  fieldBoundary.boundaryOnField.push_back(Vector2f(0, -1));

  fieldBoundary.isValid = false;
}

void FieldBoundaryProvider::draw() const
{
  int selected = -1;
  MODIFY("module:FieldBoundaryProvider:selectedCandidate", selected);

  DEBUG_DRAWING("module:FieldBoundaryProvider:boundaryCandidates", "drawingOnImage")
  {
    int num = static_cast<int>(convexBoundaryCandidates.size());
    float step = 255.0f / (num - 1);
    int pos = 0;
    for(const InImage& tmpBoundary : convexBoundaryCandidates)
    {
      const Vector2i* previ = nullptr;
      unsigned char colorMod = static_cast<unsigned char>(step * pos);
      ColorRGBA col = ColorRGBA(colorMod, colorMod, 255 - colorMod);
      if(pos == selected || selected < 0 || selected >= num)
      {
        for(const Vector2i& p : tmpBoundary)
        {
          DOT("module:FieldBoundaryProvider:boundaryCandidates", p.x(), p.y(), col, col);
          if(previ != nullptr)
          {
            LINE("module:FieldBoundaryProvider:boundaryCandidates", p.x(), p.y(), previ->x(), previ->y(), 1, Drawings::solidPen, col);
          }
          previ = &p;
        }
      }
      pos++;
    }
  }
}
