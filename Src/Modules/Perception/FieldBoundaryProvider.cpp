/**
* @author Alexis Tsogias
*/

#include <algorithm>
#include "FieldBoundaryProvider.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/Stopwatch.h"
#include "Platform/BHAssert.h"

using std::vector;

MAKE_MODULE(FieldBoundaryProvider, Perception)

void FieldBoundaryProvider::update(FieldBoundary& fieldBoundary)
{
  ASSERT(scanlienDistance > 0);

  DECLARE_DEBUG_DRAWING("module:FieldBoundary:LowerCamSpots", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:FieldBoundary:LowerCamSpotsInterpol", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:FieldBoundary:GreaterPenaltyPoint", "drawingOnImage");
  fieldBoundary.scanlineDistance = scanlienDistance;

  int horizon = static_cast<int>(theImageCoordinateSystem.origin.y);

  fieldBoundary.width = theImage.width;

  // Clear vectors.
  fieldBoundary.boundarySpots.clear();
  fieldBoundary.convexBoundary.clear();
  fieldBoundary.convexBoundaryCandidates.clear();
  fieldBoundary.boundaryInImage.clear();
  lowerCamSpotsInImage.clear();
  lowerCamSpostInterpol.clear();

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
    }

    horizon = std::max(0, horizon);

    if(theCameraInfo.camera == CameraInfo::Camera::upper)
    {
      handleLowerCamSpots();
    }
    findBundarySpots(fieldBoundary, horizon);
    bool valid = cleanupBoundarySpots(fieldBoundary.boundarySpots);;
    fieldBoundary.convexBoundaryCandidates = calcBoundaryCandidates(fieldBoundary.boundarySpots);
    findBestBoundary(fieldBoundary.convexBoundaryCandidates, fieldBoundary.boundarySpots, fieldBoundary.convexBoundary);

    if(theCameraInfo.camera == CameraInfo::Camera::upper)
    {
      fieldBoundary.isValid = valid;
    }
  }
  else
  {
    fieldBoundary.boundaryOnField.clear();
    lowerCamConvexHullOnField.clear();

    fieldBoundary.convexBoundary.push_back(Vector2<int>(0, theImage.height));
    fieldBoundary.convexBoundary.push_back(Vector2<int>(theImage.width - 1, theImage.height));

    fieldBoundary.boundaryInImage = fieldBoundary.convexBoundary;

    fieldBoundary.boundaryOnField.push_back(Vector2<float>(0, 1));
    fieldBoundary.boundaryOnField.push_back(Vector2<float>(0, -1));

    fieldBoundary.highestPoint = Vector2<int>(theImage.width / 2, theImage.height);

    fieldBoundary.isValid = false;
    return;
  }

  if(theCameraInfo.camera == CameraInfo::Camera::upper)
  {
    // Project the convexBoundary to the field.
    for(auto& p : fieldBoundary.convexBoundary)
    {
      Vector2<float> pField;
      Geometry::calculatePointOnField(p.x, p.y, theCameraMatrix, theCameraInfo, pField);
      fieldBoundary.boundaryOnField.push_back(pField);
    }

    fieldBoundary.boundaryInImage = fieldBoundary.convexBoundary;
  }
  else
  {
    // Project the convex Hull to the field.
    for(auto& p : fieldBoundary.convexBoundary)
    {
      Vector2<float> pField;
      Geometry::calculatePointOnField(p.x, p.y, theCameraMatrix, theCameraInfo, pField);
      lowerCamConvexHullOnField.push_back(pField);
    }

    // Update fieldboundary from last upper image so it is shown correctly on the lower image.
    // Do an odometry upadte on boundaryOnField
    for(auto& p : fieldBoundary.boundaryOnField)
    {
      Vector2<int> pImg;
      p.rotate(-theOdometer.odometryOffset.rotation);
      p -= theOdometer.odometryOffset.translation;
    }
    // update fieldBoundary.boundaryInImage
    for(auto& p : fieldBoundary.boundaryOnField)
    {
      Vector2<int> pImg;
      Geometry::calculatePointInImage(p, theCameraMatrix, theCameraInfo, pImg);
      fieldBoundary.boundaryInImage.push_back(pImg);
    }
    std::sort(fieldBoundary.boundaryInImage.begin(), fieldBoundary.boundaryInImage.end(), [](Vector2<int> a, Vector2<int> b)
    {
      return a.x < b.x || (a.x == b.x && a.y < b.y);
    });
    fieldBoundary.boundaryInImage = getUpperConvexHull(fieldBoundary.boundaryInImage);
  }

  // find the highest point
  Vector2<int>* heighest = &fieldBoundary.boundaryInImage.front();
  for(auto& spot : fieldBoundary.boundaryInImage)
  {
    if(spot.y < heighest->y)
    {
      heighest = &spot;
    }
  }
  fieldBoundary.highestPoint = Vector2<int>(theImage.width / 2, heighest->y);
}

void FieldBoundaryProvider::handleLowerCamSpots()
{
  InImage tmpLowerCamSpotsInImage;

  for(Vector2<float> p : lowerCamConvexHullOnField)
  {
    Vector2<int> pImg;
    p.rotate(-theOdometer.odometryOffset.rotation);
    p -= theOdometer.odometryOffset.translation;
    Geometry::calculatePointInImage(p, theCameraMatrix, theCameraInfo, pImg);
    tmpLowerCamSpotsInImage.push_back(pImg);
  }

  std::sort(tmpLowerCamSpotsInImage.begin(), tmpLowerCamSpotsInImage.end(), [](Vector2<int> a, Vector2<int> b)
  {
    return a.x < b.x || (a.x == b.x && a.y < b.y);
  });

  // Remove lower spots on the same scanline...
  if(!tmpLowerCamSpotsInImage.empty())
  {
    lowerCamSpotsInImage.push_back(tmpLowerCamSpotsInImage.front());
    Vector2<int>* prev = &tmpLowerCamSpotsInImage.front();
    for(Vector2<int>& current : tmpLowerCamSpotsInImage)
    {
      if(current.x > prev->x)
      {
        lowerCamSpotsInImage.push_back(current);
        prev = &current;
      }
    }
  }

  // Draw Spots from last frame
  for(Vector2<int>& p : lowerCamSpotsInImage)
  {
    DOT("module:FieldBoundary:LowerCamSpots", p.x, p.y, ColorClasses::green, ColorClasses::green);
  }
}

void FieldBoundaryProvider::findBundarySpots(FieldBoundary& fieldBoundary, int horizon)
{
  ASSERT(nearVertJump > 0);
  ASSERT(farVertJump > 0);

  int width = theImage.width;
  int height = theImage.height;

  vector<BoundaryScanline> scanlines;
  if(theCameraInfo.camera == CameraInfo::Camera::upper && lowerCamSpotsInImage.size() > 1)
  {
    for(int x = scanlienDistance / 2; x < width; x += scanlienDistance)
    {
      int y = clipToBoundary(lowerCamSpotsInImage, x);
      Vector2<int> p(x, y);
      lowerCamSpostInterpol.push_back(p);
      DOT("module:FieldBoundary:LowerCamSpotsInterpol", p.x, p.y, ColorClasses::black, ColorClasses::black);
      int yStart = y;
      int score = (y > height) ? (height - y) / nearVertJump : 0;
      BoundaryScanline line = {x, yStart, yStart, score, 0, &theImage[height - 1][x]};
      scanlines.push_back(line);
    }
  }
  else
  {
    for(int x = scanlienDistance / 2; x < width; x += scanlienDistance)
    {
      int yStart = height;
      theBodyContour.clipBottom(x, yStart, height);
      BoundaryScanline line = {x, yStart, yStart, 0, 0, &theImage[height - 1][x]};
      scanlines.push_back(line);
    }
  }

  // find y-coordinate where a point on the field is more than nonGreenPenaltyDistance meters away
  int yBound = 0;
  Vector2<float> pField;
  for(int y = height - 1; y > horizon; y -= 5)
  {
    Geometry::calculatePointOnField(width / 2, y, theCameraMatrix, theCameraInfo, pField);
    if(pField.abs() <= nonGreenPenaltyDistance)
    {
      yBound = y;
    }
    else
    {
      break;
    }
  }
  CROSS("module:FieldBoundary:GreaterPenaltyPoint", width / 2 , yBound, 5, 5, Drawings::ps_solid, ColorClasses::black);

  int vertJump = nearVertJump;
  for(int y = height - 1; y >= horizon; y -= vertJump)
  {
    int penalty = 1;
    int reward = 1;
    if(y < yBound)
    {
      vertJump = farVertJump;
      penalty = nonGreenPenalty;
    }

    for(BoundaryScanline& line : scanlines)
    {
      if(y < line.yStart)
      {
        if(theColorReference.isGreen(line.pImg))
        {
          line.score += reward;
        }
        else
        {
          line.score -= penalty;
        }

        if(line.maxScore <= line.score)
        {
          line.maxScore = line.score;
          line.yMax = y;
        }
      }
      line.pImg -= theImage.widthStep * vertJump;
    }
  }
  if(theCameraInfo.camera == CameraInfo::Camera::lower)
  {
    for(BoundaryScanline& line : scanlines)
    {
      int yEnd = line.yMax + static_cast<int>(minGreenCount * 1.5);
      const Image::Pixel* pImg = &theImage[line.yMax][line.x];
      if(!theColorReference.isGreen(pImg))
      {
        continue;
      }
      int greenCout = 1;
      for(int y = line.yMax + 1; y < yEnd && y < theImage.height; ++y)
      {
        if(theColorReference.isGreen(pImg))
        {
          ++greenCout;
        }
        pImg += theImage.widthStep;
      }
      if(greenCout >= minGreenCount) // line.yMax != line.yStart && TODO Add more stuff to prevent fuckups above shoulders
      {
        fieldBoundary.boundarySpots.push_back(Vector2<int>(line.x, line.yMax));
      }
    }
  }
  else
  {
    for(BoundaryScanline& line : scanlines)
    {
      fieldBoundary.boundarySpots.push_back(Vector2<int>(line.x, line.yMax));
    }
  }
}

bool FieldBoundaryProvider::cleanupBoundarySpots(InImage& boundarySpots) const
{
  ASSERT(nearVertJump >= farVertJump);
  int height = theImage.height;

  // remove trailing pointe with y == height.
  while(boundarySpots.begin() != boundarySpots.end() && boundarySpots.back().y == height)
    boundarySpots.erase(boundarySpots.end() - 1);

  // remove leading pointe with y == height.
  while(boundarySpots.begin() != boundarySpots.end() && boundarySpots.front().y == height)
    boundarySpots.erase(boundarySpots.begin());

  // Due to vertJump go up until there is no more green.
  for(Vector2<int>& point : boundarySpots)
  {
    for(int i = 0; i < nearVertJump; i++)
    {
      if(point.y > 0 && point.y < height &&  theColorReference.isGreen(&theImage[point.y - 1][point.x]))
        --point.y;
      else
        break;
    }
  }

  // make sure that the boundary contains at least 2 points.
  if(boundarySpots.size() < 2)
  {
    boundarySpots.clear();
    boundarySpots.push_back(Vector2<int>(0, height));
    boundarySpots.push_back(Vector2<int>(theImage.width - 1, height));
    return false;
  }
  else
  {
    return true;
  }
}

vector<FieldBoundaryProvider::InImage> FieldBoundaryProvider::calcBoundaryCandidates(InImage boundarySpots) const
{
  const int maxIter = 20;
  vector<InImage> boundaryCandidates;

  for(int i = 0; i < maxIter && boundarySpots.size() >= 2; ++i)
  {
    InImage boundaryCandidate = getUpperConvexHull(boundarySpots);
    boundaryCandidates.push_back(boundaryCandidate);
    if(boundaryCandidate.size() >= 2)
    {
      int maxDist = 0;
      Vector2<int>* val;

      //Length of first Line segment
      int firstDist = std::abs((boundaryCandidate.begin() + 1)->y - boundaryCandidate.front().y);
      //(*(boundaryCandidate.begin() + 1) - boundaryCandidate.front()).abs();
      //Length of last line segment
      int lastDist = std::abs((boundaryCandidate.end() - 2)->y - boundaryCandidate.back().y);
      //(*(boundaryCandidate.end() - 2) - boundaryCandidate.back()).abs();

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

      // erase the point with the maximum x-distande to its neighbous
      for(auto iter = boundaryCandidate.begin(); iter + 2 < boundaryCandidate.end(); ++iter)
      {
        int tmpDist = std::abs((iter + 1)->y - iter->y) + std::abs((iter + 2)->y - (iter + 1)->y);
        //(*(iter + 2) - *iter).abs();
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
  return boundaryCandidates;
}

void FieldBoundaryProvider::findBestBoundary(const vector<InImage>& boundaryCandidates,
    const InImage& boundarySpots, InImage& boundary) const
{
  int spotsOnLine;
  int spotsNearLine;
  int score;
  int y;
  int maxScore = 0;
  const InImage* tmpBoundary =  &boundaryCandidates.front();

  for(const InImage& boundarycandidate : boundaryCandidates)
  {
    spotsOnLine = 0;
    spotsNearLine = 0;
    for(const Vector2<int>& point : boundarySpots)
    {
      y = clipToBoundary(boundarycandidate, point.x);
      if(point.y > y && point.y - y < lowerBound)
        ++spotsNearLine;
      else if(point.y < y && y - point.y < upperBound)
        ++spotsNearLine;

      if(point.y == y)
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

inline bool FieldBoundaryProvider::isLeftOf(Vector2<int>& a, Vector2<int>& b, Vector2<int>& c) const
{
  return ((b.x - a.x) * (-c.y + a.y) - (c.x - a.x) * (-b.y + a.y) > 0);
}

FieldBoundaryProvider::InImage FieldBoundaryProvider::getUpperConvexHull(InImage& boundary) const
{
  ASSERT(boundary.size() > 1);

  //Andrew's Monotone Chain Algorithm to compute the upper hull
  InImage hull;
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
  return hull;
}

int FieldBoundaryProvider::clipToBoundary(const InImage& boundary, int x) const
{
  ASSERT(boundary.size() >= 2);

  const Vector2<int>* left = &boundary.front();
  const Vector2<int>* right = &boundary.back();

  if(x < left->x)
    right = &(*(boundary.begin() + 1));
  else if(x > right->x)
    left = &(*(boundary.end() - 2));
  else
  {
    for(const Vector2<int>& point : boundary)
    {
      if(point.x == x)
        return point.y;
      else if(point.x < x && point.x > left->x)
        left = &point;
      else if(point.x > x && point.x < right->x)
        right = &point;
    }
  }

  double m = 1.0 * (right->y - left->y) / (right->x - left->x);

  return static_cast<int>((x * m) + right->y - (right->x * m));
}
