/**
 * @file LinePerceptor.cpp
 *
 *
 * @author Felix Thielke
 * (@author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>)
 */

#include "LinePerceptor.h"

#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Approx.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/Deviation.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/LeastSquares.h"
#include "Tools/Math/Transformation.h"

MAKE_MODULE(LinePerceptor, perception)

void LinePerceptor::update(LinesPercept& linesPercept)
{
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:spots", "drawingOnImage");
  linesPercept.lines.clear();
  circleCandidates.clear();
  scanHorizontalScanlines(linesPercept);
  scanVerticalScanlines(linesPercept);
}

void LinePerceptor::update(CirclePercept& circlePercept)
{
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:CirclePoint", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:CircleCheckPoint", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:CirclePointField", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:CircleCheckPointField", "drawingOnField");

  // Find a valid center circle in the circle candidates
  for(CircleCandidate& candidate : circleCandidates)
  {
    if(candidate.spots.size() >= minSpotsOnCircle &&
       getAbsoluteDeviation(candidate.radius, theFieldDimensions.centerCircleRadius) <= maxCircleRadiusDeviation &&
       candidate.calculateError() <= maxCircleFittingError &&
       correctCircle(candidate) &&
       isCircleWhite(candidate.center, candidate.radius))
    {
      circlePercept.pos = candidate.center;
      circlePercept.lastSeen = theImage.timeStamp;

      for(const LinesPercept::Line& line : theLinesPercept.lines)
      {
        for(const Vector2f& spot : line.spotsInField)
        {
          if(candidate.getDistance(spot) > maxCircleFittingError)
            goto lineNotOnCircle;
        }
        const_cast<LinesPercept::Line&>(line).belongsToCircle = true;
      lineNotOnCircle :
        ;
      }
      return;
    }
  }

  // Construct a circle from detected lines
  clusters.clear();
  for(const LinesPercept::Line& line : theLinesPercept.lines)
  {
    for(auto it = line.spotsInField.cbegin(); it < line.spotsInField.cend() - 2; it++)
    {
      const Vector2f& a = *it;
      const Vector2f& b = *(++it);
      const Vector2f& c = *(++it);

      clusterCircleCenter(b + (((a + c) / 2) - b).normalized() * theFieldDimensions.centerCircleRadius);
    }
  }

  Vector2f center;
  size_t max = 0;
  for(const CircleCluster& cluster : clusters)
  {
    if(cluster.centers.size() > max && cluster.centers.size() >= minCircleClusterSize && isCircleWhite(cluster.center, theFieldDimensions.centerCircleRadius))
    {
      max = cluster.centers.size();
      center = cluster.center;
    }
  }
  if(max != 0)
  {
    circlePercept.pos = center;
    circlePercept.lastSeen = theImage.timeStamp;

    for(const LinesPercept::Line& line : theLinesPercept.lines)
    {
      for(const Vector2f& spot : line.spotsInField)
      {
        if(getAbsoluteDeviation((center - spot).norm(), theFieldDimensions.centerCircleRadius) > maxCircleFittingError)
          goto lineNotOnCircle2;
      }
      const_cast<LinesPercept::Line&>(line).belongsToCircle = true;
    lineNotOnCircle2 :
      ;
    }
  }
}

void LinePerceptor::clusterCircleCenter(const Vector2f& center)
{
  for(CircleCluster& cluster : clusters)
  {
    if((cluster.center - center).squaredNorm() <= sqrCircleClusterRadius)
    {
      cluster.centers.emplace_back(center);
      Vector2f newCenter(0, 0);
      for(const Vector2f& c : cluster.centers)
      {
        newCenter += c;
      }
      cluster.center = newCenter / static_cast<float>(cluster.centers.size());
      return;
    }
  }
  clusters.emplace_back(center);
}

void LinePerceptor::scanHorizontalScanlines(LinesPercept& linesPercept)
{
  spotsH.resize(theColorScanlineRegionsHorizontal.scanlines.size());
  candidates.clear();

  if(!theFieldBoundary.isValid)
    return;

  unsigned int scanlineId = 0;
  for(const ColorScanlineRegionsHorizontal::Scanline& scanline : theColorScanlineRegionsHorizontal.scanlines)
  {
    spotsH[scanlineId].clear();
    spotsH[scanlineId].reserve(128);
    if(scanline.regions.size() > 2)
    {
      for(auto region = scanline.regions.cbegin() + 1; region != scanline.regions.cend() - 1; region++)
      {
        if(region->color == FieldColors::white &&
           !isSpotInsidePlayer(region->range.left, region->range.right, scanline.y, scanline.y))
        {
          if(theFieldBoundary.getBoundaryY((region->range.left + region->range.right) / 2) > static_cast<int>(scanline.y))
          {
            goto hEndScan;
          }
          spotsH[scanlineId].emplace_back(static_cast<float>((region->range.left + region->range.right) / 2), scanline.y);
          Spot& thisSpot = spotsH[scanlineId].back();
          Vector2f corrected(theImageCoordinateSystem.toCorrected(thisSpot.image));
          Vector2f otherImage;
          if(!Transformation::imageToRobot(corrected, theCameraMatrix, theCameraInfo, thisSpot.field) ||
             !Transformation::robotToImage(Vector2f(thisSpot.field.x(), thisSpot.field.y() + theFieldDimensions.fieldLinesWidth), theCameraMatrix, theCameraInfo, otherImage) ||
             getAbsoluteDeviation(static_cast<int>((otherImage - corrected).norm()), region->range.right - region->range.left) > maxLineWidthDeviation)
          {
            spotsH[scanlineId].pop_back();
            continue;
          }
          CROSS("module:LinePerceptor:spots", thisSpot.image.x(), thisSpot.image.y(), 5, 1, Drawings::PenStyle::solidPen, ColorRGBA::red);

          if(scanlineId > 0)
          {
            bool circleFitted = false, lineFitted = false;
            for(CircleCandidate& candidate : circleCandidates)
            {
              if(candidate.getDistance(thisSpot.field) <= maxCircleFittingError)
              {
                candidate.spots.emplace_back(&thisSpot);
                candidate.fitCircle();
                circleFitted = true;
                break;
              }
            }
            for(Candidate& candidate : candidates)
            {
              if(candidate.spots.size() > 1 &&
                 candidate.getDistance(thisSpot.field) <= maxLineFittingError &&
                 isWhite(static_cast<Vector2i>(thisSpot.image.cast<int>()), static_cast<Vector2i>(candidate.spots.back()->image.cast<int>())))
              {
                if(!circleFitted)
                {
                  circleCandidates.emplace_back(candidate, &thisSpot);
                  if(circleCandidates.back().calculateError() > maxCircleFittingError)
                    circleCandidates.pop_back();
                  if(lineFitted)
                    goto hEndAdjacentSearch;
                  circleFitted = true;
                }
                if(!lineFitted)
                {
                  thisSpot.candidate = candidate.spots.front()->candidate;
                  candidate.spots.emplace_back(&thisSpot);
                  candidate.fitLine();
                  if(circleFitted)
                    goto hEndAdjacentSearch;
                  lineFitted = true;
                }
              }
            }
            if(lineFitted)
              goto hEndAdjacentSearch;
            for(const Spot& spot : spotsH[scanlineId - 1])
            {
              Candidate& candidate = candidates[spot.candidate];
              if(candidate.spots.size() == 1 &&
                 getAbsoluteDeviation(spot.image.x(), thisSpot.image.x()) < getAbsoluteDeviation(spot.image.y(), thisSpot.image.y()) &&
                 isWhite(static_cast<Vector2i>(thisSpot.image.cast<int>()), static_cast<Vector2i>(spot.image.cast<int>())))
              {
                thisSpot.candidate = candidate.spots.front()->candidate;
                candidate.spots.emplace_back(&thisSpot);
                candidate.fitLine();
                goto hEndAdjacentSearch;
              }
            }
          }
          thisSpot.candidate = (unsigned int)candidates.size();
          candidates.emplace_back(&thisSpot);
        hEndAdjacentSearch:
          ;
        }
      }
    }
    scanlineId++;
  }
hEndScan:
  ;

  Vector2f lastNearPoint;
  if(!Transformation::imageToRobot(Vector2f(theCameraInfo.width / 2, theCameraInfo.height * 4 / 5), theCameraMatrix, theCameraInfo, lastNearPoint))
    return;
  const float maxNearDistance = std::max(1000.f, lastNearPoint.x());

  for(const Candidate& candidate : candidates)
  {
    const float squaredLineLength = (candidate.spots.back()->field - candidate.spots.front()->field).squaredNorm();
    if(candidate.spots.size() >= std::max<unsigned int>(2, minSpotsPerLine) &&
       squaredLineLength >= minSquaredLineLength &&
       (candidate.spots.front()->field.x() <= maxNearDistance || squaredLineLength <= maxDistantHorizontalLength))
    {
      const Spot* from, *to;
      const bool flipped = candidate.spots.front()->image.x() > candidate.spots.back()->image.x();
      if(flipped)
      {
        from = candidate.spots.front();
        to = candidate.spots.back();
      }
      else
      {
        from = candidate.spots.back();
        to = candidate.spots.front();
      }
      linesPercept.lines.emplace_back();
      LinesPercept::Line& line = linesPercept.lines.back();
      line.firstField = from->field;
      line.lastField = to->field;
      line.line.base = line.firstField;
      line.line.direction = line.lastField - line.firstField;
      line.firstImg = static_cast<Vector2i>(from->image.cast<int>());
      line.lastImg = static_cast<Vector2i>(to->image.cast<int>());

      if(flipped)
      {
        for(auto it = candidate.spots.crbegin(); it != candidate.spots.crend(); it++)
        {
          line.spotsInField.emplace_back((*it)->field);
          line.spotsInImg.emplace_back(static_cast<Vector2i>((*it)->image.cast<int>()));
        }
      }
      else
      {
        for(const Spot* spot : candidate.spots)
        {
          line.spotsInField.emplace_back(spot->field);
          line.spotsInImg.emplace_back(static_cast<Vector2i>(spot->image.cast<int>()));
        }
      }
    }
  }
}

void LinePerceptor::scanVerticalScanlines(LinesPercept& linesPercept)
{
  spotsV.resize(theColorScanlineRegionsVerticalClipped.scanlines.size());
  candidates.clear();

  unsigned int scanlineId = 0;
  for(unsigned scanLineIndex = theColorScanlineRegionsVerticalClipped.lowResStart; scanLineIndex < theColorScanlineRegionsVerticalClipped.scanlines.size(); scanLineIndex += theColorScanlineRegionsVerticalClipped.lowResStep)
  {
    spotsV[scanlineId].clear();
    spotsV[scanlineId].reserve(128);
    const std::vector<ScanlineRegion>& regions = theColorScanlineRegionsVerticalClipped.scanlines[scanLineIndex].regions;
    if(regions.size() > 2)
    {
      for(auto region = regions.cbegin() + 1; region != regions.cend() - 1; region++)
      {
        if(region->color == FieldColors::white &&
           !isSpotInsidePlayerFeet(theColorScanlineRegionsVerticalClipped.scanlines[scanLineIndex].x, theColorScanlineRegionsVerticalClipped.scanlines[scanLineIndex].x, region->range.upper, region->range.lower))
        {
          spotsV[scanlineId].emplace_back(theColorScanlineRegionsVerticalClipped.scanlines[scanLineIndex].x, (float)(region->range.upper + region->range.lower) / 2);
          Spot& thisSpot = spotsV[scanlineId].back();
          if(!Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(thisSpot.image), theCameraMatrix, theCameraInfo, thisSpot.field))
          {
            spotsV[scanlineId].pop_back();
            continue;
          }
          CROSS("module:LinePerceptor:spots", thisSpot.image.x(), thisSpot.image.y(), 5, 1, Drawings::PenStyle::solidPen, ColorRGBA::blue);

          if(scanlineId > 0)
          {
            bool circleFitted = false, lineFitted = false;
            for(CircleCandidate& candidate : circleCandidates)
            {
              if(candidate.getDistance(thisSpot.field) <= maxCircleFittingError)
              {
                candidate.spots.emplace_back(&thisSpot);
                candidate.fitCircle();
                circleFitted = true;
                break;
              }
            }
            for(Candidate& candidate : candidates)
            {
              if(candidate.spots.size() > 1 &&
                 candidate.getDistance(thisSpot.field) <= maxLineFittingError &&
                 isWhite(static_cast<Vector2i>(thisSpot.image.cast<int>()), static_cast<Vector2i>(candidate.spots.back()->image.cast<int>())))
              {
                if(!circleFitted)
                {
                  circleCandidates.emplace_back(candidate, &thisSpot);
                  if(circleCandidates.back().calculateError() > maxCircleFittingError)
                    circleCandidates.pop_back();
                  if(lineFitted)
                    goto vEndAdjacentSearch;
                  circleFitted = true;
                }
                if(!lineFitted)
                {
                  thisSpot.candidate = candidate.spots.front()->candidate;
                  candidate.spots.emplace_back(&thisSpot);
                  candidate.fitLine();
                  if(circleFitted)
                    goto vEndAdjacentSearch;
                  lineFitted = true;
                }
              }
            }
            if(lineFitted)
              goto vEndAdjacentSearch;
            for(const Spot& spot : spotsV[scanlineId - 1])
            {
              Candidate& candidate = candidates[spot.candidate];
              if(candidate.spots.size() == 1 &&
                 getAbsoluteDeviation(spot.image.x(), thisSpot.image.x()) > getAbsoluteDeviation(spot.image.y(), thisSpot.image.y()) &&
                 isWhite(static_cast<Vector2i>(thisSpot.image.cast<int>()), static_cast<Vector2i>(spot.image.cast<int>())))
              {
                thisSpot.candidate = candidate.spots.front()->candidate;
                candidate.spots.emplace_back(&thisSpot);
                candidate.fitLine();
                goto vEndAdjacentSearch;
              }
            }
          }
          thisSpot.candidate = (unsigned int)candidates.size();
          candidates.emplace_back(&thisSpot);
        vEndAdjacentSearch:
          ;
        }
      }
    }
    scanlineId++;
  }

  for(const Candidate& candidate : candidates)
  {
    if(candidate.spots.size() >= std::max<unsigned int>(2, minSpotsPerLine))
    {
      for(LinesPercept::Line& line : linesPercept.lines)
      {
        if(candidate.getDistance(line.firstField) <= maxLineFittingError && candidate.getDistance(line.lastField) <= maxLineFittingError &&
           isWhite(static_cast<Vector2i>(candidate.spots.front()->image.cast<int>()), line.lastImg))
        {
          if(candidate.spots.front()->image.x() < line.firstImg.x())
          {
            line.firstField = candidate.spots.front()->field;
            line.firstImg = static_cast<Vector2i>(candidate.spots.front()->image.cast<int>());
          }
          if(candidate.spots.back()->image.x() > line.lastImg.x())
          {
            line.lastField = candidate.spots.back()->field;
            line.lastImg = static_cast<Vector2i>(candidate.spots.back()->image.cast<int>());
          }
          line.line.base = line.firstField;
          line.line.direction = line.lastField - line.firstField;
          for(const Spot* spot : candidate.spots)
          {
            line.spotsInField.emplace_back(spot->field);
            line.spotsInImg.emplace_back(static_cast<Vector2i>(spot->image.cast<int>()));
          }
          goto lineMerged;
        }
      }
      if((candidate.spots.back()->field - candidate.spots.front()->field).squaredNorm() >= minSquaredLineLength)
      {
        linesPercept.lines.emplace_back();
        LinesPercept::Line& line = linesPercept.lines.back();
        line.firstField = candidate.spots.front()->field;
        line.lastField = candidate.spots.back()->field;
        line.line.base = line.firstField;
        line.line.direction = line.lastField - line.firstField;
        line.firstImg = static_cast<Vector2i>(candidate.spots.front()->image.cast<int>());
        line.lastImg = static_cast<Vector2i>(candidate.spots.back()->image.cast<int>());
        for(const Spot* spot : candidate.spots)
        {
          linesPercept.lines.back().spotsInField.emplace_back(spot->field);
          linesPercept.lines.back().spotsInImg.emplace_back(static_cast<Vector2i>(spot->image.cast<int>()));
        }
      }
    lineMerged:
      ;
    }
  }
}

bool LinePerceptor::isWhite(const Vector2i& a, const Vector2i& b) const
{
  const Geometry::PixeledLine line(a, b, std::min(whiteCheckStepSize, static_cast<unsigned int>(std::ceil(std::max(getAbsoluteDeviation(a.x(), b.x()), getAbsoluteDeviation(a.y(), b.y())) * minWhiteRatio))));

  const unsigned int maxNonWhitePixels = static_cast<unsigned int>(static_cast<float>(line.size()) * (1 - minWhiteRatio));
  if(maxNonWhitePixels == line.size())
  {
    return true;
  }

  unsigned int nonWhiteCount = 0;
  for(const Vector2i& p : line)
  {
    if(theECImage.colored[p] != FieldColors::white)
    {
      nonWhiteCount++;
      if(nonWhiteCount > maxNonWhitePixels)
      {
        return false;
      }
    }
  }
  return true;
}

bool LinePerceptor::correctCircle(CircleCandidate& circle) const
{
  std::vector<Spot> spots;

  Vector2f centerInImage;
  if(!Transformation::robotToImage(circle.center, theCameraMatrix, theCameraInfo, centerInImage))
    return false;

  for(Angle a = 0_deg; a < 360_deg; a += 5_deg)
  {
    Vector2f pointOnField(circle.center.x() + cosf(a) * circle.radius, circle.center.y() + sinf(a) * circle.radius);
    Vector2f pointInImage;
    if(Transformation::robotToImage(pointOnField, theCameraMatrix, theCameraInfo, pointInImage) &&
       pointInImage.x() >= 0 && pointInImage.x() < theCameraInfo.width && pointInImage.y() >= 0 && pointInImage.y() < theCameraInfo.height)
    {
      if(theFieldBoundary.getBoundaryY(static_cast<int>(pointInImage.x())) > pointInImage.y())
        return false;

      Vector2f dir((pointInImage - centerInImage).normalized());

      Vector2f outer(pointInImage);
      while(theECImage.colored[static_cast<Vector2s>(outer.cast<short>())] != FieldColors::white)
      {
        outer += dir;
        if(!(outer.x() >= 0 && outer.x() < theCameraInfo.width && outer.y() >= 0 && outer.y() < theCameraInfo.height))
        {
          outer = pointInImage;
          while(theECImage.colored[static_cast<Vector2s>(outer.cast<short>())] != FieldColors::white)
          {
            outer -= dir;
            if(!(outer.x() >= 0 && outer.x() < theCameraInfo.width && outer.y() >= 0 && outer.y() < theCameraInfo.height))
              goto skipPoint;
          }
          pointInImage = outer;
        }
      }
      while(theECImage.colored[static_cast<Vector2s>(outer.cast<short>())] == FieldColors::white)
      {
        outer += dir;
        if(!(outer.x() >= 0 && outer.x() < theCameraInfo.width && outer.y() >= 0 && outer.y() < theCameraInfo.height))
          break;
      }
      Vector2f inner(pointInImage);
      while(theECImage.colored[static_cast<Vector2s>(inner.cast<short>())] != FieldColors::white)
      {
        inner -= dir;
        if(!(inner.x() >= 0 && inner.x() < theCameraInfo.width && inner.y() >= 0 && inner.y() < theCameraInfo.height))
          goto skipPoint;
      }
      while(theECImage.colored[static_cast<Vector2s>(inner.cast<short>())] == FieldColors::white)
      {
        inner -= dir;
        if(!(inner.x() >= 0 && inner.x() < theCameraInfo.width && inner.y() >= 0 && inner.y() < theCameraInfo.height))
          break;
      }

      outer = theImageCoordinateSystem.toCorrected(outer);
      inner = theImageCoordinateSystem.toCorrected(inner);
      Vector2f outerField, innerField;
      if(Transformation::imageToRobot(outer, theCameraMatrix, theCameraInfo, outerField) &&
         Transformation::imageToRobot(inner, theCameraMatrix, theCameraInfo, innerField) &&
         (outerField - innerField).squaredNorm() <= sqr(theFieldDimensions.fieldLinesWidth * 3 / 2))
        spots.emplace_back((outer + inner) / 2, (outerField + innerField) / 2);
    }

  skipPoint:
    ;
  }

  if(spots.size() < minSpotsOnCircle)
    return false;

  circle.spots.clear();
  for(Spot& spot : spots)
    circle.spots.emplace_back(&spot);

  circle.fitCircle();

  for(size_t i = 0; i < circle.spots.size(); ++i) // Spot*& spot : circle.spots)
  {
    const Spot*& spot = circle.spots[i];
    if(circle.getDistance(spot->field) > maxCircleFittingError)
    {
      do
      {
        spot = circle.spots.back();
        circle.spots.pop_back();
        if(circle.spots.size() < minSpotsOnCircle)
        {
          return false;
        }
      }
      while(i < circle.spots.size() && circle.getDistance(spot->field) > maxCircleFittingError);
    }
    else
    {
      CROSS("module:LinePerceptor:CirclePoint", spot->image.x(), spot->image.y(), 5, 2, Drawings::solidPen, ColorRGBA::black);
      CROSS("module:LinePerceptor:CirclePointField", spot->field.x(), spot->field.y(), 5, 2, Drawings::solidPen, ColorRGBA::black);
    }
  }

  circle.fitCircle();

  return getAbsoluteDeviation(circle.radius, theFieldDimensions.centerCircleRadius) <= maxCircleRadiusDeviation &&
         circle.calculateError() <= maxCircleFittingError;
}

bool LinePerceptor::isCircleWhite(const Vector2f& center, const float radius) const
{
  unsigned int whiteCount = 0, count = 0;
  for(Angle a = 0_deg; a < 360_deg; a += 5_deg)
  {
    Vector2f pointOnField(center.x() + cosf(a) * radius, center.y() + sinf(a) * radius);
    Vector2f pointInImage;
    if(Transformation::robotToImage(pointOnField, theCameraMatrix, theCameraInfo, pointInImage) &&
       pointInImage.x() >= 0 && pointInImage.x() < theCameraInfo.width && pointInImage.y() >= 0 && pointInImage.y() < theCameraInfo.height)
    {
      CROSS("module:LinePerceptor:CircleCheckPoint", pointInImage.x(), pointInImage.y(), 5, 2, Drawings::solidPen, ColorRGBA::black);
      CROSS("module:LinePerceptor:CircleCheckPointField", pointOnField.x(), pointOnField.y(), 5, 2, Drawings::solidPen, ColorRGBA::black);
      count++;
      if(theECImage.colored[static_cast<Vector2s>(pointInImage.cast<short>())] == FieldColors::white)
        whiteCount++;
    }
  }

  return count > 0 && static_cast<float>(whiteCount) / static_cast<float>(count) >= minCircleWhiteRatio;
}

bool LinePerceptor::isSpotInsidePlayer(const int fromX, const int toX, const int fromY, const int toY) const
{
  for(const PlayersPercept::Player& player : thePlayersPercept.players)
  {
    if(toX >= player.x1 && fromX <= player.x2 && toY >= player.y1 && fromY <= player.y2)
      return true;
  }
  return false;
}

bool LinePerceptor::isSpotInsidePlayerFeet(const int fromX, const int toX, const int fromY, const int toY) const
{
  for(const PlayersPercept::Player& player : thePlayersPercept.players)
  {
    if(toY >= player.y1 && fromY <= player.y2 &&
       ((player.detectedFeet && toX >= player.x1FeetOnly && fromX <= player.x2FeetOnly) ||
        (!player.detectedFeet && toX >= player.x1 && fromX <= player.x2)))
      return true;
  }
  return false;
}

void LinePerceptor::Candidate::fitLine()
{
  ASSERT(spots.size() > 0);

  // https://de.wikipedia.org/wiki/Lineare_Regression#Berechnung_der_Regressionsgeraden
  Vector2f avg(0, 0);
  for(const Spot* spot : spots)
  {
    avg += spot->field;
  }
  avg /= static_cast<float>(spots.size());

  float SSxx = 0, SSxy = 0;
  for(const Spot* spot : spots)
  {
    const float xDiff = spot->field.x() - avg.x();
    SSxx += sqr(xDiff);
    SSxy += xDiff * (spot->field.y() - avg.y());
  }

  if(Approx::isZero(SSxx))
  {
    // vertical lines cannot be fitted in a y=ax+b style equation
    n0 << 1, 0;
    d = avg.x();
  }
  else
  {
    const float b = SSxy / SSxx;
    const float a = avg.y() - b * avg.x();

    // https://de.wikipedia.org/wiki/Koordinatenform#Koordinatenform_einer_Geradengleichung
    // https://de.wikipedia.org/wiki/Normalenform#Berechnung
    // https://de.wikipedia.org/wiki/Hessesche_Normalform#Berechnung
    const float nLengthNegInv = (float)((a >= 0 ? 1 : -1) / sqrt(sqr(b) + 1));
    n0 << -b* nLengthNegInv, nLengthNegInv;
    d = a * nLengthNegInv;
  }
}

void LinePerceptor::CircleCandidate::fitCircle()
{
  ASSERT(spots.size() > 2);

  // Algorithm adapted from http://www.dtcenter.org/met/users/docs/write_ups/circle_fit.pdf
  Vector2f avg(0, 0);

  for(const Spot* spot : spots)
  {
    avg += spot->field;
  }
  avg /= static_cast<float>(spots.size());

  float suu = 0, suv = 0, svv = 0, c1 = 0, c2 = 0;

  for(const Spot* spot : spots)
  {
    const Vector2f normSpot(spot->field - avg);

    suu += normSpot.x() * normSpot.x();
    suv += normSpot.x() * normSpot.y();
    svv += normSpot.y() * normSpot.y();
    c1 += normSpot.x() * normSpot.x() * normSpot.x() + normSpot.x() * normSpot.y() * normSpot.y();
    c2 += normSpot.y() * normSpot.y() * normSpot.y() + normSpot.y() * normSpot.x() * normSpot.x();
  }
  c1 /= 2;
  c2 /= 2;

  const float divisor = suu * svv - suv * suv;
  if(divisor == 0)
  {
    // No solution -> no circle can be fitted
    radius = std::numeric_limits<float>::infinity();
    return;
  }
  const Vector2f c = Vector2f(svv * c1 - c2 * suv, suu * c2 - c1 * suv) / divisor;
  center = c + avg;

  radius = static_cast<float>(sqrt(c.squaredNorm() + (suu + svv) / static_cast<float>(spots.size())));
}
