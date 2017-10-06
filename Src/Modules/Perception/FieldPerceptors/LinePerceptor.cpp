/**
 * @file LinePerceptor.cpp
 *
 * Implements a module which detects lines and the center circle based on ColorScanlineRegions.
 *
 * @author Felix Thielke
 *
 * Edit "do not take first and last spot as THE line": instead, the fitted line is cutted by the first and last spot.
 *           @author Jesse Richter-Klug
 */

#include "LinePerceptor.h"

#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Approx.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/Deviation.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Transformation.h"

MAKE_MODULE(LinePerceptor, perception)

void LinePerceptor::update(LinesPercept& linesPercept)
{
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:spots", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:hessian", "drawingOnField");

  linesPercept.lines.clear();
  circleCandidates.clear();

  if(doAdvancedWidthChecks)
  {
    scanHorizontalScanlines<true>(linesPercept);
    scanVerticalScanlines<true>(linesPercept);
  }
  else
  {
    scanHorizontalScanlines<false>(linesPercept);
    scanVerticalScanlines<false>(linesPercept);
  }

  if(doExtendLines)
    extendLines(linesPercept);
}

void LinePerceptor::update(CirclePercept& circlePercept)
{
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:circlePoint", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:circleCheckPoint", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:circlePointField", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:circleCheckPointField", "drawingOnField");

  circlePercept.wasSeen = false;

  // Find a valid center circle in the circle candidates
  for(CircleCandidate& candidate : circleCandidates)
  {
    if(candidate.fieldSpots.size() >= minSpotsOnCircle &&
       getAbsoluteDeviation(candidate.radius, theFieldDimensions.centerCircleRadius) <= maxCircleRadiusDeviation &&
       candidate.calculateError() <= maxCircleFittingError &&
       correctCircle(candidate) &&
       isCircleWhite(candidate.center, candidate.radius))
    {
      circlePercept.pos = candidate.center;
      circlePercept.wasSeen = true;

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

  // Construct a circle from detected lines:
  // Cluster centers of arcs from detected lines
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

  // Find biggest valid cluster
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
    circlePercept.wasSeen = true;

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

template<bool advancedWidthChecks> void LinePerceptor::scanHorizontalScanlines(LinesPercept& linesPercept)
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
           !isSpotInsidePlayer(region->range.left, region->range.right, scanline.y, scanline.y) &&
           !isSpotInsideBall(region->range.left, region->range.right, scanline.y, scanline.y)) //Hack RoboCup17
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
                candidate.fieldSpots.emplace_back(thisSpot.field);
                leastSquaresCircleFit(candidate.fieldSpots, candidate.center, candidate.radius);
                circleFitted = true;
                break;
              }
            }
            for(Candidate& candidate : candidates)
            {
              if(candidate.spots.size() > 1 &&
                 candidate.getDistance(thisSpot.field) <= maxLineFittingError &&
                 (candidate.spots.back()->image - thisSpot.image).squaredNorm() < sqrMaxPixelDistOf2Spots && // Hack RoboCup 2017 -> TODO: make a cleaner solution
                 isWhite(static_cast<Vector2i>(thisSpot.image.cast<int>()), static_cast<Vector2i>(candidate.spots.back()->image.cast<int>())))
              {
                if(!circleFitted)
                {
                  circleCandidates.emplace_back(candidate, thisSpot.field);
                  if(circleCandidates.back().calculateError() > maxCircleFittingError)
                    circleCandidates.pop_back();
                  else
                  {
                    if(lineFitted)
                      goto hEndAdjacentSearch;
                    circleFitted = true;
                  }
                }
                if(!lineFitted && (!advancedWidthChecks || thisSpot.field.x() > maxWidthCheckDistance || getAbsoluteDeviation(theFieldDimensions.fieldLinesWidth, getLineWidthAtSpot(thisSpot, candidate.n0)) <= maxLineWidthDeviation))
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
                if(!advancedWidthChecks || (thisSpot.field.x() > maxWidthCheckDistance && spot.field.x() > maxWidthCheckDistance))
                {
                  thisSpot.candidate = candidate.spots.front()->candidate;
                  candidate.spots.emplace_back(&thisSpot);
                  candidate.fitLine();
                  goto hEndAdjacentSearch;
                }
                else
                {
                  Vector2f n0(spot.field - thisSpot.field);
                  n0 << n0.y(), n0.x();
                  n0 = n0.normalized();
                  if(std::max(thisSpot.field.x() > maxWidthCheckDistance ? 0 : getAbsoluteDeviation(theFieldDimensions.fieldLinesWidth, getLineWidthAtSpot(thisSpot, n0)), spot.field.x() > maxWidthCheckDistance ? 0 : getAbsoluteDeviation(theFieldDimensions.fieldLinesWidth, getLineWidthAtSpot(spot, n0))) <= maxLineWidthDeviation)
                  {
                    thisSpot.candidate = candidate.spots.front()->candidate;
                    candidate.spots.emplace_back(&thisSpot);
                    candidate.fitLine();
                    goto hEndAdjacentSearch;
                  }
                }
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
      CROSS("module:LinePerceptor:hessian", candidate.n0.normalized(candidate.d).x(), candidate.n0.normalized(candidate.d).y(), 50, 10, Drawings::PenStyle::solidPen, ColorRGBA::red);
      linesPercept.lines.emplace_back();
      LinesPercept::Line& line = linesPercept.lines.back();

      if(useRealLines)
      {
        line.firstField = from->field - (candidate.n0.dot(from->field) - candidate.d) * candidate.n0;
        line.lastField = to->field - (candidate.n0.dot(to->field) - candidate.d) * candidate.n0;
      }
      else
      {
        line.firstField = from->field;
        line.lastField = to->field;
      }

      line.line.base = line.firstField;
      line.line.direction = line.lastField - line.firstField;

      Vector2f temp;
      if(Transformation::robotToImage(line.firstField, theCameraMatrix, theCameraInfo, temp))
        line.firstImg = temp.cast<int>();
      else
        line.firstImg = static_cast<Vector2i>(from->image.cast<int>());

      if(Transformation::robotToImage(line.lastField, theCameraMatrix, theCameraInfo, temp))
        line.lastImg = temp.cast<int>();
      else
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

template<bool advancedWidthChecks> void LinePerceptor::scanVerticalScanlines(LinesPercept& linesPercept)
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
           !isSpotInsidePlayerFeet(theColorScanlineRegionsVerticalClipped.scanlines[scanLineIndex].x, theColorScanlineRegionsVerticalClipped.scanlines[scanLineIndex].x, region->range.upper, region->range.lower) &&
           !isSpotInsideBall(theColorScanlineRegionsVerticalClipped.scanlines[scanLineIndex].x, theColorScanlineRegionsVerticalClipped.scanlines[scanLineIndex].x, region->range.upper, region->range.lower)) //Hack RoboCup17
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
                candidate.fieldSpots.emplace_back(thisSpot.field);
                leastSquaresCircleFit(candidate.fieldSpots, candidate.center, candidate.radius);
                circleFitted = true;
                break;
              }
            }
            for(Candidate& candidate : candidates)
            {
              if(candidate.spots.size() > 1 &&
                 candidate.getDistance(thisSpot.field) <= maxLineFittingError &&
                 (candidate.spots.back()->image - thisSpot.image).squaredNorm() < sqrMaxPixelDistOf2Spots && // Hack RoboCup 2017 -> TODO: make a cleaner solution
                 isWhite(static_cast<Vector2i>(thisSpot.image.cast<int>()), static_cast<Vector2i>(candidate.spots.back()->image.cast<int>())))
              {
                if(!circleFitted)
                {
                  circleCandidates.emplace_back(candidate, thisSpot.field);
                  if(circleCandidates.back().calculateError() > maxCircleFittingError)
                    circleCandidates.pop_back();
                  else
                  {
                    if(lineFitted)
                      goto vEndAdjacentSearch;
                    circleFitted = true;
                  }
                }
                if(!lineFitted && (!advancedWidthChecks || thisSpot.field.x() > maxWidthCheckDistance || getAbsoluteDeviation(theFieldDimensions.fieldLinesWidth, getLineWidthAtSpot(thisSpot, candidate.n0)) <= maxLineWidthDeviation))
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
                if(!advancedWidthChecks || (thisSpot.field.x() > maxWidthCheckDistance && spot.field.x() > maxWidthCheckDistance))
                {
                  thisSpot.candidate = candidate.spots.front()->candidate;
                  candidate.spots.emplace_back(&thisSpot);
                  candidate.fitLine();
                  goto vEndAdjacentSearch;
                }
                else
                {
                  Vector2f n0(spot.field - thisSpot.field);
                  n0 << n0.y(), n0.x();
                  n0 = n0.normalized();
                  if(std::max(thisSpot.field.x() > maxWidthCheckDistance ? 0 : getAbsoluteDeviation(theFieldDimensions.fieldLinesWidth, getLineWidthAtSpot(thisSpot, n0)), spot.field.x() > maxWidthCheckDistance ? 0 : getAbsoluteDeviation(theFieldDimensions.fieldLinesWidth, getLineWidthAtSpot(spot, n0))) <= maxLineWidthDeviation)
                  {
                    thisSpot.candidate = candidate.spots.front()->candidate;
                    candidate.spots.emplace_back(&thisSpot);
                    candidate.fitLine();
                    goto vEndAdjacentSearch;
                  }
                }
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
        CROSS("module:LinePerceptor:hessian", candidate.n0.normalized(candidate.d).x(), candidate.n0.normalized(candidate.d).y(), 50, 10, Drawings::PenStyle::solidPen, ColorRGBA::red);
        linesPercept.lines.emplace_back();
        LinesPercept::Line& line = linesPercept.lines.back();

        if(useRealLines)
        {
          line.firstField = candidate.spots.front()->field - (candidate.n0.dot(candidate.spots.front()->field) - candidate.d) * candidate.n0;
          line.lastField = candidate.spots.back()->field - (candidate.n0.dot(candidate.spots.back()->field) - candidate.d) * candidate.n0;
        }
        else
        {
          line.firstField = candidate.spots.front()->field;
          line.lastField = candidate.spots.back()->field;
        }

        line.line.base = line.firstField;
        line.line.direction = line.lastField - line.firstField;

        Vector2f temp;
        if(Transformation::robotToImage(line.firstField, theCameraMatrix, theCameraInfo, temp))
          line.firstImg = temp.cast<int>();
        else
          line.firstImg = static_cast<Vector2i>(candidate.spots.front()->image.cast<int>());

        if(Transformation::robotToImage(line.lastField, theCameraMatrix, theCameraInfo, temp))
          line.lastImg = temp.cast<int>();
        else
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

void LinePerceptor::extendLines(LinesPercept& linesPercept) const
{
  for(LinesPercept::Line& line : linesPercept.lines)
  {
    const Vector2f step(static_cast<Vector2f>((line.firstImg - line.lastImg).cast<float>().normalized() * 2.f));

    // Extend left
    Vector2f pos(line.firstImg.cast<float>() + step);
    bool changed = false;
    for(; pos.x() >= 0 && pos.y() >= 0 && pos.x() < theECImage.colored.width && pos.y() < theECImage.colored.height && theECImage.colored[static_cast<Vector2i>(pos.cast<int>())] == FieldColors::white; pos += step, changed = true);
    if(changed)
    {
      pos -= step;
      Vector2f field;
      if(Transformation::imageToRobot(pos, theCameraMatrix, theCameraInfo, field))
      {
        line.firstImg = static_cast<Vector2i>(pos.cast<int>());
        line.firstField = field;
        line.spotsInImg.emplace_back(line.firstImg);
        line.spotsInField.emplace_back(line.firstField);
      }
    }

    // Extend right
    pos = line.lastImg.cast<float>() - step;
    changed = false;
    for(; pos.x() >= 0 && pos.y() >= 0 && pos.x() < theECImage.colored.width && pos.y() < theECImage.colored.height && theECImage.colored[static_cast<Vector2i>(pos.cast<int>())] == FieldColors::white; pos -= step, changed = true);
    if(changed)
    {
      pos += step;
      Vector2f field;
      if(Transformation::imageToRobot(pos, theCameraMatrix, theCameraInfo, field))
      {
        line.lastImg = static_cast<Vector2i>(pos.cast<int>());
        line.lastField = field;
        line.spotsInImg.emplace_back(line.lastImg);
        line.spotsInField.emplace_back(line.lastField);
      }
    }

    // Recompute line
    line.line.base = line.firstField;
    line.line.direction = line.lastField - line.firstField;
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

float LinePerceptor::getLineWidthAtSpot(const Spot& spot, const Vector2f& n0) const
{
  Vector2f dir;
  if(Transformation::robotToImage(static_cast<Vector2f>(spot.field + n0 * theFieldDimensions.fieldLinesWidth), theCameraMatrix, theCameraInfo, dir))
  {
    dir = (dir - spot.image).normalized();

    Vector2f left(spot.image - dir);
    for(Vector2i p(left.cast<int>()); p.x() >= 0 && p.y() >= 0 && p.x() < theECImage.colored.width && p.y() < theECImage.colored.height && theECImage.colored[p] == FieldColors::white; left -= dir, p = static_cast<Vector2i>(left.cast<int>()));
    left += dir;
    Vector2f right(spot.image + dir);
    for(Vector2i p(right.cast<int>()); p.x() >= 0 && p.y() >= 0 && p.x() < theECImage.colored.width && p.y() < theECImage.colored.height && theECImage.colored[p] == FieldColors::white; right += dir, p = static_cast<Vector2i>(right.cast<int>()));
    right -= dir;

    Vector2f leftField, rightField;
    if(Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(left), theCameraMatrix, theCameraInfo, leftField) && Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(right), theCameraMatrix, theCameraInfo, rightField))
      return (rightField - leftField).norm();
  }

  return theFieldDimensions.fieldLinesWidth;
}

bool LinePerceptor::correctCircle(CircleCandidate& circle) const
{
  Vector2f centerInImage;
  if(!Transformation::robotToImage(circle.center, theCameraMatrix, theCameraInfo, centerInImage))
    return false;

  circle.fieldSpots.clear();

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
      {
        circle.fieldSpots.emplace_back((outerField + innerField) / 2);
        CROSS("module:LinePerceptor:circlePoint", ((outer + inner) / 2).x(), ((outer + inner) / 2).y(), 5, 2, Drawings::solidPen, ColorRGBA::black);
        CROSS("module:LinePerceptor:circlePointField", circle.fieldSpots.back().x(), circle.fieldSpots.back().y(), 5, 2, Drawings::solidPen, ColorRGBA::black);
      }
    }

  skipPoint:
    ;
  }

  if(circle.fieldSpots.size() < minSpotsOnCircle)
    return false;

  leastSquaresCircleFit(circle.fieldSpots, circle.center, circle.radius);

  for(size_t i = 0; i < circle.fieldSpots.size(); ++i)
  {
    Vector2f& spot = circle.fieldSpots[i];
    if(circle.getDistance(spot) > maxCircleFittingError)
    {
      do
      {
        spot = circle.fieldSpots.back();
        circle.fieldSpots.pop_back();
        if(circle.fieldSpots.size() < minSpotsOnCircle)
        {
          return false;
        }
      }
      while(i < circle.fieldSpots.size() && circle.getDistance(spot) > maxCircleFittingError);
    }
  }

  leastSquaresCircleFit(circle.fieldSpots, circle.center, circle.radius);

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
      CROSS("module:LinePerceptor:circleCheckPoint", pointInImage.x(), pointInImage.y(), 5, 2, Drawings::solidPen, ColorRGBA::black);
      CROSS("module:LinePerceptor:circleCheckPointField", pointOnField.x(), pointOnField.y(), 5, 2, Drawings::solidPen, ColorRGBA::black);
      count++;
      if(theECImage.colored[static_cast<Vector2s>(pointInImage.cast<short>())] == FieldColors::white)
        whiteCount++;
    }
  }

  return count > 0 && static_cast<float>(whiteCount) / static_cast<float>(count) >= minCircleWhiteRatio;
}

bool LinePerceptor::isSpotInsidePlayer(const int fromX, const int toX, const int fromY, const int toY) const
{
  for(const PlayersImagePercept::PlayerInImage& player : thePlayersImagePercept.players)
  {
    if(toX >= player.x1 && fromX <= player.x2 && toY >= player.y1 && fromY <= player.y2)
      return true;
  }
  return false;
}

bool LinePerceptor::isSpotInsideBall(const int fromX, const int toX, const int fromY, const int toY) const
{
  if(theBallPercept.status == BallPercept::seen
     && toX >= theBallPercept.positionInImage.x() - theBallPercept.radiusInImage
     && fromX <= theBallPercept.positionInImage.x() + theBallPercept.radiusInImage
     && toY >= theBallPercept.positionInImage.y() - theBallPercept.radiusInImage
     && fromY <= theBallPercept.positionInImage.y() + theBallPercept.radiusInImage)
    return true;
  return false;
}

bool LinePerceptor::isSpotInsidePlayerFeet(const int fromX, const int toX, const int fromY, const int toY) const
{
  for(const PlayersImagePercept::PlayerInImage& player : thePlayersImagePercept.players)
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