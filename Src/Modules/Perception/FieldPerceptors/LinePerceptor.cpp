/**
 * @file LinePerceptor.cpp
 *
 * Implements a module which detects lines and the center circle based on ColorScanLineRegions.
 *
 * @author Felix Thielke
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
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:visited", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:upLow", "drawingOnImage");
  linesPercept.lines.clear();
  circleCandidates.clear();
  if(doAdvancedWidthChecks)
  {
    scanHorizontalScanLines<true>(linesPercept);
    scanVerticalScanLines<true>(linesPercept);
  }
  else
  {
    scanHorizontalScanLines<false>(linesPercept);
    scanVerticalScanLines<false>(linesPercept);
  }
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
      markLinesOnCircle(candidate.center);

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
  for(size_t n = clusters.size(); n; --n)
  {
    // Find current biggest cluster
    auto biggestCluster = clusters.begin();
    size_t maxSize = biggestCluster->centers.size();
    for(auto it = biggestCluster + 1; it < clusters.end(); it++)
    {
      const size_t curSize = it->centers.size();
      if(curSize > maxSize)
      {
        maxSize = curSize;
        biggestCluster = it;
      }
    }

    // Abort if the minimum size is not reached
    if(maxSize < minCircleClusterSize)
      break;

    // Check if the cluster is valid
    if(isCircleWhite(biggestCluster->center, theFieldDimensions.centerCircleRadius))
    {
      circlePercept.pos = biggestCluster->center;
      circlePercept.wasSeen = true;
      markLinesOnCircle(biggestCluster->center);

      break;
    }

    // Remove the cluster
    biggestCluster->centers.clear();
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

void LinePerceptor::markLinesOnCircle(const Vector2f& center)
{
  for(const LinesPercept::Line& line : theLinesPercept.lines)
  {
    for(const Vector2f& spot : line.spotsInField)
    {
      if(getAbsoluteDeviation((center - spot).norm(), theFieldDimensions.centerCircleRadius) > maxCircleFittingError)
        goto lineNotOnCircle;
    }
    const_cast<LinesPercept::Line&>(line).belongsToCircle = true;
  lineNotOnCircle :
    ;
  }
}

template<bool advancedWidthChecks> void LinePerceptor::scanHorizontalScanLines(LinesPercept& linesPercept)
{
  spotsH.resize(theColorScanLineRegionsHorizontal.scanLines.size());
  candidates.clear();

  if(!theFieldBoundary.isValid)
    return;

  unsigned int scanLineId = 0;
  for(const ColorScanLineRegionsHorizontal::ScanLine& scanLine : theColorScanLineRegionsHorizontal.scanLines)
  {
    spotsH[scanLineId].clear();
    spotsH[scanLineId].reserve(128);
    if(scanLine.regions.size() > 2)
    {
      for(auto region = scanLine.regions.cbegin() + 1; region != scanLine.regions.cend() - 1; ++region)
      {
        if(region->color == FieldColors::white)
        {
          auto before = region - 1;
          auto after = region + 1;
          for(int i = 0; i < maxSkipNumber
              && before->range.right - before->range.left <= maxSkipWidth
              && before != scanLine.regions.cbegin(); ++i, --before);
          for(int i = 0; i < maxSkipNumber
              && after->range.right - after->range.left <= maxSkipWidth
              && after + 1 != scanLine.regions.cend(); ++i, ++after);
          if(before->color == FieldColors::field &&
             after->color == FieldColors::field &&
             !isSpotInsideObstacle(region->range.left, region->range.right, scanLine.y, scanLine.y))
          {
            if(theFieldBoundary.getBoundaryY((region->range.left + region->range.right) / 2) > static_cast<int>(scanLine.y))
            {
              goto hEndScan;
            }
            spotsH[scanLineId].emplace_back(static_cast<float>((region->range.left + region->range.right) / 2), scanLine.y);
            Spot& thisSpot = spotsH[scanLineId].back();
            Vector2f corrected(theImageCoordinateSystem.toCorrected(thisSpot.image));
            Vector2f otherImage;
            if(Transformation::imageToRobot(corrected, theCameraMatrix, theCameraInfo, thisSpot.field) &&
               Transformation::robotToImage(Vector2f(thisSpot.field + thisSpot.field.normalized(theFieldDimensions.fieldLinesWidth).rotateLeft()), theCameraMatrix, theCameraInfo, otherImage))
            {
              float expectedWidth = (otherImage - corrected).norm();
              if(getAbsoluteDeviation(static_cast<int>(expectedWidth), region->range.right - region->range.left) <= maxLineWidthDeviation &&
                 before->range.right - before->range.left >= static_cast<int>(expectedWidth * greenAroundLineRatio) &&
                 after->range.right - after->range.left >= static_cast<int>(expectedWidth * greenAroundLineRatio))
                goto keepHSpot;
            }
            spotsH[scanLineId].pop_back();
            continue;

          keepHSpot:
            CROSS("module:LinePerceptor:spots", thisSpot.image.x(), thisSpot.image.y(), 5, 1, Drawings::PenStyle::solidPen, ColorRGBA::red);

            if(scanLineId > 0)
            {
              bool circleFitted = false, lineFitted = false;
              for(CircleCandidate& candidate : circleCandidates)
              {
                if(candidate.getDistance(thisSpot.field) <= maxCircleFittingError)
                {
                  candidate.addSpot(thisSpot.field);
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
              for(const Spot& spot : spotsH[scanLineId - 1])
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
                    n0.rotateLeft();
                    n0.normalize();
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
            thisSpot.candidate = static_cast<unsigned int>(candidates.size());
            candidates.emplace_back(&thisSpot);
          hEndAdjacentSearch:
            ;
          }
        }
      }
    }
    scanLineId++;
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

template<bool advancedWidthChecks> void LinePerceptor::scanVerticalScanLines(LinesPercept& linesPercept)
{
  spotsV.resize(theColorScanLineRegionsVerticalClipped.scanLines.size());
  candidates.clear();

  unsigned int scanLineId = 0;
  for(unsigned scanLineIndex = theColorScanLineRegionsVerticalClipped.lowResStart; scanLineIndex < theColorScanLineRegionsVerticalClipped.scanLines.size(); scanLineIndex += theColorScanLineRegionsVerticalClipped.lowResStep)
  {
    spotsV[scanLineId].clear();
    spotsV[scanLineId].reserve(128);
    const std::vector<ScanLineRegion>& regions = theColorScanLineRegionsVerticalClipped.scanLines[scanLineIndex].regions;
    if(regions.size() > 2)
    {
      for(auto region = regions.cbegin() + 1; region != regions.cend() - 1; ++region)
      {
        if(region->color == FieldColors::white)
        {
          auto before = region - 1;
          auto after = region + 1;
          for(int i = 0; i < maxSkipNumber
              && before->range.lower - before->range.upper <= maxSkipWidth
              && before != regions.cbegin(); ++i, --before);
          for(int i = 0; i < maxSkipNumber
              && after->range.lower - after->range.upper <= maxSkipWidth
              && after + 1 != regions.cend(); ++i, ++after);
          if(before->color == FieldColors::field &&
             after->color == FieldColors::field &&
             !isSpotInsideObstacle(theColorScanLineRegionsVerticalClipped.scanLines[scanLineIndex].x, theColorScanLineRegionsVerticalClipped.scanLines[scanLineIndex].x, region->range.upper, region->range.lower))
          {
            spotsV[scanLineId].emplace_back(theColorScanLineRegionsVerticalClipped.scanLines[scanLineIndex].x, static_cast<float>(region->range.upper + region->range.lower) / 2.f);
            Spot& thisSpot = spotsV[scanLineId].back();
            Vector2f corrected = theImageCoordinateSystem.toCorrected(thisSpot.image);
            Vector2f otherImage;
            if(Transformation::imageToRobot(corrected, theCameraMatrix, theCameraInfo, thisSpot.field) &&
               Transformation::robotToImage(Vector2f(thisSpot.field + thisSpot.field.normalized(theFieldDimensions.fieldLinesWidth)), theCameraMatrix, theCameraInfo, otherImage))
            {
              float expectedHeight = (corrected - otherImage).norm();
              if(before->range.lower - before->range.upper >= static_cast<int>(expectedHeight * greenAroundLineRatio) &&
                 after->range.lower - after->range.upper >= static_cast<int>(expectedHeight * greenAroundLineRatio))
                goto keepVSpot;
            }
            spotsV[scanLineId].pop_back();
            continue;

          keepVSpot:
            CROSS("module:LinePerceptor:spots", thisSpot.image.x(), thisSpot.image.y(), 5, 1, Drawings::PenStyle::solidPen, ColorRGBA::blue);

            if(scanLineId > 0)
            {
              bool circleFitted = false, lineFitted = false;
              for(CircleCandidate& candidate : circleCandidates)
              {
                if(candidate.getDistance(thisSpot.field) <= maxCircleFittingError)
                {
                  candidate.addSpot(thisSpot.field);
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
              for(const Spot& spot : spotsV[scanLineId - 1])
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
                    n0.rotateLeft();
                    n0.normalize();
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
            thisSpot.candidate = static_cast<unsigned int>(candidates.size());
            candidates.emplace_back(&thisSpot);
          vEndAdjacentSearch:
            ;
          }
        }
      }
    }
    scanLineId++;
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
      if(Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(pos), theCameraMatrix, theCameraInfo, field))
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
      if(Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(pos), theCameraMatrix, theCameraInfo, field))
      {
        line.lastImg = static_cast<Vector2i>(pos.cast<int>());
        line.lastField = field;
        line.spotsInImg.emplace_back(line.lastImg);
        line.spotsInField.emplace_back(line.lastField);
      }
    }

    if(trimLines)
      trimLine(line);

    // Recompute line
    line.line.base = line.firstField;
    line.line.direction = line.lastField - line.firstField;
  }
}

void LinePerceptor::trimLine(LinesPercept::Line& line) const
{
  Vector2i start = line.firstImg;
  Vector2i end = line.lastImg;
  if(start == end)
    return;

  for(int i = 0; i < 2; ++i)
  {
    int foundConsecutiveLineSpots = 0;

    Vector2f pos = i == 0 ? start.cast<float>() : end.cast<float>();
    Vector2f step = (i == 0 ? end - start : start - end).cast<float>().normalized() * 2.f;

    int left = line.firstImg.x() < line.lastImg.x() ? line.firstImg.x() : line.lastImg.x();
    int right = line.firstImg.x() < line.lastImg.x() ? line.lastImg.x() : line.firstImg.x();

    bool trimmed = false;
    for(; pos.y() >= 0 && pos.y() < theECImage.colored.height && pos.x() >= left && pos.x() <= right; pos += step)
    {
      Vector2f dir = step / 2.f;
      dir.rotateLeft();

      Vector2f lower = pos - dir, upper = pos + dir;
      for(Vector2i p(lower.cast<int>()); p.x() >= 0 && p.y() >= 0 && p.x() < static_cast<int>(theECImage.colored.width) && p.y() < static_cast<int>(theECImage.colored.height); lower -= dir, p = static_cast<Vector2i>(lower.cast<int>()))
      {
        PixelTypes::ColoredPixel pixel = theECImage.colored[p];
        if(pixel != FieldColors::white && pixel != FieldColors::none)
          break;
      }
      lower += dir;
      for(Vector2i p(upper.cast<int>()); p.x() >= 0 && p.y() >= 0 && p.x() < static_cast<int>(theECImage.colored.width) && p.y() < static_cast<int>(theECImage.colored.height); upper += dir, p = static_cast<Vector2i>(upper.cast<int>()))
      {
        PixelTypes::ColoredPixel pixel = theECImage.colored[p];
        if(pixel != FieldColors::white && pixel != FieldColors::none)
          break;
      }
      upper -= dir;

      if((upper - lower).squaredNorm() > maxWidthImage)
      {
        Vector2f lowerField, upperField;
        if(!Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(lower), theCameraMatrix, theCameraInfo, lowerField) ||
           !Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(upper), theCameraMatrix, theCameraInfo, upperField) ||
           (upperField - lowerField).norm() > theFieldDimensions.fieldLinesWidth * mFactor)
        {
          CROSS("module:LinePerceptor:upLow", lower.x(), lower.y(), 1, 1, Drawings::solidPen, ColorRGBA::yellow);
          CROSS("module:LinePerceptor:upLow", upper.x(), upper.y(), 1, 1, Drawings::solidPen, ColorRGBA::blue);
          LINE("module:LinePerceptor:upLow", lower.x(), lower.y(), upper.x(), upper.y(), 1, Drawings::solidPen, ColorRGBA::gray);
          foundConsecutiveLineSpots = 0;
          trimmed = true;
          continue;
        }
        else
        {
          CROSS("module:LinePerceptor:visited", pos.x(), pos.y(), 1, 1, Drawings::solidPen, ColorRGBA::black);
          ++foundConsecutiveLineSpots;
        }
      }
      else
      {
        CROSS("module:LinePerceptor:visited", pos.x(), pos.y(), 1, 1, Drawings::solidPen, ColorRGBA::black);
        ++foundConsecutiveLineSpots;
      }

      if(foundConsecutiveLineSpots == minConsecutiveSpots)
      {
        if(trimmed && pos.cast<int>() != start && pos.cast<int>() != end)
        {
          Vector2f field;
          if(Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(pos), theCameraMatrix, theCameraInfo, field))
          {
            line.spotsInImg.emplace_back(pos.cast<int>());
            line.spotsInField.emplace_back(field);
            pos -= (minConsecutiveSpots - 1) * step;
            if(Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(pos), theCameraMatrix, theCameraInfo, field))
            {
              if(i == 0)
              {
                line.firstImg = pos.cast<int>();
                line.firstField = field;
              }
              else
              {
                line.lastImg = pos.cast<int>();
                line.lastField = field;
              }
              line.spotsInImg.emplace_back(pos.cast<int>());
              line.spotsInField.emplace_back(field);
            }
          }
        }
        break;
      }
    }
  }

  if(start != line.firstImg || end != line.lastImg)
  {
    std::vector<Vector2i> spotsInImgTrimmed;
    spotsInImgTrimmed.reserve(line.spotsInImg.size());
    std::vector<Vector2f> spotsInFieldTrimmed;
    spotsInFieldTrimmed.reserve(line.spotsInField.size());
    for(unsigned int i = 0; i < line.spotsInImg.size(); ++i)
    {
      Vector2i spotInImage = line.spotsInImg.at(i);
      if(spotInImage.x() >= line.firstImg.x() && spotInImage.x() <= line.lastImg.x())
      {
        spotsInImgTrimmed.emplace_back(spotInImage);
        spotsInFieldTrimmed.emplace_back(line.spotsInField.at(i));
      }
    }
    line.spotsInImg = spotsInImgTrimmed;
    line.spotsInField = spotsInFieldTrimmed;
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
    dir = (theImageCoordinateSystem.fromCorrected(dir) - spot.image).normalized();

    Vector2f left(spot.image - dir);
    for(Vector2i p(left.cast<int>()); p.x() >= 0 && p.y() >= 0 && p.x() < static_cast<int>(theECImage.colored.width) && p.y() < static_cast<int>(theECImage.colored.height) && theECImage.colored[p] == FieldColors::white; left -= dir, p = static_cast<Vector2i>(left.cast<int>()));
    left += dir;
    Vector2f right(spot.image + dir);
    for(Vector2i p(right.cast<int>()); p.x() >= 0 && p.y() >= 0 && p.x() < static_cast<int>(theECImage.colored.width) && p.y() < static_cast<int>(theECImage.colored.height) && theECImage.colored[p] == FieldColors::white; right += dir, p = static_cast<Vector2i>(right.cast<int>()));
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

  centerInImage = theImageCoordinateSystem.fromCorrected(centerInImage);

  circle.fieldSpots.clear();

  for(Angle a = 0_deg; a < 360_deg; a += 5_deg)
  {
    Vector2f pointOnField(circle.center.x() + std::cos(a) * circle.radius, circle.center.y() + std::sin(a) * circle.radius);
    Vector2f pointInImage;
    if(Transformation::robotToImage(pointOnField, theCameraMatrix, theCameraInfo, pointInImage))
    {
      pointInImage = theImageCoordinateSystem.fromCorrected(pointInImage);
      if(pointInImage.x() >= 0 && pointInImage.x() < theCameraInfo.width && pointInImage.y() >= 0 && pointInImage.y() < theCameraInfo.height)
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
    }

  skipPoint:
    ;
  }

  if(circle.fieldSpots.size() < minSpotsOnCircle)
    return false;

  LeastSquares::fitCircle(circle.fieldSpots, circle.center, circle.radius);

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

  circle.fitter = LeastSquares::CircleFitter();
  circle.fitter.add(circle.fieldSpots);

  return circle.fitter.fit(circle.center, circle.radius) &&
         getAbsoluteDeviation(circle.radius, theFieldDimensions.centerCircleRadius) <= maxCircleRadiusDeviation &&
         circle.calculateError() <= maxCircleFittingError;
}

bool LinePerceptor::isCircleWhite(const Vector2f& center, const float radius) const
{
  unsigned int whiteCount = 0, count = 0;
  for(Angle a = 0_deg; a < 360_deg; a += 5_deg)
  {
    Vector2f pointOnField(center.x() + std::cos(a) * radius, center.y() + std::sin(a) * radius);
    Vector2f pointInImage;
    if(Transformation::robotToImage(pointOnField, theCameraMatrix, theCameraInfo, pointInImage))
    {
      pointInImage = theImageCoordinateSystem.fromCorrected(pointInImage);
      if(pointInImage.x() >= 0 && pointInImage.x() < theCameraInfo.width && pointInImage.y() >= 0 && pointInImage.y() < theCameraInfo.height)
      {
        CROSS("module:LinePerceptor:circleCheckPoint", pointInImage.x(), pointInImage.y(), 5, 2, Drawings::solidPen, ColorRGBA::black);
        CROSS("module:LinePerceptor:circleCheckPointField", pointOnField.x(), pointOnField.y(), 5, 2, Drawings::solidPen, ColorRGBA::black);
        count++;
        if(theECImage.colored[static_cast<Vector2s>(pointInImage.cast<short>())] == FieldColors::white)
          whiteCount++;
      }
    }
  }

  return count > 0 && static_cast<float>(whiteCount) / static_cast<float>(count) >= minCircleWhiteRatio;
}

bool LinePerceptor::isSpotInsideObstacle(const int fromX, const int toX, const int fromY, const int toY) const
{
  for(const ObstaclesImagePercept::Obstacle& obstacle : theObstaclesImagePercept.obstacles)
  {
    if(toX >= obstacle.left && fromX <= obstacle.right && toY >= obstacle.top && fromY <= obstacle.bottom)
      return true;
  }
  return false;
}
