/**
 * @file LinePerceptor.cpp
 *
 * Implements a module which detects lines and the center circle based on ColorScanLineRegions.
 *
 * @author Felix Thielke
 * @author Lukas Monnerjahn
 */

#include "LinePerceptor.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/ImageProcessing/PixelTypes.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/Deviation.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Transformation.h"

MAKE_MODULE(LinePerceptor, perception);

void LinePerceptor::update(LinesPercept& linesPercept)
{
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:spots", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:visited", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:upLow", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:isWhite", "drawingOnImage");
  linesPercept.lines.clear();
  circleCandidates.clear();
  scanHorizontalScanLines(linesPercept);
  scanVerticalScanLines(linesPercept);
  extendLines(linesPercept);
}

void LinePerceptor::update(CirclePercept& circlePercept)
{
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:circlePoint", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:circleCheckPoint", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:circlePointField", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:circleCheckPointField", "drawingOnField");
  DECLARE_DEBUG_RESPONSE("module:LinePerceptor:circleErrorStats");

  circlePercept.wasSeen = false;

  if(!circleCandidates.empty())
  {
    // find the best circle candidate to check extensively
    // the corrected circle position gets calculated only for a single candidate, to create an upper bound for the run time
    bool checkCandidate = false;
    size_t bestCandidateSpots = 0;
    CircleCandidate& bestCandidate = circleCandidates[0];

    // Find a valid center circle in the circle candidates
    for(CircleCandidate& candidate : circleCandidates)
    {
      if(candidate.fieldSpots.size() >= minSpotsOnCircle)
      {
        Angle inImageAngle = candidate.circlePartInImage();
        float maxFittingError = maxCircleFittingError;
        float maxRadiusDeviation = maxCircleRadiusDeviation;
        if(inImageAngle < circleAngleBetweenSpots && inImageAngle > minCircleAngleBetweenSpots)
        {
          // More narrow bounds for circles of which the robot sees only a small portion
          float reductionFactor = (inImageAngle - minCircleAngleBetweenSpots) / (circleAngleBetweenSpots - minCircleAngleBetweenSpots);
          maxFittingError *= reductionFactor;
          maxRadiusDeviation *= reductionFactor;
        }
        if(inImageAngle >= minCircleAngleBetweenSpots &&
           getAbsoluteDeviation(candidate.radius, theFieldDimensions.centerCircleRadius) <= maxRadiusDeviation &&
           candidate.calculateError() <= maxFittingError)
        {
          if(candidate.fieldSpots.size() > bestCandidateSpots)
          {
            checkCandidate = true;
            bestCandidateSpots = candidate.fieldSpots.size();
            bestCandidate = candidate;
          }
        }
      }
    }

    // time intensive checks
    if(checkCandidate && correctCircle(bestCandidate) &&
        isCircleWhite(bestCandidate.center, bestCandidate.radius))
    {
      circlePercept.pos = bestCandidate.center;
      circlePercept.wasSeen = true;
      markLinesOnCircle(bestCandidate.center);
      DEBUG_RESPONSE("module:LinePerceptor:circleErrorStats")
      {
        OUTPUT_TEXT("Part in image: " << bestCandidate.circlePartInImage());
        OUTPUT_TEXT("Fitting error: " << bestCandidate.calculateError());
        OUTPUT_TEXT("Radius deviation: " << getAbsoluteDeviation(bestCandidate.radius, theFieldDimensions.centerCircleRadius));
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
  for(size_t n = clusters.size(); n; --n)
  {
    // Find current biggest cluster
    auto biggestCluster = clusters.begin();
    size_t maxSize = biggestCluster->centers.size();
    for(auto it = biggestCluster + 1; it < clusters.end(); ++it)
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
void LinePerceptor::scanHorizontalScanLines(LinesPercept& linesPercept)
{
  spotsH.resize(theColorScanLineRegionsHorizontal.scanLines.size());
  candidates.clear();

  if(!theFieldBoundary.isValid)
    return;

  unsigned int scanLineId = 0;
  for(const ColorScanLineRegionsHorizontal::ScanLine& scanLine : theColorScanLineRegionsHorizontal.scanLines)
  {
    spotsH[scanLineId].clear();
    spotsH[scanLineId].reserve(scanLine.regions.size() / 2);
    if(scanLine.regions.size() > 2)
    {
      for(auto region = scanLine.regions.cbegin() + 1; region != scanLine.regions.cend() - 1; ++region)
      {
        if(region->color == PixelTypes::Color::white)
        {
          auto before = region - 1;
          auto after = region + 1;
          for(int i = 0; i < maxSkipNumber
                         && before->range.right - before->range.left <= maxSkipWidth
                         && before != scanLine.regions.cbegin(); ++i, --before);
          for(int i = 0; i < maxSkipNumber
                         && after->range.right - after->range.left <= maxSkipWidth
                         && after + 1 != scanLine.regions.cend(); ++i, ++after);
          if(before->color == PixelTypes::Color::field &&
             after->color == PixelTypes::Color::field &&
             !isSpotInsideObstacle(region->range.left, region->range.right, scanLine.y, scanLine.y))
          {
            if(theFieldBoundary.getBoundaryY((region->range.left + region->range.right) / 2) > static_cast<int>(scanLine.y))
            {
              goto hEndScan;
            }
            spotsH[scanLineId].emplace_back(static_cast<float>((region->range.left + region->range.right)) / 2.f, scanLine.y);
            Spot& thisSpot = spotsH[scanLineId].back();
            Vector2f corrected(theImageCoordinateSystem.toCorrected(thisSpot.image));
            Vector2f otherImage;
            if(Transformation::imageToRobot(corrected, theCameraMatrix, theCameraInfo, thisSpot.field) &&
               Transformation::robotToImage(Vector2f(thisSpot.field + thisSpot.field.normalized(theFieldDimensions.fieldLinesWidth).rotateLeft()), theCameraMatrix, theCameraInfo, otherImage))
            {
              float expectedWidth = (otherImage - corrected).norm();
              if(getAbsoluteDeviation(static_cast<int>(expectedWidth), region->range.right - region->range.left) <= maxLineWidthDeviationPx &&
                  (before->range.right - before->range.left >= static_cast<int>(expectedWidth * GREEN_AROUND_LINE_RATIO) ||
                      (relaxedGreenCheckAtImageBorder && before->range.left == 0)) &&
                  (after->range.right - after->range.left >= static_cast<int>(expectedWidth * GREEN_AROUND_LINE_RATIO) ||
                      (relaxedGreenCheckAtImageBorder && after->range.right == theCameraInfo.width)))
                goto keepHSpot;
            }
            spotsH[scanLineId].pop_back();
            continue;

          keepHSpot:
            CROSS("module:LinePerceptor:spots", thisSpot.image.x(), thisSpot.image.y(), 5, 3, Drawings::PenStyle::solidPen, ColorRGBA::red);

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
                      isSegmentValid(thisSpot, *candidate.spots.back(), candidate))
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
              for(const Spot& spot : spotsH[scanLineId - 1])
              {
                Candidate& candidate = candidates[spot.candidate];
                if(candidate.spots.size() == 1 &&
                   getAbsoluteDeviation(spot.image.x(), thisSpot.image.x()) < getAbsoluteDeviation(spot.image.y(), thisSpot.image.y()) &&
                    isSegmentValid(thisSpot, spot, candidate))
                {
                  thisSpot.candidate = candidate.spots.front()->candidate;
                  candidate.spots.emplace_back(&thisSpot);
                  candidate.fitLine();
                  goto hEndAdjacentSearch;
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
    if(candidate.spots.size() >= std::max<unsigned int>(2, MIN_SPOTS_PER_LINE) &&
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

void LinePerceptor::scanVerticalScanLines(LinesPercept& linesPercept)
{
  spotsV.resize(theColorScanLineRegionsVerticalClipped.scanLines.size());
  candidates.clear();

  unsigned int scanLineId = 0;
  unsigned int startIndex = highResolutionScan ? 0 : theColorScanLineRegionsVerticalClipped.lowResStart;
  unsigned int stepSize = highResolutionScan ? 1 : theColorScanLineRegionsVerticalClipped.lowResStep;
  for(unsigned scanLineIndex = startIndex; scanLineIndex < theColorScanLineRegionsVerticalClipped.scanLines.size(); scanLineIndex += stepSize)
  {
    spotsV[scanLineId].clear();
    const std::vector<ScanLineRegion>& regions = theColorScanLineRegionsVerticalClipped.scanLines[scanLineIndex].regions;
    spotsV[scanLineId].reserve(regions.size() / 2);
    if(regions.size() > 2)
    {
      for(auto region = regions.cbegin() + 1; region != regions.cend() - 1; ++region)
      {
        if(region->color == PixelTypes::Color::white)
        {
          auto before = region - 1;
          auto after = region + 1;
          for(int i = 0; i < maxSkipNumber
                         && before->range.lower - before->range.upper <= maxSkipWidth
                         && before != regions.cbegin(); ++i, --before);
          for(int i = 0; i < maxSkipNumber
                         && after->range.lower - after->range.upper <= maxSkipWidth
                         && after + 1 != regions.cend(); ++i, ++after);
          if(before->color == PixelTypes::Color::field &&
             after->color == PixelTypes::Color::field &&
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
              if((before->range.lower - before->range.upper >= static_cast<int>(expectedHeight * GREEN_AROUND_LINE_RATIO) ||
                      (relaxedGreenCheckAtImageBorder && before->range.lower == theCameraInfo.height)) &&
                  (after->range.lower - after->range.upper >= static_cast<int>(expectedHeight * GREEN_AROUND_LINE_RATIO) ||
                      (relaxedGreenCheckAtImageBorder && theCameraInfo.camera == CameraInfo::lower && after->range.upper == 0)))
                goto keepVSpot;
            }
            spotsV[scanLineId].pop_back();
            continue;

          keepVSpot:
            CROSS("module:LinePerceptor:spots", thisSpot.image.x(), thisSpot.image.y(), 5, 3, Drawings::PenStyle::solidPen, ColorRGBA::blue);

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
                    isSegmentValid(thisSpot, *candidate.spots.back(), candidate))
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
              for(const Spot& spot : spotsV[previousVerticalScanLine(thisSpot, static_cast<int>(scanLineId))])
              {
                Candidate& candidate = candidates[spot.candidate];
                if(candidate.spots.size() == 1 &&
                   getAbsoluteDeviation(spot.image.x(), thisSpot.image.x()) > getAbsoluteDeviation(spot.image.y(), thisSpot.image.y()) &&
                    isSegmentValid(thisSpot, spot, candidate))
                {
                  thisSpot.candidate = candidate.spots.front()->candidate;
                    candidate.spots.emplace_back(&thisSpot);
                    candidate.fitLine();
                    goto vEndAdjacentSearch;
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
    if(candidate.spots.size() >= std::max<unsigned int>(2, MIN_SPOTS_PER_LINE))
    {
      for(LinesPercept::Line& line : linesPercept.lines)
      {
        if(candidate.getDistance(line.firstField) <= maxLineFittingError && candidate.getDistance(line.lastField) <= maxLineFittingError &&
           isWhite(*candidate.spots.front(), Spot(line.lastImg.cast<float>(), line.lastField), candidate.n0))
        {
          if(candidate.spots.front()->image.x() < static_cast<float>(line.firstImg.x()))
          {
            line.firstField = candidate.spots.front()->field;
            line.firstImg = static_cast<Vector2i>(candidate.spots.front()->image.cast<int>());
          }
          if(candidate.spots.back()->image.x() > static_cast<float>(line.lastImg.x()))
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
  for(auto line = linesPercept.lines.begin(); line != linesPercept.lines.end();)
  {
    const Vector2f step(static_cast<Vector2f>((line->firstImg - line->lastImg).cast<float>().normalized() * 2.f));
    // Extend left
    Vector2f n0 (-step.y(), step.x()); // rotates 90 degrees

    Vector2f pos(line->firstImg.cast<float>() + step);
    bool changed = false;
    for(; pos.x() >= 0 && pos.y() >= 0 && pos.x() < static_cast<float>(theECImage.grayscaled.width) &&
          pos.y() < static_cast<float>(theECImage.grayscaled.height); pos += step, changed = true)
    {
      Vector2f pointOnField;
      if(!Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(pos), theCameraMatrix, theCameraInfo, pointOnField) ||
         !isPointWhite(pointOnField, pos.cast<int>(), n0))
        break;
    }
    if(changed)
    {
      pos -= step;
      Vector2f field;
      if(Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(pos), theCameraMatrix, theCameraInfo, field))
      {
        line->firstImg = static_cast<Vector2i>(pos.cast<int>());
        line->firstField = field;
        line->spotsInImg.emplace_back(line->firstImg);
        line->spotsInField.emplace_back(line->firstField);
      }
    }
    // Extend right
    pos = line->lastImg.cast<float>() - step;
    changed = false;
    for(; pos.x() >= 0 && pos.y() >= 0 && pos.x() < static_cast<float>(theECImage.grayscaled.width) &&
          pos.y() < static_cast<float>(theECImage.grayscaled.height); pos -= step, changed = true)
    {
      Vector2f pointOnField;
      if(!Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(pos), theCameraMatrix, theCameraInfo, pointOnField) ||
         !isPointWhite(pointOnField, pos.cast<int>(), n0))
        break;
    }
    if(changed)
    {
      pos += step;
      Vector2f field;
      if(Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(pos), theCameraMatrix, theCameraInfo, field))
      {
        line->lastImg = static_cast<Vector2i>(pos.cast<int>());
        line->lastField = field;
        line->spotsInImg.emplace_back(line->lastImg);
        line->spotsInField.emplace_back(line->lastField);
      }
    }
    if(TRIM_LINES)
      trimLine(*line);

    // Recompute line
    line->line.base = line->firstField;
    line->line.direction = line->lastField - line->firstField;

    if(line->spotsInImg.size() < MIN_SPOTS_PER_LINE || line->line.direction.squaredNorm() < minSquaredLineLength)
      line = linesPercept.lines.erase(line);
    else
      ++line;
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
    for(; pos.y() >= 0 && pos.y() < static_cast<float>(theECImage.grayscaled.height) && pos.x() >= static_cast<float>(left) && pos.x() <= static_cast<float>(right); pos += step)
    {
      Vector2f dir = step / 2.f;
      dir.rotateLeft();
      Vector2f lower = pos - dir, upper = pos + dir;
      unsigned char luminanceReference = theECImage.grayscaled[static_cast<Vector2i>(pos.cast<int>())];
      unsigned char saturationReference = theECImage.saturated[static_cast<Vector2i>(pos.cast<int>())];
      int loopCount = 0;
      for(Vector2i p(lower.cast<int>());
          p.x() >= 0 && p.y() >= 0 && p.x() < static_cast<int>(theECImage.grayscaled.width) && p.y() < static_cast<int>(theECImage.grayscaled.height);
          lower -= dir, p = static_cast<Vector2i>(lower.cast<int>()), ++loopCount)
      {
        if(loopCount > maxWidthImage ||
            theRelativeFieldColors.isFieldNearWhite(theECImage.grayscaled[p], theECImage.saturated[p],
                                                   luminanceReference, saturationReference))
          break;
      }
      lower += dir;
      for(Vector2i p(upper.cast<int>());
          p.x() >= 0 && p.y() >= 0 && p.x() < static_cast<int>(theECImage.grayscaled.width) && p.y() < static_cast<int>(theECImage.grayscaled.height);
          upper += dir, p = static_cast<Vector2i>(upper.cast<int>()), ++loopCount)
      {
        if(loopCount > maxWidthImage ||
            theRelativeFieldColors.isFieldNearWhite(theECImage.grayscaled[p], theECImage.saturated[p],
                                                   luminanceReference, saturationReference))
          break;
      }
      upper -= dir;

      if((upper - lower).squaredNorm() > maxWidthImageSquared)
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

bool LinePerceptor::isSegmentValid(const Spot& a, const Spot& b, const Candidate& candidate)
{
  Vector2f n0 = (b.field - a.field);
  n0.rotateLeft();
  n0.normalize();
  if(candidate.spots.size() > 2)
  {
    float angleDiff = n0.angleTo(candidate.n0);
    angleDiff = angleDiff <= pi_2 ? angleDiff : pi - angleDiff;
    if(angleDiff > maxNormalAngleDiff)
    {
      LINE("module:LinePerceptor:isWhite", a.image.x(), a.image.y(), b.image.x(), b.image.y(), 1, Drawings::solidPen, ColorRGBA::red);
      return false;
    }
    n0 = candidate.n0;
  }
  return isWhite(a, b, n0);
}

bool LinePerceptor::isWhite(const Spot& a, const Spot& b, const Vector2f& n0Field)
{
  const Geometry::PixeledLine line(a.image.cast<int>(), b.image.cast<int>(), std::min(static_cast<int>(whiteCheckStepSize),
                                     static_cast<int>(std::ceil(static_cast<float>(std::max(getAbsoluteDeviation(a.image.x(), b.image.x()),
                                     getAbsoluteDeviation(a.image.y(), b.image.y()))) * minWhiteRatio))));

  const auto maxNonWhitePixels = static_cast<unsigned int>(static_cast<float>(line.size()) * (1 - minWhiteRatio));
  if(maxNonWhitePixels == line.size())
  {
    CROSS("module:LinePerceptor:isWhite", a.image.x(), a.image.y(), 1, 1, Drawings::solidPen, ColorRGBA::blue);
    CROSS("module:LinePerceptor:isWhite", b.image.x(), b.image.y(), 1, 1, Drawings::solidPen, ColorRGBA::blue);
    LINE("module:LinePerceptor:isWhite", a.image.x(), a.image.y(), b.image.x(), b.image.y(), 1, Drawings::solidPen, ColorRGBA::blue);
    return true;
  }

  if(perspectivelyCorrectWhiteCheck)
  {
    Vector2f pointOnField = a.field;
    unsigned int nonWhiteCount = 0;
    COMPLEX_DRAWING("module:LinePerceptor:isWhite") debugIsPointWhite = true;
    for(const Vector2i& p : line)
    {
      if(Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(p), theCameraMatrix, theCameraInfo, pointOnField))
      {
        if(!isPointWhite(pointOnField, p, n0Field))
        {
          ++nonWhiteCount;
          if(nonWhiteCount > maxNonWhitePixels)
          {
            COMPLEX_DRAWING("module:LinePerceptor:isWhite")
            {
              debugIsPointWhite = false;
              CROSS("module:LinePerceptor:isWhite", a.image.x(), a.image.y(), 1, 1, Drawings::solidPen, ColorRGBA::red);
              CROSS("module:LinePerceptor:isWhite", b.image.x(), b.image.y(), 1, 1, Drawings::solidPen, ColorRGBA::red);
              LINE("module:LinePerceptor:isWhite", a.image.x(), a.image.y(), b.image.x(), b.image.y(), 1, Drawings::solidPen, ColorRGBA::red);
            }
            return false;
          }
        }
      }
    }
    COMPLEX_DRAWING("module:LinePerceptor:isWhite")
    {
      debugIsPointWhite = false;
      CROSS("module:LinePerceptor:isWhite", a.image.x(), a.image.y(), 1, 1, Drawings::solidPen, ColorRGBA::blue);
      CROSS("module:LinePerceptor:isWhite", b.image.x(), b.image.y(), 1, 1, Drawings::solidPen, ColorRGBA::blue);
      LINE("module:LinePerceptor:isWhite", a.image.x(), a.image.y(), b.image.x(), b.image.y(), 1, Drawings::solidPen, ColorRGBA::blue);
    }
    return true;
  }
  else
  {
    Vector2f n0Image = b.image - a.image;
    n0Image.rotateLeft();
    n0Image.normalize();
    Vector2f pointOnField;
    float s = 1.f;
    unsigned int nonWhiteCount = 0;
    COMPLEX_DRAWING("module:LinePerceptor:isWhite") debugIsPointWhite = true;
    for(const Vector2i& p : line)
    {
      if(Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(p), theCameraMatrix, theCameraInfo, pointOnField))
        s = calcWhiteCheckDistanceInImage(pointOnField);
      else
        return false;
      if(!isPointWhite(p.cast<float>(), s * n0Image))
      {
        ++nonWhiteCount;
        if(nonWhiteCount > maxNonWhitePixels)
        {
          COMPLEX_DRAWING("module:LinePerceptor:isWhite")
          {
            debugIsPointWhite = false;
            CROSS("module:LinePerceptor:isWhite", a.image.x(), a.image.y(), 1, 1, Drawings::solidPen, ColorRGBA::red);
            CROSS("module:LinePerceptor:isWhite", b.image.x(), b.image.y(), 1, 1, Drawings::solidPen, ColorRGBA::red);
            LINE("module:LinePerceptor:isWhite", a.image.x(), a.image.y(), b.image.x(), b.image.y(), 1, Drawings::solidPen, ColorRGBA::red);
          }
          return false;
        }
      }
    }
    COMPLEX_DRAWING("module:LinePerceptor:isWhite")
    {
      debugIsPointWhite = false;
      CROSS("module:LinePerceptor:isWhite", a.image.x(), a.image.y(), 1, 1, Drawings::solidPen, ColorRGBA::blue);
      CROSS("module:LinePerceptor:isWhite", b.image.x(), b.image.y(), 1, 1, Drawings::solidPen, ColorRGBA::blue);
      LINE("module:LinePerceptor:isWhite", a.image.x(), a.image.y(), b.image.x(), b.image.y(), 1, Drawings::solidPen, ColorRGBA::blue);
    }
    return true;
  }
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
      auto imageWidth = static_cast<float>(theCameraInfo.width);
      auto imageHeight = static_cast<float>(theCameraInfo.height);
      pointInImage = theImageCoordinateSystem.fromCorrected(pointInImage);
      if(pointInImage.x() >= 0 && pointInImage.x() < imageWidth && pointInImage.y() >= 0 && pointInImage.y() < imageHeight)
      {
        if(static_cast<float>(theFieldBoundary.getBoundaryY(static_cast<int>(pointInImage.x()))) > pointInImage.y())
          return false;

        Vector2f dir(pointInImage - centerInImage);
        float s = calcWhiteCheckDistance(pointOnField) / circle.radius * dir.norm();
        dir.normalize();
        Vector2f n = s * dir;

        Vector2f outer(pointInImage);
        short maxLuminance = 0;
        int maxLoops = static_cast<int>(s) + 1;
        for(int loops = 0; loops <= maxLoops && !isPointWhite(outer, n); ++loops)
        {
          outer += dir;
          if(loops == maxLoops || !(outer.x() >= 0 && outer.x() < imageWidth && outer.y() >= 0 && outer.y() < imageHeight))
          {
            outer = pointInImage;
            for(int inwardLoops = 0; inwardLoops <= maxLoops && !isPointWhite(outer, n); ++inwardLoops)
            {
              outer -= dir;
              if(inwardLoops == maxLoops || !(outer.x() >= 0 && outer.x() < imageWidth && outer.y() >= 0 && outer.y() < imageHeight))
                goto skipPoint;
            }
            break;
          }
        }
        pointInImage = outer;
        do
        {
          maxLuminance = std::max(maxLuminance, static_cast<short>(theECImage.grayscaled[static_cast<Vector2i>(outer.cast<int>())]));
          outer += dir;
        }
        while(outer.x() >= 0 && outer.x() < imageWidth && outer.y() >= 0 && outer.y() < imageHeight &&
              isPointWhite(outer, n) && theECImage.grayscaled[static_cast<Vector2i>(outer.cast<int>())] +
                                        theRelativeFieldColors.rfcParameters.minWhiteToFieldLuminanceDifference >= maxLuminance);
        outer -= dir;
        Vector2f inner(pointInImage);
        n = -n;
        while(!isPointWhite(inner, n))
        {
          inner -= dir;
          if(!(inner.x() >= 0 && inner.x() < imageWidth && inner.y() >= 0 && inner.y() < imageHeight))
            goto skipPoint;
        }
        do
        {
          maxLuminance = std::max(maxLuminance, static_cast<short>(theECImage.grayscaled[static_cast<Vector2i>(outer.cast<int>())]));
          inner -= dir;
        }
        while(inner.x() >= 0 && inner.x() < imageWidth && inner.y() >= 0 && inner.y() < imageHeight &&
              isPointWhite(inner, n) && theECImage.grayscaled[static_cast<Vector2i>(outer.cast<int>())] +
                                        theRelativeFieldColors.rfcParameters.minWhiteToFieldLuminanceDifference >= maxLuminance);
        inner += dir;

        CROSS("module:LinePerceptor:circlePoint", outer.x(), outer.y(), 5, 2, Drawings::solidPen, ColorRGBA::cyan);
        CROSS("module:LinePerceptor:circlePoint", inner.x(), inner.y(), 5, 2, Drawings::solidPen, ColorRGBA::yellow);

        outer = theImageCoordinateSystem.toCorrected(outer);
        inner = theImageCoordinateSystem.toCorrected(inner);
        Vector2f outerField, innerField;
        if(Transformation::imageToRobot(outer, theCameraMatrix, theCameraInfo, outerField) &&
           Transformation::imageToRobot(inner, theCameraMatrix, theCameraInfo, innerField) &&
           (outerField - innerField).squaredNorm() <= circleCorrectionMaxLineWidthSquared)
        {
          circle.fieldSpots.emplace_back((outerField + innerField) / 2);
          COMPLEX_DRAWING("module:LinePerceptor:circlePoint")
          {
            Vector2f uncor = (theImageCoordinateSystem.fromCorrected(outer) + theImageCoordinateSystem.fromCorrected(inner)) / 2.f;
            CROSS("module:LinePerceptor:circlePoint", uncor.x(), uncor.y(), 3, 1, Drawings::solidPen, ColorRGBA::gray);
          }
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
      if(pointInImage.x() >= 0 && pointInImage.x() < static_cast<float>(theCameraInfo.width) &&
         pointInImage.y() >= 0 && pointInImage.y() < static_cast<float>(theCameraInfo.height))
      {
        CROSS("module:LinePerceptor:circleCheckPoint", pointInImage.x(), pointInImage.y(), 5, 2, Drawings::solidPen, ColorRGBA::black);
        CROSS("module:LinePerceptor:circleCheckPointField", pointOnField.x(), pointOnField.y(), 5, 2, Drawings::solidPen, ColorRGBA::black);
        count++;
        Vector2f n0(std::cos(a), std::sin(a)); // normal vector pointing outward
        if(isPointWhite(pointOnField, pointInImage.cast<int>(), n0))
          whiteCount++;
      }
    }
  }

  return count > 0 && static_cast<float>(whiteCount) / static_cast<float>(count) >= minCircleWhiteRatio;
}

bool LinePerceptor::isPointWhite(const Vector2f& pointOnField, const Vector2i& pointInImage, const Vector2f& n0) const
{
  Vector2f nw = calcWhiteCheckDistance(pointOnField) * n0;
  Vector2f outerPointOnField = pointOnField + nw;
  Vector2f innerPointOnField = pointOnField - nw;
  Vector2f referencePointInImage;
  unsigned short luminanceReference = 0, saturationReference = 0;
  bool isOuterPointInImage = false;
  if(Transformation::robotToImage(outerPointOnField, theCameraMatrix, theCameraInfo, referencePointInImage))
  {
    referencePointInImage = theImageCoordinateSystem.fromCorrected(referencePointInImage);
    Vector2i integerReferenceInImage = referencePointInImage.cast<int>();
    if(integerReferenceInImage.x() >= 0 && integerReferenceInImage.x() < theCameraInfo.width &&
       integerReferenceInImage.y() >= 0 && integerReferenceInImage.y() < theCameraInfo.height)
    {
      luminanceReference = theECImage.grayscaled[integerReferenceInImage];
      saturationReference = theECImage.saturated[integerReferenceInImage];
      isOuterPointInImage = true;
      if(debugIsPointWhite)
      {
        CROSS("module:LinePerceptor:isWhite", referencePointInImage.x(), referencePointInImage.y(), 2, 1, Drawings::solidPen, ColorRGBA::orange);
        LINE("module:LinePerceptor:isWhite", pointInImage.x(), pointInImage.y(), referencePointInImage.x(), referencePointInImage.y(), 1, Drawings::solidPen,
             ColorRGBA::orange);
      }
    }
  }
  if(Transformation::robotToImage(innerPointOnField, theCameraMatrix, theCameraInfo, referencePointInImage))
  {
    referencePointInImage = theImageCoordinateSystem.fromCorrected(referencePointInImage);
    Vector2i integerReferenceInImage = referencePointInImage.cast<int>();
    if(integerReferenceInImage.x() >= 0 && integerReferenceInImage.x() < theCameraInfo.width &&
       integerReferenceInImage.y() >= 0 && integerReferenceInImage.y() < theCameraInfo.height)
    {
      if(isOuterPointInImage)
      {
        luminanceReference = (luminanceReference + theECImage.grayscaled[integerReferenceInImage] + 1) / 2;
        saturationReference = (saturationReference + theECImage.saturated[integerReferenceInImage]) / 2;
      }
      else
      {
        luminanceReference = theECImage.grayscaled[integerReferenceInImage];
        saturationReference = theECImage.saturated[integerReferenceInImage];
      }
      if(debugIsPointWhite)
      {
        CROSS("module:LinePerceptor:isWhite", referencePointInImage.x(), referencePointInImage.y(), 2, 1, Drawings::solidPen, ColorRGBA::orange);
        LINE("module:LinePerceptor:isWhite", pointInImage.x(), pointInImage.y(), referencePointInImage.x(), referencePointInImage.y(),
               1, Drawings::solidPen, ColorRGBA::orange);
      }
    }
  }
  return theRelativeFieldColors.isWhiteNearField(theECImage.grayscaled[pointInImage],theECImage.saturated[pointInImage],
                                                  static_cast<unsigned char>(luminanceReference), static_cast<unsigned char>(saturationReference));
}

bool LinePerceptor::isPointWhite(const Vector2f& pointInImage, const Vector2f& n) const
{
  bool isOuterPointInImage = false;
  unsigned short luminanceReference = 0, saturationReference = 0;
  Vector2i outerReference = (pointInImage + n).cast<int>();
  if(outerReference.x() >= 0 && outerReference.x() < theCameraInfo.width &&
      outerReference.y() >= 0 && outerReference.y() < theCameraInfo.height)
  {
    luminanceReference = theECImage.grayscaled[outerReference];
    saturationReference = theECImage.saturated[outerReference];
    isOuterPointInImage = true;
    if(debugIsPointWhite)
    {
      CROSS("module:LinePerceptor:isWhite", outerReference.x(), outerReference.y(), 2, 1, Drawings::solidPen, ColorRGBA::orange);
      LINE("module:LinePerceptor:isWhite", pointInImage.x(), pointInImage.y(), outerReference.x(), outerReference.y(), 1, Drawings::solidPen, ColorRGBA::orange);
    }
  }
  Vector2i innerReference = (pointInImage - n).cast<int>();
  if(innerReference.x() >= 0 && innerReference.x() < theCameraInfo.width &&
     innerReference.y() >= 0 && innerReference.y() < theCameraInfo.height)
  {
    if(isOuterPointInImage)
    {
      luminanceReference = (luminanceReference + theECImage.grayscaled[innerReference] + 1) / 2;
      saturationReference = (saturationReference + theECImage.saturated[innerReference]) / 2;
    }
    else
    {
      luminanceReference = theECImage.grayscaled[innerReference];
      saturationReference = theECImage.saturated[innerReference];
    }
    if(debugIsPointWhite)
    {
      CROSS("module:LinePerceptor:isWhite", innerReference.x(), innerReference.y(), 2, 1, Drawings::solidPen, ColorRGBA::orange);
      LINE("module:LinePerceptor:isWhite", pointInImage.x(), pointInImage.y(), innerReference.x(), innerReference.y(), 1, Drawings::solidPen, ColorRGBA::orange);
    }
  }
  Vector2i intPointInImage = pointInImage.cast<int>();
  return theRelativeFieldColors.isWhiteNearField(theECImage.grayscaled[intPointInImage],theECImage.saturated[intPointInImage],
                                                 static_cast<unsigned char>(luminanceReference), static_cast<unsigned char>(saturationReference));
}

float LinePerceptor::calcWhiteCheckDistance(const Vector2f& pointOnField) const
{
  if(pointOnField.squaredNorm() > squaredWhiteCheckNearField)
    return whiteCheckDistance * (1.f + 0.5f * sqr(1.f - squaredWhiteCheckNearField / pointOnField.squaredNorm()));
  else
    return whiteCheckDistance;
}

float LinePerceptor::calcWhiteCheckDistanceInImage(const Vector2f& pointOnField) const
{
  // sorry for these magic numbers and formulas. I can't explain them, but they kind of work
  if(theCameraInfo.camera == CameraInfo::lower)
  {
    return 15.f + 32.f * (1.f - (pointOnField.x() + 0.5f * std::abs(pointOnField.y())) / 1500.f);
  }
  else
  {
    // avoid computing a root. accurate enough for this purpose
    float yFactor = pointOnField.x() <= 0 ? 1.f : pointOnField.x() <= 350.f ? 0.25f + 0.75f * (1.f - pointOnField.x() / 350.f) : 0.25f;
    float estimatedDistance = pointOnField.x() + yFactor * std::abs(pointOnField.y());
    return 35000.f / estimatedDistance;
  }
}

int LinePerceptor::previousVerticalScanLine(const Spot& spot, int scanLineId) const
{
  ASSERT(scanLineId >= 1);
  int previousScanLine = scanLineId - 1;
  if(highResolutionScan)
    for(int i = 1; i <= 4 && scanLineId - i >= 0; ++i)
    {
      auto regions = theColorScanLineRegionsVerticalClipped.scanLines[scanLineId - i].regions;
      if(regions.empty())
        continue;
      if(static_cast<float>(regions[0].range.lower) >= spot.image.y())
        return scanLineId - i;
    }
  return previousScanLine;
}

bool LinePerceptor::isSpotInsideObstacle(const int fromX, const int toX, const int fromY, const int toY) const
{
  auto predicate = [&](const ObstaclesImagePercept::Obstacle& obstacle)
  {
    if(toX >= obstacle.left && fromX <= obstacle.right && toY >= obstacle.top && fromY <= obstacle.bottom)
      return true;
    else
      return false;
  };
  return std::any_of(theObstaclesImagePercept.obstacles.cbegin(), theObstaclesImagePercept.obstacles.cend(), predicate);
}
