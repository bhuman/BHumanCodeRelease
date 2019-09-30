/**
 * @file FieldBoundaryProvider.cpp
 *
 * This file implements a module that estimates the field boundary using
 * a RANSAC algorithm for a single straight line or two intersecting
 * straight lines.
 *
 * @author Thomas Röfer
 */

#include "FieldBoundaryProvider.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Random.h"
#include "Tools/Math/Transformation.h"
#include <limits>

MAKE_MODULE(FieldBoundaryProvider, perception)

void FieldBoundaryProvider::update(FieldBoundary& theFieldBoundary)
{
  DECLARE_DEBUG_DRAWING("module:FieldBoundaryProvider:spots", "drawingOnImage");

  // Only with a valid camera matrix, the field boundary can be computed.
  if(theCameraMatrix.isValid)
  {
    // Propagate the previous field boundary to the current image.
    if(theOtherFieldBoundary.isValid)
      predict(theFieldBoundary);
    else
      theFieldBoundary.isValid = false;

    // Find field boundary spots. These might also simply be the predicted ones.
    std::vector<Spot> spots;
    spots.reserve(theColorScanLineRegionsVertical.scanLines.size());
    STOPWATCH("FieldBoundaryProvider:findSpots")
      findSpots(theFieldBoundary, spots);

    // If enough spots were found, compute new field boundary.
    // Otherwise, the predicted one is used instead.
    if(spots.size() >= minNumberOfSpots)
    {
      std::vector<Spot> model;
      model.reserve(3);
      STOPWATCH("FieldBoundaryProvider:calcBoundary")
        calcBoundary(spots, model);

      // Fill representation with the boundary found.
      fillRepresentation(model, theFieldBoundary);
    }
  }
  else
    theFieldBoundary.isValid = false;

  if(!theFieldBoundary.isValid)
  {
    theFieldBoundary.boundaryInImage.clear();
    theFieldBoundary.boundaryOnField.clear();
  }
}

void FieldBoundaryProvider::predict(FieldBoundary& fieldBoundary) const
{
  const Pose2f invOdometryOffset = theOdometer.odometryOffset.inverse();
  fieldBoundary.boundaryInImage.clear();
  fieldBoundary.boundaryOnField.clear();
  for(Vector2f spotOnField : theOtherFieldBoundary.boundaryOnField)
  {
    Vector2f spotInImage;
    spotOnField = invOdometryOffset * spotOnField;
    if(Transformation::robotToImage(spotOnField, theCameraMatrix, theCameraInfo, spotInImage))
    {
      fieldBoundary.boundaryInImage.emplace_back(theImageCoordinateSystem.fromCorrected(spotInImage).cast<int>());
      fieldBoundary.boundaryOnField.emplace_back(spotOnField);
    }
  }
  fieldBoundary.isValid = fieldBoundary.boundaryInImage.size() > 1;
}

void FieldBoundaryProvider::findSpots(const FieldBoundary& fieldBoundary, std::vector<Spot>& spots) const
{
  for(const ColorScanLineRegionsVertical::ScanLine& scanLine : theColorScanLineRegionsVertical.scanLines)
  {
    // Use point from previous field boundary if it is above lower or below upper image.
    if(fieldBoundary.isValid)
    {
      int y = fieldBoundary.getBoundaryY(scanLine.x);
      if((theCameraInfo.camera == CameraInfo::upper && y >= theCameraInfo.height)
         || (theCameraInfo.camera == CameraInfo::lower && y < 0))
      {
        Vector2i spotInImage(scanLine.x, y);
        Vector2f spotOnField;
        if(Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(spotInImage), theCameraMatrix, theCameraInfo, spotOnField)
           && spotOnField.squaredNorm() >= sqr(minDistance))
        {
          spots.emplace_back(spotInImage, spotOnField);
          CROSS("module:FieldBoundaryProvider:spots", spotInImage.x(), spotInImage.y(), 3, 1, Drawings::solidPen, ColorRGBA::red);
          continue;
        }
      }
    }

    // Search boundary spot as the end of the most field-ish region sequence starting at the bottom.
    int score = 0;
    int maxScore = 0;
    int maxUpper = theCameraInfo.height;
    for(const ScanLineRegion& region : scanLine.regions)
    {
      score += (region.is(FieldColors::field) ? 1 : -1) * (region.range.lower - region.range.upper);
      if(score >= maxScore)
      {
        maxScore = score;
        maxUpper = region.range.upper;
      }
    }

    // Add spot if it can be projected to the field.
    Vector2i spotInImage(scanLine.x, maxUpper);
    Vector2f spotOnField;
    if(Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(spotInImage), theCameraMatrix, theCameraInfo, spotOnField)
       && spotOnField.squaredNorm() >= sqr(minDistance))
    {
      spots.emplace_back(spotInImage, spotOnField);
      CROSS("module:FieldBoundaryProvider:spots", spotInImage.x(), spotInImage.y(), 3, 1, Drawings::solidPen, ColorRGBA::orange);
    }
  }
}

void FieldBoundaryProvider::calcBoundary(const std::vector<Spot>& spots, std::vector<Spot>& model) const
{
  int minError = std::numeric_limits<int>::max();
  const int goodEnough = static_cast<int>(maxSquaredError * spots.size() * acceptanceRatio);

  for(int i = 0; i < maxNumberOfIterations && minError > goodEnough; ++i)
  {
    // Draw three unique samples sorted by their x-coordinate.
    const size_t middleIndex = Random::uniformInt(static_cast<size_t>(1), spots.size() - 2);
    const Spot& leftSpot = spots[Random::uniformInt(middleIndex - 1)];
    const Spot& middleSpot = spots[middleIndex];
    const Spot& rightSpot = spots[Random::uniformInt(middleIndex + 1, spots.size() - 1)];

    // Construct lines, second is perpendicular to first one on the field.
    Vector2f dirOnField = middleSpot.onField - leftSpot.onField;
    const Geometry::Line leftOnField(leftSpot.onField, dirOnField);
    const Geometry::Line rightOnField(rightSpot.onField, dirOnField.rotateLeft()); // Changes dirOnField!

    // Compute hypothetical corner in field coordinates.
    Vector2f inImage;
    Spot corner;
    if(Geometry::getIntersectionOfLines(leftOnField, rightOnField, corner.onField)
       && Transformation::robotToImage(corner.onField, theCameraMatrix, theCameraInfo, inImage))
    {
      corner.inImage = theImageCoordinateSystem.fromCorrected(inImage).cast<int>();

      // Corner must be right of left spot, left of the right spot, and above connecting line.
      if(corner.inImage.x() <= leftSpot.inImage.x() || corner.inImage.x() >= rightSpot.inImage.x()
         || corner.inImage.y() >= leftSpot.inImage.y() + (corner.inImage.x() - leftSpot.inImage.x())
         * (rightSpot.inImage.y() - leftSpot.inImage.y()) / (rightSpot.inImage.x() - leftSpot.inImage.x()))
        corner.inImage.x() = theCameraInfo.width; // It is not -> ignore
    }
    else
      corner.inImage.x() = theCameraInfo.width; // Corner invalid -> ignore

    const Vector2i dirLeft = middleSpot.inImage - leftSpot.inImage;
    const Vector2i dirRight = rightSpot.inImage - corner.inImage;

    size_t j = 0;

    // Accumulate errors in image coordinates for left line.
    int errorLeft = 0;
    while(j < spots.size() && spots[j].inImage.x() < corner.inImage.x() && errorLeft < minError)
    {
      const Vector2i& s = spots[j++].inImage;
      errorLeft += effectiveError(leftSpot.inImage.y() + dirLeft.y() * (s.x() - leftSpot.inImage.x()) / dirLeft.x() - s.y());
    }

    // Accumulate errors in image coordinates, assuming both a continuing left line and a separate right line.
    int errorRightLine = 0; // Assuming a separate line on the right.
    int errorRightStraight = 0;// Assuming left line continues.
    const int abortError = minError - errorLeft;
    while(j < spots.size() && std::min(errorRightLine, errorRightStraight) < abortError)
    {
      const Vector2i& s = spots[j++].inImage;
      errorRightLine += effectiveError(corner.inImage.y() + dirRight.y() * (s.x() - corner.inImage.x()) / dirRight.x() - s.y());
      errorRightStraight += effectiveError(leftSpot.inImage.y() + dirLeft.y() * (s.x() - leftSpot.inImage.x()) / dirLeft.x() - s.y());
    }

    // Update model if it is better than the best found so far.
    const int error = errorLeft + std::min(errorRightLine, errorRightStraight);
    if(error < minError)
    {
      minError = error;
      model.clear();
      model.emplace_back(leftSpot);

      if(errorRightLine < errorRightStraight)
      {
        model.emplace_back(corner);
        model.emplace_back(rightSpot);
      }
      else
        model.emplace_back(middleSpot);
    }
  }
}

void FieldBoundaryProvider::fillRepresentation(const std::vector<Spot>& model, FieldBoundary& fieldBoundary) const
{
  fieldBoundary.boundaryInImage.clear();
  fieldBoundary.boundaryOnField.clear();
  for(const Spot& spot : model)
  {
    fieldBoundary.boundaryInImage.emplace_back(spot.inImage);
    fieldBoundary.boundaryOnField.emplace_back(spot.onField);
  }
  fieldBoundary.isValid = fieldBoundary.boundaryInImage.size() > 1;
}
