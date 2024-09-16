/**
 * @file IntersectionsCandidatesProvider.cpp
 *
 * This file implements a module that detects and prepares intersections candidates for classifying.
 *
 * @author Arne Böckmann
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author Felix Thielke
 * @author Laurens Schiefelbein
 * @author Kevin Dehmlow
 * @author Roman Sablotny
 *
 */

#include <filesystem>
#include "IntersectionsCandidatesProvider.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/DebugImages.h"
#include "ImageProcessing/Image.h"
#include "Streaming/Output.h"
#include "Tools/Math/Transformation.h"

MAKE_MODULE(IntersectionsCandidatesProvider);

IntersectionsCandidatesProvider::IntersectionsCandidatesProvider() = default;

void IntersectionsCandidatesProvider::update(IntersectionCandidates& intersectionCandidates)
{
  intersectionCandidates.intersections.clear();

  DECLARE_DEBUG_DRAWING("module:IntersectionsCandidatesProvider:field", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:IntersectionsCandidatesProvider:image", "drawingOnImage");

  DEBUG_RESPONSE_ONCE("module:IntersectionsCandidatesProvider:SaveImageOnDisk")
    savePatches = !savePatches;

  // define debug image here, so it will be displayed in SimRobot, will be overwritten when intersections found
  SEND_DEBUG_IMAGE("module:IntersectionsCandidatesProvider:Patch", Image<PixelTypes::GrayscaledPixel>(patchSize, patchSize));

  // partly copied from IntersectionsProvider
  for(unsigned int i = 0; i < theLinesPercept.lines.size(); i++)
  {
    if(theLinesPercept.lines[i].belongsToCircle)
      continue;

    for(unsigned int j = i + 1; j < theLinesPercept.lines.size(); j++)
    {
      if(theLinesPercept.lines[j].belongsToCircle)
        continue;

      // get the intersection point on field
      Vector2f intersectionOnField;
      if(!Geometry::getIntersectionOfLines(theLinesPercept.lines[i].line, theLinesPercept.lines[j].line, intersectionOnField))
        continue;

      // angle between lines in the intersection point
      const float dot = theLinesPercept.lines[i].line.direction.normalized().dot(theLinesPercept.lines[j].line.direction.normalized());
      const float angle = std::acos(std::clamp(dot, -1.f, 1.f));
      const float angleDiff = std::abs(angle - pi_2);
      if(angleDiff > maxAllowedAngleDifference)
        continue;

      const LinesPercept::Line& line1 = theLinesPercept.lines[i];
      const LinesPercept::Line& line2 = theLinesPercept.lines[j];

      // get intersection point on Image
      Vector2f intersectionOnImage;
      if(!Transformation::robotToImage(intersectionOnField, theCameraMatrix, theCameraInfo, intersectionOnImage) || !isWithinBounds(intersectionOnImage))
        continue;

      // check if the intersection is on the line or in threshold
      Vector2f line1CloserEnd;
      Vector2f line1FurtherEnd;
      const float line1DistFromCloserEnd = getCloserPoint(line1.firstField, line1.lastField, intersectionOnField, line1CloserEnd, line1FurtherEnd);

      Vector2f line2CloserEnd;
      Vector2f line2FurtherEnd;
      const float line2DistFromCloserEnd = getCloserPoint(line2.firstField, line2.lastField, intersectionOnField, line2CloserEnd, line2FurtherEnd);

      const bool intersectionIsOnLine1 = isPointInSegment(line1, intersectionOnField);
      const bool intersectionIsOnLine2 = isPointInSegment(line2, intersectionOnField);

      const bool line1EndInIntersection = maxIntersectionGap > line1DistFromCloserEnd;
      const bool line2EndInIntersection = maxIntersectionGap > line2DistFromCloserEnd;

      if((!intersectionIsOnLine1 && !line1EndInIntersection) || (!intersectionIsOnLine2 && !line2EndInIntersection))
        continue;

      IntersectionsPercept::Intersection::IntersectionType type;
      if(line1EndInIntersection && line2EndInIntersection)
      {
        type = IntersectionsPercept::Intersection::L;
      }
      else if(!(line1EndInIntersection || line2EndInIntersection))
      {
        type = IntersectionsPercept::Intersection::X;
      }
      else
      {
        if(line1EndInIntersection || line2EndInIntersection)
          type = IntersectionsPercept::Intersection::T;
        else
          continue;
      }

      Vector2f intersectionOnFieldRelativeRobot;
      Matrix2f cov;
      // the position of the intersection on the field needs to be recalculated again as the transformation is not linear so the maxima of the distribution (previous value of intersection) may not be at the same location as the expected value
      theMeasurementCovariance.transformPointWithCov(intersectionOnImage, 0.f, intersectionOnFieldRelativeRobot, cov);

      Image<PixelTypes::GrayscaledPixel> patch(patchSize, patchSize);
      if(!generatePatch(intersectionOnImage, intersectionOnField, patch))
        continue;

      int robotToIntersectionDistance = static_cast<int>(intersectionOnFieldRelativeRobot.norm());
      if(savePatches)
      {
        savePatchOnDisk(patch, type, robotToIntersectionDistance);
        Image<PixelTypes::GrayscaledPixel> noneIntersectionPatch(patchSize, patchSize);
        int robotToNoneIntersectionDistance = 0;
        if(calculateNoneIntersection(intersectionOnImage, noneIntersectionPatch, robotToNoneIntersectionDistance))
          savePatchOnDisk(noneIntersectionPatch, "none", robotToNoneIntersectionDistance);
      }

      Vector2f fieldCoords = theRobotPose * intersectionOnFieldRelativeRobot;
      COMPLEX_DRAWING("module:IntersectionsCandidatesProvider:field")
      {
        DRAW_TEXT("module:IntersectionsCandidatesProvider:field", fieldCoords.x() + 5, fieldCoords.y() + 5, 10, ColorRGBA::black, std::string(1, static_cast<char>(*(TypeRegistry::getEnumName(type)))));
        CROSS("module:IntersectionsCandidatesProvider:field", fieldCoords.x(), fieldCoords.y(), 60, 30, Drawings::solidPen, ColorRGBA::blue);
      }

      COMPLEX_DRAWING("module:IntersectionsCandidatesProvider:image")
      {
        DRAW_TEXT("module:IntersectionsCandidatesProvider:image", intersectionOnImage.x() + 15, intersectionOnImage.y() + 15, 10, ColorRGBA::black, std::string(1, static_cast<char>(*(TypeRegistry::getEnumName(type)))));
        CROSS("module:IntersectionsCandidatesProvider:image", intersectionOnImage.x(), intersectionOnImage.y(), 5, 3, Drawings::solidPen, ColorRGBA::blue);
      }

      intersectionCandidates.intersections.emplace_back(type, intersectionOnFieldRelativeRobot, intersectionOnField, cov, theLinesPercept.lines[i].line.direction,
                                                        theLinesPercept.lines[j].line.direction, i, j, line1CloserEnd, line2CloserEnd,
                                                        line1FurtherEnd, line2FurtherEnd, static_cast<float>(robotToIntersectionDistance) / normFactor, patch);
    }
  }

  // overwrite debug image with the patch from the first found intersection
  if(!intersectionCandidates.intersections.empty())
    SEND_DEBUG_IMAGE("module:IntersectionsCandidatesProvider:Patch", intersectionCandidates.intersections.front().imagePatch);
}

bool IntersectionsCandidatesProvider::generatePatch(const Vector2f interImg, const Vector2f& interField, Image<PixelTypes::GrayscaledPixel>& patch) const
{
  DECLARE_DEBUG_DRAWING("module:IntersectionsCandidatesProvider:PatchFrame", "drawingOnImage");
  RECTANGLE("module:IntersectionsCandidatesProvider:PatchFrame", interImg.x() - patchSize / 2, interImg.y() - patchSize / 2, interImg.x() + patchSize / 2, interImg.y() + patchSize / 2, 3, Drawings::solidPen, ColorRGBA::blue);

  const float distanceToIntersection = interField.norm();
  const unsigned int stretchingFactor = 2; // Stretch the patch along the y-Axis by this factor
  const unsigned int maxCutoutFactor = 8; // max factor of patchSize for image cutout
  // take bigger image cutout around intersection for down sampling to patchSize
  ASSERT(maxCutoutFactor >= stretchingFactor);
  const unsigned int inputResize = std::clamp(static_cast<unsigned>(ceilf(normFactor / distanceToIntersection)), stretchingFactor, maxCutoutFactor);
  const unsigned int inputSize = patchSize * inputResize; // This is so that distant intersections do not appear too small in the patch
  const unsigned int tempOutputSize = patchSize * stretchingFactor; // Size of the patch that gets stretched
  const Vector2f center(interImg.x(), interImg.y());

  Image<PixelTypes::GrayscaledPixel> firstPatch(tempOutputSize, tempOutputSize);

  // extract patch
  PatchUtilities::extractPatch(center.cast<int>(), Vector2i(inputSize, inputSize), Vector2i(tempOutputSize, tempOutputSize), theECImage.grayscaled, firstPatch);

  // The patch height is 64p. But we need a 32x32 patch. So we cut off the first and last 16 pixels to get the center.
  const unsigned int pixelToCutOff = (tempOutputSize - patchSize) / 2;
  for(unsigned int i = pixelToCutOff; i < firstPatch.height - pixelToCutOff; i++)
  {
    for(unsigned int j = 0; j < firstPatch.width; j += stretchingFactor)
    {
      // Corrected indices for resizedPatch because we only take half of the height and every other value of the width.
      patch[i - pixelToCutOff][j - j / stretchingFactor] = firstPatch[i][j];
    }
  }
  return true;
}

// copied from IntersectionsProvider
template<typename T>
float IntersectionsCandidatesProvider::getCloserPoint(const Eigen::Matrix<T, 2, 1>& a, const Eigen::Matrix<T, 2, 1>& b, const Eigen::Matrix<T, 2, 1>& target, Eigen::Matrix<T, 2, 1>& closer, Eigen::Matrix<T, 2, 1>& further) const
{
  auto getVector2f = [](const Eigen::Matrix<T, 2, 1>& vec)
  {
    return Vector2f(vec.x(), vec.y());
  };

  const float aDist = getVector2f(a - target).squaredNorm();
  const float bDist = getVector2f(b - target).squaredNorm();
  if(aDist < bDist)
  {
    closer = a;
    further = b;
    return std::sqrt(aDist);
  }
  closer = b;
  further = a;
  return std::sqrt(bDist);
}

// copied from IntersectionsProvider
bool IntersectionsCandidatesProvider::isWithinBounds(const Vector2f& intersectionPoint) const
{
  return (intersectionPoint.x() - patchSize / 2 >= 0 &&
          intersectionPoint.x() + patchSize / 2 <= theCameraInfo.width &&
          intersectionPoint.y() - patchSize / 2 >= 0 &&
          intersectionPoint.y() + patchSize / 2 <= theCameraInfo.height);
}

bool IntersectionsCandidatesProvider::isPointInSegment(const LinesPercept::Line& line, const Vector2f& point) const
{
  const float lineSquaredNorm = (line.firstField - line.lastField).squaredNorm();
  return (line.firstField - point).squaredNorm() <= lineSquaredNorm && (line.lastField - point).squaredNorm() <= lineSquaredNorm;
}

void IntersectionsCandidatesProvider::savePatchOnDisk(const Image<PixelTypes::GrayscaledPixel>& patch,
                                                      const IntersectionsPercept::Intersection::IntersectionType type, const int distance) const
{
  std::string typeToStr(1, static_cast<char>(std::tolower(*(TypeRegistry::getEnumName(type)))));
  savePatchOnDisk(patch, typeToStr, distance);
}

void IntersectionsCandidatesProvider::savePatchOnDisk(const Image<PixelTypes::GrayscaledPixel>& patch,
                                                      const std::string& type, const int distance) const
{
  std::filesystem::path baseDir(baseDirectory);
  if(!baseDir.is_absolute())
    baseDir = std::filesystem::current_path() / baseDir;

  const std::string frameTime = std::to_string(theFrameInfo.time);
  const std::string camera = theCameraInfo.camera == CameraInfo::lower ? "lower" : "upper";

  const std::filesystem::path targetDir = baseDir / type;
  std::filesystem::create_directories(targetDir);

  const std::string fileName = logFileName + "-" + frameTime + "-" + camera + "-" + std::to_string(static_cast<int>(normFactor)) + "-" + std::to_string(distance);
  const std::filesystem::path file = targetDir / (fileName + ".bin");

  OutBinaryFile stream(file.string());

  for(unsigned int i = 0; i < patchSize; i++)
    for(unsigned int j = 0; j < patchSize; j++)
      stream << patch[i][j];
}

bool IntersectionsCandidatesProvider::calculateNoneIntersection(const Vector2f& intersectionOnImage, Image<PixelTypes::GrayscaledPixel>& patch,
                                int& robotToNoneIntersectionDistance) const
{
  Vector2f direction(1, 0);
  std::srand(theFrameInfo.time);
  switch(rand() % 4)
  {
    case 0:
      break;
    case 1:
      direction = Vector2f(-1, 0);
      break;
    case 2:
      direction = Vector2f(0, 1);
      break;
    case 3:
      direction = Vector2f(0, -1);
      break;
  }

  const Vector2f noneIntersectionOnImage(intersectionOnImage.x() + (direction.x() * static_cast<float>(patchSize)),
                                         intersectionOnImage.y() + (direction.y() * static_cast<float>(patchSize)));

  if(!isWithinBounds(noneIntersectionOnImage))
    return false;

  Vector2f noneIntersectionOnFieldRelative;
  if(!Transformation::imageToRobot(noneIntersectionOnImage, theCameraMatrix, theCameraInfo, noneIntersectionOnFieldRelative))
    return false;

  robotToNoneIntersectionDistance = static_cast<int>(noneIntersectionOnFieldRelative.norm());
  // calculate field coordinates
  const Vector2f noneIntersectionOnField = theRobotPose * noneIntersectionOnFieldRelative;

  if(!generatePatch(noneIntersectionOnImage, noneIntersectionOnField, patch))
    return false;

  return true;
}
