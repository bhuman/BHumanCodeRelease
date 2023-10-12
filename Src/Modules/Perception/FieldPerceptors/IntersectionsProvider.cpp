/**
 * @file IntersectionsProvider.cpp
 *
 * This file implements a module that detects and classifies intersections of fieldlines.
 *
 * @author Arne Böckmann
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author Felix Thielke
 * @author Laurens Schiefelbein
 */

#include "IntersectionsProvider.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/Stopwatch.h"
#include "Math/Geometry.h"
#include "Platform/File.h"
#include "Tools/Math/Projection.h"
#include "Tools/Math/Transformation.h"
#include <cmath>

MAKE_MODULE(IntersectionsProvider);


IntersectionsProvider::IntersectionsProvider() : network(&Global::getAsmjitRuntime())
{
  // Initialize model for the neural net
  model = std::make_unique<NeuralNetwork::Model>(std::string(File::getBHDir())
    + "/Config/NeuralNets/IntersectionsProvider/intersections_model.h5");
  network.compile(*model);

  if(std::abs(theFieldDimensions.yPosLeftPenaltyArea - theFieldDimensions.yPosLeftGoalArea) > theFieldDimensions.fieldLinesWidth)
    maxIntersectionGapStrict = std::min(maxIntersectionGapStrict, std::abs(theFieldDimensions.yPosLeftPenaltyArea - theFieldDimensions.yPosLeftGoalArea) * 0.7f);
  if(std::abs(theFieldDimensions.yPosLeftSideline - theFieldDimensions.yPosLeftPenaltyArea) > theFieldDimensions.fieldLinesWidth)
    maxIntersectionGapStrict = std::min(maxIntersectionGapStrict, std::abs(theFieldDimensions.yPosLeftSideline - theFieldDimensions.yPosLeftPenaltyArea) * 0.7f);
  if(std::abs(theFieldDimensions.xPosOpponentGoalArea - theFieldDimensions.xPosOpponentPenaltyArea) > theFieldDimensions.fieldLinesWidth)
    maxIntersectionGapStrict = std::min(maxIntersectionGapStrict, std::abs(theFieldDimensions.xPosOpponentGoalArea - theFieldDimensions.xPosOpponentPenaltyArea) * 0.7f);
}

void IntersectionsProvider::update(IntersectionsPercept& intersectionsPercept)
{
  DECLARE_DEBUG_RESPONSE("module:IntersectionsProvider:skipClassification");

  // check whether network has been successfully compiled
  if(!network.valid()) return;

  DECLARE_DEBUG_DRAWING("module:IntersectionsProvider:intersections", "drawingOnImage");
  intersectionsPercept.intersections.clear();

  // Prepare information about ball:
  ballIsInImageAndCanBeUsed = false;
  if(theFrameInfo.getTimeSince(theWorldModelPrediction.timeWhenBallLastSeen) < 1000 &&
     theWorldModelPrediction.ballPosition.norm() > minimumBallExclusionCheckDistance)
  {
    if(Transformation::robotToImage(theWorldModelPrediction.ballPosition, theCameraMatrix, theCameraInfo, ballPositionInImage))
    {
      ballIsInImageAndCanBeUsed = true;
      ballPositionInFieldCoordinates = theWorldModelPrediction.robotPose * theWorldModelPrediction.ballPosition;
      ballRadiusInImageScaled = Projection::getSizeByDistance(theCameraInfo, theBallSpecification.radius,
                                                              theWorldModelPrediction.ballPosition.norm()) * ballRadiusInImageScale;
    }
  }

  // Find pairs of lines that form intersections:
  for(unsigned i = 0; i < theLinesPercept.lines.size(); i++)
  {
    if(theLinesPercept.lines[i].belongsToCircle)
      continue;

    for(unsigned j = i + 1; j < theLinesPercept.lines.size(); j++)
    {
      if(theLinesPercept.lines[j].belongsToCircle)
        continue;

      //only continue if the angle between the two lines is roughly 90°
      const float dot = theLinesPercept.lines[i].line.direction.normalized().dot(theLinesPercept.lines[j].line.direction.normalized());
      const float angle = std::acos(std::clamp(dot, -1.f, 1.f));//angle between lines (in rad)
      const float angleDiff = std::abs(angle - pi_2);
      if(angleDiff > maxAllowedIntersectionAngleDifference)
      {
        continue;
      }

      Vector2f intersection;
      if(Geometry::getIntersectionOfLines(theLinesPercept.lines[i].line, theLinesPercept.lines[j].line, intersection))
      {
        // The Classification already works great for intersection that are close. So we only use the network, when the intersection is far away. In that case we relax the parameters so that more potential intersections will be generated.
        const bool useStrictParameters = intersection.norm() < minClassificationDistance;
        const float maxLengthUnrecognizedProportion = useStrictParameters ? maxLengthUnrecognizedProportionStrict : maxLengthUnrecognizedProportionRelaxed;
        const float maxIntersectionGap = useStrictParameters ? maxIntersectionGapStrict : maxLengthUnrecognizedProportionRelaxed;
        const float maxOverheadToDecleareAsEnd = useStrictParameters ? maxOverheadToDecleareAsEndStrict : maxOverheadToDecleareAsEndRelaxed;

        Vector2f lineEndCloser;//the end of line that is closest to the intersection
        Vector2f line2EndCloser;
        Vector2f lineEndFurther;//the end of line that is further away from the intersection
        Vector2f line2EndFurther;
        const float lineDist = getCloserPoint(theLinesPercept.lines[i].firstField, theLinesPercept.lines[i].lastField, intersection, lineEndCloser, lineEndFurther);
        const float line2Dist = getCloserPoint(theLinesPercept.lines[j].firstField, theLinesPercept.lines[j].lastField, intersection, line2EndCloser, line2EndFurther);

        //FIXME save line length in line to speed up a bit

        const bool intersectionIsOnLine = isPointInSegment(theLinesPercept.lines[i], intersection);
        const bool intersectionIsOnLine2 = isPointInSegment(theLinesPercept.lines[j], intersection);

        const bool intersectionGoesWithLine = lineDist < std::min((theLinesPercept.lines[i].firstField - theLinesPercept.lines[i].lastField).norm() * maxLengthUnrecognizedProportion, maxIntersectionGap);
        const bool intersectionGoesWithLine2 = line2Dist < std::min((theLinesPercept.lines[j].firstField - theLinesPercept.lines[j].lastField).norm() * maxLengthUnrecognizedProportion, maxIntersectionGap);

        if((!intersectionIsOnLine && !intersectionGoesWithLine) || (!intersectionIsOnLine2 && !intersectionGoesWithLine2))
          continue;
        if(ballIsInImageAndCanBeUsed)
        {
          Vector2f intersectionInImage;
          if(Transformation::robotToImage(intersection, theCameraMatrix, theCameraInfo, intersectionInImage))
            if((intersectionInImage - ballPositionInImage).norm() < ballRadiusInImageScaled)
              continue;
        }

        //from here a corner is present

        //in the case that the intersection is on a line we must calc if the line goes through or ends in it
        // (note: if the intersection is not on a line we assume it ends there)
        //beside every other calc is in field coord this decision is made by image coords,
        //                          because the motive of the variance is the image itself
        bool lineIsEnd(true);
        bool line2IsEnd(true);

        if(intersectionIsOnLine || intersectionIsOnLine2)
        {
          auto getVector2f = [&](const Vector2i& vi)
          {
            return Vector2f(vi.x(), vi.y());
          };

          auto minLength = [&](const Vector2f& a, const Vector2f& b)
          {
            const float aSL = a.squaredNorm();
            const float bSL = b.squaredNorm();
            return aSL > bSL ? sqrt(bSL) : sqrt(aSL);
          };

          const Geometry::Line lineImg(getVector2f(theLinesPercept.lines[i].firstImg), getVector2f(theLinesPercept.lines[i].lastImg) - getVector2f(theLinesPercept.lines[i].firstImg));
          const Geometry::Line line2Img(getVector2f(theLinesPercept.lines[j].firstImg), getVector2f(theLinesPercept.lines[j].lastImg) - getVector2f(theLinesPercept.lines[j].firstImg));

          Vector2f intersectionImg;
          if(Geometry::getIntersectionOfLines(lineImg, line2Img, intersectionImg))
          {
            if(intersectionIsOnLine)
              lineIsEnd = maxOverheadToDecleareAsEnd >= minLength(lineImg.base - intersectionImg, getVector2f(theLinesPercept.lines[i].lastImg) - intersectionImg);

            if(intersectionIsOnLine2)
              line2IsEnd = maxOverheadToDecleareAsEnd >= minLength(line2Img.base - intersectionImg, getVector2f(theLinesPercept.lines[j].lastImg) - intersectionImg);
          }
        }

        // The 4 was a 2 before the RoboCup German Open 2018.
        lineIsEnd |= (lineEndCloser - intersection).squaredNorm() < sqr(4 * theFieldDimensions.fieldLinesWidth);
        line2IsEnd |= (line2EndCloser - intersection).squaredNorm() < sqr(4 * theFieldDimensions.fieldLinesWidth);

        if(lineIsEnd && line2IsEnd) // L
          addIntersection(intersectionsPercept, IntersectionsPercept::Intersection::L, intersection, lineEndFurther - intersection, line2EndFurther - intersection, i, j);
        else if(lineIsEnd == line2IsEnd)// X
          addIntersection(intersectionsPercept, IntersectionsPercept::Intersection::X, intersection, theLinesPercept.lines[i].line.direction, theLinesPercept.lines[j].line.direction, i, j);
        else // T
          if(lineIsEnd) //line is vertical
          {
            Vector2f vertical = lineEndFurther - intersection;
            Vector2f horizontal = line2EndFurther - intersection;
            enforceTIntersectionDirections(vertical, horizontal);
            addIntersection(intersectionsPercept, IntersectionsPercept::Intersection::T, intersection, vertical, horizontal, i, j);
          }
          else //line is horizontal
          {
            Vector2f vertical = line2EndFurther - intersection;
            Vector2f horizontal = lineEndFurther - intersection;
            enforceTIntersectionDirections(vertical, horizontal);
            addIntersection(intersectionsPercept, IntersectionsPercept::Intersection::T, intersection, vertical, horizontal, j, i);
          }
      }
    }
  }
}

float IntersectionsProvider::getCloserPoint(const Vector2f& a, const Vector2f& b, const Vector2f target, Vector2f& closer, Vector2f& further) const
{
  const float aDist = (a - target).squaredNorm();
  const float bDist = (b - target).squaredNorm();
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

bool IntersectionsProvider::isPointInSegment(const LinesPercept::Line& line, const Vector2f& point) const
{
  const float lineSquaredNorm = (line.firstField - line.lastField).squaredNorm();
  return (line.firstField - point).squaredNorm() <= lineSquaredNorm && (line.lastField - point).squaredNorm() <= lineSquaredNorm;
}

// note: removed const
void IntersectionsProvider::addIntersection(IntersectionsPercept& intersectionsPercept, IntersectionsPercept::Intersection::IntersectionType type,
                                            Vector2f& intersection, const Vector2f& dir1, const Vector2f& dir2,
                                            unsigned line1, unsigned line2)
{
  Vector2f pInImg;
  if(Transformation::robotToImage(intersection, theCameraMatrix, theCameraInfo, pInImg))
  {
    pInImg = theImageCoordinateSystem.fromCorrected(pInImg);
    DEBUG_RESPONSE_NOT("module:IntersectionsProvider:skipClassification")
    {
      if(threshold > 0.f && !IntersectionsProvider::classifyIntersection(type, intersection, pInImg))
        return;
    }
    Matrix2f cov;

    // the position of the intersection on the field needs to be recalculated again as the transformation is not linear so the maxima of the distribution (previous value of intersection) may not be at the same location as the expected value
    theMeasurementCovariance.transformWithCov(pInImg, 0.f, intersection, cov);
    intersectionsPercept.intersections.emplace_back(type, intersection, cov, dir1, dir2, line1, line2);
  }

  COMPLEX_DRAWING("module:IntersectionsProvider:intersections")
  {
    DRAW_TEXT("module:IntersectionsProvider:intersections", pInImg.x(), pInImg.y(), 30, ColorRGBA::black, TypeRegistry::getEnumName(type));
  }
}

void IntersectionsProvider::enforceTIntersectionDirections(const Vector2f& vertical, Vector2f& horizontal) const
{
  Vector2f vertical90 = vertical;
  vertical90.rotate(pi_2); //vertical rotated by +90°
  if(vertical90.angleTo(horizontal) > pi_2)
  {
    horizontal.mirror();
  }
}

bool IntersectionsProvider::classifyIntersection(IntersectionsPercept::Intersection::IntersectionType& type, const Vector2f& intersection, const Vector2f& pInImg)
{
  // If the distance between intersection and robot is bigger than the distance threshold, the intersection will not be classified by the network.
  if(!isWithinBounds(pInImg) || intersection.norm() < minClassificationDistance)
    return true;

  STOPWATCH("module:IntersectionsProvider:network") {
    const float distanceToIntersection = intersection.norm();

    const int inputSize = distanceToIntersection < 5700 ? 140 : 64; // This is so that distant intersections do not appear too small in the patch
    const int stretchingFactor = 2; // Stretch the patch along the y-Axis by this factor
    const int tempPatchSize = patchSize * stretchingFactor; // Size of the patch that gets stretched
    const Vector2f center(pInImg.x(), pInImg.y());

    Image<PixelTypes::GrayscaledPixel> patch(tempPatchSize, tempPatchSize);
    Image<PixelTypes::GrayscaledPixel> resizedPatch(patchSize, patchSize);

    // extract patch
    PatchUtilities::extractPatch(center.cast<int>(), Vector2i(inputSize, inputSize), Vector2i(tempPatchSize, tempPatchSize), theECImage.grayscaled, patch);

    // The patch height is 64p. But we need a 32x32 patch. So we cut off the first and last 16 pixels to get the center.
    for(unsigned int i = patchSize/2; i < patch.height - patchSize/2; i++)
    {
      // Stretch the image along the x-Axis by taking every other value of the image-width.
      for(unsigned int j = 0; j < patch.width; j += stretchingFactor)
      {
        // Corrected indices for resizedPatch because we only take half of the height and every other value of the width.
        resizedPatch[i - patchSize/2][j - (j/2)] = patch[i][j];
      }
    }

    PatchUtilities::extractPatch(Vector2i(patchSize/2, patchSize/2), Vector2i(patchSize, patchSize), Vector2i(patchSize, patchSize), resizedPatch, network.input(0).data());

    network.apply();
    float l = network.output(0)[0];
    float none = network.output(0)[1];
    float t = network.output(0)[2];
    float x = network.output(0)[3];

    if(none >= threshold + 0.15f)
      return false;
    else if(l >= threshold)
    {
      type = IntersectionsPercept::Intersection::L;
      return true;
    }
    else if(t >= threshold)
    {
      type = IntersectionsPercept::Intersection::T;
      return true;
    }
    // We want to be especially sure before we classify an intersection as x.
    else if(x >= threshold + 0.1f)
    {
      type = IntersectionsPercept::Intersection::X;
      return true;
    }
  }
  // If no prediction passes the threshold, take the original prediction by the IntersectionsProvider.
  return true;
}

bool IntersectionsProvider::isWithinBounds(const Vector2f& intersectionPoint) {
  return (intersectionPoint.x() - patchSize / 2 >= 0 &&
          intersectionPoint.x() + patchSize / 2 <= theCameraInfo.width &&
          intersectionPoint.y() - patchSize / 2 >= 0 &&
          intersectionPoint.y() + patchSize / 2 <= theCameraInfo.height);
}
