
#include "IntersectionsProvider.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Projection.h"
#include "Tools/Math/Transformation.h"

MAKE_MODULE(IntersectionsProvider, perception)

void IntersectionsProvider::update(IntersectionsPercept& intersectionsPercept)
{
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
      ballPositionInFieldCoordinates = Transformation::robotToField(theWorldModelPrediction.robotPose, theWorldModelPrediction.ballPosition);
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
      const float angle = std::acos(dot);//angle between lines (in rad)
      const float angleDiff = std::abs(angle - pi_2);
      if(angleDiff > maxAllowedIntersectionAngleDifference)
      {
        continue;
      }

      Vector2f intersection;
      if(Geometry::getIntersectionOfLines(theLinesPercept.lines[i].line, theLinesPercept.lines[j].line, intersection))
      {
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

void IntersectionsProvider::addIntersection(IntersectionsPercept& intersectionsPercept, IntersectionsPercept::Intersection::IntersectionType type,
    const Vector2f& intersection, const Vector2f& dir1, const Vector2f& dir2,
    unsigned line1, unsigned line2) const
{
  COMPLEX_DRAWING("module:IntersectionsProvider:intersections")
  {
    Vector2f pInImg;
    if(Transformation::robotToImage(intersection, theCameraMatrix, theCameraInfo, pInImg))
      DRAWTEXT("module:IntersectionsProvider:intersections", pInImg.x(), pInImg.y(), 30,
               ColorRGBA::black, TypeRegistry::getEnumName(type));
  }
  intersectionsPercept.intersections.emplace_back(type, intersection, dir1, dir2, line1, line2);
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
