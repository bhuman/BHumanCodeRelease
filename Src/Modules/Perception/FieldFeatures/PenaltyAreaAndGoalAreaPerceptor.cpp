/**
 * @file PenaltyAreaAndGoalAreaPerceptor.cpp
 *
 * Implementation of a module that tries to find the penalty area in the current field percepts.
 * The module searches two different combinations of field lines and intersections and computes a field feature if
 * one of those is found.
 *
 * The first approach is looking for three parallel lines, corresponding to the goal-, goal area- and penalty area lines, and either corner of the penalty area (A or D).
 * The second approach is used, if the first is not successful and looks for two intersections, corresponding to the penalty area and goal area (A and B or C and D).
 *
 * ========================(goal line)=======================
 *    |         |                             |         |
 *    |         |                             |         |
 *    |         B-----(goal area line)--------C         |
 *    |                                                 |
 *    |                         X                       |
 *    A------------------(penalty area line)------------D
 *
 * @author Nico Wellbrock
 * @author Tim Gebers
 */

#include "PenaltyAreaAndGoalAreaPerceptor.h"
#include "Math/Geometry.h"
#include "Debugging/DebugDrawings.h"

void PenaltyAreaAndGoalAreaPerceptor::update(PenaltyAreaAndGoalArea& penaltyAreaAndGoalArea)
{
  DECLARE_DEBUG_DRAWING("module:PenaltyAreaAndGoalAreaPerceptor:field", "drawingOnField");

  if(searchParallelLines() && searchIntersectionOnPenaltyAreaLine())
  {
    // This drawing marks the detected lines and intersections
    COMPLEX_DRAWING("module:PenaltyAreaAndGoalAreaPerceptor:field")
    {
      LINE("module:PenaltyAreaAndGoalAreaPerceptor:field", penaltyAreaLine.first.x(), penaltyAreaLine.first.y(), penaltyAreaLine.last.x(), penaltyAreaLine.last.y(), 10, Drawings::solidPen, ColorRGBA::violet);
      LINE("module:PenaltyAreaAndGoalAreaPerceptor:field", goalAreaLine.first.x(), goalAreaLine.first.y(), goalAreaLine.last.x(), goalAreaLine.last.y(), 10, Drawings::solidPen, ColorRGBA::violet);
      LINE("module:PenaltyAreaAndGoalAreaPerceptor:field", goalLine.first.x(), goalLine.first.y(), goalLine.last.x(), goalLine.last.y(), 10, Drawings::solidPen, ColorRGBA::violet);
      CROSS("module:PenaltyAreaAndGoalAreaPerceptor:field", intersectionOnPenaltyAreaLine.pos.x(), intersectionOnPenaltyAreaLine.pos.y(), 10, 10, Drawings::solidPen, ColorRGBA::violet);
    }

    computeFeatureFrom3Lines(penaltyAreaAndGoalArea);

    return;
  }
  else if(searchTwoIntersectionsOfPenaltyAndGoalArea())
  {
    computeFeatureFrom2Corners(penaltyAreaAndGoalArea);

    return;
  }

  penaltyAreaAndGoalArea.isValid = false;
}

void PenaltyAreaAndGoalAreaPerceptor::computeFeatureFrom3Lines(PenaltyAreaAndGoalArea& penaltyAreaAndGoalArea)
{
  Matrix2f inverseRotationMatrix = (Matrix2f() << std::cos(penaltyAreaLine.alpha), -std::sin(penaltyAreaLine.alpha), std::sin(penaltyAreaLine.alpha), std::cos(penaltyAreaLine.alpha)).finished();
  Matrix2f rotationMatrix = inverseRotationMatrix.transpose();
  Vector2f penaltyLineCenter = (penaltyAreaLine.first + penaltyAreaLine.last) / 2;
  // Take x-coordinate from penalty area line and y-coordinate from intersection
  Vector2f pos = rotationMatrix * penaltyLineCenter;
  pos.y() = (rotationMatrix * intersectionOnPenaltyAreaLine.pos).y();
  pos = inverseRotationMatrix * pos;

  // dir2 == dir1 + 90_deg, thus we are looking at the left corner (when facing the goal) if dir1 is pointing along the goal area line
  // Check if intersection is on left or right side of penalty area line
  Angle angle1 = intersectionOnPenaltyAreaLine.dir1.angleTo((penaltyAreaLine.last - penaltyAreaLine.first).normalized());
  Angle angle2 = intersectionOnPenaltyAreaLine.dir2.angleTo((penaltyAreaLine.first - penaltyAreaLine.last).normalized());
  if(angle1 <= maxAngleDeviationParallelVectors)
  {
    Pose2f perceivedPose(penaltyAreaLine.alpha, pos);
    perceivedPose = perceivedPose * (Vector2f(penaltyAreaToGoalDistance / 2, -penaltyAreaLineLength / 2));
    perceivedPose.rotation = penaltyAreaLine.alpha;
    penaltyAreaAndGoalArea = perceivedPose;
  }
  else if(angle2 <= maxAngleDeviationParallelVectors)
  {
    Pose2f perceivedPose(penaltyAreaLine.alpha, pos);
    perceivedPose = perceivedPose * Vector2f(penaltyAreaToGoalDistance / 2, penaltyAreaLineLength / 2);
    perceivedPose.rotation = penaltyAreaLine.alpha;
    penaltyAreaAndGoalArea = perceivedPose;
  }
  else
  {
    penaltyAreaAndGoalArea.isValid = false;
    return;
  }
  penaltyAreaAndGoalArea.isValid = true;
  // Compute covariance matrix
  Pose2f computedPose;
  if(penaltyAreaAndGoalArea.pickMorePlausiblePose(theWorldModelPrediction.robotPose, computedPose))
  {
    const float s = std::sin(penaltyAreaAndGoalArea.rotation);
    const float c = std::cos(penaltyAreaAndGoalArea.rotation);
    const Matrix2f angleRotationMatrix = (Matrix2f() << c, -s, s, c).finished();
    const Matrix2f covXR = angleRotationMatrix * penaltyAreaLine.cov * angleRotationMatrix.transpose();
    const Matrix2f covY = angleRotationMatrix * intersectionOnPenaltyAreaLine.cov * angleRotationMatrix.transpose();
    const float xVariance = covXR(0, 0); // Depends on penaltyAreaLine
    const float yVariance = covY(1, 1);  // Depends on intersection
    const float sqrPenaltyAreaLineLength = (penaltyAreaLine.last - penaltyAreaLine.first).squaredNorm();
    const float angleVariance = sqr(std::atan(std::sqrt(4.f * xVariance / sqrPenaltyAreaLineLength)));
    penaltyAreaAndGoalArea.covOfAbsoluteRobotPose << xVariance, 0.f, 0.f, 0.f, yVariance, 0.f, 0.f, 0.f, angleVariance;
  }
  else
    penaltyAreaAndGoalArea.isValid = false;
}

void PenaltyAreaAndGoalAreaPerceptor::computeFeatureFrom2Corners(PenaltyAreaAndGoalArea& penaltyAreaAndGoalArea)
{
  // Check if intersection is on left or right side of penalty area
  Angle angle = intersectionOnPenaltyAreaLine.dir1.angleTo((intersectionOnGoalAreaLine.pos - intersectionOnPenaltyAreaLine.pos).normalized());
  if(angle >= maxAngleDeviationTwoCorners && 180_deg - maxAngleDeviationTwoCorners >= angle)
  {
    Pose2f perceivedPose(intersectionOnPenaltyAreaLine.dir2.angle(), intersectionOnPenaltyAreaLine.pos);
    perceivedPose = perceivedPose * Vector2f(penaltyAreaToGoalDistance / 2, -penaltyAreaLineLength / 2);
    perceivedPose.rotation = intersectionOnPenaltyAreaLine.dir2.angle();
    penaltyAreaAndGoalArea = perceivedPose;
  }
  else
  {
    Pose2f perceivedPose(intersectionOnPenaltyAreaLine.dir1.angle(), intersectionOnPenaltyAreaLine.pos);
    perceivedPose = perceivedPose * Vector2f(penaltyAreaToGoalDistance / 2, penaltyAreaLineLength / 2);
    perceivedPose.rotation = intersectionOnPenaltyAreaLine.dir1.angle();
    penaltyAreaAndGoalArea = perceivedPose;
  }
  penaltyAreaAndGoalArea.isValid = true;
  // compute covariance matrix
  Pose2f computedPose;
  if(penaltyAreaAndGoalArea.pickMorePlausiblePose(theWorldModelPrediction.robotPose, computedPose))
  {
    const Matrix2x3f relativeLandmarkJacobianPenaltyIntersection = relativeLandmarkJacobian(penaltyAreaAndGoalArea, intersectionOnPenaltyAreaLine.pos);
    const Matrix2x3f relativeLandmarkJacobianGoalIntersection = relativeLandmarkJacobian(penaltyAreaAndGoalArea, intersectionOnGoalAreaLine.pos);
    const Matrix3f intersectionOnPenaltyAreaLineCov = relativeLandmarkJacobianPenaltyIntersection.transpose() * intersectionOnPenaltyAreaLine.cov.inverse() * relativeLandmarkJacobianPenaltyIntersection;
    const Matrix3f intersectionOnGoalAreaLineCov = relativeLandmarkJacobianGoalIntersection.transpose() * intersectionOnGoalAreaLine.cov.inverse() * relativeLandmarkJacobianGoalIntersection;
    penaltyAreaAndGoalArea.covOfAbsoluteRobotPose = (intersectionOnPenaltyAreaLineCov + intersectionOnGoalAreaLineCov).inverse();
  }
  else
    penaltyAreaAndGoalArea.isValid = false;
}


Matrix2x3f PenaltyAreaAndGoalAreaPerceptor::relativeLandmarkJacobian(const Pose2f& pose, const Vector2f& globalLandmark)
{
  const float c = std::cos(pose.rotation), s = std::sin(pose.rotation);
  const Vector2f rightColumn = (Matrix2f() << -s, c, -c, -s).finished() * (globalLandmark - pose.translation);
  return (Matrix2x3f() << -c, -s, rightColumn.x(),
    s, -c, rightColumn.y()).finished();
}

bool PenaltyAreaAndGoalAreaPerceptor::searchParallelLines()
{
  const std::vector<FieldLines::Line>& lines = theFieldLines.lines;

  // We need at least three lines
  if(lines.size() < 3)
    return false;

  // Look for lines that are parallel and have the right distance to each other
  for(auto penaltyAreaLineIt = lines.begin(); penaltyAreaLineIt < lines.end(); penaltyAreaLineIt++)
  {
    for(auto goalAreaLineIt = lines.begin(); goalAreaLineIt < lines.end(); goalAreaLineIt++)
    {
      if(penaltyAreaLineIt == goalAreaLineIt || (std::abs(goalAreaLineIt->alpha - penaltyAreaLineIt->alpha) > maxAngleDeviationParallelLines)
         || std::abs(Geometry::getDistanceToLine(Geometry::Line(penaltyAreaLineIt->first, penaltyAreaLineIt->first - penaltyAreaLineIt->last), goalAreaLineIt->first) - penaltyAreaToGoalAreaDistance) > maxLineDistanceDeviation)
        continue;

      for(auto goalLineIt = lines.begin(); goalLineIt < lines.end(); goalLineIt++)
      {
        if(goalLineIt != goalAreaLineIt && goalLineIt != penaltyAreaLineIt
           && (std::abs(goalLineIt->alpha - penaltyAreaLineIt->alpha) <= maxAngleDeviationParallelLines)
           && std::abs(Geometry::getDistanceToLine(Geometry::Line(goalAreaLineIt->first, goalAreaLineIt->last - goalAreaLineIt->first), goalLineIt->first) - goalAreaToGoalDistance) <= maxLineDistanceDeviation)
        {
          penaltyAreaLine = *penaltyAreaLineIt;
          goalAreaLine = *goalAreaLineIt;
          goalLine = *goalLineIt;
          return true;
        }
      }
    }
  }
  return false;
}

bool PenaltyAreaAndGoalAreaPerceptor::searchTwoIntersectionsOfPenaltyAndGoalArea()
{
  const std::vector<FieldLineIntersections::Intersection>& intersections = theFieldLineIntersections.intersections;

  // We need at least two intersections
  if(intersections.size() < 2)
    return false;

  // Look for intersections that have the right distance from each other and where the directions are parallel
  for(auto intersectionOnPenaltyAreaLineIt = intersections.begin(); intersectionOnPenaltyAreaLineIt < intersections.end(); intersectionOnPenaltyAreaLineIt++)
  {
    if(intersectionOnPenaltyAreaLineIt->type == FieldLineIntersections::Intersection::IntersectionType::L)
    {
      for(auto intersectionOnGoalAreaLineIt = intersections.begin(); intersectionOnGoalAreaLineIt < intersections.end(); intersectionOnGoalAreaLineIt++)
      {
        if(intersectionOnPenaltyAreaLineIt != intersectionOnGoalAreaLineIt && intersectionOnPenaltyAreaLineIt->type == FieldLineIntersections::Intersection::IntersectionType::L
           && std::abs(Geometry::distance(intersectionOnGoalAreaLineIt->pos, intersectionOnPenaltyAreaLineIt->pos) - penaltyAndGoalAreaIntersectionDistance) < maxIntersectionDistanceDeviation
           && (std::abs(intersectionOnGoalAreaLineIt->dir1.angle() - intersectionOnPenaltyAreaLineIt->dir1.angle()) < maxAngleDeviationParallelLines)
           && (Geometry::distance(intersectionOnGoalAreaLineIt->pos, intersectionOnPenaltyAreaLineIt->pos)) >
           (Geometry::distance(intersectionOnGoalAreaLineIt->pos, (intersectionOnPenaltyAreaLineIt->pos + intersectionOnPenaltyAreaLineIt->dir1 + intersectionOnPenaltyAreaLineIt->dir2))))
        {
          intersectionOnPenaltyAreaLine = *intersectionOnPenaltyAreaLineIt;
          intersectionOnGoalAreaLine = *intersectionOnGoalAreaLineIt;
          return true;
        }
      }
    }
  }
  return false;
}

bool PenaltyAreaAndGoalAreaPerceptor::searchIntersectionOnPenaltyAreaLine()
{
  for(const auto& intersection : theFieldLineIntersections.intersections)
  {
    // Test if intersection is on penalty area line
    if(intersection.type == FieldLineIntersections::Intersection::IntersectionType::L
       && (Geometry::distance(intersection.pos, penaltyAreaLine.first) <= maxPointDeviation || Geometry::distance(intersection.pos, penaltyAreaLine.last) <= maxPointDeviation))
    {
      intersectionOnPenaltyAreaLine = intersection;
      return true;
    }
  }
  return false;
}

MAKE_MODULE(PenaltyAreaAndGoalAreaPerceptor);
