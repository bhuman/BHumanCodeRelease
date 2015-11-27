/**
* @file FieldModel.h
*
* This file implements a submodule that represents the robot's environment for self-localization
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
* @author Colin Graf
*/

#include "FieldModel.h"
#include "SelfLocator.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Tools/Math/Geometry.h"

FieldModel::FieldModel(const FieldDimensions& fieldDimensions, const SelfLocatorBase::Parameters& parameters,
                       const CameraMatrix& cameraMatrix):
  parameters(parameters), cameraMatrix(cameraMatrix)
{
  // Initialize goal posts
  goalPosts[0] = Vector2f(fieldDimensions.xPosOwnGoalPost, fieldDimensions.yPosRightGoal);
  goalPosts[1] = Vector2f(fieldDimensions.xPosOwnGoalPost, fieldDimensions.yPosLeftGoal);
  goalPosts[2] = Vector2f(fieldDimensions.xPosOpponentGoalPost, fieldDimensions.yPosLeftGoal);
  goalPosts[3] = Vector2f(fieldDimensions.xPosOpponentGoalPost, fieldDimensions.yPosRightGoal);
  unknownGoalAcceptanceThreshold = (2.f * fieldDimensions.yPosLeftGoal) / 3.f;
  knownGoalAcceptanceThreshold = (2.f * fieldDimensions.yPosLeftGoal) / 1.5f;
  
  // Initialize penalty marks
  ownPenaltyMark =      Vector2f(fieldDimensions.xPosOwnPenaltyMark, 0.f);
  opponentPenaltyMark = Vector2f(fieldDimensions.xPosOpponentPenaltyMark, 0.f);

  // Initialize list of relevant field lines
  for(size_t i = 0, count = fieldDimensions.fieldLines.lines.size(); i < count; ++i)
  {
    const FieldDimensions::LinesTable::Line& fieldLine = fieldDimensions.fieldLines.lines[i];
    if(!fieldLine.isPartOfCircle && (fieldLine.to - fieldLine.from).norm() > 300.f)
    {
      FieldLine relevantFieldLine;
      relevantFieldLine.start = fieldLine.from;
      relevantFieldLine.end = fieldLine.to;
      relevantFieldLine.dir = relevantFieldLine.end - relevantFieldLine.start;
      relevantFieldLine.dir.normalize();
      relevantFieldLine.length = (fieldLine.to - fieldLine.from).norm();
      relevantFieldLine.vertical = std::abs(fieldLine.from.y() - fieldLine.to.y()) < 0.001f;
      fieldLines.push_back(relevantFieldLine);
    }
  }
  if(parameters.goalFrameIsPerceivedAsLines)
  {
    for(size_t i = 0, count = fieldDimensions.goalFrameLines.lines.size(); i < count; ++i)
    {
      const FieldDimensions::LinesTable::Line& fieldLine = fieldDimensions.goalFrameLines.lines[i];
      FieldLine relevantFieldLine;
      relevantFieldLine.start = fieldLine.from;
      relevantFieldLine.end = fieldLine.to;
      relevantFieldLine.dir = relevantFieldLine.end - relevantFieldLine.start;
      relevantFieldLine.dir.normalize();
      relevantFieldLine.length = (fieldLine.to - fieldLine.from).norm();
      relevantFieldLine.vertical = std::abs(fieldLine.from.y() - fieldLine.to.y()) < 0.001f;
      fieldLines.push_back(relevantFieldLine);
    }
  }

  // Initialize corner lists:
  // X
  xCorners.push_back(Vector2f(fieldDimensions.xPosHalfWayLine, fieldDimensions.centerCircleRadius));
  xCorners.push_back(Vector2f(fieldDimensions.xPosHalfWayLine, -fieldDimensions.centerCircleRadius));
  // T
  tCorners = xCorners;
  tCorners.push_back(Vector2f(fieldDimensions.xPosHalfWayLine, fieldDimensions.yPosRightSideline));
  tCorners.push_back(Vector2f(fieldDimensions.xPosHalfWayLine, fieldDimensions.yPosLeftSideline));
  tCorners.push_back(Vector2f(fieldDimensions.xPosOwnGroundline, fieldDimensions.yPosLeftPenaltyArea));
  tCorners.push_back(Vector2f(fieldDimensions.xPosOwnGroundline, fieldDimensions.yPosRightPenaltyArea));
  tCorners.push_back(Vector2f(fieldDimensions.xPosOpponentGroundline, fieldDimensions.yPosLeftPenaltyArea));
  tCorners.push_back(Vector2f(fieldDimensions.xPosOpponentGroundline, fieldDimensions.yPosRightPenaltyArea));
  // L
  lCorners = tCorners;
  lCorners.push_back(Vector2f(fieldDimensions.xPosOpponentGroundline, fieldDimensions.yPosRightSideline));
  lCorners.push_back(Vector2f(fieldDimensions.xPosOpponentGroundline, fieldDimensions.yPosLeftSideline));
  lCorners.push_back(Vector2f(fieldDimensions.xPosOwnGroundline, fieldDimensions.yPosRightSideline));
  lCorners.push_back(Vector2f(fieldDimensions.xPosOwnGroundline, fieldDimensions.yPosLeftSideline));
  lCorners.push_back(Vector2f(fieldDimensions.xPosOwnPenaltyArea, fieldDimensions.yPosRightPenaltyArea));
  lCorners.push_back(Vector2f(fieldDimensions.xPosOwnPenaltyArea, fieldDimensions.yPosLeftPenaltyArea));
  lCorners.push_back(Vector2f(fieldDimensions.xPosOpponentPenaltyArea, fieldDimensions.yPosRightPenaltyArea));
  lCorners.push_back(Vector2f(fieldDimensions.xPosOpponentPenaltyArea, fieldDimensions.yPosLeftPenaltyArea));
  if(parameters.goalFrameIsPerceivedAsLines)
  {
    lCorners.push_back(Vector2f(fieldDimensions.xPosOwnGoal, fieldDimensions.yPosLeftGoal));
    lCorners.push_back(Vector2f(fieldDimensions.xPosOwnGoal, fieldDimensions.yPosRightGoal));
    lCorners.push_back(Vector2f(fieldDimensions.xPosOpponentGoal, fieldDimensions.yPosLeftGoal));
    lCorners.push_back(Vector2f(fieldDimensions.xPosOpponentGoal, fieldDimensions.yPosRightGoal));
  }
}

bool FieldModel::getAssociatedPenaltyMark(const Pose2f& robotPose, const Vector2f& penaltyMarkPercept, Vector2f& associatedPenaltyMark) const
{
  const Vector2f penaltyMarkInWorld = robotPose * penaltyMarkPercept;
  associatedPenaltyMark = penaltyMarkInWorld.x() <= 0.f ? ownPenaltyMark : opponentPenaltyMark;
  const float differenceBetweenPerceptAndModel = (penaltyMarkInWorld - associatedPenaltyMark).norm();
  return differenceBetweenPerceptAndModel < parameters.penaltyMarkAssociationDistance;
}

bool FieldModel::getAssociatedUnknownGoalPost(const Pose2f& robotPose, const Vector2f& goalPercept, Vector2f& associatedPost) const
{
  const Vector2f postInWorld = robotPose * goalPercept;
  if(postInWorld.x() <= 0.f) // own half
  {
    if(postInWorld.y() <= 0.f) // right post
      associatedPost = goalPosts[0];
    else // left post
      associatedPost = goalPosts[1];
  }
  else // opponent half
  {
    if(postInWorld.y() > 0.f) // left post
      associatedPost = goalPosts[2];
    else // right post
      associatedPost = goalPosts[3];
  }
  return goalPostIsValid(postInWorld, associatedPost, robotPose, unknownGoalAcceptanceThreshold);
}

bool FieldModel::getAssociatedKnownGoalPost(const Pose2f& robotPose, const Vector2f& goalPercept, bool isLeft, Vector2f& associatedPost) const
{
  const Vector2f postInWorld = robotPose * goalPercept;
  if(postInWorld.x() <= 0.f) // own half
  {
     if(isLeft) // right post [!]
       associatedPost = goalPosts[0];
     else // left post
       associatedPost = goalPosts[1];
  }
  else // opponent half
  {
    if(isLeft) // left post
      associatedPost = goalPosts[2];
    else // right post
      associatedPost = goalPosts[3];
  }
  return goalPostIsValid(postInWorld, associatedPost, robotPose, knownGoalAcceptanceThreshold);
}

int FieldModel::getIndexOfAssociatedLine(const Pose2f& robotPose, const Vector2f& start, const Vector2f& end) const
{
  Vector2f startOnField = robotPose * start;
  Vector2f endOnField = robotPose * end;
  Vector2f dirOnField = endOnField - startOnField;
  dirOnField.normalize();
  Vector2f orthogonalOnField(dirOnField.y(), -dirOnField.x());
  float sqrLineAssociationCorridor = sqr(parameters.lineAssociationCorridor);
  Vector2f intersection, orthogonalProjection;

  int index = -1;
  for(unsigned int i=0; i<fieldLines.size(); ++i)
  {
    const FieldLine& fieldLine = fieldLines[i];
    if(getSqrDistanceToLine(fieldLine.start, fieldLine.dir, fieldLine.length, startOnField) > sqrLineAssociationCorridor ||
       getSqrDistanceToLine(fieldLine.start, fieldLine.dir, fieldLine.length, endOnField) > sqrLineAssociationCorridor)
      continue;
    if(!intersectLineWithLine(startOnField, orthogonalOnField, fieldLine.start, fieldLine.dir, intersection))
      continue;
    if(getSqrDistanceToLine(startOnField, dirOnField, intersection) > sqrLineAssociationCorridor)
      continue;
    if(!intersectLineWithLine(endOnField, orthogonalOnField, fieldLine.start, fieldLine.dir, intersection))
      continue;
    if(getSqrDistanceToLine(startOnField, dirOnField, intersection) > sqrLineAssociationCorridor)
      continue;
    if(index != -1) // ambiguous?
    {
      index = -1;
      break;
    }
    index = i;
  }
  return index;
}

bool FieldModel::getAssociatedCorner(const Pose2f& robotPose, const LinePercept::Intersection& intersection, Vector2f& associatedCorner) const
{
  const std::vector< Vector2f >* corners = &lCorners;
  if(intersection.type == LinePercept::Intersection::T)
    corners = &tCorners;
  else if(intersection.type == LinePercept::Intersection::X)
    corners = &xCorners;
  const Vector2f pointWorld = robotPose * intersection.pos;
  const float sqrThresh = parameters.cornerAssociationDistance * parameters.cornerAssociationDistance;
  for(unsigned int i=0; i < corners->size(); ++i)
  {
    const Vector2f& c = corners->at(i);
    // simple implementation for testing:
    if((pointWorld - c).squaredNorm() < sqrThresh)
    {
      associatedCorner = c;
      return true;
    }
  }
  return false;
}

float FieldModel::getSqrDistanceToLine(const Vector2f& base, const Vector2f& dir, float length, const Vector2f& point) const
{
  float l = (point.x() - base.x()) * dir.x() + (point.y() - base.y()) * dir.y();
  if(l < 0)
    l = 0;
  else if(l > length)
    l = length;
  return ((base + dir * l) - point).squaredNorm();
}

bool FieldModel::intersectLineWithLine(const Vector2f& lineBase1, const Vector2f& lineDir1,
    const Vector2f& lineBase2, const Vector2f& lineDir2, Vector2f& intersection) const
{
  const float h = lineDir1.x() * lineDir2.y() - lineDir1.y() * lineDir2.x();
  if(h == 0.f)
    return false;
  float scale = ((lineBase2.x() - lineBase1.x()) * lineDir1.y() - (lineBase2.y() - lineBase1.y()) * lineDir1.x()) / h;
  intersection.x() = lineBase2.x() + lineDir2.x() * scale;
  intersection.y() = lineBase2.y() + lineDir2.y() * scale;
  return true;
}

float FieldModel::getSqrDistanceToLine(const Vector2f& base, const Vector2f& dir, const Vector2f& point) const
{
  const float l = (point.x() - base.x()) * dir.x() + (point.y() - base.y()) * dir.y();
  return ((base + dir * l) - point).squaredNorm();
}

bool FieldModel::goalPostIsValid(const Vector2f& observedPosition, const Vector2f& modelPosition,
                                 const Pose2f& robotPose, float goalAcceptanceThreshold) const
{
  // If the difference between model and perception is below a certain minimum, the post is always valid.
  // The minimum is one third of the goal width:
  const float observationDifference = (observedPosition - modelPosition).norm();
  if(observationDifference < goalAcceptanceThreshold)
    return true;
  // Check, if the angle to the observed goal post is small enough:
  const float modelAngle = Geometry::angleTo(robotPose, modelPosition);
  const float observedAngle = Geometry::angleTo(robotPose, observedPosition);
  const float observedDistance = (robotPose.translation - observedPosition).norm();
  if(std::abs(Angle::normalize(modelAngle - observedAngle)) > parameters.goalAssociationMaxAngle)
    return false;
  // If the angle is OK, check the distance deviation.
  const float modelDistance = (robotPose.translation - modelPosition).norm();
  const float modelDistanceAsAngle = (pi_2 - std::atan2(cameraMatrix.translation.z(), modelDistance));
  const float observedDistanceAsAngle = (pi_2 - std::atan2(cameraMatrix.translation.z(), observedDistance));
  return std::abs(modelDistanceAsAngle - observedDistanceAsAngle) < parameters.goalAssociationMaxAngularDistance;
}
