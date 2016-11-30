/**
 * @file PerceptRegistration.cpp
 *
 * Implementation of a class that uses recent field feature obervations
 * and associates them with their real world pendants.
 *
 * @author <A href="mailto:tlaue@uni-bremen.de">Tim Laue</A>
 */

#include "PerceptRegistration.h"
#include "UKFPose2D.h"
#include "Tools/Debugging/DebugDrawings.h"


PerceptRegistration::PerceptRegistration(const CameraMatrix& cameraMatrix,
    const CirclePercept& circlePercept,
    const FieldDimensions& fieldDimensions,
    const FrameInfo& frameInfo,
    const GoalFeature& goalFeature,
    const GoalFrame& goalFrame,
    const FieldLineIntersections& fieldLineIntersections,
    const FieldLines& fieldLines,
    const MidCircle& midCircle,
    const MidCorner& midCorner,
    const MotionInfo& motionInfo,
    const OuterCorner& outerCorner,
    const OwnSideModel& ownSideModel,
    const PenaltyArea& penaltyArea,
    const PenaltyMarkPercept& penaltyMarkPercept,
    const float& goalAssociationMaxAngle,
    const float& goalAssociationMaxAngularDistance,
    bool         goalFrameIsPerceivedAsLines,
    const float& lineAssociationCorridor,
    const float& centerCircleAssociationDistance,
    const float& penaltyMarkAssociationDistance,
    const float& intersectionAssociationDistance,
    const float& globalPoseAssociationDistance) :
  theCameraMatrix(cameraMatrix), theCirclePercept(circlePercept), theFieldDimensions(fieldDimensions),
  theFrameInfo(frameInfo), theGoalFeature(goalFeature), theGoalFrame(goalFrame), theFieldLineIntersections(fieldLineIntersections),
  theFieldLines(fieldLines), theMidCircle(midCircle), theMidCorner(midCorner), theMotionInfo(motionInfo), theOuterCorner(outerCorner),
  theOwnSideModel(ownSideModel), thePenaltyArea(penaltyArea),
  thePenaltyMarkPercept(penaltyMarkPercept), goalAssociationMaxAngle(goalAssociationMaxAngle),
  goalAssociationMaxAngularDistance(goalAssociationMaxAngularDistance),
  lineAssociationCorridor(lineAssociationCorridor), centerCircleAssociationDistance(centerCircleAssociationDistance),
  penaltyMarkAssociationDistance(penaltyMarkAssociationDistance), intersectionAssociationDistance(intersectionAssociationDistance),
  globalPoseAssociationDistance(globalPoseAssociationDistance)
{
  // Initialize goal posts
  goalPosts[0] = Vector2f(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosRightGoal);
  goalPosts[1] = Vector2f(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosLeftGoal);
  goalPosts[2] = Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal);
  goalPosts[3] = Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal);
  unknownGoalAcceptanceThreshold = (2.f * theFieldDimensions.yPosLeftGoal) / 3.f;
  knownGoalAcceptanceThreshold = (2.f * theFieldDimensions.yPosLeftGoal) / 1.5f;

  // Initialize penalty marks
  ownPenaltyMark =      Vector2f(theFieldDimensions.xPosOwnPenaltyMark, 0.f);
  opponentPenaltyMark = Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, 0.f);

  // Initialize list of relevant field lines
  for(size_t i = 0, count = theFieldDimensions.fieldLines.lines.size(); i < count; ++i)
  {
    const FieldDimensions::LinesTable::Line& fieldLine = theFieldDimensions.fieldLines.lines[i];
    if(!fieldLine.isPartOfCircle && (fieldLine.to - fieldLine.from).norm() > 300.f)
    {
      FieldLine relevantFieldLine;
      relevantFieldLine.start = fieldLine.from;
      relevantFieldLine.end = fieldLine.to;
      relevantFieldLine.dir = relevantFieldLine.end - relevantFieldLine.start;
      relevantFieldLine.dir.normalize();
      relevantFieldLine.length = (fieldLine.to - fieldLine.from).norm();
      relevantFieldLine.vertical = std::abs(fieldLine.from.y() - fieldLine.to.y()) < 0.001f;
      if(relevantFieldLine.vertical)
        verticalFieldLines.push_back(relevantFieldLine);
      else
        horizontalFieldLines.push_back(relevantFieldLine);
    }
  }
  if(goalFrameIsPerceivedAsLines)
  {
    for(size_t i = 0, count = theFieldDimensions.goalFrameLines.lines.size(); i < count; ++i)
    {
      const FieldDimensions::LinesTable::Line& fieldLine = theFieldDimensions.goalFrameLines.lines[i];
      FieldLine relevantFieldLine;
      relevantFieldLine.start = fieldLine.from;
      relevantFieldLine.end = fieldLine.to;
      relevantFieldLine.dir = relevantFieldLine.end - relevantFieldLine.start;
      relevantFieldLine.dir.normalize();
      relevantFieldLine.length = (fieldLine.to - fieldLine.from).norm();
      relevantFieldLine.vertical = std::abs(fieldLine.from.y() - fieldLine.to.y()) < 0.001f;
      if(relevantFieldLine.vertical)
        verticalFieldLines.push_back(relevantFieldLine);
      else
        horizontalFieldLines.push_back(relevantFieldLine);
    }
  }

  // Initialize corner lists:
  // X
  xIntersections.push_back(Vector2f(theFieldDimensions.xPosHalfWayLine, theFieldDimensions.centerCircleRadius));
  xIntersections.push_back(Vector2f(theFieldDimensions.xPosHalfWayLine, -theFieldDimensions.centerCircleRadius));
  // T
  tIntersections = xIntersections;
  tIntersections.push_back(Vector2f(theFieldDimensions.xPosHalfWayLine, theFieldDimensions.yPosRightSideline));
  tIntersections.push_back(Vector2f(theFieldDimensions.xPosHalfWayLine, theFieldDimensions.yPosLeftSideline));
  tIntersections.push_back(Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftPenaltyArea));
  tIntersections.push_back(Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightPenaltyArea));
  tIntersections.push_back(Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftPenaltyArea));
  tIntersections.push_back(Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightPenaltyArea));
  // L
  lIntersections = tIntersections;
  lIntersections.push_back(Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightSideline));
  lIntersections.push_back(Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftSideline));
  lIntersections.push_back(Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightSideline));
  lIntersections.push_back(Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftSideline));
  lIntersections.push_back(Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosRightPenaltyArea));
  lIntersections.push_back(Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea));
  lIntersections.push_back(Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosRightPenaltyArea));
  lIntersections.push_back(Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea));
  if(goalFrameIsPerceivedAsLines)
  {
    lIntersections.push_back(Vector2f(theFieldDimensions.xPosOwnGoal, theFieldDimensions.yPosLeftGoal));
    lIntersections.push_back(Vector2f(theFieldDimensions.xPosOwnGoal, theFieldDimensions.yPosRightGoal));
    lIntersections.push_back(Vector2f(theFieldDimensions.xPosOpponentGoal, theFieldDimensions.yPosLeftGoal));
    lIntersections.push_back(Vector2f(theFieldDimensions.xPosOpponentGoal, theFieldDimensions.yPosRightGoal));
  }

  // Initialize time stamps
  lastGoalPostCovarianceUpdate = 0;
  lastPenaltyMarkCovarianceUpdate = 0;
  lastCirclePerceptCovarianceUpdate = 0;
}

void PerceptRegistration::update(const Pose2f& theRobotPose, RegisteredPercepts& registeredPercepts,
                                 const Vector2f& robotRotationDeviationInStand, const Vector2f& robotRotationDeviation)
{
  robotPose = theRobotPose;
  inverseCameraMatrix = theCameraMatrix.inverse();
  currentRotationDeviation = theMotionInfo.isStanding() ? robotRotationDeviationInStand : robotRotationDeviation;

  if(theFieldLines.lines.size() > lineCovarianceUpdates.size())
  {
    lineCovarianceUpdates.resize(theFieldLines.lines.size(), 0);
    lineCovariances.resize(theFieldLines.lines.size());

  }
  if(theFieldLineIntersections.intersections.size() > intersectionCovarianceUpdates.size())
  {
    intersectionCovarianceUpdates.resize(theFieldLineIntersections.intersections.size(), 0);
    intersectionCovariances.resize(theFieldLineIntersections.intersections.size());
  }

  registeredPercepts.lines.clear();
  registeredPercepts.landmarks.clear();
  registeredPercepts.poses.clear();
  registeredPercepts.totalNumberOfPerceivedLines     = registerLines(registeredPercepts.lines);
  registeredPercepts.totalNumberOfPerceivedLandmarks = registerLandmarks(registeredPercepts.landmarks);
  registeredPercepts.totalNumberOfPerceivedPoses     = registerPoses(registeredPercepts.poses);
  draw();
  // Dummy code to avoid warnings:
  bool dummy = !theOwnSideModel.stillInOwnSide;
  dummy = !dummy;
}

int PerceptRegistration::registerPoses(std::vector<RegisteredPose>& poses)
{
  return registerPose(poses, theMidCircle) +
         registerPose(poses, theMidCorner) +
         registerPose(poses, theGoalFrame) +
         registerPose(poses, theOuterCorner) +
         registerPose(poses, thePenaltyArea) +
         registerPose(poses, theGoalFeature);
}

int PerceptRegistration::registerPose(std::vector<RegisteredPose>& poses, const FieldFeature& feature)
{
  if(feature.isValid)
  {
    RegisteredPose newPose;
    if(pickPoseFromFieldFeature(robotPose, feature, newPose.pose))
    {
      if((robotPose.translation - newPose.pose.translation).norm() < globalPoseAssociationDistance)
      {
        newPose.p = feature;
        poses.push_back(newPose);
      }
    }
    return 1;
  }
  return 0;
}

bool PerceptRegistration::pickPoseFromFieldFeature(const Pose2f& robotPose, const FieldFeature& fieldFeature, Pose2f& pickedPose) const
{
  const Pose2f& p1 = fieldFeature.getGlobalRobotPosition().pos1;
  const Pose2f& p2 = fieldFeature.getGlobalRobotPosition().pos2;
  const float sqrDst1 = (robotPose.translation - p1.translation).squaredNorm();
  const float sqrDst2 = (robotPose.translation - p2.translation).squaredNorm();
  float angle1 = robotPose.rotation - p1.rotation;
  float angle2 = robotPose.rotation - p2.rotation;
  angle1 = std::abs(Angle::normalize(angle1));
  angle2 = std::abs(Angle::normalize(angle2));
  if(angle1 < angle2 && sqrDst1 < sqrDst2)
  {
    pickedPose = p1;
    return true;
  }
  else if(angle2 < angle1 && sqrDst2 < sqrDst1)
  {
    pickedPose = p2;
    return true;
  }
  return false;
}

int PerceptRegistration::registerLandmarks(std::vector<RegisteredLandmark>& landmarks)
{
  int numOfLandmarks(0);
  if(thePenaltyMarkPercept.timeLastSeen == theFrameInfo.time)
  {
    Vector2f penaltyMarkInWorld;
    if(getAssociatedPenaltyMark(thePenaltyMarkPercept.positionOnField, penaltyMarkInWorld))
    {
      RegisteredLandmark newLandmark;
      newLandmark.w = penaltyMarkInWorld;
      newLandmark.p = thePenaltyMarkPercept.positionOnField;
      if(lastPenaltyMarkCovarianceUpdate != theFrameInfo.time)
      {
        penaltyMarkCovariance = UKFPose2D::getCovOfPointInWorld(newLandmark.p, 0.f, theCameraMatrix, inverseCameraMatrix, currentRotationDeviation);
        lastPenaltyMarkCovarianceUpdate = theFrameInfo.time;
      }
      newLandmark.cov = penaltyMarkCovariance;
      landmarks.push_back(newLandmark);
    }
    numOfLandmarks++;
  }
  if(theCirclePercept.lastSeen == theFrameInfo.time && !theMidCircle.isValid)
  {
    const Vector2f circleInWorld = robotPose * theCirclePercept.pos;
    if(circleInWorld.norm() <= centerCircleAssociationDistance)
    {
      RegisteredLandmark newLandmark;
      newLandmark.w = Vector2f(0.f, 0.f);
      newLandmark.p = theCirclePercept.pos;
      if(lastCirclePerceptCovarianceUpdate != theFrameInfo.time)
      {
        circlePerceptCovariance = UKFPose2D::getCovOfPointInWorld(newLandmark.p, 0.f, theCameraMatrix, inverseCameraMatrix, currentRotationDeviation);
        lastCirclePerceptCovarianceUpdate = theFrameInfo.time;
      }
      newLandmark.cov = circlePerceptCovariance;
      landmarks.push_back(newLandmark);
    }
    numOfLandmarks++;
  }
  // If we detected some of the special corner constellations (which will be used differently),
  // we do not need any of the "normal" intersections anymore.
  if(!theGoalFrame.isValid && !theMidCorner.isValid && !theOuterCorner.isValid && !thePenaltyArea.isValid)
  {
    for(unsigned int i = 0; i < theFieldLineIntersections.intersections.size(); ++i)
    {
      const auto& intersection = theFieldLineIntersections.intersections[i];
      Vector2f intersectionInWorld;
      if(getAssociatedIntersection(intersection, intersectionInWorld))
      {
        RegisteredLandmark newLandmark;
        newLandmark.w = intersectionInWorld;
        newLandmark.p = intersection.pos;
        if(intersectionCovarianceUpdates[i] < theFrameInfo.time)
        {
          intersectionCovariances[i] = UKFPose2D::getCovOfPointInWorld(newLandmark.p, 0.f, theCameraMatrix, inverseCameraMatrix, currentRotationDeviation);
          intersectionCovarianceUpdates[i] = theFrameInfo.time;
        }
        newLandmark.cov = intersectionCovariances[i];
        landmarks.push_back(newLandmark);
      }
    }
    numOfLandmarks += static_cast<int>(theFieldLineIntersections.intersections.size());
  }
  return numOfLandmarks;
}

int PerceptRegistration::registerLines(std::vector<RegisteredLine>& lines)
{
  // If we have some of the high level percepts, do not register the low level ones
  if(theGoalFrame.isValid || theMidCorner.isValid || theOuterCorner.isValid || thePenaltyArea.isValid || theMidCircle.isValid)
    return 0;

  // Iterate over all observed lines:
  for(unsigned int i = 0; i < theFieldLines.lines.size(); ++i)
  {
    const auto& line = theFieldLines.lines[i];
    const FieldLine* fieldLine = getPointerToAssociatedLine(line.first, line.last);
    if(fieldLine)
    {
      RegisteredLine newLine;
      newLine.pStart    = line.first;
      newLine.pEnd      = line.last;
      newLine.pCenter   = (line.first + line.last) * 0.5f;
      newLine.pDir      = line.last - line.first;
      newLine.pDir.normalize();
      newLine.wStart    = (*fieldLine).start;
      newLine.wEnd      = (*fieldLine).end;
      newLine.vertical  = (*fieldLine).vertical;
      if(lineCovarianceUpdates[i] < theFrameInfo.time)
      {
        lineCovariances[i] = UKFPose2D::getCovOfPointInWorld(newLine.pCenter, 0.f, theCameraMatrix, inverseCameraMatrix, currentRotationDeviation);
        lineCovarianceUpdates[i] = theFrameInfo.time;
      }
      newLine.covCenter = lineCovariances[i];
      lines.push_back(newLine);
    }
  }
  return static_cast<int>(theFieldLines.lines.size());
}

bool PerceptRegistration::getAssociatedIntersection(const FieldLineIntersections::Intersection& intersection, Vector2f& associatedIntersection) const
{
  const std::vector< Vector2f >* corners = &lIntersections;
  if(intersection.type == FieldLineIntersections::Intersection::T)
    corners = &tIntersections;
  else if(intersection.type == FieldLineIntersections::Intersection::X)
    corners = &xIntersections;
  const Vector2f pointWorld = robotPose * intersection.pos;
  const float sqrThresh = intersectionAssociationDistance * intersectionAssociationDistance;
  for(unsigned int i = 0; i < corners->size(); ++i)
  {
    const Vector2f& c = corners->at(i);
    // simple implementation for testing:
    if((pointWorld - c).squaredNorm() < sqrThresh)
    {
      associatedIntersection = c;
      return true;
    }
  }
  return false;
}

bool PerceptRegistration::getAssociatedPenaltyMark(const Vector2f& penaltyMarkPercept, Vector2f& associatedPenaltyMark) const
{
  const Vector2f penaltyMarkInWorld = robotPose * penaltyMarkPercept;
  associatedPenaltyMark = penaltyMarkInWorld.x() <= 0.f ? ownPenaltyMark : opponentPenaltyMark;
  const float differenceBetweenPerceptAndModel = (penaltyMarkInWorld - associatedPenaltyMark).norm();
  return differenceBetweenPerceptAndModel < penaltyMarkAssociationDistance;
}

bool PerceptRegistration::getAssociatedUnknownGoalPost(const Vector2f& goalPercept, Vector2f& associatedPost) const
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
  return goalPostIsValid(postInWorld, associatedPost, unknownGoalAcceptanceThreshold);
}

bool PerceptRegistration::getAssociatedKnownGoalPost(const Vector2f& goalPercept, bool isLeft, Vector2f& associatedPost) const
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
  return goalPostIsValid(postInWorld, associatedPost, knownGoalAcceptanceThreshold);
}

const PerceptRegistration::FieldLine* PerceptRegistration::getPointerToAssociatedLine(const Vector2f& start, const Vector2f& end) const
{
  const Vector2f startOnField = robotPose * start;
  const Vector2f endOnField = robotPose * end;
  Vector2f dirOnField = endOnField - startOnField;
  dirOnField.normalize();
  Vector2f orthogonalOnField(dirOnField.y(), -dirOnField.x());
  const float lineLength = (start - end).norm();
  // Throw away short lines on center circle (as they are currently not in the field lines)
  if(lineLength < 500.f)
  {
    const float centerCircleCorridorInner = theFieldDimensions.centerCircleRadius * 0.8f;
    const float centerCircleCorridorOuter = theFieldDimensions.centerCircleRadius * 1.2f;
    const float distStart = startOnField.norm();
    const float distEnd = endOnField.norm();
    if(distStart > centerCircleCorridorInner && distStart < centerCircleCorridorOuter &&
       distEnd > centerCircleCorridorInner && distEnd < centerCircleCorridorOuter)
      return 0;
  }
  const float sqrLineAssociationCorridor = sqr(lineAssociationCorridor);
  Vector2f intersection;
  bool isVertical = std::abs(dirOnField.x()) > std::abs(dirOnField.y());
  const std::vector<FieldLine>& fieldLines = isVertical ? verticalFieldLines : horizontalFieldLines;

  for(unsigned int i = 0; i < fieldLines.size(); ++i)
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
    return &(fieldLines[i]);
  }
  return 0;
}

float PerceptRegistration::getSqrDistanceToLine(const Vector2f& base, const Vector2f& dir, float length, const Vector2f& point) const
{
  float l = (point.x() - base.x()) * dir.x() + (point.y() - base.y()) * dir.y();
  if(l < 0)
    l = 0;
  else if(l > length)
    l = length;
  return ((base + dir * l) - point).squaredNorm();
}

bool PerceptRegistration::intersectLineWithLine(const Vector2f& lineBase1, const Vector2f& lineDir1,
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

float PerceptRegistration::getSqrDistanceToLine(const Vector2f& base, const Vector2f& dir, const Vector2f& point) const
{
  const float l = (point.x() - base.x()) * dir.x() + (point.y() - base.y()) * dir.y();
  return ((base + dir * l) - point).squaredNorm();
}

bool PerceptRegistration::goalPostIsValid(const Vector2f& observedPosition, const Vector2f& modelPosition,
    float goalAcceptanceThreshold) const
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
  if(std::abs(Angle::normalize(modelAngle - observedAngle)) > goalAssociationMaxAngle)
    return false;
  // If the angle is OK, check the distance deviation.
  const float modelDistance = (robotPose.translation - modelPosition).norm();
  const float modelDistanceAsAngle = (pi_2 - std::atan2(theCameraMatrix.translation.z(), modelDistance));
  const float observedDistanceAsAngle = (pi_2 - std::atan2(theCameraMatrix.translation.z(), observedDistance));
  return std::abs(modelDistanceAsAngle - observedDistanceAsAngle) < goalAssociationMaxAngularDistance;
}

void PerceptRegistration::draw()
{
  DECLARE_DEBUG_DRAWING("module:PerceptRegistration:fieldModel", "drawingOnField");
  COMPLEX_DRAWING("module:PerceptRegistration:fieldModel")
  {
    for(const auto& line : verticalFieldLines)
    {
      LINE("module:PerceptRegistration:fieldModel", line.start.x(), line.start.y(), line.end.x(), line.end.y(),
           40, Drawings::solidPen, ColorRGBA(128, 0, 255, 100));
    }
    for(const auto& line : horizontalFieldLines)
    {
      LINE("module:PerceptRegistration:fieldModel", line.start.x(), line.start.y(), line.end.x(), line.end.y(),
           40, Drawings::solidPen, ColorRGBA(255, 0, 128, 100));
    }
  }
}
