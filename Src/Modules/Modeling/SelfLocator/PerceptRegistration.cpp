/**
 * @file PerceptRegistration.cpp
 *
 * Implementation of a class that uses recent field feature observations
 * and associates them with their real world pendants.
 *
 * @author <A href="mailto:tlaue@uni-bremen.de">Tim Laue</A>
 */

#include "PerceptRegistration.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Modeling/Measurements.h"
#include "Tools/Modeling/UKFPose2D.h"

PerceptRegistration::PerceptRegistration(const CameraInfo& cameraInfo, const CameraMatrix& cameraMatrix,
    const CirclePercept& circlePercept,
    const FieldDimensions& fieldDimensions,
    const FrameInfo& frameInfo,
    const GameInfo& gameInfo,
    const OwnTeamInfo& ownTeamInfo,
    const GoalFrame& goalFrame,
    const FieldLineIntersections& fieldLineIntersections,
    const FieldLines& fieldLines,
    const MidCircle& midCircle,
    const MidCorner& midCorner,
    const OuterCorner& outerCorner,
    const PenaltyArea& penaltyArea,
    const PenaltyMarkPercept& penaltyMarkPercept,
    bool         goalFrameIsPerceivedAsLines,
    const float& lineAssociationCorridor,
    const float& longLineAssociationCorridor,
    const float& centerCircleAssociationDistance,
    const float& penaltyMarkAssociationDistance,
    const float& intersectionAssociationDistance,
    const float& minimumLineLengthForAssociationToLongHorizontalLine,
    const float& globalPoseAssociationMaxDistanceDeviation,
    const Angle& globalPoseAssociationMaxAngularDeviation,
    const float& covarianceScalarLongHorizontalLine,
    const float& minLengthLongHorizontalLine,
    const float& yDifferenceLongHorizontalLine) :
  theCameraInfo(cameraInfo),
  theCameraMatrix(cameraMatrix), theCirclePercept(circlePercept), theFieldDimensions(fieldDimensions),
  theFrameInfo(frameInfo), theGameInfo(gameInfo), theOwnTeamInfo(ownTeamInfo),
  theGoalFrame(goalFrame), theFieldLineIntersections(fieldLineIntersections),
  theFieldLines(fieldLines), theMidCircle(midCircle), theMidCorner(midCorner), theOuterCorner(outerCorner),
  thePenaltyArea(penaltyArea), thePenaltyMarkPercept(penaltyMarkPercept),
  lineAssociationCorridor(lineAssociationCorridor), longLineAssociationCorridor(longLineAssociationCorridor),
  centerCircleAssociationDistance(centerCircleAssociationDistance),
  penaltyMarkAssociationDistance(penaltyMarkAssociationDistance), intersectionAssociationDistance(intersectionAssociationDistance),
  minimumLineLengthForAssociationToLongHorizontalLine(minimumLineLengthForAssociationToLongHorizontalLine),
  globalPoseAssociationMaxDistanceDeviation(globalPoseAssociationMaxDistanceDeviation),
  globalPoseAssociationMaxAngularDeviation(globalPoseAssociationMaxAngularDeviation),
  covarianceScalarLongHorizontalLine(covarianceScalarLongHorizontalLine), minLengthLongHorizontalLine(minLengthLongHorizontalLine),
  yDifferenceLongHorizontalLine(yDifferenceLongHorizontalLine)
{
  // Initialize goal posts
  goalPosts[0] = Vector2f(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosRightGoal);
  goalPosts[1] = Vector2f(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosLeftGoal);
  goalPosts[2] = Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal);
  goalPosts[3] = Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal);
  penaltyAreaWidth = 2.f * std::abs(theFieldDimensions.yPosLeftPenaltyArea);
  goalAcceptanceThreshold = (2.f * theFieldDimensions.yPosLeftGoal) / 3.f;

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
      relevantFieldLine.isLong = relevantFieldLine.length > penaltyAreaWidth;
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
      relevantFieldLine.isLong = relevantFieldLine.length > penaltyAreaWidth;
      relevantFieldLine.vertical = std::abs(fieldLine.from.y() - fieldLine.to.y()) < 0.001f;
      if(relevantFieldLine.vertical)
        verticalFieldLines.push_back(relevantFieldLine);
      else
        horizontalFieldLines.push_back(relevantFieldLine);
    }
  }
  // Search center line in list of all lines:
  centerLine = 0;
  for(auto& fieldLine : horizontalFieldLines)
  {
    if(fieldLine.start.x() != theFieldDimensions.xPosHalfWayLine)
      continue;
    if(fieldLine.end.x() != theFieldDimensions.xPosHalfWayLine)
      continue;
    centerLine = &fieldLine;
    return;
  }
  ASSERT(centerLine != 0);

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

  // Initialize timestamps
  lastGoalPostCovarianceUpdate = 0;
  lastPenaltyMarkCovarianceUpdate = 0;
  lastCirclePerceptCovarianceUpdate = 0;
}

void PerceptRegistration::update(const Pose2f& theRobotPose, RegisteredPercepts& registeredPercepts,
                                 const Pose3f& inverseCameraMatrix, const Vector2f& currentRotationDeviation, bool drawRegisteredElements)
{
  robotPose = theRobotPose;
  this->inverseCameraMatrix = inverseCameraMatrix;
  this->currentRotationDeviation = currentRotationDeviation;

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
  if(drawRegisteredElements)
    draw(registeredPercepts, theRobotPose);
}

int PerceptRegistration::registerPoses(std::vector<RegisteredPose>& poses)
{
  return registerPose(poses, theMidCircle) +
         registerPose(poses, theMidCorner) +
         registerPose(poses, theGoalFrame) +
         registerPose(poses, theOuterCorner) +
         registerPose(poses, thePenaltyArea);
}

int PerceptRegistration::registerPose(std::vector<RegisteredPose>& poses, const FieldFeature& feature)
{
  if(feature.isValid)
  {
    RegisteredPose newPose;
    if(pickPoseFromFieldFeature(robotPose, feature, newPose.pose))
    {
      if((robotPose.translation - newPose.pose.translation).norm() < globalPoseAssociationMaxDistanceDeviation)
      {
        Angle angularDifference(robotPose.rotation - newPose.pose.rotation);
        angularDifference.normalize();
        if(std::abs(angularDifference) < globalPoseAssociationMaxAngularDeviation)
        {
          newPose.p = feature;
          poses.push_back(newPose);
        }
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
  if(thePenaltyMarkPercept.wasSeen)
  {
    Vector2f penaltyMarkInWorld;
    if(getAssociatedPenaltyMark(thePenaltyMarkPercept.positionOnField, penaltyMarkInWorld))
    {
      RegisteredLandmark newLandmark;
      newLandmark.w = penaltyMarkInWorld;
      newLandmark.p = thePenaltyMarkPercept.positionOnField;
      if(lastPenaltyMarkCovarianceUpdate != theFrameInfo.time)
      {
        penaltyMarkCovariance = Measurements::positionToCovarianceMatrixInRobotCoordinates(newLandmark.p, 0.f, theCameraMatrix, inverseCameraMatrix, currentRotationDeviation);
        lastPenaltyMarkCovarianceUpdate = theFrameInfo.time;
      }
      newLandmark.cov = penaltyMarkCovariance;
      landmarks.push_back(newLandmark);
    }
    numOfLandmarks++;
  }
  if(theCirclePercept.wasSeen && !theMidCircle.isValid)
  {
    const Vector2f circleInWorld = robotPose * theCirclePercept.pos;
    if(circleInWorld.norm() <= centerCircleAssociationDistance)
    {
      RegisteredLandmark newLandmark;
      newLandmark.w = Vector2f(0.f, 0.f);
      newLandmark.p = theCirclePercept.pos;
      if(lastCirclePerceptCovarianceUpdate != theFrameInfo.time)
      {
        // Some workaround stuff as the distance to a perceived center circle center
        // might be very low (resulting in a very small covariance) but still be a subject
        // to high noise as the center is computer and not directly perceived.
        // Thus, a higher covariance is computer for close circles.
        const float circleDistance = theCirclePercept.pos.norm();
        const float centerCircleDiameter = theFieldDimensions.centerCircleRadius * 2.f;
        Vector2f increasedCirclePos = theCirclePercept.pos;
        if(circleDistance < centerCircleDiameter)
        {
          if(circleDistance < 10.f)
            increasedCirclePos = Vector2f(centerCircleDiameter, 0.f);
          else
            increasedCirclePos *= centerCircleDiameter / circleDistance;
        }
        circlePerceptCovariance = Measurements::positionToCovarianceMatrixInRobotCoordinates(increasedCirclePos, 0.f, theCameraMatrix, inverseCameraMatrix, currentRotationDeviation);
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
          intersectionCovariances[i] = Measurements::positionToCovarianceMatrixInRobotCoordinates(newLandmark.p, 0.f, theCameraMatrix, inverseCameraMatrix, currentRotationDeviation);
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
        lineCovariances[i] = Measurements::positionToCovarianceMatrixInRobotCoordinates(newLine.pCenter, 0.f, theCameraMatrix, inverseCameraMatrix, currentRotationDeviation);
        lineCovarianceUpdates[i] = theFrameInfo.time;
        // Reduce covariance, if line is long and horizontal:
        if(lineIsLongAndHorizontalInImage(newLine))
          lineCovariances[i] *= covarianceScalarLongHorizontalLine;
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

bool PerceptRegistration::getAssociatedGoalPost(const Vector2f& goalPostPercept, Vector2f& associatedGoalPost) const
{
  const Vector2f goalPostPerceptionInWorld = robotPose * goalPostPercept;
  int idxOfClosestPost = 0;
  float distToClosestPost = (goalPostPerceptionInWorld - goalPosts[0]).norm();
  for(int i = 1; i < 4; ++i)
  {
    const float distToPost = (goalPostPerceptionInWorld - goalPosts[i]).norm();
    if(distToPost < distToClosestPost)
    {
      distToClosestPost = distToPost;
      idxOfClosestPost = i;
    }
  }
  associatedGoalPost = goalPosts[idxOfClosestPost];
  return distToClosestPost <= goalAcceptanceThreshold;
}

const PerceptRegistration::FieldLine* PerceptRegistration::getPointerToAssociatedLine(const Vector2f& start, const Vector2f& end) const
{
  const Vector2f startOnField = robotPose * start;
  const Vector2f endOnField = robotPose * end;
  Vector2f dirOnField = endOnField - startOnField;
  dirOnField.normalize();
  const Vector2f orthogonalOnField(dirOnField.y(), -dirOnField.x());
  bool isVertical = std::abs(dirOnField.x()) > std::abs(dirOnField.y());
  const float lineLength = (start - end).norm();
  const bool perceivedLineIsLong = lineLength > penaltyAreaWidth;
  // Throw away lines on center circle (as they are currently not in the field lines)
  // To save computing time, do check for lines of up to medium length, only.
  // To avoid deleting short penalty area side lines, only consider lines that start somewhere near the field center
  if(lineLength < 2000.f && (startOnField.norm() < 2.f * theFieldDimensions.centerCircleRadius
     || end.norm() < 2.f * theFieldDimensions.centerCircleRadius) &&
     lineCouldBeOnCenterCircle(startOnField, dirOnField))
    return nullptr;
  // Throw away short lines before kickoff (false positives on center circle that are confused with the center line)
  // when the other team has kickoff
  if(lineLength < 1000.f && iAmBeforeKickoffInTheCenterOfMyHalfLookingForward() &&
     theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber)
    return nullptr;
  const float sqrLineAssociationCorridor     = sqr(lineAssociationCorridor);
  const float sqrLongLineAssociationCorridor = sqr(longLineAssociationCorridor);
  Vector2f intersection;
  const std::vector<FieldLine>& fieldLines = isVertical ? verticalFieldLines : horizontalFieldLines;

  for(unsigned int i = 0; i < fieldLines.size(); ++i)
  {
    const FieldLine& fieldLine = fieldLines[i];
    if(lineLength > 1.3f * fieldLine.length)
      continue;
    const float currentSqrCorridor = (fieldLine.isLong && perceivedLineIsLong) ? sqrLongLineAssociationCorridor : sqrLineAssociationCorridor;
    if(getSqrDistanceToLine(fieldLine.start, fieldLine.dir, fieldLine.length, startOnField) > currentSqrCorridor ||
       getSqrDistanceToLine(fieldLine.start, fieldLine.dir, fieldLine.length, endOnField) > currentSqrCorridor)
      continue;
    if(!intersectLineWithLine(startOnField, orthogonalOnField, fieldLine.start, fieldLine.dir, intersection))
      continue;
    if(getSqrDistanceToLine(startOnField, dirOnField, intersection) > currentSqrCorridor)
      continue;
    if(!intersectLineWithLine(endOnField, orthogonalOnField, fieldLine.start, fieldLine.dir, intersection))
      continue;
    if(getSqrDistanceToLine(startOnField, dirOnField, intersection) > currentSqrCorridor)
      continue;
    // Exlude potential short lines on the center circle that might become associated to the center line,
    if(lineLength < minimumLineLengthForAssociationToLongHorizontalLine &&
       theCameraInfo.camera == CameraInfo::upper &&
       fieldLine.isLong && !fieldLine.vertical && fieldLine.start.x() == 0.f)
      continue;
    return &(fieldLines[i]);
  }

  // If this point has been reached, no matching line has been found.
  // However, in READY and SET, we try to consider a wide tolerance to match the center line. This
  // might solve some rarely occurring localization imprecisions before kickoff.
  if(!isVertical && iAmBeforeKickoffAndTheLineIsProbablyTheCenterLine(startOnField, endOnField, dirOnField, orthogonalOnField, lineLength))
  {
    return centerLine;
  }
  return nullptr;
}

// THIS IS SOMEHOW HACKED. COULD BE IMPROVED IN THE FUTURE. T.L.
bool PerceptRegistration::iAmBeforeKickoffAndTheLineIsProbablyTheCenterLine(const Vector2f& lineStart,
                                                                            const Vector2f& lineEnd,
                                                                            const Vector2f& direction,
                                                                            const Vector2f& orthogonal,
                                                                            float length) const
{
  if(!iAmBeforeKickoffInTheCenterOfMyHalfLookingForward())
    return false;
  // Check minimum length
  if(length < 2500.f)
    return false;
  // Check compatibility of line:
  Vector2f intersection;
  const float sqrLineAssociationCorridor = sqr(2000.f);
  if(getSqrDistanceToLine(centerLine->start, centerLine->dir, centerLine->length, lineStart) > sqrLineAssociationCorridor ||
     getSqrDistanceToLine(centerLine->start, centerLine->dir, centerLine->length, lineEnd) > sqrLineAssociationCorridor)
    return false;
  if(!intersectLineWithLine(lineStart, orthogonal, centerLine->start, centerLine->dir, intersection))
    return false;
  if(getSqrDistanceToLine(lineStart, direction, intersection) > sqrLineAssociationCorridor)
    return false;
  if(!intersectLineWithLine(lineEnd, orthogonal, centerLine->start, centerLine->dir, intersection))
    return false;
  if(getSqrDistanceToLine(lineStart, direction, intersection) > sqrLineAssociationCorridor)
    return false;
  return true;
}

bool PerceptRegistration::iAmBeforeKickoffInTheCenterOfMyHalfLookingForward() const
{
  // Game state must be correct:
  if(theGameInfo.state != STATE_SET && theGameInfo.state != STATE_READY)
    return false;
  // Robot must be in own half and look forward:
  if(robotPose.translation.x() >= 0.f || std::abs(robotPose.rotation) >= 25_deg ||
     robotPose.translation.x() <= theFieldDimensions.xPosOwnPenaltyArea)
    return false;
  return true;
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

bool PerceptRegistration::lineCouldBeOnCenterCircle(const Vector2f& lineStart, const Vector2f& direction) const
{
  Geometry::Line line;
  line.base = lineStart;
  line.direction = direction;
  const float distToCenter = Geometry::getDistanceToLine(line, Vector2f(0.f, 0.f));
  const float centerCircleCorridorInner = theFieldDimensions.centerCircleRadius * 0.4f;
  const float centerCircleCorridorOuter = theFieldDimensions.centerCircleRadius * 1.7f;
  return distToCenter > centerCircleCorridorInner && distToCenter < centerCircleCorridorOuter;
}

bool PerceptRegistration::lineIsLongAndHorizontalInImage(const RegisteredLine& line) const
{
  const float lineLength = (line.pStart - line.pEnd).norm();
  if(lineLength < minLengthLongHorizontalLine)
    return false;
  Vector2f pStartImage;
  Vector2f pEndImage;
  if(Transformation::robotToImage(line.pStart, theCameraMatrix, theCameraInfo, pStartImage) &&
     Transformation::robotToImage(line.pEnd, theCameraMatrix, theCameraInfo, pEndImage))
  {
    if(std::abs(pStartImage.y() - pEndImage.y()) > yDifferenceLongHorizontalLine)
    {
      return false;
    }
  }
  return true;
}

void PerceptRegistration::draw(const RegisteredPercepts& registeredPercepts, const Pose2f& theRobotPose)
{
  for(const auto& line : registeredPercepts.lines)
  {
    // Associated model line in absolute field coordinates:
    LINE("module:SelfLocator:perceptRegistrationWorld", line.wStart.x(), line.wStart.y(), line.wEnd.x(), line.wEnd.y(),
         80, Drawings::solidPen, ColorRGBA(128, 0, 255, 180));
    // Attach drawing of covariance (has to be drawn relative to the robot)
    COVARIANCE_ELLIPSES_2D("module:SelfLocator:perceptRegistrationRobot", line.covCenter, line.pCenter);
  }
  ColorRGBA c99(255,255,100,100);
  ColorRGBA c95(255,128,50,100);
  ColorRGBA c68(255,100,100,100);
  for(const auto& landmark : registeredPercepts.landmarks)
  {
    // Drawing of covariance (has to be drawn relative to the robot)
    COVARIANCE_ELLIPSES_2D_OWN_COLORS("module:SelfLocator:perceptRegistrationRobot", landmark.cov, landmark.p, c99, c95, c68);
  }
}
