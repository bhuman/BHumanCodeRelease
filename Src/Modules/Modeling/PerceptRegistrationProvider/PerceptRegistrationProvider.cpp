/**
 * @file PerceptRegistrationProvider.cpp
 *
 * This file implements a module,
 * which provides functions to assign perceived field elements
 * to their counterparts in the model of the field.
 * These pairs are later used in the self-localization
 * process to update the robot's pose estimates..
 *
 * @author Tim Laue
 */

/*
 * TODO:
 * - Current implementation considers elements twice, if they are part of a FieldFeature.
 *   Think about handling this in a reasonable way
 */

#include "PerceptRegistrationProvider.h"
#include "Tools/Modeling/Measurements.h"

MAKE_MODULE(PerceptRegistrationProvider, modeling);

PerceptRegistrationProvider::PerceptRegistrationProvider()
{
  // Initialize time stamps
  timeOfLastPenaltyMarkCovarianceUpdate = 0;

  // Initialize penalty marks
  ownPenaltyMarkWorldModel =      Vector2f(theFieldDimensions.xPosOwnPenaltyMark, 0.f);
  opponentPenaltyMarkWorldModel = Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, 0.f);

  // Initialize goal posts
  ownGoalPostsWorldModel[0] = Vector2f(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosRightGoal);
  ownGoalPostsWorldModel[1] = Vector2f(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosLeftGoal);
  opponentGoalPostsWorldModel[0] = Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal);
  opponentGoalPostsWorldModel[1] = Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal);

  // Initialize list for lines in the world model
  auto const& listOfLines = goalFrameIsPerceivedAsLines ? theFieldDimensions.fieldLinesWithGoalFrame.lines : theFieldDimensions.fieldLines.lines;
  for(auto const& fieldLine : listOfLines)
  {
    if(!fieldLine.isPartOfCircle) // TODO: Create list for circle matching later
    {
      WorldModelFieldLine newLine(fieldLine.from, fieldLine.to);
      if(newLine.length > theFieldDimensions.penaltyMarkSize) // Penalty mark is not a line here
      {
        const bool isVertical = std::abs(newLine.dir.x()) > std::abs(newLine.dir.y());
        if(isVertical)
          verticalLinesWorldModel.push_back(newLine);
        else
          horizontalLinesWorldModel.push_back(newLine);
      }
    }
  }

  // Compute deviations
  // If we see false positive center circles, they are sometimes seen in large corners.
  // The closest constellation is near the crossing of the centerline and the side line:
  Vector2f posOfClosestPotentialFalsePositiveCenterCircle(theFieldDimensions.centerCircleRadius, theFieldDimensions.yPosLeftSideline - theFieldDimensions.centerCircleRadius);
  maxCenterCircleDeviation = posOfClosestPotentialFalsePositiveCenterCircle.norm();
  maxCenterCircleDeviation *= 0.8f; // Some additional tolerance
  maxGoalPostDeviation = 2.f * theFieldDimensions.yPosLeftGoal / 3.f; // One third of the total goal width
}

void PerceptRegistrationProvider::update(PerceptRegistration& perceptRegistration)
{
  preprocessMeasurements(perceptRegistration);
  perceptRegistration.registerAbsolutePoseMeasurements = [this, &perceptRegistration](const Pose2f& pose, std::vector<RegisteredAbsolutePoseMeasurement>& absolutePoseMeasurements) -> void
  {
    absolutePoseMeasurements.clear();
    if(perceptRegistration.totalNumberOfAvailableAbsolutePoseMeasurements > 0)
      registerAbsolutePoseMeasurements(pose, absolutePoseMeasurements);
  };
  perceptRegistration.registerLandmarks = [this, &perceptRegistration](const Pose2f& pose, std::vector<RegisteredLandmark>& landmarks) -> void
  {
    landmarks.clear();
    if(perceptRegistration.totalNumberOfAvailableLandmarks > 0)
      registerLandmarks(pose, landmarks);
  };
  perceptRegistration.registerLines = [this, &perceptRegistration](const Pose2f& pose, std::vector<RegisteredLine>& lines) -> void
  {
    lines.clear();
    if(perceptRegistration.totalNumberOfAvailableLines > 0)
    {
      perceptRegistration.totalNumberOfIgnoredLines = 0;
      registerLines(pose, lines, perceptRegistration.totalNumberOfIgnoredLines);
      ASSERT(perceptRegistration.totalNumberOfIgnoredLines <= perceptRegistration.totalNumberOfAvailableLines);
    }
  };
}

void PerceptRegistrationProvider::preprocessMeasurements(PerceptRegistration& perceptRegistration)
{
  // Reset counters:
  perceptRegistration.totalNumberOfAvailableAbsolutePoseMeasurements = 0;
  perceptRegistration.totalNumberOfAvailableLandmarks = 0;
  perceptRegistration.totalNumberOfAvailableLines = 0;

  // Precompute and initialize stuff:
  inverseCameraMatrix = theCameraMatrix.inverse();
  currentRotationDeviation = theMotionInfo.executedPhase == MotionPhase::stand ? robotRotationDeviationInStand : robotRotationDeviation;
  if(theFieldLineIntersections.intersections.size() > timesOfLastIntersectionCovarianceUpdates.size())
  {
    intersectionCovariances.resize(theFieldLineIntersections.intersections.size());
    timesOfLastIntersectionCovarianceUpdates.resize(theFieldLineIntersections.intersections.size(), 0);
  }
  if(theGoalPostsPercept.goalPosts.size() > timesOfLastGoalPostCovarianceUpdates.size())
  {
    goalPostsCovariances.resize(theGoalPostsPercept.goalPosts.size());
    timesOfLastGoalPostCovarianceUpdates.resize(theGoalPostsPercept.goalPosts.size(), 0);
  }
  if(theFieldLines.lines.size() > timesOfLastLineCovarianceUpdates.size())
  {
    lineCovariances.resize(theFieldLines.lines.size());
    timesOfLastLineCovarianceUpdates.resize(theFieldLines.lines.size(), 0);
  }

  // Count the number of perceived field elements (needed for computing the validity):
  // --- Field Features:
  if(thePenaltyMarkWithPenaltyAreaLine.isValid)
    perceptRegistration.totalNumberOfAvailableAbsolutePoseMeasurements++;
  if(theMidCircle.isValid)
    perceptRegistration.totalNumberOfAvailableAbsolutePoseMeasurements++;
  // --- Landmarks:
  if(thePenaltyMarkPercept.wasSeen)
    perceptRegistration.totalNumberOfAvailableLandmarks++;
  if(theCirclePercept.wasSeen)
    perceptRegistration.totalNumberOfAvailableLandmarks++;
  for(auto const& goalPost : theGoalPostsPercept.goalPosts)
    if(goalPost.baseInImage)
      perceptRegistration.totalNumberOfAvailableLandmarks++;
  perceptRegistration.totalNumberOfAvailableLandmarks += static_cast<int>(theFieldLineIntersections.intersections.size());
  // --- Lines:
  perceptRegistration.totalNumberOfAvailableLines = static_cast<int>(theFieldLines.lines.size());
}

void PerceptRegistrationProvider::registerAbsolutePoseMeasurements(const Pose2f& pose, std::vector<RegisteredAbsolutePoseMeasurement>& absolutePoseMeasurements)
{
  registerSingleAbsolutePoseMeasurement(pose, thePenaltyMarkWithPenaltyAreaLine, absolutePoseMeasurements);
  registerSingleAbsolutePoseMeasurement(pose, theMidCircle, absolutePoseMeasurements);
}

void PerceptRegistrationProvider::registerSingleAbsolutePoseMeasurement(const Pose2f& pose, const FieldFeature& measurement, std::vector<RegisteredAbsolutePoseMeasurement>& absolutePoseMeasurements)
{
  if(measurement.isValid)
  {
    RegisteredAbsolutePoseMeasurement newPose;
    if(pickPoseFromFieldFeature(pose, measurement, newPose.absolutePoseOnField))
    {
      if((pose.translation - newPose.absolutePoseOnField.translation).norm() < globalPoseAssociationMaxDistanceDeviation)
      {
        Angle angularDifference(pose.rotation - newPose.absolutePoseOnField.rotation);
        angularDifference.normalize();
        if(std::abs(angularDifference) < globalPoseAssociationMaxAngularDeviation)
        {
          newPose.perceivedRelativePose = measurement;
          absolutePoseMeasurements.push_back(newPose);
        }
      }
    }
  }
}

bool PerceptRegistrationProvider::pickPoseFromFieldFeature(const Pose2f& robotPose, const FieldFeature& fieldFeature, Pose2f& pickedPose) const
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

void PerceptRegistrationProvider::registerLandmarks(const Pose2f& pose, std::vector<RegisteredLandmark>& landmarks)
{
  // Register the penalty mark: +++++++++++++++++++++++++++++++++++++++++++++++++++++++
  if(thePenaltyMarkPercept.wasSeen)
  {
    Vector2f penaltyMarkWorldModel;
    if(getCorrespondingPenaltyMark(pose, thePenaltyMarkPercept.positionOnField, penaltyMarkWorldModel))
    {
      RegisteredLandmark newLandmark;
      newLandmark.model = penaltyMarkWorldModel;
      newLandmark.percept = thePenaltyMarkPercept.positionOnField;
      if(timeOfLastPenaltyMarkCovarianceUpdate != theFrameInfo.time)
      {
        penaltyMarkCovariance = Measurements::positionToCovarianceMatrixInRobotCoordinates(newLandmark.percept, 0.f, theCameraMatrix, inverseCameraMatrix, currentRotationDeviation);
        timeOfLastPenaltyMarkCovarianceUpdate = theFrameInfo.time;
      }
      newLandmark.covPercept = penaltyMarkCovariance;
      landmarks.push_back(newLandmark);
    }
  }
  // Register the center circle: ooooooooooooooooooooooooooooooooooooooooooooooooooooo
  if(theCirclePercept.wasSeen)
  {
    const Vector2f circleInWorld = pose * theCirclePercept.pos;
    if(circleInWorld.norm() <= maxCenterCircleDeviation) // We assume that the CENTER circle is always at (0,0)
    {
      RegisteredLandmark newLandmark;
      newLandmark.model = Vector2f(0.f, 0.f);
      newLandmark.percept = theCirclePercept.pos;
      if(timeOfLastCenterCircleCovarianceUpdate != theFrameInfo.time)
      {
        // The distance to a perceived center circle center
        // might be extremely small (which results in an extremely small covariance) as
        // the center is not perceived directly but is computed based on lines on the circle.
        // An extreme case would be a robot standing in the center of the center circle.
        // However, the measurement of the circle is definitely not as precise as
        // the small covariance might indicate. This could lead to significant localization
        // errors that rotate the robot in the wrong direction. B-Human will das nicht.
        // Thus, a minimum is required for the covariance. This is realized by assuming a minimum
        // distance to the center circle (the covariance computation is based on this distance).
        const float circleDistance = theCirclePercept.pos.norm();
        const float minimumCircleDistance = theFieldDimensions.centerCircleRadius * 0.7834f; // Based on optimization.                        Not.
        centerCircleCovariance = Measurements::positionToCovarianceMatrixInRobotCoordinates(
          circleDistance < minimumCircleDistance ? Vector2f(minimumCircleDistance, 0.f) : theCirclePercept.pos,
          0.f, theCameraMatrix, inverseCameraMatrix, currentRotationDeviation);
        timeOfLastCenterCircleCovarianceUpdate = theFrameInfo.time;
      }
      newLandmark.covPercept = centerCircleCovariance;
      landmarks.push_back(newLandmark);
    }
  }
  // Register the intersections / crossings of field lines: TXL TXL TXL TXL TXL TXL TXL TXL TXL TXL TXL
  for(unsigned int i = 0; i < theFieldLineIntersections.intersections.size(); ++i)
  {
    const auto& intersection = theFieldLineIntersections.intersections[i];
    Vector2f intersectionInWorldModel;
    if(getCorrespondingIntersection(pose, intersection, intersectionInWorldModel))
    {
      RegisteredLandmark newLandmark;
      newLandmark.model = intersectionInWorldModel;
      newLandmark.percept = intersection.pos;
      if(timesOfLastIntersectionCovarianceUpdates[i] < theFrameInfo.time)
      {
        intersectionCovariances[i] = Measurements::positionToCovarianceMatrixInRobotCoordinates(newLandmark.percept, 0.f, theCameraMatrix, inverseCameraMatrix, currentRotationDeviation);
        timesOfLastIntersectionCovarianceUpdates[i] = theFrameInfo.time;
      }
      newLandmark.covPercept = intersectionCovariances[i];
      landmarks.push_back(newLandmark);
    }
  }
  // Register goal posts: I I I I I I I I I I I I I I I I I I I I I I I I I I I I I I I I I I I I I
  for(unsigned int i = 0; i < theGoalPostsPercept.goalPosts.size(); ++i)
  {
    const auto& goalPost = theGoalPostsPercept.goalPosts[i];
    if(!goalPost.baseInImage)
      continue;
    Vector2f goalPostInWorldModel;
    if(getCorrespondingGoalPost(pose, goalPost.relativePosition, goalPostInWorldModel))
    {
      RegisteredLandmark newLandmark;
      newLandmark.model = goalPostInWorldModel;
      newLandmark.percept = goalPost.relativePosition;
      if(timesOfLastGoalPostCovarianceUpdates[i] < theFrameInfo.time)
      {
        goalPostsCovariances[i] = Measurements::positionToCovarianceMatrixInRobotCoordinates(newLandmark.percept, 0.f, theCameraMatrix, inverseCameraMatrix, currentRotationDeviation);
        timesOfLastGoalPostCovarianceUpdates[i] = theFrameInfo.time;
      }
      newLandmark.covPercept = goalPostsCovariances[i];
      landmarks.push_back(newLandmark);
    }
  }
}

void PerceptRegistrationProvider::registerLines(const Pose2f& pose, std::vector<RegisteredLine>& lines, int& numberOfIgnoredLines)
{
  numberOfIgnoredLines = 0;
  // Iterate over all observed lines and try to match each line:
  for(unsigned int i = 0; i < theFieldLines.lines.size(); ++i)
  {
    const auto& line = theFieldLines.lines[i];
    bool lineIsOnCenterCircle;
    const WorldModelFieldLine* fieldLineWorld = getPointerToCorrespondingLineInWorldModel(pose, line.first, line.last, line.length, lineIsOnCenterCircle);
    if(fieldLineWorld != nullptr || lineIsOnCenterCircle)
    {
      // Compute covariance of point on center of line percept (if not already in buffer)
      if(timesOfLastLineCovarianceUpdates[i] < theFrameInfo.time)
      {
        // TODO: Consider kind of covariance scaling similar to center circle percept.
        const Vector2f linePerceptCenter = (line.first + line.last) * 0.5f;
        lineCovariances[i] = Measurements::positionToCovarianceMatrixInRobotCoordinates(linePerceptCenter, 0.f, theCameraMatrix, inverseCameraMatrix, currentRotationDeviation);
        timesOfLastLineCovarianceUpdates[i] = theFrameInfo.time;
      }
      // Normal case: Line in the world model was found and pointer references it:
      if(fieldLineWorld != nullptr)
      {
        RegisteredLine newLine(line.first, line.last, fieldLineWorld->start, fieldLineWorld->end,
                               lineCovariances[i]);
        lines.push_back(newLine);
      }
      // Alternative case: Line appears to be a part of the center circle:
      else if(registerLinesOnCenterCircle) // the condition if(lineIsOnCenterCircle) must be true here
      {
        // The world model is just used for drawing, create a fake line from the center
        // of the perceived line to the center circle.
        const Vector2f fakeLineStartInWorld(pose * ((line.first + line.last) * 0.5f));
        const Vector2f fakeLineEndInWorld(0.f,0.f);
        RegisteredLine newLine(line.first, line.last, fakeLineStartInWorld, fakeLineEndInWorld,
                               lineCovariances[i], true);
        lines.push_back(newLine);
      }
      // Remaining case: line appears to be on the center circle but should be ignored
      else
      {
        numberOfIgnoredLines++;
      }
    }
  }
}

bool PerceptRegistrationProvider::getCorrespondingPenaltyMark(const Pose2f& pose, const Vector2f& penaltyMarkPercept, Vector2f& penaltyMarkWorldModel) const
{
  const Vector2f penaltyMarkInWorld = pose * penaltyMarkPercept;
  penaltyMarkWorldModel = penaltyMarkInWorld.x() <= 0.f ? ownPenaltyMarkWorldModel : opponentPenaltyMarkWorldModel;
  const float differenceBetweenPerceptAndModel = (penaltyMarkInWorld - penaltyMarkWorldModel).norm();
  return differenceBetweenPerceptAndModel <= maxPenaltyMarkDeviation;
}

bool PerceptRegistrationProvider::getCorrespondingGoalPost(const Pose2f& pose, const Vector2f& goalPostPercept, Vector2f& goalPostWorldModel) const
{
  const Vector2f goalPostInWorld = pose * goalPostPercept;
  const Vector2f* listOfGoalPosts = goalPostInWorld.x() > 0.f ? &opponentGoalPostsWorldModel[0] : &ownGoalPostsWorldModel[0];
  for(int i=0; i<2; i++)
  {
    if((listOfGoalPosts[i] - goalPostInWorld).norm() <= maxGoalPostDeviation)
    {
      goalPostWorldModel = listOfGoalPosts[i];
      return true;
    }
  }
  return false;
}

bool PerceptRegistrationProvider::getCorrespondingIntersection(const Pose2f& pose, const FieldLineIntersections::Intersection& intersectionPercept, Vector2f& intersectionWorldModel) const
{
  const std::vector< Vector2f >* intersectionList = 0;
  if(intersectionPercept.type == FieldLineIntersections::Intersection::X)
  {
    intersectionList = &theFieldDimensions.corners[FieldDimensions::xCorner];
  }
  else if(intersectionPercept.type == FieldLineIntersections::Intersection::T)
  {
    int section = intersectionDirectionTo90DegreeSection(pose, intersectionPercept.dir1);
    switch(section)
    {
      case 0:   intersectionList = &theFieldDimensions.corners[FieldDimensions::tCorner0];   break;
      case 90:  intersectionList = &theFieldDimensions.corners[FieldDimensions::tCorner90];  break;
      case 180: intersectionList = &theFieldDimensions.corners[FieldDimensions::tCorner180]; break;
      default:  intersectionList = &theFieldDimensions.corners[FieldDimensions::tCorner270]; break;
    }
  }
  else if(intersectionPercept.type == FieldLineIntersections::Intersection::L)
  {
    int section = intersectionDirectionTo90DegreeSection(pose, intersectionPercept.dir1);
    switch(section)
    {
      case 0:   intersectionList = &theFieldDimensions.corners[FieldDimensions::lCorner0];   break;
      case 90:  intersectionList = &theFieldDimensions.corners[FieldDimensions::lCorner90];  break;
      case 180: intersectionList = &theFieldDimensions.corners[FieldDimensions::lCorner180]; break;
      default:  intersectionList = &theFieldDimensions.corners[FieldDimensions::lCorner270]; break;
    }
  }
  if(intersectionList != 0)
  {
    // Check for empty list. This might happen for special configurations of demo fields.
    if(intersectionList->size() == 0)
      return false;
    // Initialize variables to store the real world intersection that is closest to the percept:
    const Vector2f perceptWorld = pose * intersectionPercept.pos;
    Vector2f closestIntersectionWorld = intersectionList->at(0);
    float sqrDistanceToClosestIntersectionWorld = (perceptWorld - closestIntersectionWorld).squaredNorm();
    // Iterate over list to find the closest intersection:
    for(unsigned int i = 1; i < intersectionList->size(); ++i)
    {
      const Vector2f& intersection = intersectionList->at(i);
      const float sqrDistance = (perceptWorld - intersection).squaredNorm();
      if(sqrDistance < sqrDistanceToClosestIntersectionWorld)
      {
        sqrDistanceToClosestIntersectionWorld = sqrDistance;
        closestIntersectionWorld = intersection;
      }
    }
    // Check, if closest intersection is close enough:
    if(sqrDistanceToClosestIntersectionWorld < maxIntersectionDeviation * maxIntersectionDeviation)
    {
      intersectionWorldModel = closestIntersectionWorld;
      return true;
    }
  }
  return false;
}

int PerceptRegistrationProvider::intersectionDirectionTo90DegreeSection(const Pose2f& pose, const Vector2f& dir) const
{
  Vector2f directionInFieldCoordinates = dir;
  directionInFieldCoordinates.rotate(-pose.rotation);
  const float degrees = toDegrees(directionInFieldCoordinates.angle()) + 180.f;
  if(degrees < 45 || degrees >= 315)
    return 0;
  else if(degrees >= 45 && degrees < 135)
    return 90;
  else if(degrees >= 135 && degrees < 225)
    return 180;
  else // if(degrees >= 225 && degrees < 315)
    return 270;
}

const PerceptRegistrationProvider::WorldModelFieldLine*
PerceptRegistrationProvider::getPointerToCorrespondingLineInWorldModel(const Pose2f& pose, const Vector2f& start, const Vector2f& end, float lineLength, bool& isPartOfCenterCircle) const
{
  const Vector2f startOnField = pose * start;
  const Vector2f endOnField = pose * end;
  Vector2f dirOnField = endOnField - startOnField;
  dirOnField.normalize();
  const bool isVertical = std::abs(dirOnField.x()) > std::abs(dirOnField.y());
  const float sqrLineAssociationCorridor = sqr(lineAssociationCorridor);

  // Lines that are detected on the center circle are a bit special:
  if(lineShouldBeMatchedWithCenterCircle(startOnField, endOnField, dirOnField, lineLength))
  {
    // There is no actual line that is associated here. Measurement update of the UKF hypotheses
    // has to create a virtual thing that is measured.
    isPartOfCenterCircle = true;
    return nullptr;
  }
  isPartOfCenterCircle = false;
  // If this point is reached, the line is matched against the "normal" field lines:
  const std::vector<WorldModelFieldLine>& worldModelLines = isVertical ? verticalLinesWorldModel : horizontalLinesWorldModel;
  for(unsigned int i = 0; i < worldModelLines.size(); ++i)
  {
    const WorldModelFieldLine& worldModelLine = worldModelLines[i];
    // A perceived line cannot be longer than the original line:
    if(lineLength > 1.25f * worldModelLine.length)
      continue;
    // The begin as well as the end of the perceived line should not be too far away from the model:
    if(getSqrDistanceToLineSegment(worldModelLine.start, worldModelLine.dir, worldModelLine.length, startOnField) > sqrLineAssociationCorridor ||
       getSqrDistanceToLineSegment(worldModelLine.start, worldModelLine.dir, worldModelLine.length, endOnField) > sqrLineAssociationCorridor)
      continue;
    // Not other conditions implemented yet. Some more might be added later (see old implementation).
    // ...
    // Check association of perceived line to center line. Often, short schniptzels of the center circle
    // become falsely associated to the center line, causing a translational error in the self-localization.
    // Thus, any perceived center line must be longer than the center circle diameter, if it was perceived
    // from a distance that is larger than the diameter.
    // Please note that these are just frickelparameters, but I do not think that we should
    // add some separate parameters to the module.
    const float diameter = 2.f * theFieldDimensions.centerCircleRadius;
    if(worldModelLine.isCenterLine && lineLength < diameter &&
       start.norm() > diameter && end.norm() > diameter)
    {
      return nullptr;
    }
    return &(worldModelLines[i]);
  }
  // Hmmm, no matching line has been found ...
  return nullptr;
}

bool PerceptRegistrationProvider::lineShouldBeMatchedWithCenterCircle(const Vector2f& start, const Vector2f& end, const Vector2f& dir, float lineLength) const
{
  // A line on the center circle
  // - is short,
  if(lineLength > 2000.f)
    return false;
  // - has start and end points that are about one center circle radius away from the field center,
  const float distToStart = start.norm();
  const float distToEnd   = end.norm();
  const float centerCircleCorridorInner = theFieldDimensions.centerCircleRadius * 0.4f;
  const float centerCircleCorridorOuter = theFieldDimensions.centerCircleRadius * 1.7f;
  if(distToStart < centerCircleCorridorInner || distToStart > centerCircleCorridorOuter ||
     distToEnd   < centerCircleCorridorInner || distToEnd   > centerCircleCorridorOuter)
    return false;
  // - and remains at a certain distance to the center at all:
  Geometry::Line line;
  line.base = start;
  line.direction = dir;
  const float distToCenter = std::abs(Geometry::getDistanceToLine(line, Vector2f::Zero()));
  return distToCenter > centerCircleCorridorInner && distToCenter < centerCircleCorridorOuter;
}

float PerceptRegistrationProvider::getSqrDistanceToLineSegment(const Vector2f& base, const Vector2f& dir, float length, const Vector2f& point) const
{
  float l = (point.x() - base.x()) * dir.x() + (point.y() - base.y()) * dir.y();
  if(l < 0)
    l = 0;
  else if(l > length)
    l = length;
  return ((base + dir * l) - point).squaredNorm();
}
