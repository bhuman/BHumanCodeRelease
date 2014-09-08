/**
* @file Modules/Infrastructure/OracledPerceptsProvider.h
*
* This file implements a module that provides precepts based on simulated data.
*
* @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
*/

#include "OracledPerceptsProvider.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Math/Probabilistics.h"

OracledPerceptsProvider::OracledPerceptsProvider()
{
  // Four goal posts
  goalPosts.push_back(Vector2<>(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal));
  goalPosts.push_back(Vector2<>(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal));
  goalPosts.push_back(Vector2<>(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosLeftGoal));
  goalPosts.push_back(Vector2<>(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosRightGoal));
  // Five points roughly approximating the center circle
  ccPoints.push_back(Vector2<>(0.f, 0.f));
  ccPoints.push_back(Vector2<>(0.f, theFieldDimensions.centerCircleRadius));
  ccPoints.push_back(Vector2<>(0.f, -theFieldDimensions.centerCircleRadius));
  ccPoints.push_back(Vector2<>(theFieldDimensions.centerCircleRadius, 0.f));
  ccPoints.push_back(Vector2<>(-theFieldDimensions.centerCircleRadius, 0.f));
  // Half of the intersections
  LinePercept::Intersection oppLeftCorner;
  oppLeftCorner.type = LinePercept::Intersection::L;
  oppLeftCorner.pos = Vector2<>(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftSideline);
  oppLeftCorner.dir1 = Vector2<>(-1.f, 0.f);
  oppLeftCorner.dir2 = Vector2<>(0.f, -1.f);
  intersections.push_back(oppLeftCorner);
  LinePercept::Intersection oppLeftPenaltyArea;
  oppLeftPenaltyArea.type = LinePercept::Intersection::T;
  oppLeftPenaltyArea.pos = Vector2<>(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftPenaltyArea);
  oppLeftPenaltyArea.dir1 = Vector2<>(-1.f, 0.f);
  oppLeftPenaltyArea.dir2 = Vector2<>(0.f, -1.f);
  intersections.push_back(oppLeftPenaltyArea);
  LinePercept::Intersection oppRightPenaltyArea;
  oppRightPenaltyArea.type = LinePercept::Intersection::T;
  oppRightPenaltyArea.pos = Vector2<>(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightPenaltyArea);
  oppRightPenaltyArea.dir1 = Vector2<>(-1.f, 0.f);
  oppRightPenaltyArea.dir2 = Vector2<>(0.f, -1.f);
  intersections.push_back(oppRightPenaltyArea);
  LinePercept::Intersection oppRightCorner;
  oppRightCorner.type = LinePercept::Intersection::L;
  oppRightCorner.pos = Vector2<>(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightSideline);
  oppRightCorner.dir1 = Vector2<>(-1.f, 0.f);
  oppRightCorner.dir2 = Vector2<>(0.f, 1.f);
  intersections.push_back(oppRightCorner);
  LinePercept::Intersection oppLeftPenaltyCorner;
  oppLeftPenaltyCorner.type = LinePercept::Intersection::L;
  oppLeftPenaltyCorner.pos = Vector2<>(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea);
  oppLeftPenaltyCorner.dir1 = Vector2<>(1.f, 0.f);
  oppLeftPenaltyCorner.dir2 = Vector2<>(0.f, -1.f);
  intersections.push_back(oppLeftPenaltyCorner);
  LinePercept::Intersection oppRightPenaltyCorner;
  oppRightPenaltyCorner.type = LinePercept::Intersection::L;
  oppRightPenaltyCorner.pos = Vector2<>(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosRightPenaltyArea);
  oppRightPenaltyCorner.dir1 = Vector2<>(1.f, 0.f);
  oppRightPenaltyCorner.dir2 = Vector2<>(0.f, 1.f);
  intersections.push_back(oppRightPenaltyCorner);
  LinePercept::Intersection leftCenterLineCrossing;
  leftCenterLineCrossing.type = LinePercept::Intersection::T;
  leftCenterLineCrossing.pos = Vector2<>(0.f, theFieldDimensions.yPosLeftSideline);
  leftCenterLineCrossing.dir1 = Vector2<>(1.f, 0.f);
  leftCenterLineCrossing.dir2 = Vector2<>(0.f, -1.f);
  intersections.push_back(leftCenterLineCrossing);
  LinePercept::Intersection leftCenterCircleCrossing;
  leftCenterCircleCrossing.type = LinePercept::Intersection::X;
  leftCenterCircleCrossing.pos = Vector2<>(0.f, theFieldDimensions.centerCircleRadius);
  leftCenterCircleCrossing.dir1 = Vector2<>(1.f, 0.f);
  leftCenterCircleCrossing.dir2 = Vector2<>(0.f, -1.f);
  intersections.push_back(leftCenterCircleCrossing);
  // The other half of the intersections is mirrored to the first half:
  const size_t numOfIntersections = intersections.size();
  for(unsigned int i = 0; i < numOfIntersections; i++)
  {
    LinePercept::Intersection mirroredIntersection = intersections[i];
    mirroredIntersection.pos = Pose2D(pi) * mirroredIntersection.pos;
    mirroredIntersection.dir1 = Pose2D(pi) * mirroredIntersection.dir1;
    mirroredIntersection.dir2 = Pose2D(pi) * mirroredIntersection.dir2;
    intersections.push_back(mirroredIntersection);
  }
  // The lines
  std::pair<Vector2<>, Vector2<>> line;
  // ground lines and center line:
  line.first =  Vector2<>(0, theFieldDimensions.yPosLeftSideline);
  line.second = Vector2<>(0, theFieldDimensions.yPosRightSideline);
  lines.push_back(line);
  line.first =  Vector2<>(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftSideline);
  line.second = Vector2<>(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightSideline);
  lines.push_back(line);
  line.first =  Vector2<>(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftSideline);
  line.second = Vector2<>(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightSideline);
  lines.push_back(line);
  // side lines:
  line.first =  Vector2<>(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftSideline);
  line.second = Vector2<>(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftSideline);
  lines.push_back(line);
  line.first =  Vector2<>(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightSideline);
  line.second = Vector2<>(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightSideline);
  lines.push_back(line);
  // opponent penalty area:
  line.first =  Vector2<>(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftPenaltyArea);
  line.second = Vector2<>(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea);
  lines.push_back(line);
  line.first =  Vector2<>(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightPenaltyArea);
  line.second = Vector2<>(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosRightPenaltyArea);
  lines.push_back(line);
  line.first =  Vector2<>(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea);
  line.second = Vector2<>(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosRightPenaltyArea);
  lines.push_back(line);
  // own penalty area:
  line.first =  Vector2<>(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftPenaltyArea);
  line.second = Vector2<>(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea);
  lines.push_back(line);
  line.first =  Vector2<>(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightPenaltyArea);
  line.second = Vector2<>(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosRightPenaltyArea);
  lines.push_back(line);
  line.first =  Vector2<>(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea);
  line.second = Vector2<>(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosRightPenaltyArea);
  lines.push_back(line);
  // The field boundary
  line.first =  Vector2<>(theFieldDimensions.xPosOpponentFieldBorder, theFieldDimensions.yPosLeftFieldBorder);
  line.second = Vector2<>(theFieldDimensions.xPosOpponentFieldBorder, theFieldDimensions.yPosRightFieldBorder);
  fieldBoundaryLines.push_back(line);
  line.first =  Vector2<>(theFieldDimensions.xPosOwnFieldBorder, theFieldDimensions.yPosLeftFieldBorder);
  line.second = Vector2<>(theFieldDimensions.xPosOwnFieldBorder, theFieldDimensions.yPosRightFieldBorder);
  fieldBoundaryLines.push_back(line);
  line.first =  Vector2<>(theFieldDimensions.xPosOpponentFieldBorder, theFieldDimensions.yPosLeftFieldBorder);
  line.second = Vector2<>(theFieldDimensions.xPosOwnFieldBorder, theFieldDimensions.yPosLeftFieldBorder);
  fieldBoundaryLines.push_back(line);
  line.first =  Vector2<>(theFieldDimensions.xPosOpponentFieldBorder, theFieldDimensions.yPosRightFieldBorder);
  line.second = Vector2<>(theFieldDimensions.xPosOwnFieldBorder, theFieldDimensions.yPosRightFieldBorder);
  fieldBoundaryLines.push_back(line);
}

void OracledPerceptsProvider::update(BallPercept& ballPercept)
{
  ballPercept.status = BallPercept::notSeen;
  if(!theCameraMatrix.isValid)
    return;
  if(theGroundTruthWorldState.balls.size() != 0)
  {
    const Vector2<>& ballOnField = theGroundTruthWorldState.balls[0];
    Vector2<> ballOffset = theGroundTruthWorldState.ownPose.invert() * ballOnField;
    if(ballOffset.abs() > ballMaxVisibleDistance)
      return;
    if(randomFloat() > ballRecognitionRate)
      return;
    Geometry::Circle circle;
    if(Geometry::calculateBallInImage(ballOffset, theCameraMatrix, theCameraInfo, theFieldDimensions.ballRadius, circle))
    {
      if((circle.center.x >= -circle.radius / 1.5f) &&
         (circle.center.x < theCameraInfo.width + circle.radius / 1.5f) &&
         (circle.center.y >= -circle.radius / 1.5f) &&
         (circle.center.y < theCameraInfo.height + circle.radius / 1.5f))
      {
        ballPercept.status = BallPercept::seen;
        ballPercept.positionInImage = circle.center;
        ballPercept.radiusInImage = circle.radius;
        ballPercept.relativePositionOnField = ballOffset;
        // Add some noise
        if(applyBallNoise)
        {
          applyNoise(ballCenterInImageStdDev, ballPercept.positionInImage);
          Transformation::imageToRobotHorizontalPlane(ballPercept.positionInImage, theFieldDimensions.ballRadius, theCameraMatrix,
              theCameraInfo, ballPercept.relativePositionOnField);
        }
      }
    }
  }
}

void OracledPerceptsProvider::update(GoalPercept& goalPercept)
{
  goalPercept.goalPosts.clear();
  if(!theCameraMatrix.isValid)
    return;
  const Pose2D robotPoseInv = theGroundTruthWorldState.ownPose.invert();
  for(unsigned int i = 0; i < goalPosts.size(); i++)
  {
    const Vector2<> relativePostPos = robotPoseInv * goalPosts[i];
    if(relativePostPos.abs() > goalPostMaxVisibleDistance)
      continue;
    if(randomFloat() > goalPostRecognitionRate)
      continue;
    Vector2<> postInImage;
    if(pointIsInImage(relativePostPos, postInImage))
    {
      GoalPost newPost; // GoalPost::IS_UNKNOWN is the default side information
      newPost.positionInImage.x = static_cast<int>(std::floor(postInImage.x + 0.5f));
      newPost.positionInImage.y = static_cast<int>(std::floor(postInImage.y + 0.5f));
      newPost.positionOnField = relativePostPos;
      // Add some noise:
      if(applyGoalPostNoise)
      {
        applyNoise(ballCenterInImageStdDev, newPost.positionInImage);
        Transformation::imageToRobot(newPost.positionInImage.x, newPost.positionInImage.y, theCameraMatrix, theCameraInfo, newPost.positionOnField);
      }
      // If a part of the goal bar might be in the image, there is also a side information:
      const Vector3<> relativeBarPos(relativePostPos.x, relativePostPos.y, theFieldDimensions.goalHeight);
      Vector2<> barInImage;
      if(Transformation::robotToImage(relativeBarPos, theCameraMatrix, theCameraInfo, barInImage))
      {
        if((barInImage.x >= 0.f) && (barInImage.x < static_cast<float>(theCameraInfo.width)) &&
           (barInImage.y >= 0.f) && (barInImage.y < static_cast<float>(theCameraInfo.height)))
        {
          if((goalPosts[i].x > 0 && goalPosts[i].y > 0) || (goalPosts[i].x < 0 && goalPosts[i].y < 0))
            newPost.position = GoalPost::IS_LEFT;
          else
            newPost.position = GoalPost::IS_RIGHT;
        }
      }
      goalPercept.goalPosts.push_back(newPost);
    }
  }
  if(goalPercept.goalPosts.size())
    goalPercept.timeWhenGoalPostLastSeen = theFrameInfo.time;
  if(goalPercept.goalPosts.size() == 2)
  {
    goalPercept.timeWhenCompleteGoalLastSeen = theFrameInfo.time;
    float angleToFirst  = std::atan2(goalPercept.goalPosts[0].positionOnField.y, goalPercept.goalPosts[0].positionOnField.x);
    float angleToSecond = std::atan2(goalPercept.goalPosts[1].positionOnField.y, goalPercept.goalPosts[1].positionOnField.x);
    if(angleToFirst < angleToSecond)
    {
      goalPercept.goalPosts[0].position = GoalPost::IS_RIGHT;
      goalPercept.goalPosts[1].position = GoalPost::IS_LEFT;
    }
    else
    {
      goalPercept.goalPosts[0].position = GoalPost::IS_LEFT;
      goalPercept.goalPosts[1].position = GoalPost::IS_RIGHT;
    }
  }
}

void OracledPerceptsProvider::update(LinePercept& linePercept)
{
  // Initialize percept and local data:
  linePercept.intersections.clear();
  linePercept.lines.clear();
  linePercept.circle.found = false;
  if(!theCameraMatrix.isValid)
    return;
  updateViewPolygon();
  const Pose2D robotPoseInv = theGroundTruthWorldState.ownPose.invert();

  // Find center circle (at least one out of five center circle points must be inside the current image)
  bool pointFound = false;
  if((theGroundTruthWorldState.ownPose.translation.abs() <= centerCircleMaxVisibleDistance) &&
     (randomFloat() < centerCircleRecognitionRate))
  {
    for(unsigned int i = 0; i < ccPoints.size(); ++i)
    {
      const Vector2<> relPos = robotPoseInv * ccPoints[i];
      Vector2<> posInImage;
      if(pointIsInImage(relPos, posInImage))
      {
        pointFound = true;
        break;
      }
    }
  }
  if(pointFound)
  {
    linePercept.circle.found = true;
    linePercept.circle.lastSeen = theFrameInfo.time;
    linePercept.circle.pos = robotPoseInv * Vector2<>(0.f, 0.f);
    // Add some noise:
    if(applyCenterCircleNoise)
    {
      Vector2<> nPImg;
      Transformation::robotToImage(linePercept.circle.pos, theCameraMatrix, theCameraInfo, nPImg);
      applyNoise(centerCircleCenterInImageStdDev, nPImg);
      Transformation::imageToRobot(nPImg.x, nPImg.y, theCameraMatrix, theCameraInfo, linePercept.circle.pos);
    }
  }

  // Find intersections:
  for(unsigned int i = 0; i < intersections.size(); i++)
  {
    const Vector2<> relativeIntersectionPos = robotPoseInv * intersections[i].pos;
    if(relativeIntersectionPos.abs() > intersectionMaxVisibleDistance)
      continue;
    if(randomFloat() > intersectionRecognitionRate)
      continue;
    Vector2<> intersectionInImage;
    if(pointIsInImage(relativeIntersectionPos, intersectionInImage))
    {
      LinePercept::Intersection newIntersection;
      newIntersection.pos = relativeIntersectionPos;
      newIntersection.type = intersections[i].type;
      newIntersection.dir1 = (Pose2D(robotPoseInv.rotation) * intersections[i].dir1);
      newIntersection.dir2 = (Pose2D(robotPoseInv.rotation) * intersections[i].dir2);
      if(applyIntersectionNoise)
      {
        applyNoise(intersectionPosInImageStdDev, intersectionInImage);
        Transformation::imageToRobot(intersectionInImage.x, intersectionInImage.y, theCameraMatrix, theCameraInfo, newIntersection.pos);
        // noise on directions is not implemented, but if you need it, feel free to add it right here
      }
      linePercept.intersections.push_back(newIntersection);
    }
  }

  // Find lines:
  for(unsigned int i = 0; i < lines.size(); i++)
  {
    if(randomFloat() > lineRecognitionRate)
      continue;
    Vector2<> start, end;
    if(partOfLineIsVisible(lines[i], start, end))
    {
      LinePercept::Line line;
      line.first = robotPoseInv * start;
      line.last = robotPoseInv * end;
      if(line.first.abs() > lineMaxVisibleDistance || line.last.abs() > lineMaxVisibleDistance)
        continue;
      line.midLine = (i == 0);
      Vector2<> pImg;
      Transformation::robotToImage(line.first, theCameraMatrix, theCameraInfo, pImg);
      line.startInImage = pImg;
      Transformation::robotToImage(line.last, theCameraMatrix, theCameraInfo, pImg);
      line.endInImage = pImg;
      if(applyLineNoise)
      {
        applyNoise(linePosInImageStdDev, line.startInImage);
        applyNoise(linePosInImageStdDev, line.endInImage);
        Transformation::imageToRobot(line.startInImage.x, line.startInImage.y, theCameraMatrix, theCameraInfo, line.first);
        Transformation::imageToRobot(line.endInImage.x, line.endInImage.y, theCameraMatrix, theCameraInfo, line.last);
      }
      line.dead = false;

      line.alpha = (line.first - line.last).angle() + pi_2;
      while(line.alpha < 0)
        line.alpha += pi;
      while(line.alpha >= pi)
        line.alpha -= pi;
      const float c = std::cos(line.alpha),
                  s = std::sin(line.alpha);
      line.d = line.first.x * c + line.first.y * s;

      line.segments.clear(); // has to remain empty
      linePercept.lines.push_back(line);
    }
  }
}

void OracledPerceptsProvider::update(RobotPercept& robotPercept)
{
  robotPercept.robots.clear();
  if(!theCameraMatrix.isValid)
    return;
  for(unsigned int i = 0; i < theGroundTruthWorldState.blueRobots.size(); ++i)
    createRobotBox(theGroundTruthWorldState.blueRobots[i], true, robotPercept);
  for(unsigned int i = 0; i < theGroundTruthWorldState.redRobots.size(); ++i)
    createRobotBox(theGroundTruthWorldState.redRobots[i], false, robotPercept);
}

void OracledPerceptsProvider::update(FieldBoundary& fieldBoundary)
{
  // Initialize percept and local data:
  fieldBoundary.boundaryInImage.clear();
  fieldBoundary.boundaryOnField.clear();
  fieldBoundary.boundarySpots.clear();
  fieldBoundary.convexBoundary.clear();
  if(!theCameraMatrix.isValid)
  {
    fieldBoundary.convexBoundary.push_back(Vector2<int>(0, theCameraInfo.height));
    fieldBoundary.convexBoundary.push_back(Vector2<int>(theCameraInfo.width - 1, theCameraInfo.height));
    fieldBoundary.boundaryInImage = fieldBoundary.convexBoundary;
    fieldBoundary.boundaryOnField.push_back(Vector2<float>(0, 1));
    fieldBoundary.boundaryOnField.push_back(Vector2<float>(0, -1));
    fieldBoundary.highestPoint = Vector2<int>(theCameraInfo.width / 2, theCameraInfo.height);
    fieldBoundary.isValid = false;
    return;
  }
  updateViewPolygon();
  const Pose2D robotPoseInv = theGroundTruthWorldState.ownPose.invert();

  // Find boundary lines:
  for(unsigned int i = 0; i < fieldBoundaryLines.size(); i++)
  {
    Vector2<> start, end;
    if(partOfLineIsVisible(fieldBoundaryLines[i], start, end))
    {
      fieldBoundary.boundaryOnField.push_back(start);
      fieldBoundary.boundaryOnField.push_back(end);
      Vector2<> pImgStart, pImgEnd;
      Transformation::robotToImage(start, theCameraMatrix, theCameraInfo, pImgStart);
      Transformation::robotToImage(end, theCameraMatrix, theCameraInfo, pImgEnd);
      fieldBoundary.boundaryInImage.push_back(Vector2<int>(static_cast<int>(pImgStart.x), static_cast<int>(pImgStart.y)));
      fieldBoundary.boundaryInImage.push_back(Vector2<int>(static_cast<int>(pImgEnd.x), static_cast<int>(pImgEnd.y)));
    }
  }
  fieldBoundary.isValid = fieldBoundary.boundaryOnField.size() != 0;
  if(fieldBoundary.boundaryOnField.size() < 2)
  {
    fieldBoundary.convexBoundary.push_back(Vector2<int>(0, theCameraInfo.height));
    fieldBoundary.convexBoundary.push_back(Vector2<int>(theCameraInfo.width - 1, theCameraInfo.height));
    fieldBoundary.boundaryInImage = fieldBoundary.convexBoundary;
    fieldBoundary.boundaryOnField.push_back(Vector2<float>(0, 1));
    fieldBoundary.boundaryOnField.push_back(Vector2<float>(0, -1));
    fieldBoundary.highestPoint = Vector2<int>(theCameraInfo.width / 2, theCameraInfo.height);
    fieldBoundary.isValid = false;
  }
}

void OracledPerceptsProvider::createRobotBox(const GroundTruthWorldState::GroundTruthRobot& robot, bool isBlue, RobotPercept& robotPercept)
{
  const Pose2D robotPoseInv = theGroundTruthWorldState.ownPose.invert();
  Vector2<> relativeRobotPos = robotPoseInv * robot.pose.translation;
  if(relativeRobotPos.abs() > robotMaxVisibleDistance)
    return;
  if(randomFloat() > robotRecognitionRate)
    return;
  Vector2<> robotInImage;
  if(pointIsInImage(relativeRobotPos, robotInImage))
  {
    if(applyRobotNoise)
    {
      applyNoise(robotPosInImageStdDev, robotInImage);
      Transformation::imageToRobot(robotInImage.x, robotInImage.y, theCameraMatrix, theCameraInfo, relativeRobotPos);
    }
    RobotPercept::RobotBox r;
    r.x1 = r.x2 = r.x1FeetOnly = r.x2FeetOnly = r.realCenterX = static_cast<int>(std::floor(robotInImage.x + 0.5f));
    r.y1 = r.y2 = static_cast<int>(std::floor(robotInImage.y + 0.5f));
    r.lowerCamera = theCameraInfo.camera == CameraInfo::lower;
    r.detectedJersey = true;
    r.detectedFeet   = true;
    r.teamRed = !isBlue;
    r.fallen  = false;
    robotPercept.robots.push_back(r);
  }
}

bool OracledPerceptsProvider::pointIsInImage(const Vector2<>& p, Vector2<>& pImg) const
{
  if(Transformation::robotToImage(p, theCameraMatrix, theCameraInfo, pImg))
  {
    if((pImg.x >= 0) && (pImg.x < theCameraInfo.width) && (pImg.y >= 0) && (pImg.y < theCameraInfo.height))
    {
      return true;
    }
  }
  return false;
}

void OracledPerceptsProvider::updateViewPolygon()
{
  // code is copied from FieldCoverageProvider::drawFieldView()
  const Vector3<> vectorToCenter(1, 0, 0);

  RotationMatrix r = theCameraMatrix.rotation;
  r.rotateY(theCameraInfo.openingAngleHeight / 2);
  r.rotateZ(theCameraInfo.openingAngleWidth / 2);
  Vector3<> vectorToCenterWorld = r * vectorToCenter;

  const float a1 = theCameraMatrix.translation.x,
              a2 = theCameraMatrix.translation.y,
              a3 = theCameraMatrix.translation.z;
  float b1 = vectorToCenterWorld.x,
        b2 = vectorToCenterWorld.y,
        b3 = vectorToCenterWorld.z,
        f = a3 / b3;
  Vector2<> pof = Vector2<>(a1 - f * b1, a2 - f * b2);

  if(f > 0.f)
    viewPolygon[0] = theGroundTruthWorldState.ownPose.translation;
  else
    viewPolygon[0] = (theGroundTruthWorldState.ownPose + pof).translation;

  r = theCameraMatrix.rotation;
  r.rotateY(theCameraInfo.openingAngleHeight / 2);
  r.rotateZ(-(theCameraInfo.openingAngleWidth / 2));
  vectorToCenterWorld = r * vectorToCenter;

  b1 = vectorToCenterWorld.x;
  b2 = vectorToCenterWorld.y;
  b3 = vectorToCenterWorld.z;
  f = a3 / b3;
  pof = Vector2<>(a1 - f * b1, a2 - f * b2);

  if(f > 0.f)
    viewPolygon[1] = theGroundTruthWorldState.ownPose.translation;
  else
    viewPolygon[1] = (theGroundTruthWorldState.ownPose + pof).translation;

  r = theCameraMatrix.rotation;
  r.rotateY(-(theCameraInfo.openingAngleHeight / 2));
  r.rotateZ(-(theCameraInfo.openingAngleWidth / 2));
  vectorToCenterWorld = r * vectorToCenter;

  b1 = vectorToCenterWorld.x;
  b2 = vectorToCenterWorld.y;
  b3 = vectorToCenterWorld.z;
  f = a3 / b3;
  pof = Vector2<>(a1 - f * b1, a2 - f * b2);

  const float maxDist = std::sqrt(4.f * theFieldDimensions.xPosOpponentFieldBorder * theFieldDimensions.xPosOpponentFieldBorder +
                                  4.f * theFieldDimensions.yPosLeftFieldBorder * theFieldDimensions.yPosLeftFieldBorder);
  if(f > 0.f)
    viewPolygon[2] = theGroundTruthWorldState.ownPose.translation + Vector2<>(maxDist, 0).rotate(theGroundTruthWorldState.ownPose.rotation + (-theCameraInfo.openingAngleWidth / 2) + theCameraMatrix.rotation.getZAngle());
  else
    viewPolygon[2] = (theGroundTruthWorldState.ownPose + pof).translation;

  r = theCameraMatrix.rotation;
  r.rotateY(-(theCameraInfo.openingAngleHeight / 2));
  r.rotateZ(theCameraInfo.openingAngleWidth / 2);
  vectorToCenterWorld = r * vectorToCenter;

  b1 = vectorToCenterWorld.x;
  b2 = vectorToCenterWorld.y;
  b3 = vectorToCenterWorld.z;
  f = a3 / b3;
  pof = Vector2<>(a1 - f * b1, a2 - f * b2);

  if(f > 0.f)
    viewPolygon[3] = theGroundTruthWorldState.ownPose.translation + Vector2<>(maxDist, 0).rotate(theGroundTruthWorldState.ownPose.rotation + (theCameraInfo.openingAngleWidth / 2) + theCameraMatrix.rotation.getZAngle());
  else
    viewPolygon[3] = (theGroundTruthWorldState.ownPose + pof).translation;
}

bool OracledPerceptsProvider::partOfLineIsVisible(const std::pair<Vector2<>, Vector2<>>& line, Vector2<>& start, Vector2<>& end) const
{
  // First case: both points are inside:
  if(Geometry::isPointInsideConvexPolygon(viewPolygon, 4, line.first) && Geometry::isPointInsideConvexPolygon(viewPolygon, 4, line.second))
  {
    start = line.first;
    end = line.second;
    return true;
  }
  // Second case: start is inside but end is outside
  if(Geometry::isPointInsideConvexPolygon(viewPolygon, 4, line.first) && !Geometry::isPointInsideConvexPolygon(viewPolygon, 4, line.second))
  {
    start = line.first;
    for(int i = 0; i < 4; i++)
    {
      if(Geometry::checkIntersectionOfLines(line.first, line.second, viewPolygon[i], viewPolygon[(i + 1) % 4]))
      {
        Geometry::getIntersectionOfLines(Geometry::Line(line.first, line.second - line.first),
                                         Geometry::Line(viewPolygon[i], viewPolygon[(i + 1) % 4] - viewPolygon[i]), end);
        return true;
      }
    }
  }
  // Third case: end is inside but start is outside
  if(!Geometry::isPointInsideConvexPolygon(viewPolygon, 4, line.first) && Geometry::isPointInsideConvexPolygon(viewPolygon, 4, line.second))
  {
    start = line.second;
    for(int i = 0; i < 4; i++)
    {
      if(Geometry::checkIntersectionOfLines(line.first, line.second, viewPolygon[i], viewPolygon[(i + 1) % 4]))
      {
        Geometry::getIntersectionOfLines(Geometry::Line(line.first, line.second - line.first),
                                         Geometry::Line(viewPolygon[i], viewPolygon[(i + 1) % 4] - viewPolygon[i]), end);
        return true;
      }
    }
  }
  // Fourth case: both points are outside the polygon but maybe intersect it:
  std::vector<Vector2<>> intersectionPoints;
  for(int i = 0; i < 4; i++)
  {
    if(Geometry::checkIntersectionOfLines(line.first, line.second, viewPolygon[i], viewPolygon[(i + 1) % 4]))
    {
      Vector2<> point;
      Geometry::getIntersectionOfLines(Geometry::Line(line.first, line.second - line.first),
                                       Geometry::Line(viewPolygon[i], viewPolygon[(i + 1) % 4] - viewPolygon[i]), point);
      intersectionPoints.push_back(point);
    }
  }
  // There are some more special cases that could be treated but in general, there should be two intersections that are not at the same place.
  // Other cases are ignored here
  if(intersectionPoints.size() == 2 && (intersectionPoints[0] - intersectionPoints[1]).abs() > 100.f)
  {
    start = intersectionPoints[0];
    end   = intersectionPoints[1];
    return true;
  }
  // Sorry, nothing found ...
  return false;
}

void OracledPerceptsProvider::applyNoise(float standardDeviation, Vector2<>& p) const
{
  p.x += sampleNormalDistribution(standardDeviation);
  p.y += sampleNormalDistribution(standardDeviation);
}

void OracledPerceptsProvider::applyNoise(float standardDeviation, Vector2<int>& p) const
{
  const float errorX = sampleNormalDistribution(standardDeviation);
  const float errorY = sampleNormalDistribution(standardDeviation);
  p.x += static_cast<int>(floor(errorX + 0.5f));
  p.y += static_cast<int>(floor(errorY + 0.5f));
}

MAKE_MODULE(OracledPerceptsProvider, Cognition Infrastructure)
