/**
* @file Modules/Infrastructure/OracledPerceptsProvider.h
*
* This file implements a module that provides precepts based on simulated data.
*
* @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
*/

#include "OracledPerceptsProvider.h"
#include "Tools/Global.h"
#include "Tools/Settings.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Math/Probabilistics.h"
#include "Tools/Math/RotationMatrix.h"
#include "Tools/Modeling/Obstacle.h"

OracledPerceptsProvider::OracledPerceptsProvider()
{
  // Four goal posts
  goalPosts.push_back(Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal));
  goalPosts.push_back(Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal));
  goalPosts.push_back(Vector2f(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosLeftGoal));
  goalPosts.push_back(Vector2f(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosRightGoal));
  // Two penalty marks
  penaltyMarks.push_back(Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, 0.f));
  penaltyMarks.push_back(Vector2f(theFieldDimensions.xPosOwnPenaltyMark, 0.f));
  // Five points roughly approximating the center circle
  ccPoints.push_back(Vector2f::Zero());
  ccPoints.push_back(Vector2f(0.f, theFieldDimensions.centerCircleRadius));
  ccPoints.push_back(Vector2f(0.f, -theFieldDimensions.centerCircleRadius));
  ccPoints.push_back(Vector2f(theFieldDimensions.centerCircleRadius, 0.f));
  ccPoints.push_back(Vector2f(-theFieldDimensions.centerCircleRadius, 0.f));
  // Half of the intersections
  LinePercept::Intersection oppLeftCorner;
  oppLeftCorner.type = LinePercept::Intersection::L;
  oppLeftCorner.pos = Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftSideline);
  oppLeftCorner.dir1 = Vector2f(-1.f, 0.f);
  oppLeftCorner.dir2 = Vector2f(0.f, -1.f);
  intersections.push_back(oppLeftCorner);
  LinePercept::Intersection oppLeftPenaltyArea;
  oppLeftPenaltyArea.type = LinePercept::Intersection::T;
  oppLeftPenaltyArea.pos = Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftPenaltyArea);
  oppLeftPenaltyArea.dir1 = Vector2f(-1.f, 0.f);
  oppLeftPenaltyArea.dir2 = Vector2f(0.f, -1.f);
  intersections.push_back(oppLeftPenaltyArea);
  LinePercept::Intersection oppRightPenaltyArea;
  oppRightPenaltyArea.type = LinePercept::Intersection::T;
  oppRightPenaltyArea.pos = Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightPenaltyArea);
  oppRightPenaltyArea.dir1 = Vector2f(-1.f, 0.f);
  oppRightPenaltyArea.dir2 = Vector2f(0.f, -1.f);
  intersections.push_back(oppRightPenaltyArea);
  LinePercept::Intersection oppRightCorner;
  oppRightCorner.type = LinePercept::Intersection::L;
  oppRightCorner.pos = Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightSideline);
  oppRightCorner.dir1 = Vector2f(-1.f, 0.f);
  oppRightCorner.dir2 = Vector2f(0.f, 1.f);
  intersections.push_back(oppRightCorner);
  LinePercept::Intersection oppLeftPenaltyCorner;
  oppLeftPenaltyCorner.type = LinePercept::Intersection::L;
  oppLeftPenaltyCorner.pos = Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea);
  oppLeftPenaltyCorner.dir1 = Vector2f(1.f, 0.f);
  oppLeftPenaltyCorner.dir2 = Vector2f(0.f, -1.f);
  intersections.push_back(oppLeftPenaltyCorner);
  LinePercept::Intersection oppRightPenaltyCorner;
  oppRightPenaltyCorner.type = LinePercept::Intersection::L;
  oppRightPenaltyCorner.pos = Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosRightPenaltyArea);
  oppRightPenaltyCorner.dir1 = Vector2f(1.f, 0.f);
  oppRightPenaltyCorner.dir2 = Vector2f(0.f, 1.f);
  intersections.push_back(oppRightPenaltyCorner);
  LinePercept::Intersection leftCenterLineCrossing;
  leftCenterLineCrossing.type = LinePercept::Intersection::T;
  leftCenterLineCrossing.pos = Vector2f(0.f, theFieldDimensions.yPosLeftSideline);
  leftCenterLineCrossing.dir1 = Vector2f(1.f, 0.f);
  leftCenterLineCrossing.dir2 = Vector2f(0.f, -1.f);
  intersections.push_back(leftCenterLineCrossing);
  LinePercept::Intersection leftCenterCircleCrossing;
  leftCenterCircleCrossing.type = LinePercept::Intersection::X;
  leftCenterCircleCrossing.pos = Vector2f(0.f, theFieldDimensions.centerCircleRadius);
  leftCenterCircleCrossing.dir1 = Vector2f(1.f, 0.f);
  leftCenterCircleCrossing.dir2 = Vector2f(0.f, -1.f);
  intersections.push_back(leftCenterCircleCrossing);
  // The other half of the intersections is mirrored to the first half:
  const size_t numOfIntersections = intersections.size();
  for(unsigned int i = 0; i < numOfIntersections; i++)
  {
    LinePercept::Intersection mirroredIntersection = intersections[i];
    mirroredIntersection.pos = Pose2f(pi) * mirroredIntersection.pos;
    mirroredIntersection.dir1 = Pose2f(pi) * mirroredIntersection.dir1;
    mirroredIntersection.dir2 = Pose2f(pi) * mirroredIntersection.dir2;
    intersections.push_back(mirroredIntersection);
  }
  // The lines
  std::pair<Vector2f, Vector2f> line;
  // ground lines and center line:
  line.first =  Vector2f(0, theFieldDimensions.yPosLeftSideline);
  line.second = Vector2f(0, theFieldDimensions.yPosRightSideline);
  lines.push_back(line);
  line.first =  Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftSideline);
  line.second = Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightSideline);
  lines.push_back(line);
  line.first =  Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftSideline);
  line.second = Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightSideline);
  lines.push_back(line);
  // side lines:
  line.first =  Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftSideline);
  line.second = Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftSideline);
  lines.push_back(line);
  line.first =  Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightSideline);
  line.second = Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightSideline);
  lines.push_back(line);
  // opponent penalty area:
  line.first =  Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftPenaltyArea);
  line.second = Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea);
  lines.push_back(line);
  line.first =  Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightPenaltyArea);
  line.second = Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosRightPenaltyArea);
  lines.push_back(line);
  line.first =  Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea);
  line.second = Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosRightPenaltyArea);
  lines.push_back(line);
  // own penalty area:
  line.first =  Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftPenaltyArea);
  line.second = Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea);
  lines.push_back(line);
  line.first =  Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightPenaltyArea);
  line.second = Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosRightPenaltyArea);
  lines.push_back(line);
  line.first =  Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea);
  line.second = Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosRightPenaltyArea);
  lines.push_back(line);
  // The field boundary
  line.first = Vector2f(theFieldDimensions.xPosOwnFieldBorder, theFieldDimensions.yPosLeftFieldBorder);
  line.second =  Vector2f(theFieldDimensions.xPosOpponentFieldBorder, theFieldDimensions.yPosLeftFieldBorder);
  fieldBoundaryLines.push_back(line);
  line.first =  Vector2f(theFieldDimensions.xPosOpponentFieldBorder, theFieldDimensions.yPosLeftFieldBorder);
  line.second = Vector2f(theFieldDimensions.xPosOpponentFieldBorder, theFieldDimensions.yPosRightFieldBorder);
  fieldBoundaryLines.push_back(line);
  line.first =  Vector2f(theFieldDimensions.xPosOpponentFieldBorder, theFieldDimensions.yPosRightFieldBorder);
  line.second = Vector2f(theFieldDimensions.xPosOwnFieldBorder, theFieldDimensions.yPosRightFieldBorder);
  fieldBoundaryLines.push_back(line);
  line.first = Vector2f(theFieldDimensions.xPosOwnFieldBorder, theFieldDimensions.yPosRightFieldBorder);
  line.second =  Vector2f(theFieldDimensions.xPosOwnFieldBorder, theFieldDimensions.yPosLeftFieldBorder);
  fieldBoundaryLines.push_back(line);
}

void OracledPerceptsProvider::update(BallPercept& ballPercept)
{
  ballPercept.status = BallPercept::notSeen;
  if(!theCameraMatrix.isValid)
    return;
  if(theGroundTruthWorldState.balls.size() != 0)
  {
    const Vector2f ballOnField = theGroundTruthWorldState.balls[0];
    Vector2f ballOffset = theGroundTruthWorldState.ownPose.inverse() * ballOnField;
    if(ballOffset.norm() > ballMaxVisibleDistance)
      return;
    if(randomFloat() > ballRecognitionRate)
      return;
    Geometry::Circle circle;
    if(Geometry::calculateBallInImage(ballOffset, theCameraMatrix, theCameraInfo, theFieldDimensions.ballRadius, circle))
    {
      if((circle.center.x() >= -circle.radius / 1.5f) &&
         (circle.center.x() < theCameraInfo.width + circle.radius / 1.5f) &&
         (circle.center.y() >= -circle.radius / 1.5f) &&
         (circle.center.y() < theCameraInfo.height + circle.radius / 1.5f))
      {
        ballPercept.status = BallPercept::seen;
        ballPercept.positionInImage = circle.center;
        ballPercept.radiusInImage = circle.radius;
        ballPercept.relativePositionOnField = ballOffset;
        // Add some noise
        if(applyBallNoise)
        {
          applyNoise(ballCenterInImageStdDev, ballPercept.positionInImage);
          if(!Transformation::imageToRobotHorizontalPlane(ballPercept.positionInImage, theFieldDimensions.ballRadius, theCameraMatrix,theCameraInfo,ballPercept.relativePositionOnField))
          {
            ballPercept.status = BallPercept::notSeen;
            return;
          }
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
  const Pose2f robotPoseInv = theGroundTruthWorldState.ownPose.inverse();
  for(unsigned int i = 0; i < goalPosts.size(); i++)
  {
    const Vector2f relativePostPos = robotPoseInv * goalPosts[i];
    if(relativePostPos.norm() > goalPostMaxVisibleDistance)
      continue;
    if(randomFloat() > goalPostRecognitionRate)
      continue;
    Vector2f postInImage;
    if(pointIsInImage(relativePostPos, postInImage))
    {
      GoalPost newPost; // GoalPost::IS_UNKNOWN is the default side information
      newPost.positionInImage.x() = static_cast<int>(std::floor(postInImage.x() + 0.5f));
      newPost.positionInImage.y() = static_cast<int>(std::floor(postInImage.y() + 0.5f));
      newPost.positionOnField = relativePostPos;
      // Add some noise:
      if(applyGoalPostNoise)
      {
        applyNoise(ballCenterInImageStdDev, newPost.positionInImage);
        if(!Transformation::imageToRobot(newPost.positionInImage.x(), newPost.positionInImage.y(), theCameraMatrix, theCameraInfo, newPost.positionOnField))
        {
          continue;
        }
      }
      // If a part of the goal bar might be in the image, there is also a side information:
      const Vector3f relativeBarPos(relativePostPos.x(), relativePostPos.y(), theFieldDimensions.goalHeight);
      Vector2f barInImage;
      if(Transformation::robotToImage(relativeBarPos, theCameraMatrix, theCameraInfo, barInImage))
      {
        if((barInImage.x() >= 0.f) && (barInImage.x() < static_cast<float>(theCameraInfo.width)) &&
           (barInImage.y() >= 0.f) && (barInImage.y() < static_cast<float>(theCameraInfo.height)))
        {
          if((goalPosts[i].x() > 0 && goalPosts[i].y() > 0) || (goalPosts[i].x() < 0 && goalPosts[i].y() < 0))
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
    float angleToFirst = std::atan2(goalPercept.goalPosts[0].positionOnField.y(), goalPercept.goalPosts[0].positionOnField.x());
    float angleToSecond = std::atan2(goalPercept.goalPosts[1].positionOnField.y(), goalPercept.goalPosts[1].positionOnField.x());
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
  const Pose2f robotPoseInv = theGroundTruthWorldState.ownPose.inverse();

  // Find center circle (at least one out of five center circle points must be inside the current image)
  bool pointFound = false;
  if((theGroundTruthWorldState.ownPose.translation.norm() <= centerCircleMaxVisibleDistance) &&
     (randomFloat() < centerCircleRecognitionRate))
  {
    for(unsigned int i = 0; i < ccPoints.size(); ++i)
    {
      const Vector2f relPos = robotPoseInv * ccPoints[i];
      Vector2f posInImage;
      if(pointIsInImage(relPos, posInImage))
      {
        pointFound = true;
        break;
      }
    }
  }
  if(pointFound)
  {
    linePercept.circle.pos = robotPoseInv * Vector2f::Zero();
    // Add some noise:
    linePercept.circle.found = true;
    if(applyCenterCircleNoise)
    {
      Vector2f nPImg;
      if(Transformation::robotToImage(linePercept.circle.pos, theCameraMatrix, theCameraInfo, nPImg))
      {
        applyNoise(centerCircleCenterInImageStdDev, nPImg);
        if(!Transformation::imageToRobot(nPImg, theCameraMatrix, theCameraInfo, linePercept.circle.pos))
        {
          linePercept.circle.found = false;
        }
      }
    }
    if(linePercept.circle.found)
      linePercept.circle.lastSeen = theFrameInfo.time;
  }

  // Find intersections:
  for(unsigned int i = 0; i < intersections.size(); i++)
  {
    const Vector2f relativeIntersectionPos = robotPoseInv * intersections[i].pos;
    if(relativeIntersectionPos.norm() > intersectionMaxVisibleDistance)
      continue;
    if(randomFloat() > intersectionRecognitionRate)
      continue;
    Vector2f intersectionInImage;
    if(pointIsInImage(relativeIntersectionPos, intersectionInImage))
    {
      LinePercept::Intersection newIntersection;
      newIntersection.pos = relativeIntersectionPos;
      newIntersection.type = intersections[i].type;
      newIntersection.dir1 = (Pose2f(robotPoseInv.rotation) * intersections[i].dir1);
      newIntersection.dir2 = (Pose2f(robotPoseInv.rotation) * intersections[i].dir2);
      bool success = true;
      if(applyIntersectionNoise)
      {
        applyNoise(intersectionPosInImageStdDev, intersectionInImage);
        success = Transformation::imageToRobot(intersectionInImage, theCameraMatrix, theCameraInfo, newIntersection.pos);
        // noise on directions is not implemented, but if you need it, feel free to add it right here
      }
      if(success)
      {
        linePercept.intersections.push_back(newIntersection);
      }
    }
  }

  // Find lines:
  for(unsigned int i = 0; i < lines.size(); i++)
  {
    if(randomFloat() > lineRecognitionRate)
      continue;
    Vector2f start, end;
    if(partOfLineIsVisible(lines[i], start, end))
    {
      LinePercept::Line line;
      line.first = robotPoseInv * start;
      line.last = robotPoseInv * end;
      if(line.first.norm() > lineMaxVisibleDistance || line.last.norm() > lineMaxVisibleDistance)
        continue;
      line.midLine = (i == 0);
      Vector2f pImg;
      if(Transformation::robotToImage(line.first, theCameraMatrix, theCameraInfo, pImg))
      {
        line.startInImage = pImg;
        if(Transformation::robotToImage(line.last, theCameraMatrix, theCameraInfo, pImg))
        {
          line.endInImage = pImg;
          bool success = true;
          if(applyLineNoise)
          {
            applyNoise(linePosInImageStdDev, line.startInImage);
            applyNoise(linePosInImageStdDev, line.endInImage);
            success = Transformation::imageToRobot(line.startInImage.x(), line.startInImage.y(), theCameraMatrix, theCameraInfo, line.first) &&
              Transformation::imageToRobot(line.endInImage.x(), line.endInImage.y(), theCameraMatrix, theCameraInfo, line.last);
          }
          if(success)
          {
            line.alpha = (line.first - line.last).angle() + pi_2;
            while(line.alpha < 0)
              line.alpha += pi;
            while(line.alpha >= pi)
              line.alpha -= pi;
            const float c = std::cos(line.alpha),
              s = std::sin(line.alpha);
            line.d = line.first.x() * c + line.first.y() * s;
            linePercept.lines.push_back(line);
          }
        }
      }
    }
  }
}

void OracledPerceptsProvider::update(PenaltyMarkPercept& penaltyMarkPercept)
{
  if(!theCameraMatrix.isValid)
    return;
  const Pose2f robotPoseInv = theGroundTruthWorldState.ownPose.inverse();
  for(auto& pos : penaltyMarks)
  {
    Vector2f relativeMarkPos = robotPoseInv * pos;
    if(relativeMarkPos.norm() > penaltyMarkMaxVisibleDistance)
      continue;
    if(randomFloat() > penaltyMarkRecognitionRate)
      continue;
    Vector2f penaltyMarkInImage;
    if(pointIsInImage(relativeMarkPos, penaltyMarkInImage))
    {
      bool success = true;
      if(applyPenaltyMarkNoise)
      {
        applyNoise(penaltyMarkPosInImageStdDev, penaltyMarkInImage);
        success = Transformation::imageToRobot(penaltyMarkInImage, theCameraMatrix, theCameraInfo, relativeMarkPos);
      }
      if(success)
      {
        penaltyMarkPercept.positionOnField = relativeMarkPos;
        penaltyMarkPercept.position        = Vector2i(static_cast<int>(penaltyMarkInImage.x()), static_cast<int>(penaltyMarkInImage.y()));
        penaltyMarkPercept.timeLastSeen    = theFrameInfo.time;
      }
    }
  }
}

void OracledPerceptsProvider::update(PlayersPercept& playersPercept)
{
  playersPercept.players.clear();
  if(!theCameraMatrix.isValid || !Global::settingsExist())
    return;

  // Simulation scene should only use blue and red for now
  ASSERT(Global::getSettings().teamColor == Settings::blue || Global::getSettings().teamColor == Settings::red);
  
  const bool isBlue = Global::getSettings().teamColor == Settings::blue;
  for(unsigned int i = 0; i < theGroundTruthWorldState.bluePlayers.size(); ++i)
    createPlayerBox(theGroundTruthWorldState.bluePlayers[i], !isBlue, playersPercept);
  for(unsigned int i = 0; i < theGroundTruthWorldState.redPlayers.size(); ++i)
    createPlayerBox(theGroundTruthWorldState.redPlayers[i], isBlue, playersPercept);
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
    fieldBoundary.convexBoundary.push_back(Vector2i(0, theCameraInfo.height));
    fieldBoundary.convexBoundary.push_back(Vector2i(theCameraInfo.width - 1, theCameraInfo.height));
    fieldBoundary.boundaryInImage = fieldBoundary.convexBoundary;
    fieldBoundary.boundaryOnField.push_back(Vector2f(0, 1));
    fieldBoundary.boundaryOnField.push_back(Vector2f(0, -1));
    fieldBoundary.isValid = false;
    return;
  }
  updateViewPolygon();
  const Pose2f robotPoseInv = theGroundTruthWorldState.ownPose.inverse();

  // Find boundary lines:
  for(unsigned int i = 0; i < fieldBoundaryLines.size(); i++)
  {
    Vector2f start, end;
    if(partOfLineIsVisible(fieldBoundaryLines[i], start, end))
    {
      Vector2f pRobotStart = robotPoseInv * start;
      Vector2f pRobotEnd = robotPoseInv * end;
      fieldBoundary.boundaryOnField.push_back(pRobotStart);
      fieldBoundary.boundaryOnField.push_back(pRobotEnd);
      Vector2f pImgStart, pImgEnd;
      VERIFY(Transformation::robotToImage(pRobotStart, theCameraMatrix, theCameraInfo, pImgStart));
      VERIFY(Transformation::robotToImage(pRobotEnd, theCameraMatrix, theCameraInfo, pImgEnd));
      fieldBoundary.boundaryInImage.push_back(pImgStart.cast<int>());
      fieldBoundary.boundaryInImage.push_back(pImgEnd.cast<int>());
    }
  }
  fieldBoundary.isValid = fieldBoundary.boundaryOnField.size() != 0;
  if(fieldBoundary.boundaryOnField.size() < 2)
  {
    fieldBoundary.convexBoundary.push_back(Vector2i(0, theCameraInfo.height));
    fieldBoundary.convexBoundary.push_back(Vector2i(theCameraInfo.width - 1, theCameraInfo.height));
    fieldBoundary.boundaryInImage = fieldBoundary.convexBoundary;
    fieldBoundary.boundaryOnField.push_back(Vector2f(0, 1));
    fieldBoundary.boundaryOnField.push_back(Vector2f(0, -1));
    fieldBoundary.isValid = false;
  }
}

void OracledPerceptsProvider::createPlayerBox(const GroundTruthWorldState::GroundTruthPlayer& player, bool isOpponent, PlayersPercept& playersPercept)
{
  const Pose2f robotPoseInv = theGroundTruthWorldState.ownPose.inverse();
  Vector2f relativePlayerPos = robotPoseInv * player.pose.translation;
  if(relativePlayerPos.norm() > playerMaxVisibleDistance)
    return;
  if(randomFloat() > playerRecognitionRate)
    return;
  Vector2f playerInImage;
  if(pointIsInImage(relativePlayerPos, playerInImage))
  {
    bool success = true;
    if(applyPlayerNoise)
    {
      applyNoise(playerPosInImageStdDev, playerInImage);
      success = Transformation::imageToRobot(playerInImage, theCameraMatrix, theCameraInfo, relativePlayerPos);
    }
    if(success)
    {
      PlayersPercept::Player p;
      p.x1 = p.x2 = p.x1FeetOnly = p.x2FeetOnly = p.realCenterX = static_cast<int>(std::floor(playerInImage.x() + 0.5f));
      p.y1 = p.y2 = static_cast<int>(std::floor(playerInImage.y() + 0.5f));
      p.lowerCamera = theCameraInfo.camera == CameraInfo::lower;
      p.detectedJersey = true;
      p.detectedFeet   = true;
      p.ownTeam = !isOpponent;
      p.fallen = !player.upright;
      p.rightOnField = p.centerOnField = p.leftOnField = relativePlayerPos;
      p.rightOnField.normalize(Obstacle::getRobotDepth());
      p.leftOnField.normalize(Obstacle::getRobotDepth());
      p.rightOnField.rotateRight();
      p.leftOnField.rotateLeft();
      p.rightOnField += p.centerOnField;
      p.leftOnField += p.centerOnField;

      playersPercept.players.push_back(p);
    }
  }
}

bool OracledPerceptsProvider::pointIsInImage(const Vector2f& p, Vector2f& pImg) const
{
  if(Transformation::robotToImage(p, theCameraMatrix, theCameraInfo, pImg))
  {
    if((pImg.x() >= 0) && (pImg.x() < theCameraInfo.width) && (pImg.y() >= 0) && (pImg.y() < theCameraInfo.height))
    {
      return true;
    }
  }
  return false;
}

void OracledPerceptsProvider::updateViewPolygon()
{
  // code is copied from FieldCoverageProvider::drawFieldView()
  const Vector3f vectorToCenter(1, 0, 0);

  RotationMatrix r = theCameraMatrix.rotation;
  r.rotateY(theCameraInfo.openingAngleHeight / 2);
  r.rotateZ(theCameraInfo.openingAngleWidth / 2);
  Vector3f vectorToCenterWorld = r * vectorToCenter;

  const float a1 = theCameraMatrix.translation.x(),
              a2 = theCameraMatrix.translation.y(),
              a3 = theCameraMatrix.translation.z();
  float b1 = vectorToCenterWorld.x(),
        b2 = vectorToCenterWorld.y(),
        b3 = vectorToCenterWorld.z(),
        f = a3 / b3;
  Vector2f pof = Vector2f(a1 - f * b1, a2 - f * b2);

  if(f > 0.f)
    viewPolygon[0] = theGroundTruthWorldState.ownPose.translation;
  else
    viewPolygon[0] = (theGroundTruthWorldState.ownPose + pof).translation;

  r = theCameraMatrix.rotation;
  r.rotateY(theCameraInfo.openingAngleHeight / 2);
  r.rotateZ(-(theCameraInfo.openingAngleWidth / 2));
  vectorToCenterWorld = r * vectorToCenter;

  b1 = vectorToCenterWorld.x();
  b2 = vectorToCenterWorld.y();
  b3 = vectorToCenterWorld.z();
  f = a3 / b3;
  pof = Vector2f(a1 - f * b1, a2 - f * b2);

  if(f > 0.f)
    viewPolygon[1] = theGroundTruthWorldState.ownPose.translation;
  else
    viewPolygon[1] = (theGroundTruthWorldState.ownPose + pof).translation;

  r = theCameraMatrix.rotation;
  r.rotateY(-(theCameraInfo.openingAngleHeight / 2));
  r.rotateZ(-(theCameraInfo.openingAngleWidth / 2));
  vectorToCenterWorld = r * vectorToCenter;

  b1 = vectorToCenterWorld.x();
  b2 = vectorToCenterWorld.y();
  b3 = vectorToCenterWorld.z();
  f = a3 / b3;
  pof = Vector2f(a1 - f * b1, a2 - f * b2);

  const float maxDist = std::sqrt(4.f * theFieldDimensions.xPosOpponentFieldBorder * theFieldDimensions.xPosOpponentFieldBorder +
                                  4.f * theFieldDimensions.yPosLeftFieldBorder * theFieldDimensions.yPosLeftFieldBorder);
  if(f > 0.f)
    viewPolygon[2] = theGroundTruthWorldState.ownPose.translation + Vector2f(maxDist, 0).rotate(theGroundTruthWorldState.ownPose.rotation + (-theCameraInfo.openingAngleWidth / 2) + theCameraMatrix.rotation.getZAngle());
  else
    viewPolygon[2] = (theGroundTruthWorldState.ownPose + pof).translation;

  r = theCameraMatrix.rotation;
  r.rotateY(-(theCameraInfo.openingAngleHeight / 2));
  r.rotateZ(theCameraInfo.openingAngleWidth / 2);
  vectorToCenterWorld = r * vectorToCenter;

  b1 = vectorToCenterWorld.x();
  b2 = vectorToCenterWorld.y();
  b3 = vectorToCenterWorld.z();
  f = a3 / b3;
  pof = Vector2f(a1 - f * b1, a2 - f * b2);

  if(f > 0.f)
    viewPolygon[3] = theGroundTruthWorldState.ownPose.translation + Vector2f(maxDist, 0).rotate(theGroundTruthWorldState.ownPose.rotation + (theCameraInfo.openingAngleWidth / 2) + theCameraMatrix.rotation.getZAngle());
  else
    viewPolygon[3] = (theGroundTruthWorldState.ownPose + pof).translation;
}

bool OracledPerceptsProvider::partOfLineIsVisible(const std::pair<Vector2f, Vector2f>& line, Vector2f& start, Vector2f& end) const
{
  // First case: both points are inside:
  if(Geometry::isPointInsideConvexPolygon(viewPolygon, 4, line.first) &&
     Geometry::isPointInsideConvexPolygon(viewPolygon, 4, line.second))
  {
    start = line.first;
    end = line.second;
    return true;
  }
  // Second case: start is inside but end is outside
  if(Geometry::isPointInsideConvexPolygon(viewPolygon, 4, line.first) &&
     !Geometry::isPointInsideConvexPolygon(viewPolygon, 4, line.second))
  {
    start = line.first;
    for(int i = 0; i < 4; i++)
    {
      if(Geometry::checkIntersectionOfLines(line.first, line.second, viewPolygon[i], viewPolygon[(i + 1) % 4]))
      {
        if(Geometry::getIntersectionOfLines(Geometry::Line(line.first, line.second - line.first),
                                            Geometry::Line(viewPolygon[i], viewPolygon[(i + 1) % 4] - viewPolygon[i]), end))
        {
          return true;
        }
      }
    }
    return false; // should not happen ...
  }
  // Third case: end is inside but start is outside
  if(!Geometry::isPointInsideConvexPolygon(viewPolygon, 4, line.first) &&
     Geometry::isPointInsideConvexPolygon(viewPolygon, 4, line.second))
  {
    start = line.second;
    for(int i = 0; i < 4; i++)
    {
      if(Geometry::checkIntersectionOfLines(line.first, line.second, viewPolygon[i], viewPolygon[(i + 1) % 4]))
      {
        if(Geometry::getIntersectionOfLines(Geometry::Line(line.first, line.second - line.first),
                                            Geometry::Line(viewPolygon[i], viewPolygon[(i + 1) % 4] - viewPolygon[i]), end))
        {
          return true;
        }
      }
    }
    return false; // should not happen ...
  }
  // Fourth case: both points are outside the polygon but maybe intersect it:
  std::vector<Vector2f> intersectionPoints;
  for(int i = 0; i < 4; i++)
  {
    if(Geometry::checkIntersectionOfLines(line.first, line.second, viewPolygon[i], viewPolygon[(i + 1) % 4]))
    {
      Vector2f point;
      if(Geometry::getIntersectionOfLines(Geometry::Line(line.first, line.second - line.first),
                                          Geometry::Line(viewPolygon[i], viewPolygon[(i + 1) % 4] - viewPolygon[i]), point))
      {
        intersectionPoints.push_back(point);
      }
    }
  }
  // There are some more special cases that could be treated but in general, there should be two intersections that are not at the same place.
  // Other cases are ignored here
  if(intersectionPoints.size() == 2 && (intersectionPoints[0] - intersectionPoints[1]).norm() > 100.f)
  {
    start = intersectionPoints[0];
    end   = intersectionPoints[1];
    return true;
  }
  // Sorry, nothing found ...
  return false;
}

void OracledPerceptsProvider::applyNoise(float standardDeviation, Vector2f& p) const
{
  p.x() += sampleNormalDistribution(standardDeviation);
  p.y() += sampleNormalDistribution(standardDeviation);
}

void OracledPerceptsProvider::applyNoise(float standardDeviation, Vector2i& p) const
{
  const float errorX = sampleNormalDistribution(standardDeviation);
  const float errorY = sampleNormalDistribution(standardDeviation);
  p.x() += static_cast<int>(floor(errorX + 0.5f));
  p.y() += static_cast<int>(floor(errorY + 0.5f));
}

MAKE_MODULE(OracledPerceptsProvider, cognitionInfrastructure)
