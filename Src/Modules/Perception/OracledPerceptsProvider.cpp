/**
 * @file Modules/Infrastructure/OracledPerceptsProvider.cpp
 *
 * This file implements a module that provides percepts based on simulated data.
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#include "OracledPerceptsProvider.h"
#include "Streaming/Global.h"
#include "Framework/Settings.h"
#include "Math/Geometry.h"
#include "Math/Probabilistics.h"
#include "Tools/Math/Projection.h"
#include "Math/RotationMatrix.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Modeling/Obstacle.h"

MAKE_MODULE(OracledPerceptsProvider);

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
  IntersectionsPercept::Intersection oppLeftCorner;
  oppLeftCorner.type = IntersectionsPercept::Intersection::L;
  oppLeftCorner.pos = Vector2f(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosLeftTouchline);
  oppLeftCorner.dir1 = Vector2f(-1.f, 0.f);
  oppLeftCorner.dir2 = Vector2f(0.f, -1.f);
  intersections.push_back(oppLeftCorner);
  IntersectionsPercept::Intersection oppLeftPenaltyArea;
  oppLeftPenaltyArea.type = IntersectionsPercept::Intersection::T;
  oppLeftPenaltyArea.pos = Vector2f(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosLeftPenaltyArea);
  oppLeftPenaltyArea.dir1 = Vector2f(-1.f, 0.f);
  oppLeftPenaltyArea.dir2 = Vector2f(0.f, -1.f);
  intersections.push_back(oppLeftPenaltyArea);
  IntersectionsPercept::Intersection oppRightPenaltyArea;
  oppRightPenaltyArea.type = IntersectionsPercept::Intersection::T;
  oppRightPenaltyArea.pos = Vector2f(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosRightPenaltyArea);
  oppRightPenaltyArea.dir1 = Vector2f(-1.f, 0.f);
  oppRightPenaltyArea.dir2 = Vector2f(0.f, -1.f);
  intersections.push_back(oppRightPenaltyArea);
  IntersectionsPercept::Intersection oppRightCorner;
  oppRightCorner.type = IntersectionsPercept::Intersection::L;
  oppRightCorner.pos = Vector2f(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosRightTouchline);
  oppRightCorner.dir1 = Vector2f(-1.f, 0.f);
  oppRightCorner.dir2 = Vector2f(0.f, 1.f);
  intersections.push_back(oppRightCorner);
  IntersectionsPercept::Intersection oppLeftPenaltyCorner;
  oppLeftPenaltyCorner.type = IntersectionsPercept::Intersection::L;
  oppLeftPenaltyCorner.pos = Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea);
  oppLeftPenaltyCorner.dir1 = Vector2f(1.f, 0.f);
  oppLeftPenaltyCorner.dir2 = Vector2f(0.f, -1.f);
  intersections.push_back(oppLeftPenaltyCorner);
  IntersectionsPercept::Intersection oppRightPenaltyCorner;
  oppRightPenaltyCorner.type = IntersectionsPercept::Intersection::L;
  oppRightPenaltyCorner.pos = Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosRightPenaltyArea);
  oppRightPenaltyCorner.dir1 = Vector2f(1.f, 0.f);
  oppRightPenaltyCorner.dir2 = Vector2f(0.f, 1.f);
  intersections.push_back(oppRightPenaltyCorner);
  IntersectionsPercept::Intersection leftHalfwayLineCrossing;
  leftHalfwayLineCrossing.type = IntersectionsPercept::Intersection::T;
  leftHalfwayLineCrossing.pos = Vector2f(0.f, theFieldDimensions.yPosLeftTouchline);
  leftHalfwayLineCrossing.dir1 = Vector2f(1.f, 0.f);
  leftHalfwayLineCrossing.dir2 = Vector2f(0.f, -1.f);
  intersections.push_back(leftHalfwayLineCrossing);
  IntersectionsPercept::Intersection leftCenterCircleCrossing;
  leftCenterCircleCrossing.type = IntersectionsPercept::Intersection::X;
  leftCenterCircleCrossing.pos = Vector2f(0.f, theFieldDimensions.centerCircleRadius);
  leftCenterCircleCrossing.dir1 = Vector2f(1.f, 0.f);
  leftCenterCircleCrossing.dir2 = Vector2f(0.f, -1.f);
  intersections.push_back(leftCenterCircleCrossing);
  // The other half of the intersections is mirrored to the first half:
  const size_t numOfIntersections = intersections.size();
  for(unsigned int i = 0; i < numOfIntersections; i++)
  {
    IntersectionsPercept::Intersection mirroredIntersection = intersections[i];
    mirroredIntersection.pos = Pose2f(pi) * mirroredIntersection.pos;
    mirroredIntersection.dir1 = Pose2f(pi) * mirroredIntersection.dir1;
    mirroredIntersection.dir2 = Pose2f(pi) * mirroredIntersection.dir2;
    intersections.push_back(mirroredIntersection);
  }
  // The lines
  std::pair<Vector2f, Vector2f> line;
  // goal lines and halfway line:
  line.first =  Vector2f(0, theFieldDimensions.yPosLeftTouchline);
  line.second = Vector2f(0, theFieldDimensions.yPosRightTouchline);
  lines.push_back(line);
  line.first =  Vector2f(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosLeftTouchline);
  line.second = Vector2f(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosRightTouchline);
  lines.push_back(line);
  line.first =  Vector2f(theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.yPosLeftTouchline);
  line.second = Vector2f(theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.yPosRightTouchline);
  lines.push_back(line);
  // touchlines:
  line.first =  Vector2f(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosLeftTouchline);
  line.second = Vector2f(theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.yPosLeftTouchline);
  lines.push_back(line);
  line.first =  Vector2f(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosRightTouchline);
  line.second = Vector2f(theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.yPosRightTouchline);
  lines.push_back(line);
  // opponent penalty area:
  line.first =  Vector2f(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosLeftPenaltyArea);
  line.second = Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea);
  lines.push_back(line);
  line.first =  Vector2f(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosRightPenaltyArea);
  line.second = Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosRightPenaltyArea);
  lines.push_back(line);
  line.first =  Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea);
  line.second = Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosRightPenaltyArea);
  lines.push_back(line);
  // opponent goal area:
  line.first = Vector2f(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosLeftGoalArea);
  line.second = Vector2f(theFieldDimensions.xPosOpponentGoalArea, theFieldDimensions.yPosLeftGoalArea);
  lines.push_back(line);
  line.first = Vector2f(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosRightGoalArea);
  line.second = Vector2f(theFieldDimensions.xPosOpponentGoalArea, theFieldDimensions.yPosRightGoalArea);
  lines.push_back(line);
  line.first = Vector2f(theFieldDimensions.xPosOpponentGoalArea, theFieldDimensions.yPosLeftGoalArea);
  line.second = Vector2f(theFieldDimensions.xPosOpponentGoalArea, theFieldDimensions.yPosRightGoalArea);
  lines.push_back(line);
  // own penalty area:
  line.first =  Vector2f(theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.yPosLeftPenaltyArea);
  line.second = Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea);
  lines.push_back(line);
  line.first =  Vector2f(theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.yPosRightPenaltyArea);
  line.second = Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosRightPenaltyArea);
  lines.push_back(line);
  line.first =  Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea);
  line.second = Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosRightPenaltyArea);
  lines.push_back(line);
  // own goal area:
  line.first = Vector2f(theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.yPosLeftGoalArea);
  line.second = Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosLeftGoalArea);
  lines.push_back(line);
  line.first = Vector2f(theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.yPosRightGoalArea);
  line.second = Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosRightGoalArea);
  lines.push_back(line);
  line.first = Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosLeftGoalArea);
  line.second = Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosRightGoalArea);
  lines.push_back(line);
  // The field boundary
  line.first =  Vector2f(theFieldDimensions.xPosOwnFieldBorder, theFieldDimensions.yPosLeftFieldBorder);
  line.second = Vector2f(theFieldDimensions.xPosOpponentFieldBorder, theFieldDimensions.yPosLeftFieldBorder);
  fieldBoundaryLines.push_back(line);
  line.first =  Vector2f(theFieldDimensions.xPosOpponentFieldBorder, theFieldDimensions.yPosLeftFieldBorder);
  line.second = Vector2f(theFieldDimensions.xPosOpponentFieldBorder, theFieldDimensions.yPosRightFieldBorder);
  fieldBoundaryLines.push_back(line);
  line.first =  Vector2f(theFieldDimensions.xPosOpponentFieldBorder, theFieldDimensions.yPosRightFieldBorder);
  line.second = Vector2f(theFieldDimensions.xPosOwnFieldBorder, theFieldDimensions.yPosRightFieldBorder);
  fieldBoundaryLines.push_back(line);
  line.first =  Vector2f(theFieldDimensions.xPosOwnFieldBorder, theFieldDimensions.yPosRightFieldBorder);
  line.second = Vector2f(theFieldDimensions.xPosOwnFieldBorder, theFieldDimensions.yPosLeftFieldBorder);
  fieldBoundaryLines.push_back(line);
}

void OracledPerceptsProvider::update(BallPercept& ballPercept)
{
  ballPercept.radiusOnField = theBallSpecification.radius;
  ballPercept.status = BallPercept::notSeen;
  if(!theCameraMatrix.isValid || theGroundTruthWorldState.balls.size() == 0)
    return;

  if(Random::bernoulli(ballFalsePositiveRate))
    falseBallPercept(ballPercept);
  else
    trueBallPercept(ballPercept);
}

void OracledPerceptsProvider::trueBallPercept(BallPercept& ballPercept)
{
  const Vector2f ballOnField = theGroundTruthWorldState.balls[0].position.head<2>();
  Vector2f ballOffset = theGroundTruthWorldState.ownPose.inverse() * ballOnField;
  if(ballOffset.norm() > ballMaxVisibleDistance || isPointBehindObstacle(ballOnField))
    return;
  if(Random::bernoulli(1. - ballRecognitionRate))
    return;
  Geometry::Circle circle;
  if(Projection::calculateBallInImage(ballOffset, theCameraMatrix, theCameraInfo, theBallSpecification.radius, circle))
  {
    if((circle.center.x() >= -circle.radius / 1.5f) &&
       (circle.center.x() < theCameraInfo.width + circle.radius / 1.5f) &&
       (circle.center.y() >= -circle.radius / 1.5f) &&
       (circle.center.y() < theCameraInfo.height + circle.radius / 1.5f))
    {
      ballPercept.status = BallPercept::seen;
      ballPercept.positionInImage = circle.center;
      ballPercept.radiusInImage = circle.radius;
      ballPercept.positionOnField = ballOffset;
      // Add some noise
      if(applyBallNoise)
      {
        applyNoise(ballCenterInImageStdDev, ballPercept.positionInImage);
        if(!theMeasurementCovariance.transformWithCovLegacy(ballPercept.positionInImage, theBallSpecification.radius, Vector2f(0.04f, 0.06f),
                                                            ballPercept.positionOnField, ballPercept.covarianceOnField))
        {
          ballPercept.status = BallPercept::notSeen;
          return;
        }
      }
    }
  }
}

void OracledPerceptsProvider::falseBallPercept(BallPercept& ballPercept)
{
  std::vector<BallPercept> possiblePercepts;
  const Pose2f robotPoseInv = theGroundTruthWorldState.ownPose.inverse();

  auto addPercept = [&](const Vector2f& positionOnField)
  {
    if(isPointBehindObstacle(positionOnField))
      return;
    const Vector2f relativePosition = robotPoseInv * positionOnField;
    Geometry::Circle circle;
    if(Projection::calculateBallInImage(relativePosition, theCameraMatrix, theCameraInfo, theBallSpecification.radius, circle))
    {
      if((circle.center.x() >= -circle.radius / 1.5f) &&
         (circle.center.x() < theCameraInfo.width + circle.radius / 1.5f) &&
         (circle.center.y() >= -circle.radius / 1.5f) &&
         (circle.center.y() < theCameraInfo.height + circle.radius / 1.5f))
      {
        BallPercept percept;
        percept.positionInImage = circle.center;
        percept.positionOnField = relativePosition;
        percept.radiusInImage = circle.radius;
        percept.status = BallPercept::seen;
        possiblePercepts.emplace_back(percept);
      }
    }
  };

  for(size_t i = 0; i < goalPosts.size(); i++)
    addPercept(goalPosts[i]);
  for(size_t i = 0; i < theGroundTruthWorldState.ownTeamPlayers.size(); ++i)
    addPercept(theGroundTruthWorldState.ownTeamPlayers[i].pose.translation);
  for(size_t i = 0; i < theGroundTruthWorldState.opponentTeamPlayers.size(); ++i)
    addPercept(theGroundTruthWorldState.opponentTeamPlayers[i].pose.translation);
  for(size_t i = 0; i < intersections.size(); ++i)
    addPercept(intersections[i].pos);
  for(size_t i = 0; i < penaltyMarks.size(); ++i)
    addPercept(penaltyMarks[i]);

  if(possiblePercepts.size())
  {
    auto percept = possiblePercepts[Random::uniformInt(possiblePercepts.size() - 1)];
    if(!theMeasurementCovariance.transformWithCovLegacy(percept.positionInImage, theBallSpecification.radius, Vector2f(0.04f, 0.06f),
                                                        percept.positionOnField, percept.covarianceOnField))
      ballPercept = percept;
  }
}

void OracledPerceptsProvider::update(LinesPercept& linesPercept)
{
  // Initialize percept and local data:
  linesPercept.lines.clear();

  if(!theCameraMatrix.isValid)
    return;
  updateViewPolygon();
  const Pose2f robotPoseInv = theGroundTruthWorldState.ownPose.inverse();

  // Find lines:
  for(unsigned int i = 0; i < lines.size(); i++)
  {
    if(Random::bernoulli(1. - lineRecognitionRate))
      continue;
    Vector2f start, end;
    if(partOfLineIsVisible(lines[i], start, end))
    {
      LinesPercept::Line line;
      line.firstField = robotPoseInv * start;
      line.lastField = robotPoseInv * end;
      if(line.firstField.norm() > lineMaxVisibleDistance || line.lastField.norm() > lineMaxVisibleDistance)
        continue;
      Vector2f pImg;
      if(Transformation::robotToImage(line.firstField, theCameraMatrix, theCameraInfo, pImg))
      {
        Vector2f startInImage = pImg;
        if(Transformation::robotToImage(line.lastField, theCameraMatrix, theCameraInfo, pImg))
        {
          Vector2f endInImage = pImg;
          bool success = true;
          if(applyLineNoise)
          {
            applyNoise(linePosInImageStdDev, startInImage);
            applyNoise(linePosInImageStdDev, endInImage);
            success = Transformation::imageToRobot(startInImage.x(), startInImage.y(), theCameraMatrix, theCameraInfo, line.firstField) &&
                      Transformation::imageToRobot(endInImage.x(), endInImage.y(), theCameraMatrix, theCameraInfo, line.lastField);
          }
          if(success)
          {
            line.firstImg = Vector2i(int(startInImage.x()), int(startInImage.y()));
            line.lastImg = Vector2i(int(endInImage.x()), int(endInImage.y()));
            line.spotsInImg.push_back(line.firstImg);
            line.spotsInImg.push_back(line.lastImg);
            line.spotsInField.push_back(line.firstField);
            line.spotsInField.push_back(line.lastField);
            line.line = Geometry::Line(line.firstField, line.lastField - line.firstField);
            linesPercept.lines.push_back(line);
          }
        }
      }
    }
  }
}

void OracledPerceptsProvider::update(CirclePercept& circlePercept)
{
  circlePercept.wasSeen = false;
  if(!theCameraMatrix.isValid)
    return;

  // Find center circle (at least one out of five center circle points must be inside the current image)
  const Pose2f robotPoseInv = theGroundTruthWorldState.ownPose.inverse();
  bool pointFound = false;
  if((theGroundTruthWorldState.ownPose.translation.norm() <= centerCircleMaxVisibleDistance) &&
     Random::bernoulli(centerCircleRecognitionRate))
  {
    for(unsigned int i = 0; i < ccPoints.size(); ++i)
    {
      const Vector2f relPos = robotPoseInv * ccPoints[i];
      Vector2f posInImage;
      if(Projection::pointIsInImage(theCameraMatrix, theCameraInfo, relPos, posInImage))
      {
        pointFound = true;
        break;
      }
    }
  }
  if(pointFound)
  {
    Vector2f circlePos(robotPoseInv * Vector2f::Zero());
    // Add some noise:
    if(applyCenterCircleNoise)
    {
      Vector2f nPImg;
      if(Transformation::robotToImage(circlePos, theCameraMatrix, theCameraInfo, nPImg))
      {
        applyNoise(centerCircleCenterInImageStdDev, nPImg);
        if(Transformation::imageToRobot(nPImg, theCameraMatrix, theCameraInfo, circlePos)) // TODO right?
        {
          circlePercept.pos = circlePos;
          circlePercept.wasSeen = true;
          const float distToCenter = circlePercept.pos.norm();
          const Vector2f virtualPosForCovariance(distToCenter > theFieldDimensions.centerCircleRadius ? distToCenter : theFieldDimensions.centerCircleRadius, 0.f);
          circlePercept.cov = theMeasurementCovariance.computeForRelativePosition(virtualPosForCovariance);
        }
      }
    }
  }
}

void OracledPerceptsProvider::update(PenaltyMarkPercept& penaltyMarkPercept)
{
  penaltyMarkPercept.wasSeen = false;
  if(!theCameraMatrix.isValid)
    return;
  const Pose2f robotPoseInv = theGroundTruthWorldState.ownPose.inverse();
  for(auto& pos : penaltyMarks)
  {
    Vector2f relativeMarkPos = robotPoseInv * pos;
    if(relativeMarkPos.norm() > penaltyMarkMaxVisibleDistance || isPointBehindObstacle(pos))
      continue;
    if(Random::bernoulli(1. - penaltyMarkRecognitionRate))
      continue;
    else if(Random::bernoulli(penaltyMarkFalsePositiveRate))
    {
      falsePenaltyMarkPercept(penaltyMarkPercept);
      continue;
    }

    Vector2f penaltyMarkInImage;
    if(Projection::pointIsInImage(theCameraMatrix, theCameraInfo, relativeMarkPos, penaltyMarkInImage))
    {
      if(applyPenaltyMarkNoise)
        applyNoise(penaltyMarkPosInImageStdDev, penaltyMarkInImage);

      if(theMeasurementCovariance.transformPointWithCov(penaltyMarkInImage, 0.f, penaltyMarkPercept.positionOnField, penaltyMarkPercept.covarianceOnField))
      {
        penaltyMarkPercept.positionInImage = Vector2i(static_cast<int>(penaltyMarkInImage.x()), static_cast<int>(penaltyMarkInImage.y()));
        penaltyMarkPercept.wasSeen = true;
      }
    }
  }
}

void OracledPerceptsProvider::falsePenaltyMarkPercept(PenaltyMarkPercept& penaltyMarkPercept)
{
  Vector2f falseMarkPos = theGroundTruthWorldState.ownPose.inverse() * penaltyMarks[0];
  falseMarkPos.y() *= Random::bernoulli(0.5) ? -penaltyMarkFalseDeviationFactor : penaltyMarkFalseDeviationFactor;
  if(Random::bernoulli(0.5))
    falseMarkPos.x() *= penaltyMarkFalseDeviationFactor;
  Vector2f penaltyMarkInImage;
  if(Projection::pointIsInImage(theCameraMatrix, theCameraInfo, falseMarkPos, penaltyMarkInImage)
     && theMeasurementCovariance.transformPointWithCov(penaltyMarkInImage, 0.f, penaltyMarkPercept.positionOnField, penaltyMarkPercept.covarianceOnField))
  {
    penaltyMarkPercept.positionInImage = penaltyMarkInImage.cast<int>();
    penaltyMarkPercept.wasSeen = true;
  }
}

void OracledPerceptsProvider::update(ObstaclesImagePercept& obstaclesImagePercept)
{
  obstaclesImagePercept.obstacles.clear();
  if(!theCameraMatrix.isValid || !Global::settingsExist())
    return;

  for(unsigned int i = 0; i < theGroundTruthWorldState.ownTeamPlayers.size(); ++i)
    if(!isPointBehindObstacle(theGroundTruthWorldState.ownTeamPlayers[i].pose.translation))
      createPlayerBox(theGroundTruthWorldState.ownTeamPlayers[i], obstaclesImagePercept);
  for(unsigned int i = 0; i < theGroundTruthWorldState.opponentTeamPlayers.size(); ++i)
    if(!isPointBehindObstacle(theGroundTruthWorldState.opponentTeamPlayers[i].pose.translation))
      createPlayerBox(theGroundTruthWorldState.opponentTeamPlayers[i], obstaclesImagePercept);
}

void OracledPerceptsProvider::update(ObstaclesFieldPercept& obstaclesFieldPercept)
{
  obstaclesFieldPercept.obstacles.clear();
  if(!theCameraMatrix.isValid || !Global::settingsExist())
    return;

  for(unsigned int i = 0; i < theGroundTruthWorldState.ownTeamPlayers.size(); ++i)
    if(!isPointBehindObstacle(theGroundTruthWorldState.ownTeamPlayers[i].pose.translation))
      createPlayerOnField(theGroundTruthWorldState.ownTeamPlayers[i], false, obstaclesFieldPercept);
  for(unsigned int i = 0; i < theGroundTruthWorldState.opponentTeamPlayers.size(); ++i)
    if(!isPointBehindObstacle(theGroundTruthWorldState.opponentTeamPlayers[i].pose.translation))
      createPlayerOnField(theGroundTruthWorldState.opponentTeamPlayers[i], true, obstaclesFieldPercept);
}

void OracledPerceptsProvider::update(FieldBoundary& fieldBoundary)
{
  // Initialize percept and local data:
  fieldBoundary.boundaryInImage.clear();
  fieldBoundary.boundaryOnField.clear();
  if(!theCameraMatrix.isValid)
  {
    fieldBoundary.boundaryInImage.push_back(Vector2i(0, theCameraInfo.height));
    fieldBoundary.boundaryInImage.push_back(Vector2i(theCameraInfo.width - 1, theCameraInfo.height));
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
    fieldBoundary.boundaryInImage.push_back(Vector2i(0, theCameraInfo.height));
    fieldBoundary.boundaryInImage.push_back(Vector2i(theCameraInfo.width - 1, theCameraInfo.height));
    fieldBoundary.boundaryOnField.push_back(Vector2f(0, 1));
    fieldBoundary.boundaryOnField.push_back(Vector2f(0, -1));
    fieldBoundary.isValid = false;
  }
}

void OracledPerceptsProvider::createPlayerBox(const GroundTruthWorldState::GroundTruthPlayer& player, ObstaclesImagePercept& obstaclesImagePercept)
{
  const Pose2f robotPoseInv = theGroundTruthWorldState.ownPose.inverse();
  Vector2f relativePlayerPos = robotPoseInv * player.pose.translation;
  if(relativePlayerPos.norm() > playerMaxVisibleDistance)
    return;
  if(Random::bernoulli(1. - playerRecognitionRate))
    return;
  Vector2f playerInImage;
  if(Projection::pointIsInImage(theCameraMatrix, theCameraInfo, relativePlayerPos, playerInImage))
  {
    bool success = true;
    if(applyPlayerNoise)
    {
      applyNoise(playerPosInImageStdDev, playerInImage);
      success = Transformation::imageToRobot(playerInImage, theCameraMatrix, theCameraInfo, relativePlayerPos);
    }
    if(success)
    {
      ObstaclesImagePercept::Obstacle o;
      o.left = o.right = static_cast<int>(std::floor(playerInImage.x() + 0.5f));
      o.top = o.bottom = static_cast<int>(std::floor(playerInImage.y() + 0.5f));
      o.bottomFound = true;
      o.fallen = !player.upright;
      obstaclesImagePercept.obstacles.push_back(o);
    }
  }
}

void OracledPerceptsProvider::createPlayerOnField(const GroundTruthWorldState::GroundTruthPlayer& player, bool isOpponent, ObstaclesFieldPercept& obstaclesFieldPercept)
{
  const Pose2f robotPoseInv = theGroundTruthWorldState.ownPose.inverse();
  Vector2f relativePlayerPos = robotPoseInv * player.pose.translation;
  if(relativePlayerPos.norm() > playerMaxVisibleDistance)
    return;
  if(Random::bernoulli(1. - playerRecognitionRate))
    return;
  if(Random::bernoulli(playerFalsePositiveRate))
  {
    Vector2f falsePlayerTranslation;
    falsePlayerTranslation.x() = Random::uniform(theFieldDimensions.xPosOwnFieldBorder, theFieldDimensions.xPosOpponentFieldBorder);
    falsePlayerTranslation.y() = Random::uniform(theFieldDimensions.yPosRightFieldBorder, theFieldDimensions.yPosLeftFieldBorder);
    relativePlayerPos = robotPoseInv * falsePlayerTranslation;
  }
  Vector2f playerInImage;
  if(Projection::pointIsInImage(theCameraMatrix, theCameraInfo, relativePlayerPos, playerInImage))
  {
    bool success = true;
    if(applyPlayerNoise)
    {
      applyNoise(playerPosInImageStdDev, playerInImage);
      success = Transformation::imageToRobot(playerInImage, theCameraMatrix, theCameraInfo, relativePlayerPos);
    }
    if(success)
    {
      ObstaclesFieldPercept::Obstacle o;
      if(player.number != 1)
        o.type = isOpponent ? ObstaclesFieldPercept::opponentPlayer : ObstaclesFieldPercept::ownPlayer;
      else
        o.type = isOpponent ? ObstaclesFieldPercept::opponentGoalkeeper : ObstaclesFieldPercept::ownGoalkeeper;
      o.fallen = !player.upright;
      o.right = o.center = o.left = relativePlayerPos;
      o.right.normalize(theRobotDimensions.robotDepth);
      o.left.normalize(theRobotDimensions.robotDepth);
      o.right.rotateRight();
      o.left.rotateLeft();
      o.right += o.center;
      o.left += o.center;

      Vector2f inImage;
      if(Transformation::robotToImage(o.center, theCameraMatrix, theCameraInfo, inImage)
         && theMeasurementCovariance.transformPointWithCov(inImage, 0.f, o.center, o.covariance))
        obstaclesFieldPercept.obstacles.push_back(o);
    }
  }
}

void OracledPerceptsProvider::updateViewPolygon()
{
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
  p.x() += Random::normal(standardDeviation);
  p.y() += Random::normal(standardDeviation);
}

void OracledPerceptsProvider::applyNoise(float standardDeviation, Vector2i& p) const
{
  const float errorX = Random::normal(standardDeviation);
  const float errorY = Random::normal(standardDeviation);
  p.x() += static_cast<int>(floor(errorX + 0.5f));
  p.y() += static_cast<int>(floor(errorY + 0.5f));
}

void OracledPerceptsProvider::applyNoise(float standardDeviation, float& angle) const
{
  const float error = Random::normal(standardDeviation);
  angle += error;
}

bool OracledPerceptsProvider::isPointBehindObstacle(const Vector2f& pointGlo) const
{
  const Pose2f robotPoseInv = theGroundTruthWorldState.ownPose.inverse();
  const float sqrDistTopoint = (pointGlo - theGroundTruthWorldState.ownPose.translation).squaredNorm();
  const Angle pointAngle = (robotPoseInv * pointGlo).angle();

  auto isBehind = [&](const Vector2f& obstacle)
  {
    const float sqrDistToObstacle = (obstacle - theGroundTruthWorldState.ownPose.translation).squaredNorm();
    if(sqrDistToObstacle < 10 || sqrDistTopoint <= sqrDistToObstacle)
      return false;

    const Vector2f obstacleRel = robotPoseInv * obstacle;
    const Vector2f obstacleThickness = obstacleRel.normalized(obstacleCoverageThickness).rotate(pi_2);

    const Angle leftPointAngle = (obstacleRel + obstacleThickness).angle();
    const Angle rightPointAngle = (obstacleRel - obstacleThickness).angle();

    return pointAngle < leftPointAngle && pointAngle > rightPointAngle; //would not work on behind the robot, but we can not see anything there too
  };

  for(const GroundTruthWorldState::GroundTruthPlayer& player : theGroundTruthWorldState.ownTeamPlayers)
    if(player.upright && isBehind(player.pose.translation))
      return true;

  for(const GroundTruthWorldState::GroundTruthPlayer& player : theGroundTruthWorldState.opponentTeamPlayers)
    if(player.upright && isBehind(player.pose.translation))
      return true;

  return false;
}
