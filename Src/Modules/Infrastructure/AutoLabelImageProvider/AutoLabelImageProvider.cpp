#include "AutoLabelImageProvider.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Angle.h"
#include "Tools/Math/Constants.h"
#include "Tools/Math/Random.h"
#include "Tools/Range.h"
#include <algorithm>
#include <limits>

MAKE_MODULE(AutoLabelImageProvider, infrastructure)

void AutoLabelImageProvider::update(LabelImage& labelImage)
{
  DECLARE_DEBUG_DRAWING3D("module:AutoLabelImageProvider", "field");
  DECLARE_DEBUG_DRAWING("module:AutoLabelImageProvider:image", "drawingOnImage");

  if(theCameraInfo.camera != CameraInfo::upper)
    return;

  auto worldPlayers = theGroundTruthWorldState.firstTeamPlayers;
  worldPlayers.insert(worldPlayers.end(), theGroundTruthWorldState.secondTeamPlayers.begin(), theGroundTruthWorldState.secondTeamPlayers.end());

  labelImage.annotations.clear();

  for(const auto& player : worldPlayers)
  {
    if(!player.upright)
      continue;

    std::vector<Vector3f> robot3DBoundingBox = getPlainRobot3DBoundingBox();

    LabelImage::Annotation playerAnnotation = {};
    playerAnnotation.labelType = LabelImage::Annotation::Robot;

    move3DBoundingBox(robot3DBoundingBox, theGroundTruthWorldState.ownPose.inverse() * player.pose);
    drawRelative3DBoundingBox(robot3DBoundingBox);
    if(projectRelative3DBoundingBoxToAnnotation(robot3DBoundingBox, playerAnnotation))
    {
      Vector2f playerRelativeTranslation = theGroundTruthWorldState.ownPose.translation - player.pose.translation;
      playerAnnotation.rotation = Angle::normalize<float>(player.pose.rotation - playerRelativeTranslation.angle());
      playerAnnotation.distance = playerRelativeTranslation.norm();
      labelImage.annotations.push_back(playerAnnotation);
    }

    labelImage.valid = true;
    labelImage.evaluated = true;

    // Determine the new position of the robot at the end of the update call so that their positions in the world
    // state are updated in the next cycle
    auto p = Pose2f::random(
      {-pi, pi},
      {-robotPlacementAreaLength, robotPlacementAreaLength},
      {-robotPlacementAreaWidth, robotPlacementAreaWidth}
    );
    if(calculateDistanceToClosestRobot(p.translation) > minDistanceToRobots)
      teleportRobot(Random::uniformInt<int>(minRobotPlacementId, maxRobotPlacementId), p);
  }
}

std::vector<Vector3f> AutoLabelImageProvider::getPlainRobot3DBoundingBox()
{
  return
  {
    // Head
    {-robotCenter, robotWidthHead / 2, robotHeight},
    {robotDepthHead - robotCenter, robotWidthHead / 2, robotHeight},
    {-robotCenter, -robotWidthHead / 2, robotHeight},
    {robotDepthHead - robotCenter, -robotWidthHead / 2, robotHeight},
    // Arms
    {0.f, robotWidthHands / 2, robotHandsHeight},
    {0.f, -robotWidthHands / 2, robotHandsHeight},
    // Feet
    {-robotCenter, robotWidthBody / 2, 0.f},
    {robotDepthFeet - robotCenter, robotWidthBody / 2, 0.f},
    {-robotCenter, -robotWidthBody / 2, 0.f},
    {robotDepthFeet - robotCenter, -robotWidthBody / 2, 0.f},
  };
}

void AutoLabelImageProvider::drawRelative3DBoundingBox(const std::vector<Vector3f>& relative3DBoundingBox)
{
  std::vector<Vector3f> robot3DAbsoluteBoundingBox;

  for(const auto& p : relative3DBoundingBox)
  {
    Vector2f absolutePose2D = Transformation::robotToField(theGroundTruthWorldState.ownPose, Vector2f(p.x(), p.y()));
    Vector3f absolutePose(absolutePose2D.x(), absolutePose2D.y(), p.z());
    CROSS3D("module:AutoLabelImageProvider", absolutePose.x(), absolutePose.y(), absolutePose.z(), 10, 5, ColorRGBA::black);
  }
}

void AutoLabelImageProvider::move3DBoundingBox(std::vector<Vector3f>& box, const Pose2f& pose)
{
  for(auto& p : box)
    p << pose * p.head<2>(), p.z();
}

bool AutoLabelImageProvider::projectRelative3DBoundingBoxToAnnotation(const std::vector<Vector3f>& box, LabelImage::Annotation& annotation)
{
  annotation.correct = true;
  annotation.upperLeft = {theCameraInfo.width, theCameraInfo.height};
  annotation.lowerRight = {0, 0};

  for(const auto& p : box)
  {
    Vector2f point;
    if(Transformation::robotToImage(p, theCameraMatrix, theCameraInfo, point))
    {
      if(point.x() < annotation.upperLeft.x())
        annotation.upperLeft.x() = point.x();
      if(point.x() > annotation.lowerRight.x())
        annotation.lowerRight.x() = point.x();
      if(point.y() < annotation.upperLeft.y())
        annotation.upperLeft.y() = point.y();
      if(point.y() > annotation.lowerRight.y())
        annotation.lowerRight.y() = point.y();

      CROSS("module:AutoLabelImageProvider:image", point.x(), point.y(), 5, 1, Drawings::solidPen, ColorRGBA::black);
    }
  }

  return annotation.upperLeft.x() < theCameraInfo.width - marginBoundingBox
         && annotation.upperLeft.y() < theCameraInfo.height - marginBoundingBox
         && annotation.lowerRight.x() > marginBoundingBox
         && annotation.lowerRight.y() > marginBoundingBox;
}

void AutoLabelImageProvider::teleportRobot(int robotId, const Pose2f& pose)
{
  OUTPUT(idConsole, text, "mvo RoboCup.extras.robot" << robotId << " " << -pose.translation.x() << " " << -pose.translation.y() << " 330 0 0 " << pose.rotation);
}

float AutoLabelImageProvider::calculateDistanceToClosestRobot(const Vector2f& position)
{
  std::vector<Vector2f> robotPositions;

  for(const auto& player : theGroundTruthWorldState.firstTeamPlayers)
    robotPositions.push_back(player.pose.translation);

  for(const auto& player : theGroundTruthWorldState.secondTeamPlayers)
    robotPositions.push_back(player.pose.translation);

  robotPositions.push_back(theGroundTruthWorldState.ownPose.translation);

  std::vector<float> robotDistances(robotPositions.size());
  std::transform(robotPositions.begin(), robotPositions.end(), robotDistances.begin(), [position](const auto& player) { return (position - player).norm(); });
  return *std::min_element(robotDistances.begin(), robotDistances.end());
}
