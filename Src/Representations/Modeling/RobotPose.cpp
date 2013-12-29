/**
* @file RobotPose.cpp
*
* contains the implementation of the streaming operators
* for the class RobotPose
*/

#include "RobotPose.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

void RobotPose::draw(bool teamRed)
{
  DECLARE_DEBUG_DRAWING("representation:RobotPose", "drawingOnField");
  Vector2<> bodyPoints[4] = {Vector2<>(55, 90),
                             Vector2<>(-55, 90),
                             Vector2<>(-55, -90),
                             Vector2<>(55, -90)
                            };
  for(int i = 0; i < 4; i++)
    bodyPoints[i] = *this * bodyPoints[i];
  Vector2<> dirVec(200, 0);
  dirVec = *this * dirVec;
  const ColorRGBA ownTeamColorForDrawing = teamRed ? ColorRGBA(255, 0, 0) : ColorRGBA(0, 0, 255);
  LINE("representation:RobotPose", translation.x, translation.y, dirVec.x, dirVec.y,
       20, Drawings::ps_solid, ColorClasses::white);
  POLYGON("representation:RobotPose", 4, bodyPoints, 20, Drawings::ps_solid,
          ownTeamColorForDrawing, Drawings::bs_solid, ColorClasses::white);
  CIRCLE("representation:RobotPose", translation.x, translation.y, 42, 0,
         Drawings::ps_solid, ownTeamColorForDrawing, Drawings::bs_solid, ownTeamColorForDrawing);

  DECLARE_DEBUG_DRAWING("representation:RobotPose:deviation", "drawingOnField");
  if(deviation < 100000.f)
    DRAWTEXT("representation:RobotPose:deviation", -3000, -2300, 100, ColorRGBA(0xff, 0xff, 0xff), "pose deviation: " << deviation);
  else
    DRAWTEXT("representation:RobotPose:deviation", -3000, -2300, 100, ColorRGBA(0xff, 0xff, 0xff), "pose deviation: unknown");

  DECLARE_DEBUG_DRAWING3D("representation:RobotPose", "field",
  {
    LINE3D("representation:RobotPose", translation.x, translation.y, 10,
           dirVec.x, dirVec.y, 10, 1, ownTeamColorForDrawing);
    for(int i = 0; i < 4; ++i)
    {
      const Vector2<> p1 = bodyPoints[i];
      const Vector2<> p2 = bodyPoints[(i + 1) & 3];
      LINE3D("representation:RobotPose", p1.x, p1.y, 10,
             p2.x, p2.y, 10, 1, ownTeamColorForDrawing);
    }
  });

  DECLARE_DEBUG_DRAWING("origin:RobotPose", "drawingOnField"); // Set the origin to the robot's current position
  DECLARE_DEBUG_DRAWING("origin:RobotPoseWithoutRotation", "drawingOnField");
  ORIGIN("origin:RobotPose", translation.x, translation.y, rotation);
  ORIGIN("origin:RobotPoseWithoutRotation", translation.x, translation.y, 0);
}

void GroundTruthRobotPose::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:GroundTruthRobotPose", "drawingOnField");
  ColorRGBA transparentWhite(ColorClasses::white);
  transparentWhite.a = 128;
  Vector2<> bodyPoints[4] = {Vector2<>(55, 90),
                             Vector2<>(-55, 90),
                             Vector2<>(-55, -90),
                             Vector2<>(55, -90)
                            };
  for(int i = 0; i < 4; i++)
    bodyPoints[i] = *this * bodyPoints[i];
  Vector2<> dirVec(200, 0);
  dirVec = *this * dirVec;
  const ColorRGBA ownTeamColorForDrawing(0, 0, 0, 128);
  LINE("representation:GroundTruthRobotPose", translation.x, translation.y, dirVec.x, dirVec.y,
       20, Drawings::ps_solid, transparentWhite);
  POLYGON("representation:GroundTruthRobotPose", 4, bodyPoints, 20, Drawings::ps_solid,
          ownTeamColorForDrawing, Drawings::bs_solid, transparentWhite);
  CIRCLE("representation:GroundTruthRobotPose", translation.x, translation.y, 42, 0,
         Drawings::ps_solid, ownTeamColorForDrawing, Drawings::bs_solid, ownTeamColorForDrawing);

  DECLARE_DEBUG_DRAWING("origin:GroundTruthRobotPose", "drawingOnField"); // Set the origin to the robot's ground truth position
  DECLARE_DEBUG_DRAWING("origin:GroundTruthRobotPoseWithoutRotation", "drawingOnField");
  ORIGIN("origin:GroundTruthRobotPose", translation.x, translation.y, rotation);
  ORIGIN("origin:GroundTruthRobotPoseWithoutRotation", translation.x, translation.y, 0);
}

RobotPoseCompressed::RobotPoseCompressed(const RobotPose& robotPose)
: translation(robotPose.translation),
  deviation(robotPose.deviation)
{
  float normalizedAngle = normalize(robotPose.rotation);
  int discretizedAngle = (int)(normalizedAngle * 128.0f / pi);
  if(discretizedAngle > 127)
    discretizedAngle = -128;
  rotation = (char) discretizedAngle;
  validity = (unsigned char) (robotPose.validity * 255.f);
}

RobotPoseCompressed::operator RobotPose() const
{
  RobotPose robotPose;
  robotPose.translation = Vector2<>(translation);
  robotPose.rotation = (float) rotation * pi / 128.f;
  robotPose.validity = (float) validity / 255.f;
  robotPose.deviation = deviation;
  return robotPose;
}
