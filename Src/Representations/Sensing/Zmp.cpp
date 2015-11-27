#include "Zmp.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/RingBuffer.h"
#include <utility>
#include <iostream>

void Zmp::draw() const
{
  float offset = 0.f;
  if(Blackboard::getInstance().exists("RobotDimensions"))
  {
    const RobotDimensions& rd = (const RobotDimensions&)Blackboard::getInstance()["RobotDimensions"];
    offset = -rd.footHeight;
  }
  DECLARE_DEBUG_DRAWING3D("representation:Zmp:zmpL", "LFoot");
  DECLARE_DEBUG_DRAWING3D("representation:Zmp:zmpR", "RFoot");
  CROSS3D("representation:Zmp:zmpL", zmpInLeftSole.x(), zmpInLeftSole.y(), offset, 50, 50, ColorRGBA::blue);
  CROSS3D("representation:Zmp:zmpR", zmpInRightSole.x(), zmpInRightSole.y(), offset, 50, 50, ColorRGBA::green);

  DECLARE_DEBUG_DRAWING("representation:Zmp:zmp", "drawingOnField");

  Pose2f leftTransform(90_deg, Vector2f(-50, 0));
  Pose2f rightTransform(90_deg, Vector2f(50, 0));

  drawFoot(rightTransform, 10, false);
  drawFoot(leftTransform, 10, true);

  const Vector2f left = (leftTransform * zmpInLeftSole) * 10;
  const Vector2f right = (rightTransform * zmpInRightSole) * 10;

  CROSS("representation:Zmp:zmp", left.x(), left.y(), 40, 15, Drawings::solidPen, ColorRGBA::blue);
  CROSS("representation:Zmp:zmp", right.x(), right.y(), 40, 15, Drawings::solidPen, ColorRGBA::red);

  const Vector2f leftRot = (leftTransform * rotZmpInLeftSole) * 10;
  const Vector2f rightRot = (rightTransform * rotZmpInRightSole) * 10;

  CROSS("representation:Zmp:zmp", leftRot.x(), leftRot.y(), 45, 15, Drawings::solidPen, ColorRGBA::green);
  CROSS("representation:Zmp:zmp", rightRot.x(), rightRot.y(), 45, 15, Drawings::solidPen, ColorRGBA::green);

  const Vector2f leftJoint = (leftTransform * jointZmpInLeftSole) * 10;
  const Vector2f rightJoint = (rightTransform * jointZmpInRightSole) * 10;

  CROSS("representation:Zmp:zmp", leftJoint.x(), leftJoint.y(), 45, 15, Drawings::solidPen, ColorRGBA::yellow);
  CROSS("representation:Zmp:zmp", rightJoint.x(), rightJoint.y(), 45, 15, Drawings::solidPen, ColorRGBA::yellow);

  const Vector2f leftCom = (leftTransform * comInLeftSole) * 10;
  const Vector2f rightCom = (rightTransform * comInRightSole) * 10;
  CIRCLE("representation:Zmp:zmp", leftCom.x(), leftCom.y(), 10, 15, Drawings::solidPen, ColorRGBA::cyan, Drawings::noBrush, ColorRGBA::red);
  CIRCLE("representation:Zmp:zmp", rightCom.x(), rightCom.y(), 10, 15, Drawings::solidPen, ColorRGBA::magenta, Drawings::noBrush, ColorRGBA::red);
}

void Zmp::drawFoot(const Pose2f& offset, const float scale, const bool left) const
{
  for(size_t i = 0; i < Drawings3D::footPoints.size(); ++i)
  {
    Vector2f start = Drawings3D::footPoints[i].head<2>() * 1000.f;
    const int endIdx = (i + 1) % static_cast<int>(Drawings3D::footPoints.size());
    Vector2f end = Drawings3D::footPoints[endIdx].head<2>() * 1000.f;
    if(left)
    {
      start.y() = -start.y();
      end.y() = -end.y();
    }
    start = (offset * start) * scale;
    end = (offset * end) * scale;

    LINE("representation:Zmp:zmp", start.x(), start.y(), end.x(), end.y(), 3, Drawings::solidPen, ColorRGBA::red);
  }
  const Vector2f center = (offset * Vector2f::Zero() * scale);
  CROSS("representation:Zmp:zmp", center.x(), center.y(), 60, 30, Drawings::solidPen, left ? ColorRGBA::blue : ColorRGBA::red);
}
