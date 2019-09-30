/**
 * @file FieldFeature.cpp
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "FieldFeature.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/RobotParts/Joints.h"
#include "Tools/Math/Geometry.h"

void FieldFeature::draw() const
{
  if(Blackboard::getInstance().exists("CameraInfo"))
  {
    std::string thread = static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]).camera == CameraInfo::upper ? "Upper" : "Lower";
    DEBUG_DRAWING("representation:FieldFeature:field", "drawingOnField")
      THREAD("representation:FieldFeature:field", thread);
    DEBUG_DRAWING("representation:MarkedField:field", "drawingOnField")
      THREAD("representation:MarkedField:field", thread);
    DEBUG_DRAWING("representation:MarkedField:imageText", "drawingOnImage")
      THREAD("representation:MarkedField:imageText", thread);
    DEBUG_DRAWING("representation:MarkedField:image", "drawingOnImage")
      THREAD("representation:MarkedField:image", thread);
    DECLARE_DEBUG_DRAWING3D("representation:MarkedField", "robot");

    if(!isValid)
      return;

    COMPLEX_DRAWING("representation:FieldFeature:field")
    {
      const RobotPoseToFF poses = getGlobalRobotPosition();
      if(Blackboard::getInstance().exists("CameraMatrix"))
      {
        const CameraMatrix& theCameraMatrix = static_cast<const CameraMatrix&>(Blackboard::getInstance()["CameraMatrix"]);
        DRAW_ROBOT_POSE_WITH_HEAD_ROTATION("representation:FieldFeature:field", poses.pos1, ColorRGBA::blue, theCameraMatrix.rotation.getZAngle());
        DRAW_ROBOT_POSE_WITH_HEAD_ROTATION("representation:FieldFeature:field", poses.pos2, ColorRGBA::blue, theCameraMatrix.rotation.getZAngle());
      }
      else
      {
        DRAW_ROBOT_POSE("representation:FieldFeature:field", poses.pos1, ColorRGBA::blue);
        DRAW_ROBOT_POSE("representation:FieldFeature:field", poses.pos2, ColorRGBA::blue);
      }
    }

    for(auto& markedPoint : markedPoints)
      markedPoint.draw();

    for(auto& markedLine : markedLines)
      markedLine.draw();

    for(auto& markedIntersection : markedIntersections)
      markedIntersection.draw();
  }
}

void FieldFeature::clear()
{
  markedIntersections.clear();
  markedLines.clear();
  markedPoints.clear();
}

const FieldFeature::RobotPoseToFF FieldFeature::getGlobalRobotPosition() const
{
  return FieldFeature::RobotPoseToFF(getGlobalFeaturePosition() * this->inverse());
}

bool FieldFeature::isLikeEnoughACorrectPerception(const float searchXRadius, const float searchYRadius, const FieldLines& theFieldLines, const Vector2f offset, const unsigned allowedLines)
{
  auto isMarked = [&](const size_t i)
  {
    for(const MarkedLine& mark : markedLines)
      if(mark.lineIndex == i)
        return true;

    return false;
  };

  const Pose2f theInverse((*this + Pose2f(offset)).inverse());

  auto goesThroughBox = [&](const FieldLines::Line& line)
  {
    Vector2f point1(theInverse * line.first);
    Vector2f point2(theInverse * line.last);

    if((std::abs(point1.x()) <= searchXRadius && std::abs(point1.y()) <= searchYRadius)
       || (std::abs(point2.x()) <= searchXRadius && std::abs(point2.y()) <= searchYRadius))
      return true;

    auto isIntersecting = [&](float(*getPrimaryValue)(const Vector2f&, const float), float(*getSecondaryValue)(const Vector2f&),
                              const float rPrimary, const float rSecundary)
    {
      auto getPrimaryValue2 = [&](const Vector2f& a) {return getPrimaryValue(a, rPrimary); };

      return getPrimaryValue2(point1) * getPrimaryValue2(point2) < 0.f
             && std::abs(getSecondaryValue(point1 +
                                           std::abs(getPrimaryValue2(point1)) / (std::abs(getPrimaryValue2(point1)) + std::abs(getPrimaryValue2(point2)))
                                           * (point2 - point1))) < rSecundary + 1.f;
    };

    return isIntersecting([](const Vector2f& a, const float) {return a.x(); }, [](const Vector2f& a) {return a.y(); }, searchXRadius, searchYRadius)
      || isIntersecting([](const Vector2f& a, const float r) {return a.y() - r; }, [](const Vector2f& a) {return a.x(); }, searchYRadius, searchXRadius)
      || isIntersecting([](const Vector2f& a, const float r) {return a.y() + r; }, [](const Vector2f& a) {return a.x(); }, searchYRadius, searchXRadius);
  };

  unsigned count = 0u;

  for(size_t i = 0; i < theFieldLines.lines.size(); ++i)
    if(!isMarked(i) && goesThroughBox(theFieldLines.lines[i]))
      if(++count > allowedLines)
        return false;

  return true;
}
