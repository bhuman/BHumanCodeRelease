/**
 * @file FieldFeature.cpp
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "FieldFeature.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Debugging/DebugDrawings.h"

void FieldFeature::draw() const
{
  if(Blackboard::getInstance().exists("CameraInfo"))
  {
    std::string thread = static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]).camera == CameraInfo::upper ? "Upper" : "Lower";
    DEBUG_DRAWING("representation:FieldFeature:field", "drawingOnField")
      THREAD("representation:FieldFeature:field", thread);

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
  }
}

const FieldFeature::RobotPoseToFF FieldFeature::getGlobalRobotPosition() const
{
  return FieldFeature::RobotPoseToFF(getGlobalFeaturePosition() * this->inverse());
}
