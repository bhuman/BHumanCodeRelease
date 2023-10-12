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
    const CameraInfo& theCameraInfo = static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]);
    DEBUG_DRAWING("representation:FieldFeature:field", "drawingOnField")
      THREAD("representation:FieldFeature:field", theCameraInfo.getThreadName());

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

bool FieldFeature::pickMorePlausiblePose(const Pose2f& robotPose, Pose2f& pickedPose) const
{
  if(!isValid)
    return false;
  const Pose2f& p1 = getGlobalRobotPosition().pos1;
  const Pose2f& p2 = getGlobalRobotPosition().pos2;
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
