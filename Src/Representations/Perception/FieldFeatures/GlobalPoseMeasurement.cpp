/**
 * @file GlobalPoseMeasurement.cpp
 *
 * This file implements a representation of a measurement of a pose on the field.
 *
 * @author Arne Hasselbring
 */

#include "GlobalPoseMeasurement.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Debugging/DebugDrawings.h"
#include "Framework/Blackboard.h"

void GlobalPoseMeasurement::draw() const
{
  DEBUG_DRAWING("representation:GlobalPoseMeasurement", "drawingOnField")
  {
    if(Blackboard::getInstance().exists("CameraInfo"))
      THREAD("representation:GlobalPoseMeasurement", static_cast<CameraInfo&>(Blackboard::getInstance()["CameraInfo"]).getThreadName());
    if(isValid)
    {
      COVARIANCE_ELLIPSES_2D("representation:GlobalPoseMeasurement", (covarianceOnField.topLeftCorner<2, 2>()), (poseOnField.translation.head<2>()));
      DRAW_ROBOT_POSE_ROTATIONAL_STANDARD_DEVIATION("representation:GlobalPoseMeasurement", poseOnField, std::sqrt(covarianceOnField(2, 2)), ColorRGBA::black);
    }
  }
}
