/**
 * @file JointPlay.h
 * @author Philip Reichenberg
 */

#pragma once

#include "RobotParts/Joints.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/EnumIndexedArray.h"

STREAMABLE(JointPlay,
{
  ENUM(JointStatus,
  {,
    allFine,
    sensorJump,
    broken,
    damaged,
  });

  STREAMABLE(JointState,
  {,
    (JointPlay::JointStatus)(JointPlay::allFine) status,
    (Rangea) requestBoundary, /**< The current expected minand max joint positions. */
    (Angle) velocity,
    (Angle) lastExecutedRequest,
  });

  std::vector<Joints::Joint> getJointsWithSensorJump(const JointPlay::JointStatus status) const;

  void draw() const,

  (float)(0.f) qualityOfRobotHardware, /**< 1.f means good robot, 0.f means bad robot */
  (bool)(false) isCalibrated, /**< The robot walked enough to trust the calibration. */
  (ENUM_INDEXED_ARRAY(JointState, Joints::Joint)) jointState,
});
