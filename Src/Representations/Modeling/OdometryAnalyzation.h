
#pragma once

#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose2f.h"

STREAMABLE(OdometryAnalyzation,
{
  OdometryAnalyzation() = default;
  void draw() const,

  (Pose2f) startPoint,
  (Pose2f) currentEndPoint,
  (Matrix2f) covariance,
});
