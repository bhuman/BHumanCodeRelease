#pragma once

#include "JointRequest.h"

STREAMABLE_WITH_BASE(ModifiedJointRequest, JointAngles,
{
  ModifiedJointRequest& operator=(const JointAngles& jointAngles);
  void draw() const,
});
