/**
 * @file FieldFeatureCheck.h
 *
 * This file declares a macro to declare the verify method for a field feature.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author Tim Laue
 */

#pragma once

#include "Platform/BHAssert.h"
#include "Representations/Modeling/RobotPose.h"

#ifdef TARGET_ROBOT
#define VERIFY_FIELD_FEATURE ;
#else
#define VERIFY_FIELD_FEATURE \
  void verify() const \
  { \
    if(!this->isValid) return; \
    ASSERT(std::isfinite(translation.x())); \
    ASSERT(std::isfinite(translation.y())); \
    ASSERT(std::isfinite(rotation)); \
    ASSERT(rotation >= -pi); \
    ASSERT(rotation <= pi); \
    ASSERT(std::isnormal(covOfAbsoluteRobotPose(0, 0))); \
    ASSERT(std::isnormal(covOfAbsoluteRobotPose(1, 1))); \
    ASSERT(std::isnormal(covOfAbsoluteRobotPose(2, 2))); \
    ASSERT(std::isfinite(covOfAbsoluteRobotPose(0, 1))); \
    ASSERT(std::isfinite(covOfAbsoluteRobotPose(0, 2))); \
    ASSERT(std::isfinite(covOfAbsoluteRobotPose(1, 0))); \
    ASSERT(std::isfinite(covOfAbsoluteRobotPose(1, 2))); \
    ASSERT(std::isfinite(covOfAbsoluteRobotPose(2, 0))); \
    ASSERT(std::isfinite(covOfAbsoluteRobotPose(2, 1))); \
  }
#endif
