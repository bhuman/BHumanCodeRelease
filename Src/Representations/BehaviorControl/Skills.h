/**
 * @file Skills.h
 *
 * This file declares all skills that are used by the current behavior.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Configuration/CalibrationRequest.h"
#include "Representations/Configuration/KickInfo.h"
#include "Representations/MotionControl/ArmKeyFrameRequest.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/BehaviorControl/Framework/Skill/Skill.h"
#include "Tools/BehaviorControl/HeadOrientation.h"
#include "Tools/BehaviorControl/Interception.h"
#include "Math/Angle.h"
#include "Math/Eigen.h"
#include "Math/Range.h"
#include "Math/Pose2f.h"
#include "RobotParts/Arms.h"
#include <limits>
#include <string>

namespace Skills
{
#include "SkillInterfaces.h"
}
