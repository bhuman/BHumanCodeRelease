/**
 * @file BehaviorBase.h
 *
 * This file declares a base class for the parts
 * of the behavior that are written as C++ code.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Representations/BehaviorControl/BallSearchAreas.h"
#include "Representations/BehaviorControl/ClearTarget.h"
#include "Representations/BehaviorControl/DribbleTarget.h"
#include "Representations/BehaviorControl/ExpectedGoals.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/FieldInterceptBall.h"
#include "Representations/BehaviorControl/IndirectKick.h"
#include "Representations/BehaviorControl/Libraries/LibPosition.h"
#include "Representations/BehaviorControl/PassEvaluation.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/BehaviorParameters.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/GlobalOpponentsModel.h"
#include "Representations/Modeling/GlobalTeammatesModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeammatesBallModel.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Framework/Blackboard.h"
#include "Framework/Module.h"

// Representations listed here will be available in classes derived from \c BehaviorBase.
#define VISIT_BEHAVIOR_BASE_REPRESENTATIONS(_) \
  _(BallSearchAreas) \
  _(BallSpecification) \
  _(BehaviorParameters) \
  _(ClearTarget) \
  _(DribbleTarget) \
  _(ExpectedGoals) \
  _(FieldBall) \
  _(FieldInterceptBall) \
  _(FieldDimensions) \
  _(FrameInfo) \
  _(GameState) \
  _(GlobalOpponentsModel) \
  _(GlobalTeammatesModel) \
  _(IndirectKick) \
  _(LibPosition) \
  _(MotionInfo) \
  _(PassEvaluation) \
  _(RobotPose) \
  _(TeammatesBallModel)

class BehaviorBase
{
public:
  /** Virtual destructor for polymorphism and release of blackboard references. */
  virtual ~BehaviorBase()
  {
#define FREE_MEMBER(type) Blackboard::getInstance().free(#type);
    VISIT_BEHAVIOR_BASE_REPRESENTATIONS(FREE_MEMBER)
#undef FREE_MEMBER
  }

  /** Resets a behavior before it is first executed. */
  virtual void reset() {}

  /** Is executed every frame before any behavior is executed. */
  virtual void preProcess() {}

  /** Is executed every frame after all behaviors have been executed. */
  virtual void postProcess() {}

  /**
   * Adds the required representations to a module info list.
   * @param info The module info to modify.
   */
  static void addToModuleInfo(std::vector<ModuleBase::Info>& info)
  {
#define ADD_TO_MODULE_INFO(type) info.emplace_back(#type, nullptr);
    VISIT_BEHAVIOR_BASE_REPRESENTATIONS(ADD_TO_MODULE_INFO)
#undef ADD_TO_MODULE_INFO
  }

protected:
#define ALLOC_MEMBER(type) const type& the##type = Blackboard::getInstance().alloc<type>(#type);
  VISIT_BEHAVIOR_BASE_REPRESENTATIONS(ALLOC_MEMBER)
#undef ALLOC_MEMBER
};

#undef VISIT_BEHAVIOR_BASE_REPRESENTATIONS
