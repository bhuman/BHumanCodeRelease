/**
 * @file BehaviorControl.h
 *
 * This file declares a module that describes the robot behavior.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Representations/BehaviorControl/ActivationGraph.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/Module/Module.h"

#include "Tools/BehaviorOptionRegistry.h"

MODULE(BehaviorControl,
{,
  REQUIRES(ActivationGraph),
  REQUIRES(FrameInfo),
  REQUIRES(RobotPose),
  PROVIDES(ActivationGraph),
  PROVIDES(ArmMotionRequest),
  PROVIDES(BehaviorStatus),
  PROVIDES(HeadMotionRequest),
  PROVIDES(MotionRequest),
});

class BehaviorControl : public BehaviorControlBase
{
public:
  /**
   * Constructor.
   */
  BehaviorControl();
  /**
   * Creates extended module info (union of this module's info and requirements of all behavior options).
   * @return The extended module info.
   */
  static std::vector<ModuleBase::Info> getExtModuleInfo();
private:
  /**
   * Updates the activation graph.
   * @param activationGraph The provided activation graph.
   */
  void update(ActivationGraph& activationGraph) override;
  /**
   * Updates the arm motion request.
   * @param armMotionRequest The provided arm motion request.
   */
  void update(ArmMotionRequest& armMotionRequest) override { armMotionRequest = registry.theArmMotionRequest; }
  /**
   * Updates the behavior status.
   * @param behaviorStatus The provided behavior status.
   */
  void update(BehaviorStatus& behaviorStatus) override { behaviorStatus = registry.theBehaviorStatus; }
  /**
   * Updates the head motion request.
   * @param headMotionRequest The provided head motion request.
   */
  void update(HeadMotionRequest& headMotionRequest) override { headMotionRequest = registry.theHeadMotionRequest; }
  /**
   * Updates the motion request.
   * @param motionRequest The provided motion request.
   */
  void update(MotionRequest& motionRequest) override { motionRequest = registry.theMotionRequest; }

  BehaviorOptionRegistry registry; /**< The object that contains instances of all active behavior options. */
  BehaviorOptionInterface& soccerBehavior; /** A reference to the root soccer behavior. */
  BehaviorOptionInterface& headControl2018; /** A reference to the head control behavior. */
};
