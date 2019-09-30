/**
 * @file KeyFrameArms.cpp
 *
 * This file implements the implementation of the KeyFrameArms
 * and KeyFrameSingleArm skills.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/ArmKeyFrameEngineOutput.h"
#include "Representations/MotionControl/ArmMotionSelection.h"
#include "Representations/MotionControl/ArmMotionRequest.h"

SKILL_IMPLEMENTATION(KeyFrameArmsImpl,
{,
  IMPLEMENTS(KeyFrameArms),
  IMPLEMENTS(KeyFrameSingleArm),
  REQUIRES(ArmKeyFrameEngineOutput),
  REQUIRES(ArmMotionSelection),
  REQUIRES(LibCheck),
  MODIFIES(ArmMotionRequest),
});

class KeyFrameArmsImpl : public KeyFrameArmsImplBase
{
  void execute(const KeyFrameArms& p) override
  {
    setRequest(p.motion, Arms::left, p.fast);
    setRequest(p.motion, Arms::right, p.fast);
  }

  bool isDone(const KeyFrameArms& p) const override
  {
    return requestIsExecuted(p.motion, Arms::left) && requestIsExecuted(p.motion, Arms::right);
  }

  void execute(const KeyFrameSingleArm& p) override
  {
    setRequest(p.motion, p.arm, p.fast);
  }

  bool isDone(const KeyFrameSingleArm& p) const override
  {
    return requestIsExecuted(p.motion, p.arm);
  }

  void setRequest(ArmKeyFrameRequest::ArmKeyFrameId motion, Arms::Arm arm, bool fast)
  {
    theArmMotionRequest.armMotion[arm] = ArmMotionRequest::keyFrame;
    theArmMotionRequest.armKeyFrameRequest.arms[arm].motion = motion;
    theArmMotionRequest.armKeyFrameRequest.arms[arm].fast = fast;
    theLibCheck.setArm(arm);
  }

  bool requestIsExecuted(ArmKeyFrameRequest::ArmKeyFrameId motion, Arms::Arm arm) const
  {
    return theArmMotionSelection.targetArmMotion[arm] == ArmMotionSelection::keyFrameS &&
           theArmKeyFrameEngineOutput.arms[arm].motion == motion;
  }
};

MAKE_SKILL_IMPLEMENTATION(KeyFrameArmsImpl);
