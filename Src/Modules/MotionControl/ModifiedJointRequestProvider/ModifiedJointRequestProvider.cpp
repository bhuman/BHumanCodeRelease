/*
 * @file ModifiedJointRequestProvider.cpp
 * @author Philip Reichenberg
 */
#include "ModifiedJointRequestProvider.h"
#include "Framework/Settings.h"

MAKE_MODULE(ModifiedJointRequestProvider);

ModifiedJointRequestProvider::ModifiedJointRequestProvider()
{
  FOREACH_ENUM(Joints::Joint, joint)
    shouldResetValue[joint] = false;
}

void ModifiedJointRequestProvider::update(ModifiedJointRequest& theModifiedJointRequest)
{
  theModifiedJointRequest.angles.fill(0);

  if(theMotionInfo.lastMotionPhaseStarted != lastSupportFootSwitch)
  {
    lastSupportFootSwitch = theMotionInfo.lastMotionPhaseStarted;
    const Joints::Joint supportKnee = theMotionInfo.walkPhaseIsLeftPhase ? Joints::rKneePitch : Joints::lKneePitch;
    // The robot is walking and the support foot switched OR the robot starts walking now
    if((wasLastWalking && lastIsLeftPhase != theMotionInfo.walkPhaseIsLeftPhase) || (!wasLastWalking && theMotionInfo.executedPhase == MotionPhase::walk))
      FOREACH_ENUM(Joints::Joint, joint)
        shouldResetValue[joint] = supportKnee != joint;
    wasLastWalking = theMotionInfo.executedPhase == MotionPhase::walk;
    lastIsLeftPhase = theMotionInfo.walkPhaseIsLeftPhase;
  }

  const Rangea removeIRange(-removeIValueSpeed * Global::getSettings().motionCycleTime, removeIValueSpeed * Global::getSettings().motionCycleTime);

  FOREACH_ENUM(Joints::Joint, joint)
  {
    ASSERT(theJointRequest.angles[joint] != JointAngles::ignore);
    if(theJointRequest.angles[joint] == JointAngles::off || theJointRequest.stiffnessData.stiffnesses[joint] == 0)
    {
      requestControllerI[joint] = 0.f;
      continue;
    }

    const Angle error = theJointPlay.jointState[joint].lastExecutedRequest - theJointAngles.angles[joint];
    const Rangef& factorRange = wasLastWalking ? iFactor[joint].walk : iFactor[joint].stand;
    const float factor = error >= 0.f ? factorRange.max : factorRange.min;
    const Angle errorChange = error - lastJointError[joint].average();

    shouldResetValue[joint] &= theFrameInfo.getTimeSince(lastSupportFootSwitch) < removeDuration && requestControllerI[joint] != 0.f && (error * requestControllerI[joint] <= 0.f && factor > 0.f);

    const bool removeIValue = iFactor[joint].removeFactor > 0.f && errorChange * requestControllerI[joint] < 0.f;
    const bool resetPreviousI = (shouldResetValue[joint] && joint >= Joints::firstLegJoint) || (factor == 0.f);
    // Remove I value from legs
    if(resetPreviousI)
      requestControllerI[joint] -= removeIRange.limit(requestControllerI[joint]);
    if(!resetPreviousI || (error * requestControllerI[joint] <= 0.f && factor > 0.f))
    {
      if(removeIValue)
        requestControllerI[joint] -= Rangea(std::min(0.f, -errorChange * iFactor[joint].removeFactor), std::max(0.f, -errorChange * iFactor[joint].removeFactor)).limit(requestControllerI[joint]);
      if(!removeIValue || error * requestControllerI[joint] < 0.f)
        requestControllerI[joint] += factor * error;
      const Rangea controlRange(-iFactor[joint].maxControlValue, iFactor[joint].maxControlValue);
      controlRange.clamp(requestControllerI[joint]);
    }
    lastJointError[joint].push_front(error);
    theModifiedJointRequest.angles[joint] = requestControllerI[joint];
  }
}
