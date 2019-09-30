/**
 * @file Modules/MotionControl/ArmMotionCombinator.cpp
 * This file declares a module that combines the arm motions created by the different modules.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * based on a module created by @author Thomas Röfer
 */

#include "ArmMotionCombinator.h"
#include "MotionCombinator.h"

MAKE_MODULE(ArmMotionCombinator, motionControl)

void ArmMotionCombinator::update(ArmJointRequest& armJointRequest)
{
  const JointRequest* armJointRequests[ArmMotionSelection::numOfArmMotions];
  armJointRequests[ArmMotionSelection::walkArms] = &theWalkingEngineOutput;
  armJointRequests[ArmMotionSelection::fallArms] = &theFallEngineOutput;
  armJointRequests[ArmMotionSelection::kickArms] = &theKickEngineOutput;
  armJointRequests[ArmMotionSelection::specialActionArms] = &theSpecialActionsOutput;
  armJointRequests[ArmMotionSelection::standArms] = &theStandArmRequest;
  armJointRequests[ArmMotionSelection::getUpArms] = &theGetUpEngineOutput;

  armJointRequests[ArmMotionSelection::clearS] = &theSupersedeArmMotionEngineOutput;
  armJointRequests[ArmMotionSelection::keyFrameS] = &theArmKeyFrameEngineOutput;
  armJointRequests[ArmMotionSelection::keyPoseS] = &theSupersedeArmMotionEngineOutput;
  armJointRequests[ArmMotionSelection::pointAtS] = &thePointAtEngineOutput;

  auto combinateArmMotions = [&](Arms::Arm const arm)
  {
    const Joints::Joint startJoint = arm == Arms::left ? Joints::lShoulderPitch : Joints::rShoulderPitch;
    const Joints::Joint endJoint = arm == Arms::left ? Joints::lHand : Joints::rHand;
    MotionUtilities::copy(*armJointRequests[theArmMotionSelection.targetArmMotion[arm]], armJointRequest, theStiffnessSettings, startJoint, endJoint);

    ASSERT(armJointRequest.isValid());

    if(theArmMotionSelection.armRatios[arm][theArmMotionSelection.targetArmMotion[arm]] == 1.f)
    {
      lastJointAngles = theJointAngles;
    }
    else
    {
      const bool interpolateStiffness = !(theLegMotionSelection.targetMotion != MotionRequest::specialAction &&
                                          theLegMotionSelection.specialActionRequest.specialAction == SpecialActionRequest::playDead &&
                                          theLegMotionSelection.ratios[MotionRequest::specialAction] > 0.f &&
                                          theArmMotionSelection.armRatios[arm][ArmMotionRequest::none] > 0);

      for(int i = 0; i < ArmMotionSelection::numOfArmMotions; ++i)
      {
        if(i != theArmMotionSelection.targetArmMotion[arm] && theArmMotionSelection.armRatios[arm][i] > 0)
        {
          MotionUtilities::interpolate(*armJointRequests[i], *armJointRequests[theArmMotionSelection.targetArmMotion[arm]], theArmMotionSelection.armRatios[arm][i], armJointRequest, interpolateStiffness, theStiffnessSettings, lastJointAngles, startJoint, endJoint);
        }
      }
    }

    ASSERT(armJointRequest.isValid());
  };

  combinateArmMotions(Arms::left);
  combinateArmMotions(Arms::right);
}

void ArmMotionCombinator::update(ArmMotionInfo& armMotionInfo)
{
  auto setArmMotionInfo = [&](Arms::Arm const arm)
  {
    if(theArmMotionSelection.armRatios[arm][theArmMotionSelection.targetArmMotion[arm]] == 1.f)
    {
      armMotionInfo.armMotion[arm] = theArmMotionSelection.targetArmMotion[arm] < ArmMotionSelection::firstNonBodyMotion ?
                                     ArmMotionRequest::none :
                                     ArmMotionRequest::ArmRequest(theArmMotionSelection.targetArmMotion[arm] - ArmMotionSelection::firstNonBodyMotion + 1);

      switch(theArmMotionSelection.targetArmMotion[arm])
      {
        case ArmMotionSelection::keyFrameS:
          armMotionInfo.armKeyFrameRequest = theArmMotionSelection.armKeyFrameRequest;
          break;
        default:
          break;
      }
    }
  };

  setArmMotionInfo(Arms::left);
  setArmMotionInfo(Arms::right);
}
