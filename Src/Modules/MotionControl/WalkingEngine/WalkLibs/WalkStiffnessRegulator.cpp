/**
 * @file WalKStiffnessRegulator.cpp
 * Helper functions to determine the stiffness while walking/standing
 * @author Philip Reichenberg
 */

#include "WalkStiffnessRegulator.h"
#include "Math/BHMath.h"

void WalkStiffnessRegulator::setStiffness(JointRequest& request,
                                          const WalkStiffnessParameters& params, StiffnessState state,
                                          const bool groundContact,
                                          const bool isLeftPhase)
{
  if(!groundContact)
  {
    for(unsigned int index = Joints::firstLegJoint; index < Joints::numOfJoints; index++)
      request.stiffnessData.stiffnesses[index] = params.pickedUpStiffness; // Make sure the robot does not break if something is wrong
    return;
  }

  int legStiffness = params.walkStiffness;
  switch(state)
  {
    case StandStill:
      legStiffness = StiffnessData::useDefault;
      break;
    case StandHigh:
      legStiffness = params.lowStiffnessLegs;
      break;
    default:
      break; // Already set to walk stiffness
  }

  for(std::size_t i = Joints::firstLegJoint; i < Joints::numOfJoints; i++)
    request.stiffnessData.stiffnesses[i] = legStiffness;

  request.stiffnessData.stiffnesses[Joints::waistYaw] = legStiffness;

  switch(state)
  {
    case StandHigh:
      request.stiffnessData.stiffnesses[Joints::lAnklePitch] = params.lowStiffnessAnklePitch;
      request.stiffnessData.stiffnesses[Joints::rAnklePitch] = params.lowStiffnessAnklePitch;
      break;
    case Walk:
      request.stiffnessData.stiffnesses[isLeftPhase ? Joints::lAnklePitch : Joints::rAnklePitch] = params.lowWalkStiffness;
      break;
    default:
      break; // no special handling in all other cases
  }
}

StiffnessState WalkStiffnessRegulator::getState(const WalkState walkState, const float standFactor, const int timeWhenStandBegan,
                                                const int timeWhenStandHighBegan, const WalkStiffnessParameters& params,
                                                const bool standingStill, const bool isKick)
{
  if(isKick)
    return StiffnessState::Kick;

  if(walkState == standing && standFactor == 1.f && timeWhenStandHighBegan > params.lowStiffnessDelay)
    return StiffnessState::StandHigh;

  const bool applyEnergySavingStand = walkState == standing && standFactor == 0.f && timeWhenStandBegan > params.standStiffnessDelay;
  if(applyEnergySavingStand && standingStill)
    return StiffnessState::StandStill;
  else if(applyEnergySavingStand)
    return StiffnessState::Stand;
  else if(walkState == standing)
    return StiffnessState::Interpolating;

  return StiffnessState::Walk;
}
