/**
 * @file WalKStiffnessRegulator.h
 * Helper functions to determine the stiffness while walking/standing
 * @author Philip Reichenberg
 */

#include "Representations/Sensing/InertialData.h"
#include "Streaming/AutoStreamable.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Sensing/JointAnglePred.h"
#include "Representations/Sensing/JointPlay.h"

STREAMABLE(WalkStiffnessParameters,
{,
  (int) walkStiffness, /**< Joint stiffness while walking in %. */
  (int) pickedUpStiffness, /**< Joint stiffness when no ground contact is present (in %). */
  (int) lowStiffnessDelay, /**< Low stiffness delay after high stand interpolation started. */
  (int) standStiffnessDelay, /**< The time in stand before the stiffness is lowered (in ms). */
  (int) lowStiffnessLegs, /**< Low stiffness setting for leg joints (except AnklePitch). */
  (int) lowStiffnessAnklePitch, /**< Low stiffness setting for ankle pitch joints. */
  (int) lowWalkStiffness, /**< The swing foot always uses a lower stiffness for the ankle pitch. */
});

enum StiffnessState
{
  Interpolating,
  Stand,
  StandStill,
  StandHigh,
  Walk,
  Kick,
};

enum WalkState
{
  standing,
  starting,
  walking,
  stopping
};

namespace WalkStiffnessRegulator
{
  void setStiffness(JointRequest& request,
                    const WalkStiffnessParameters& params, StiffnessState state,
                    const bool groundContact, const bool isLeftPhase);

  StiffnessState getState(const WalkState walkState, const float standFactor, const int timeWhenStandBegan,
                          const int timeWhenStandHighBegan, const WalkStiffnessParameters& params,
                          const bool standingStill, const bool isKick);
};
