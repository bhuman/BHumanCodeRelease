/**
 * @file ModifiedJointRequestProvider.h
 *
 * This module provides a modification of the JointRequest
 * @author Philip Reichenberg
 */

#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/ModifiedJointRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/JointPlay.h"
#include "Framework/Module.h"

MODULE(ModifiedJointRequestProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(JointAngles),
  REQUIRES(JointPlay),
  REQUIRES(JointRequest),
  REQUIRES(MotionInfo),
  PROVIDES(ModifiedJointRequest),
  LOADS_PARAMETERS(
  {
    STREAMABLE(IParameter,
    {,
      (Rangef) stand, /**< I factor for not walking. */
      (Rangef) walk, /**< I factor for walking. */
      (float) removeFactor, /**< Remove factor for I-Value when the error changes direction. Used to remove the I-Value faster. */
      (Angle) maxControlValue, /**< Clip the artificial integral controller part into this range. */
    }),
    (ENUM_INDEXED_ARRAY(IParameter, Joints::Joint)) iFactor, /**< i factor for a joint controller. */
    (Angle) removeIValueSpeed, /**< Switching support foot forces a reset of the I-Values with this speed. */
    (int) removeDuration, /**< Duration for the reset. */
  }),
});

class ModifiedJointRequestProvider : public ModifiedJointRequestProviderBase
{
  void update(ModifiedJointRequest& theModifiedJointRequest) override;

  unsigned lastSupportFootSwitch = 0;
  bool shouldResetValue[Joints::numOfJoints];
  bool lastIsLeftPhase = false;
  bool wasLastWalking = false;

  Angle requestControllerI[Joints::numOfJoints]; /**< The artificial integral part for the joint request. */
  RingBufferWithSum<Angle, 3> lastJointError[Joints::numOfJoints]; /**< The artificial integral part for the joint request. */

public:
  ModifiedJointRequestProvider();
};
