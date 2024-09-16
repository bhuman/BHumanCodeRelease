/**
 * @file JointPlayTranslationProvider.h
 * This module calculates the position errors of the request and the measured positions.
 * @author Philip Reichenberg
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Sensing/JointPlay.h"
#include "Representations/Sensing/JointPlayTranslation.h"
#include "Representations/Sensing/JointAnglePred.h"
#include "Math/RingBuffer.h"
#include "Tools/Motion/LowPassFilterPR.h"

MODULE(JointPlayTranslationProvider,
{,
  REQUIRES(JointAngles),
  REQUIRES(JointAnglePred),
  REQUIRES(JointPlay),
  USES(JointRequest),
  PROVIDES(JointPlayTranslation),
  DEFINES_PARAMETERS(
  {,
    (float)(0.2f) factor, /**< Parameter for the offset filter. */
    (float)(0.4f) fastFactor, /**< Parameter for the offset filter. */
  }),
});

class JointPlayTranslationProvider : public JointPlayTranslationProviderBase
{
  void update(JointPlayTranslation& theJointPlayTranslation) override;

  LowPassFilterPR offsetFilter[Joints::numOfJoints];

public:
  JointPlayTranslationProvider();
};
