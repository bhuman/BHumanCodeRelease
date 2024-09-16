/**
 * @file JointPlayAnalyzer.h
 * This modules calculated the delta between the measured joint position of it min and max value within a given time frame.
 * This way, the joint play can be semi automatically be determined, by just moving the joints by hand.
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Sensing/JointPlayData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"

MODULE(JointPlayAnalyzer,
{,
  REQUIRES(EnhancedKeyStates),
  REQUIRES(JointAngles),
  REQUIRES(JointRequest),
  PROVIDES(JointPlayData),
  DEFINES_PARAMETERS(
  {,
    (unsigned int)(10) minMaxRange, /**< Time window to check for the min and max values of a joint position over time. */
  }),
});

class JointPlayAnalyzer : public JointPlayAnalyzerBase
{
  void update(JointPlayData& theJointPlayData) override;

  Rangea minMaxValue[Joints::numOfJoints]; /**< The current min and max values. */
  Rangea maxRequestChange[Joints::numOfJoints]; /**< The request changes. */

  unsigned int counter = 0; /**< Number of samples. */
  bool lastJointRequestIsValid = false; /**< Was the update method executed once, so we can trust the last joint request? */
  JointRequest lastRequest; /**< The last joint request. */

public:
  JointPlayAnalyzer();
};
