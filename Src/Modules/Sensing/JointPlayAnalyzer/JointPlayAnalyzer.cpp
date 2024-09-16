/**
 * @file JointPlayAnalyzer.cpp
 * @author Philip Reichenberg
 */

#include "JointPlayAnalyzer.h"

MAKE_MODULE(JointPlayAnalyzer);

JointPlayAnalyzer::JointPlayAnalyzer()
{
  for(std::size_t joint = 0; joint < Joints::numOfJoints; joint++)
    minMaxValue[joint] = maxRequestChange[joint] = Rangea(0, 0);
}

void JointPlayAnalyzer::update(JointPlayData& theJointPlayData)
{
  DEBUG_RESPONSE_ONCE("module:JointPlayAnalyzer:reset")
  {
    for(std::size_t joint = 0; joint < Joints::numOfJoints; joint++)
      theJointPlayData.play[joint] = 0;
  }

  if(theEnhancedKeyStates.pressed[KeyStates::headMiddle] && (theEnhancedKeyStates.pressed[KeyStates::headFront] || theEnhancedKeyStates.pressed[KeyStates::headRear]))
    for(std::size_t joint = 0; joint < Joints::numOfJoints; joint++)
      theJointPlayData.play[joint] = 0;

  if(theJointAngles.timestamp == 0)
    return;
  if(lastJointRequestIsValid)
  {
    counter++;
    for(std::size_t joint = 0; joint < Joints::numOfJoints; joint++)
    {
      const Angle requestChange = theJointRequest.angles[joint] - lastRequest.angles[joint];
      maxRequestChange[joint].min = std::min(maxRequestChange[joint].min, requestChange);
      maxRequestChange[joint].max = std::min(maxRequestChange[joint].max, requestChange);
      // TODO energy saving is disabled. No need for request delta calculation
      minMaxValue[joint].min = std::min(minMaxValue[joint].min, Angle(theJointAngles.angles[joint] - maxRequestChange[joint].min));
      minMaxValue[joint].max = std::max(minMaxValue[joint].max, Angle(theJointAngles.angles[joint] - maxRequestChange[joint].max));

      if(counter >= minMaxRange)
        theJointPlayData.play[joint] = std::max(theJointPlayData.play[joint], Angle(minMaxValue[joint].max - minMaxValue[joint].min));
    }
  }
  else
  {
    for(std::size_t joint = 0; joint < Joints::numOfJoints; joint++)
      minMaxValue[joint] = Rangea(theJointAngles.angles[joint], theJointAngles.angles[joint]);
  }

  if(counter >= minMaxRange)
  {
    counter = 0;
    for(std::size_t joint = 0; joint < Joints::numOfJoints; joint++)
    {
      maxRequestChange[joint] = Rangea(0, 0);
      minMaxValue[joint] = Rangea(theJointAngles.angles[joint], theJointAngles.angles[joint]);
    }
  }

  lastRequest = theJointRequest;
  lastJointRequestIsValid = true;
}
