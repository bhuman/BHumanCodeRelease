/*
 * @file NaoQiImageDetector.cpp
 * @author Philip Reichenberg
 */

#include "NaoQiImageDetector.h"
#include "Platform/SystemCall.h"
#include <cmath>

MAKE_MODULE(NaoQiImageDetector);

void NaoQiImageDetector::update(NaoQiImageInfo& theNaoQiImageInfo)
{
  if(SystemCall::getMode() != SystemCall::physicalRobot)
    theNaoQiImageInfo.isCorrectImage = true;
  if(theNaoQiImageInfo.isCorrectImage || theFrameInfo.time < startTime)
    return;

  bool noData = true;
  // Check joint positions sensors
  FOREACH_ENUM(Joints::Joint, joint)
  {
    if(joint == Joints::lHand || joint == Joints::rHand)
      continue;
    noData &= valueRange.isInside(theJointAngles.angles[joint]);
  }
  // FSR checks do not work, as those values jump between 0 and a higher value randomly
  // IMU data does work even with a wrong image

  theNaoQiImageInfo.isCorrectImage = !noData;

  if(!theNaoQiImageInfo.isCorrectImage && theFrameInfo.getTimeSince(lastSirene) > wrongImageTime)
  {
    lastSirene = theFrameInfo.time;
    SystemCall::say("You need to flash me with the NAO! Qi! Image! 2!8!5! I can not read my sensor values!", true);
    SystemCall::say("After visiting the repair you need to flash me with the correct image!", true);
  }
}
