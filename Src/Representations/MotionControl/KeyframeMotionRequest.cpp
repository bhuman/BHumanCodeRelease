/**
 * @file KeyframeMotionRequest.cpp
 *
 * This file implements functions for the \c KeyframeMotionRequest.
 *
 * @author Arne Hasselbring
 */

#include "KeyframeMotionRequest.h"
#include "Platform/BHAssert.h"

KeyframeMotionRequest KeyframeMotionRequest::fromDiveRequest(MotionRequest::Dive::Request diveRequest)
{
  KeyframeMotionRequest keyframeMotionRequest;
  switch(diveRequest)
  {
    case MotionRequest::Dive::prepare:
      keyframeMotionRequest.keyframeMotion = sitDownKeeper;
      keyframeMotionRequest.mirror = false;
      break;
    case MotionRequest::Dive::jumpLeft:
    case MotionRequest::Dive::jumpRight:
      keyframeMotionRequest.keyframeMotion = keeperJumpLeft;
      keyframeMotionRequest.mirror = diveRequest == MotionRequest::Dive::jumpRight;
      break;
    case MotionRequest::Dive::squatArmsBackLeft:
    case MotionRequest::Dive::squatArmsBackRight:
      keyframeMotionRequest.keyframeMotion = genuflectStand;
      keyframeMotionRequest.mirror = diveRequest == MotionRequest::Dive::squatArmsBackRight;
      break;
    case MotionRequest::Dive::squatWideArmsBackLeft:
    case MotionRequest::Dive::squatWideArmsBackRight:
      keyframeMotionRequest.keyframeMotion = genuflectStandDefender;
      keyframeMotionRequest.mirror = diveRequest == MotionRequest::Dive::squatWideArmsBackRight;
      break;
    default:
      FAIL("Unknown dive motion.");
  }
  return keyframeMotionRequest;
}

KeyframeMotionRequest KeyframeMotionRequest::fromSpecialRequest(MotionRequest::Special::Request specialRequest)
{
  KeyframeMotionRequest keyframeMotionRequest;
  switch(specialRequest)
  {
    case MotionRequest::Special::demoBannerWave:
      keyframeMotionRequest.keyframeMotion = demoBannerWave;
      break;
    case MotionRequest::Special::demoBannerWaveInitial:
      keyframeMotionRequest.keyframeMotion = demoBannerWaveInitial;
      break;
    default:
      FAIL("Unknown special motion.");
  }
  keyframeMotionRequest.mirror = false;
  return keyframeMotionRequest;
}
