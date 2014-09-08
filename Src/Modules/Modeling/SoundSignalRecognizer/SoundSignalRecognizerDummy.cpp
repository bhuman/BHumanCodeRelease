/*
 * @file SoundSignalRecognizerDummy.cpp
 *
 * Just some debug responses for behavior testing
 */

#include "SoundSignalRecognizerDummy.h"
#include "Tools/Debugging/Debugging.h"

MAKE_MODULE(SoundSignalRecognizerDummy, Modeling)

void SoundSignalRecognizerDummy::update(SoundSignal& soundSignal)
{
  DEBUG_RESPONSE_ONCE("module:SoundSignalRecognizer:pfeif", soundSignal.lastTimeWhistleDetected = theFrameInfo.time;);
  DEBUG_RESPONSE_ONCE("module:SoundSignalRecognizer:trommel", soundSignal.lastTimeAFSKSignalDetected = theFrameInfo.time;);
  DEBUG_RESPONSE_ONCE("module:SoundSignalRecognizer:einfachTon", soundSignal.lastTimeOfIncomingSound = theFrameInfo.time;);
}