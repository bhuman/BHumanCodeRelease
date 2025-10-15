/**
 * @file RefereeSignalDetector.cpp
 *
 * This file implements a module that detects referee gestures.
 *
 * @author Thomas Röfer
 */

#include "RefereeSignalDetector.h"
#include "Debugging/Debugging.h"
#include "Platform/SystemCall.h"
#include "Platform/Time.h"
#include <algorithm>

MAKE_MODULE(RefereeSignalDetector);

#if defined TARGET_ROBOT && defined NDEBUG
#define DEBUG(code)
#else
#define DEBUG(code) code
#endif

RefereeSignalDetector::RefereeSignalDetector()
{
  history.reserve(bufferSize);
}

void RefereeSignalDetector::update(RefereeSignal& theRefereeSignal)
{
  DEBUG(DECLARE_DEBUG_RESPONSE("debug data:module:RefereeSignalDetector:history"));

  if(!theOptionalCameraImage.image.has_value() || theGameState.state == GameState::beforeHalf || theGameState.state == GameState::timeout )
  {
    history.clear();
    std::fill(histogram.begin(), histogram.end(), 0);
    newSignal = true;
    return;
  }

  lastHistoryUpdate = Time::getCurrentSystemTime();

  updateHistogram(theRefereeGesture.gesture);
  const RefereeSignal::Signal signal = getPredominantSignal();
  if(signal != RefereeSignal::none && (newSignal || signal != theRefereeSignal.signal))
  {
    newSignal = false;
    theRefereeSignal.signal = signal;
    theRefereeSignal.timeWhenDetected = theOptionalCameraImage.image.value().timestamp;
  }

  // Publish history for debugging purposes.
  DEBUG(STREAMABLE(Helper,
        {,
          (std::vector<RefereeGesture::Gesture>) history,
        }) helper;
        helper.history.resize(history.size());
        std::copy(history.begin(), history.end(), helper.history.begin());
        MODIFY("module:RefereeSignalDetector:history", helper));
}

void RefereeSignalDetector::updateHistogram(const RefereeGesture::Gesture gesture)
{
  if(history.size() == history.capacity())
    --histogram[history.back()];
  history.push_front(gesture);
  ++histogram[gesture];
}

RefereeSignal::Signal RefereeSignalDetector::getPredominantSignal() const
{
  std::array<int, RefereeSignal::numOfSignals> histogram = this->histogram;

  // Determine most frequent gesture, ignoring "none".
  const auto predominantGesture = std::max_element(histogram.begin() + 1, histogram.end());

  // Return most frequent gesture if predominant enough, otherwise "none".
  return *predominantGesture < minDetectionRatio * history.capacity() ? RefereeGesture::none
         : static_cast<RefereeGesture::Gesture>(predominantGesture - histogram.begin());
}
