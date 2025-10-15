/**
 * @file RefereeSignalDetector.h
 *
 * This file declares a module that detects referee signals from recently
 * perceived referee gestures.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Framework/Module.h"
#include "MathBase/RingBuffer.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/RefereeSignal.h"
#include "Representations/Perception/ImagePreprocessing/OptionalCameraImage.h"

MODULE(RefereeSignalDetector,
{,
  REQUIRES(GameState),
  REQUIRES(OptionalCameraImage),
  REQUIRES(RefereeGesture),
  PROVIDES(RefereeSignal),
  LOADS_PARAMETERS(
  {,
    (unsigned) bufferSize, /**< Number of gesture detections buffered for majority vote. */
    (float) minDetectionRatio, /**< Minimum ratio of buffered gestures that make up an accepted gesture. */
  }),
});

class RefereeSignalDetector : public RefereeSignalDetectorBase
{
  RingBuffer<RefereeGesture::Gesture> history; /**< Recently detected gestures. */
  std::array<int, RefereeGesture::numOfGestures> histogram = {}; /**< How often was each gesture detected recently? */
  unsigned lastHistoryUpdate = 0; /**< For updating the gesture history in SimRobot as slow as on the real robot. */
  bool newSignal = true; /**< Is the next signal detected a new one? */

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theRefereeSignal The representation updated.
   */
  void update(RefereeSignal& theRefereeSignal) override;

  /**
   * Update the histogram of the recently detected gestures.
   * @param gesture The latest gesture detected. Can be \c none.
   */
  void updateHistogram(const RefereeGesture::Gesture gesture);

  /**
   * Compute the predominant signal from the histogram of recently detected gestures.
   * @return The predominant signal. Is \c none if no signal is predominant enough.
   */
  RefereeSignal::Signal getPredominantSignal() const;

public:
  /**
   * Constructor.
   * Initializes the gesture history.
   */
  RefereeSignalDetector();
};
