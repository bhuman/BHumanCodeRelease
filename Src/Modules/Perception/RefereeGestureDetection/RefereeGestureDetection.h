/**
 * @file RefereeGestureDetection.h
 *
 * This file declares a module that detects referee gestures.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Framework/Module.h"
#include "Math/Range.h"
#include "MathBase/RingBuffer.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Perception/ImagePreprocessing/OptionalCameraImage.h"
#include "Representations/Perception/RefereePercept/Keypoints.h"
#include "Representations/Perception/RefereePercept/RefereePercept.h"

MODULE(RefereeGestureDetection,
{,
  REQUIRES(FieldDimensions),
  REQUIRES(GameState),
  REQUIRES(Keypoints),
  REQUIRES(OptionalCameraImage),
  REQUIRES(RobotPose),
  PROVIDES(RefereePercept),
  LOADS_PARAMETERS(
  {
    STREAMABLE(Constraint,
    {,
      (Keypoints::Keypoint) from, /**< The first keypoint the position of which is checked. */
      (Keypoints::Keypoint) to, /**< The second keypoint the position of which is checked. */
      (Rangef) distance, /**< The distance between the two keypoints must be in this range (in pixels). */
      (Rangea) direction, /**< The direction from \c from to \c to must be in this range (in radians). */
    });

    STREAMABLE(Rule,
    {,
      (RefereePercept::Gesture) gesture, /**< The gesture detected by this rule. */
      (std::vector<Constraint>) constraints, /**< The constraints that must be satisfied to detect the gesture. */
    }),

    (unsigned) bufferSize, /**< Number of gesture detections buffered for majority vote. */
    (float) minDetectionRatio, /**< Minimum ratio of buffered gestures that make up an accepted gesture. */
    (bool) mustCrossMiddle, /**< Must there be keypoints in both halves of the image to accept a gesture? */
    (float) yThreshold, /**< At least one point must be below this threshold. */
    (std::vector<Rule>) rules, /**< The rules made up of constraints that must be satisfied to detect a gesture. */
  }),
});

class RefereeGestureDetection : public RefereeGestureDetectionBase
{
  RingBuffer<RefereePercept::Gesture> history; /**< Recently detected gestures. */
  std::array<int, RefereePercept::numOfGestures> histogram = {}; /**< How often was each gesture detected recently? */
  unsigned lastHistoryUpdate = 0; /**< For updating the gesture history in SimRobot as slow as on the real robot. */

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theRefereePercept The representation updated.
   */
  void update(RefereePercept& theRefereePercept) override;

  /**
   * Checks whether keypoints exist on both sides of the image.
   * The referee should stand in the middle of the image. Therefore, there must be
   * points on both sides.
   * @return Could this detection be the referee?
   */
  bool crossesMiddle() const;

  /**
   * Checks whether at least one point is below the horizon.
   * This should exclude people on grand stands.
   * @return At least one point below horizon?
   */
  bool pointsLowEnough() const;

  /**
   * Get a keypoint returned by the network. The order to its left/right counterpart
   * is maintained under the assumption that the referee faces this robot.
   * @param keypoint The keypoint from the perspective of the observed referee.
   * @return The network output for the keypoint.
   */
  const Keypoints::Point& getOrdered(const Keypoints::Keypoint keypoint) const;

  /**
   * Detect a gesture based on the network output.
   * @return The keypoint detected. Can be \c none.
   */
  RefereePercept::Gesture detectGesture() const;

  /**
   * Update the histogram of the recently detected gestures.
   * @param gesture The latest gesture detected. Can be \c none.
   */
  void updateHistogram(const RefereePercept::Gesture gesture);

  /**
   * Compute the predominant gesture from the histogram of recently detected gestures.
   * @return The predominant gesture. Is \c none if no gesture is predominant enough.
   */
  RefereePercept::Gesture getPredominantGesture() const;

public:
  /**
   * Constructor.
   * Initializes the gesture history.
   */
  RefereeGestureDetection();
};
