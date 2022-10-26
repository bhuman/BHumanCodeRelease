/**
 * @file RefereeGestureDetection.h
 *
 * This file declares a module that detects referee gestures.
 *
 * @author <a href="mailto:aylu@uni-bremen.de">Ayleen Lührsen</a>
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/RefereePercept/Keypoints.h"
#include "Representations/Perception/RefereePercept/RefereePercept.h"
#include "Streaming/EnumIndexedArray.h"
#include <vector>

/*Exists for Debugging purposes*/
ENUM(RefArmPose,
{,
  none,
  down,
  degree45,
  degree90,
  up,
  forward,
  folded,
  wing,
  spreadwing,
});
STREAMABLE(Rule,
{,
  (Rangea) angleBodyToUpperArm, // Angle between upper arm and a vertical line
  (Rangea) angleUpperToLower, // Angle between upper Arm and lower arm, with the elbow as angle if you want so
  (Rangef) distanceRatio, // distance from sum of upper and lower arm in relation to body
});

MODULE(RefereeGestureDetection,
{,
  REQUIRES(Keypoints),
  REQUIRES(CameraInfo),
  PROVIDES(RefereePercept),
  LOADS_PARAMETERS(
  {,
    (ENUM_INDEXED_ARRAY(Rule, RefArmPose)) constraints, /**Rules that define the pose of the arm*/
    (int) posePortion, /**Size of the buffer is divided by this number to determine the minimum amount of frames a pose needs to be recognized before we apply it to the percept*/
  }),
});

RefArmPose leftArmPose;
RefArmPose rightArmPose;
int leftArmPoseInt = 0;
int rightArmPoseInt = 0;
RingBuffer<RefereePercept::Gesture, 10> poseMemory;
ENUM_INDEXED_ARRAY(int, RefereePercept::Gesture) poseAppearances;

class RefereeGestureDetection : public RefereeGestureDetectionBase
{
  /** current position in the memories of the robot regarding the  **/
  // int memCtr = 0;
  /** to be able to convert a Keypoint to the array index instead of iterating / switch case **/
  ENUM_INDEXED_ARRAY(int, Keypoints::Keypoint) kpToIndex = { -1, -1, -1, -1, -1, 0, 1, 2, 3, 4, 5, -1, -1, -1, -1, -1, -1 };

  /**
   * This method calculates the position that may be assigned to the RefereePercept. It uses the KeypointRepresentation from KeypointProvider, and uses them to calculate a position. It then fills a buffer with them, and in each round the first pose that fills a given portion of the buffer will be selected as the pose.
   * @param theDummyRepresentation The representation updated.
   */
  void update(RefereePercept& theRefereePercept) override;
};
