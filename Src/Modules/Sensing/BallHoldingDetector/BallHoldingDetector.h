/**
 * @file BallHoldingDetector.h
 *
 * This module implements a detection for a ball stuck between both legs, e.g. after a duel or interception step.
 *
 * @author Harm Thordsen
 * @author Philip Reichenberg
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Modeling/BallLostModel.h"
#include "Representations/Sensing/BallHoldingState.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/RobotModel.h"

MODULE(BallHoldingDetector,
{,
  REQUIRES(BallLostModel),
  REQUIRES(FallDownState),
  REQUIRES(FieldBall),
  REQUIRES(GameState),
  REQUIRES(JointRequest),
  REQUIRES(MassCalibration),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  PROVIDES(BallHoldingState),
  DEFINES_PARAMETERS(
  {,
    (int)(100) minTimeSinceBallWasSeen, /**< Ball is not seen since 100 ms */
    (int)(1500) maxTimeSinceBallWasSeen, /**< Ball is not seen for max 1000 ms. */
    (float)(1000.f) maxDistanceToBall, /**< Ball is close to us */
    (float)(7.f) minLegYDiff, /**< Legs are spread apart by this distance. */
  }),
});

class BallHoldingDetector : public BallHoldingDetectorBase
{
  void update(BallHoldingState& theBallHoldingState) override;

  RingBuffer<float, 3> lastLeftSoleY, lastRightSoleY;

  // Around half a second, as motion has 83.333 frames
  RingBufferWithSum<float, 41> ringBufferLeft;
  RingBufferWithSum<float, 41> ringBufferRight;
};
