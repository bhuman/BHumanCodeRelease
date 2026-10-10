/**
 * @file BallKickInformationProvider.h
 *
 * For the behavior it is important to know when the ball was kicked or touched.
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Modeling/BallKickInformation.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Framework/Module.h"

MODULE(BallKickInformationProvider,
{,
  REQUIRES(BallSpecification),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(MotionRequest),
  REQUIRES(OdometryDataPreview),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  REQUIRES(TorsoMatrix),
  PROVIDES(BallKickInformation),
  DEFINES_PARAMETERS(
  {,
    (float)(500.f) minBallDistance, /**< Ball must be this close to actually start checking for potential collisions. */
    (float)(400.f) ballDistanceAfterKick, /**< Ball must be this far away to accept it was kicked. */
    (float)(40.f) detectionRadius, /**< Ball must be this close to a checked point to accept a kick. Gets overwritten with the largest distance between two neighbor points. */
    (int)(1500) ballSeenThreshold, /**< Ball must be seen to detect kick. */
  }),
});

class BallKickInformationProvider : public BallKickInformationProviderBase
{
public:
  BallKickInformationProvider();
private:

  bool ballWasTouched = false;
  std::vector<Vector3f> soleOffsets;
  Vector2f ballRelativeWhenClose = Vector2f::Zero();
  Pose2f odometryWhenClose;

  /**
   * Add some comment here
   */
  void update(BallKickInformation& theBallKickInformation);
};
