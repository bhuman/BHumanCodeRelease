/**
 * @file InterceptBallInformationProvider.h
 *
 * Provides information about intercepting the ball for the Behavior
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/FieldInterceptBall.h"
#include "Representations/BehaviorControl/InterceptBallInformation.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Infrastructure/EmergencyButtonState.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Framework/Module.h"

MODULE(InterceptBallInformationProvider,
{,
  REQUIRES(BallModel),
  REQUIRES(EmergencyButtonState),
  REQUIRES(FieldBall),
  REQUIRES(FieldInterceptBall),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(RobotPose),
  REQUIRES(TeamData),
  PROVIDES(InterceptBallInformation),
  DEFINES_PARAMETERS(
  {,
    (float)(2000.f) rememberPassTime,
    (Angle)(20_deg) maxBallRollAngleDiff,
    (Angle)(90_deg) goalBallAngleDiff,
    (Angle)(20_deg) goalBallAngleHysteresis,
    (float)(700.f) minBallDistanceForWalkOut,
  }),
});

class InterceptBallInformationProvider : public InterceptBallInformationProviderBase
{
  void update(InterceptBallInformation& theInterceptBallInformation) override;

  Angle passAngle = 0_deg;
  Vector2f oldTeammateStart = Vector2f::Zero();
  Vector2f oldTeammateTarget = Vector2f::Zero();
  unsigned lastPassToOtherTeammatePlanned = 0;
  unsigned lastFrameInfo = 0;
};
