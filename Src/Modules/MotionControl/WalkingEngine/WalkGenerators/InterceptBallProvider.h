/**
 * @file InterceptBallProvider.h
 *
 * This file declares a module that ...
 *
 * @author Florian Scholz
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/KickInfo.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/MotionControl/InterceptBallGenerator.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Representations/MotionControl/WalkKickGenerator.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Sensing/TorsoMatrix.h"

MODULE(InterceptBallProvider,
{,
  REQUIRES(BallSpecification),
  REQUIRES(FieldBall),
  REQUIRES(OdometryDataPreview),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  REQUIRES(RobotPose),
  REQUIRES(KickInfo),
  REQUIRES(TorsoMatrix),
  REQUIRES(WalkGenerator),
  REQUIRES(WalkKickGenerator),
  PROVIDES(InterceptBallGenerator),

  LOADS_PARAMETERS(
  {,
    (float) timeOffset,
    (float) footRectLengthOffset,
    (float) footRectYOffset,
  }),
});

class InterceptBallProvider : public InterceptBallProviderBase
{
  /**
   * This method is called when the representation provided needs to be updated.
   * @param theInterceptBallGenerator The representation updated.
   */
  void update(InterceptBallGenerator& theInterceptBallGenerator) override;

  /**
   * This method is used to check the requested KickType
   * @param kickType The requsted kicktype
   * @return Wheather the requested KickType is allowed
   */
  bool checkKickType(KickInfo::KickType kickType);

  /**
   * This method clips a provided pose such that the WalkingEngined doesn't complain
   * @param pose Pose to be clipped
   * @param isLeftPhase If the current phase is a left phase
   * @param lastPhase The last motion phase
   * @param motionRequest The current motion request
   */
  void clipPose(Pose2f& pose, bool isLeftPhase, const MotionPhase& lastPhase, const MotionRequest& motionRequest);

  Geometry::Rect leftFootRect; /**< The rectangle in front of the left foot */
  Geometry::Rect rightFootRect; /**< The rectangle in front of the right foot */

  Vector2f leftFootBackRight; /**< The back right point of the left foot rectangle */
  Vector2f leftFootFrontLeft; /**< The fron left point of the left foot rectangle */

  Vector2f rightFootBackRight; /**< The right back point of the right foot rectangle */
  Vector2f rightFootFrontLeft; /**< The fron left point of the right foot rectangle */
};
