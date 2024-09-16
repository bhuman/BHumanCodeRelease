/**
 * @file ClearTargetProvider.h
 *
 * This file declares a module that calculates the ball position after the execution of the ClearBall-Skill.
 *
 * @author Nico Holsten
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/BehaviorControl/ClearTarget.h"
#include "Representations/BehaviorControl/ExpectedGoals.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/FieldInterceptBall.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/GlobalOpponentsModel.h"
#include "Representations/Modeling/GlobalTeammatesModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/SectorWheel.h"

MODULE(ClearTargetProvider,
{,
  REQUIRES(BallSpecification),
  REQUIRES(ExpectedGoals),
  REQUIRES(FieldBall),
  REQUIRES(FieldInterceptBall),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(KickInfo),
  REQUIRES(ObstacleModel),
  REQUIRES(RobotPose),
  REQUIRES(GlobalTeammatesModel),
  REQUIRES(GlobalOpponentsModel),
  PROVIDES(ClearTarget),
  DEFINES_PARAMETERS(
  {,
    (float)(0.01f) hysteresisSector, /**< Bonus for last sector */
    (float)(0.01f) hysteresisShotType, /**< Bonus for last shot */
    (float)(0.1f) teammateSummand, /**< Summand for teammate or obstacle next to current sector */
    (float)(100.f) ballGoalPostTangentOffset, /**< This amount of space should fit between the goal post and the ball. */
    (Angle)(15_deg) smallSectorRange, /* We divide big sectors in smaller sectors of this size. */
    (Angle)(10_deg) smallSector, /* We don't pass into a sector which has a range <= this size. */
    (Angle)(30_deg) largeSector, /* A sector with a range larger than this will be divided into smaller ones */
    (float)(500.f) minDistanceToXLine,  /* Minimum target distance to XGoalLine */
    (float)(1000.f) minDistanceToYLine, /* Minimum target distance to YFieldBorderLine */
    //@todo add useful shot types
    (std::vector<KickInfo::KickType>)({KickInfo::walkForwardsLeftLong, KickInfo::walkForwardsRightLong, KickInfo::walkForwardsRightAlternative,
                                       KickInfo::walkForwardsLeftAlternative}) availableKicks, /**< The kicks that may be selected. TODO use commented kicks during set pieces */
    (std::vector<KickInfo::KickType>)({KickInfo::forwardFastRightLong, KickInfo::forwardFastLeftLong, KickInfo::forwardFastLeftPass, KickInfo::forwardFastRightPass}) extraKicksSetPieces,  /**< The kicks that may be add during setPieces.*/
    (KickInfo::KickType)(KickInfo::numOfKickTypes) lastKickType, /**< TODO (Also, why is this a parameter?) */
    (SectorWheel::Sector)({}) lastSector, /**< The sector that has been selected in the previous frame. */
  }),
});

class ClearTargetProvider : public ClearTargetProviderBase
{
public:
  void update(ClearTarget& theClearTarget) override;

private:
  /**
   * TODO
   */
  void calcBestKick();
  void createSmallSectors(Angle& var, Angle end, std::list<SectorWheel::Sector>& newSmallSectors, SectorWheel::Sector& newSmallSector);

  /**
   * TODO
   */
  float getAngleRating(const Angle candidateAngle, const Vector2f basePosition, const float targetDistance, int number = 0);

  /**
   * TODO
   */
  bool isNotNearLine(const Vector2f targetPosition);

  /**
   * TODO
   */
  bool isTeammateCloseToBall(const Angle candidateAngle, const Vector2f basePosition, const float targetDistance);

  KickInfo::KickType bestKick = KickInfo::numOfKickTypes; /**< The kick with which the ball should be cleared */
  SectorWheel::Sector bestSector; /**< The sector the ball should be cleared into */
  Pose2f bestKickPoseRelative; /**< The best pose of the bestKick */
  unsigned timeWhenBestKickWasUpdated = 0; /**< The last time when bestKick was computed. */
};
