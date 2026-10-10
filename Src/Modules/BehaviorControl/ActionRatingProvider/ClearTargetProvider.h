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
#include "Representations/BehaviorControl/IndirectKick.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
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
  REQUIRES(FieldDimensions),
  REQUIRES(GameState),
  REQUIRES(IndirectKick),
  REQUIRES(KickInfo),
  REQUIRES(ObstacleModel),
  REQUIRES(RobotPose),
  REQUIRES(GlobalTeammatesModel),
  REQUIRES(GlobalOpponentsModel),
  PROVIDES(ClearTarget),
  LOADS_PARAMETERS(
  {,
    (float) hysteresisSector, /**< Bonus for last sector */
    (float) hysteresisShotType, /**< Bonus for last shot */
    (float) teammateSummand, /**< Summand for teammate or obstacle next to current sector */
    (float) ballGoalPostTangentOffset, /**< This amount of space should fit between the goal post and the ball. */
    (Angle) smallSectorRange, /* We divide big sectors in smaller sectors of this size. */
    (Angle) smallSector, /* We don't pass into a sector which has a range <= this size. */
    (Angle) largeSector, /* A sector with a range larger than this will be divided into smaller ones */
    (float) minDistanceToXLine,  /* Minimum target distance to XGoalLine */
    (float) minDistanceToYLine, /* Minimum target distance to YFieldBorderLine */
    (Rangef) minKickDistance, /**< Kick must be minimum of this distance. */
    (Rangea) scaleKickDistanceAngleRange, /**< Reduce min kick strength based on kick angle towards own goal. */
    (float) kickDistanceStepSize, /**< Kick length search is done in this step size. */
    (float) additionalObstacleWidth, /**< Extra distance to obstacles. */
    (Rangef) lastKickRatingBonus, /**< Increase bonus rating for last kick direction if close to the ball. These are the ratings. */
    (Rangef) ballDistanceRatingInterpolation, /**< Increase bonus rating for last kick direction if close to the ball. These are the ball distances. */
    (Rangea) kickDirectionHystersisRange, /**< Interpolate bonus rating based on how close to the last best kick direction the current kick is. */
    //@todo add useful shot types
    (std::vector<KickInfo::KickType>) availableKicks, /**< The kicks that may be selected. TODO use commented kicks during set pieces */
    (std::vector<KickInfo::KickType>) extraKicksSetPieces,  /**< The kicks that may be add during setPieces.*/
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
  void calcBestKick(ClearTarget& theClearTarget);
  void createSmallSectors(Angle& var, Angle end, std::list<SectorWheel::Sector>& newSmallSectors, SectorWheel::Sector& newSmallSector);

  /**
   * Get rating for kick.
   *
   * @param candidateAngle Kick direction in field coordinates.
   * @param basePosition Start position in field coordinates. Normally it is the ball.
   * @param targetDistance Kick length.
   * @param number Text for Drawings.
   */
  float getAngleRating(const Angle candidateAngle, const Vector2f basePosition, const float targetDistance, int number = 0);

  /**
   * Check whether the end position is near the field border.
   * @param targetPosition Field position to check.
   */
  bool isNotNearLine(const Vector2f targetPosition);

  /**
   * Checks if a potential kick could be considered as a pass
   * @param candidateAngle Kick direction in field coordinates.
   * @param basePosition Start position in field coordinates. Normally it is the ball.
   * @param targetDistance Kick length.
   */
  bool isTeammateCloseToBall(const Angle candidateAngle, const Vector2f basePosition, const float targetDistance);

  KickInfo::KickType bestKick = KickInfo::numOfKickTypes; /**< The kick with which the ball should be cleared */
  KickInfo::KickType lastKickType = KickInfo::numOfKickTypes;
  float bestKickLength = 0.f;
  float lastKickLength = 20000.f;
  SectorWheel::Sector lastSector = {};
  SectorWheel::Sector bestSector; /**< The sector the ball should be cleared into */
  Angle bestKickDirection = 0_deg; /**< Best kick direction in global coordinates. */
  Angle lastBestKickDirection = 0_deg; /**< Last best kick direction in global coordinates. */
};
