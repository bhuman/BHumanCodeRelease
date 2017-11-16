/**
 * @file Representations/BehaviorControl/SPLStandardBehaviorStatus.h
 * The file declares a struct that contains those parts of the current behavior state
 * that are officially part of the SPLStandardMessage
 *
 * @author Tim Laue
 */

#pragma once
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Representations/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"
#include <cstdint>

/**
 * @struct SPLStandardBehaviorStatus
 * A struct that contains standard data about the current behavior state.
 */
STREAMABLE(SPLStandardBehaviorStatus, COMMA public BHumanMessageParticle<idSPLStandardBehaviorStatus>
{
  /** BHumanMessageParticle functions */
  void operator >> (BHumanMessage& m) const override
  {
    m.theBSPLStandardMessage.walkingTo[0] = walkingTo.x();
    m.theBSPLStandardMessage.walkingTo[1] = walkingTo.y();
    m.theBSPLStandardMessage.shootingTo[0] = shootingTo.x();
    m.theBSPLStandardMessage.shootingTo[1] = shootingTo.y();
    m.theBSPLStandardMessage.intention = intention;
    m.theBSPLStandardMessage.averageWalkSpeed = averageWalkSpeed;
    m.theBSPLStandardMessage.maxKickDistance = maxKickDistance;
  }
  void operator << (const BHumanMessage& m) override
  {
    walkingTo = Vector2f(m.theBSPLStandardMessage.walkingTo[0], m.theBSPLStandardMessage.walkingTo[1]);
    shootingTo = Vector2f(m.theBSPLStandardMessage.shootingTo[0], m.theBSPLStandardMessage.shootingTo[1]);
    intention = m.theBSPLStandardMessage.intention;
    averageWalkSpeed = m.theBSPLStandardMessage.averageWalkSpeed;
    maxKickDistance = m.theBSPLStandardMessage.maxKickDistance;
  },

  (Vector2f)(Vector2f::Zero()) walkingTo,
  (Vector2f)(Vector2f::Zero()) shootingTo,
  (int8_t)(0) intention,
  (int16_t)(220) averageWalkSpeed,
  (int16_t)(7000) maxKickDistance,
});
