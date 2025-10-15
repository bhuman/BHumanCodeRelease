/**
 * @file WalkSpeedConversion.h
 *
 * This file defines some functions to convert a requested speed type into walk speed ratios.
 *
 * @author Philip Reichenberg
 */

#pragma once
#include "Math/Angle.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Tools/Motion/ReduceWalkSpeedType.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Math/BHMath.h"
#include "Tools/BehaviorControl/KickSelection.h"

namespace WalkSpeedConversion
{
  constexpr float bufferTime = 3000.f; /**< Buffer time in ms to ensure a safety margin when reaching the target. */
  const Rangef ballDistanceSpeedReductionRange(500.f, 1000.f); /**< Reduce walking speed based on distance to ball. */
  const Rangef targetDistanceSpeedReductionRange(250.f, 750.f); /**< Reduce walking speed based on distance to target. */

  inline Pose2f convertSpeedRatio(ReduceWalkSpeedType::ReduceWalkSpeedType reduceWalkSpeedType, const Pose2f& speed, const Pose2f& target,
                                  const FrameInfo& theFrameInfo, const GameState& theGameState,
                                  const FieldBall& theFieldBall, const WalkingEngineOutput& theWalkingEngineOutput)
  {
    Pose2f walkingSpeedRatio = speed;
    const float targetDistance = target.translation.norm();

    switch(reduceWalkSpeedType)
    {
      case ReduceWalkSpeedType::slow:
      {
        const float ballDistance = theFieldBall.recentBallEndPositionRelative().norm();
        walkingSpeedRatio.rotation = mapToRange(ballDistance, ballDistanceSpeedReductionRange.min, ballDistanceSpeedReductionRange.max, 1.f, theWalkingEngineOutput.energyEfficientWalkSpeed.rotation / theWalkingEngineOutput.maxSpeed.rotation);
        walkingSpeedRatio.translation.x() = mapToRange(ballDistance, ballDistanceSpeedReductionRange.min, ballDistanceSpeedReductionRange.max, 1.f, theWalkingEngineOutput.energyEfficientWalkSpeed.translation.x() / theWalkingEngineOutput.maxSpeed.translation.x());
        walkingSpeedRatio.translation.y() = mapToRange(ballDistance, ballDistanceSpeedReductionRange.min, ballDistanceSpeedReductionRange.max, 1.f, theWalkingEngineOutput.energyEfficientWalkSpeed.translation.y() / theWalkingEngineOutput.maxSpeed.translation.y());
        break;
      }
      case ReduceWalkSpeedType::normal:
      {
        walkingSpeedRatio.rotation = std::min(speed.rotation, Angle(Rangef::ZeroOneRange().limit(theWalkingEngineOutput.noEfficientWalkSpeed.rotation / theWalkingEngineOutput.maxSpeed.rotation)));
        walkingSpeedRatio.translation.x() = std::min(speed.translation.x(), Rangef::ZeroOneRange().limit(theWalkingEngineOutput.noEfficientWalkSpeed.translation.x() / theWalkingEngineOutput.maxSpeed.translation.x()));
        walkingSpeedRatio.translation.y() = std::min(speed.translation.y(), Rangef::ZeroOneRange().limit(theWalkingEngineOutput.noEfficientWalkSpeed.translation.y() / theWalkingEngineOutput.maxSpeed.translation.y()));
        break;
      }
      case ReduceWalkSpeedType::timeBased:
      {
        ASSERT(theGameState.timeWhenStateEnds != 0);

        // Time to reach the target
        const float ttrpSlow = KickSelection::calcTTRP(target, Pose2f(theWalkingEngineOutput.energyEfficientWalkSpeed.rotation, theWalkingEngineOutput.energyEfficientWalkSpeed.translation.x(), theWalkingEngineOutput.energyEfficientWalkSpeed.translation.y()));
        const float ttrpNormal = KickSelection::calcTTRP(target, Pose2f(theWalkingEngineOutput.noEfficientWalkSpeed.rotation, theWalkingEngineOutput.noEfficientWalkSpeed.translation.x(), theWalkingEngineOutput.noEfficientWalkSpeed.translation.y()));

        // Time left in current state
        const float timeLeft = std::max(-theFrameInfo.getTimeSince(theGameState.timeWhenStateEnds) - bufferTime, 0.f);

        // Calculate scale factor based on time left vs ttrp
        walkingSpeedRatio.rotation = mapToRange(timeLeft, ttrpNormal, ttrpSlow,
                                                theWalkingEngineOutput.noEfficientWalkSpeed.rotation / theWalkingEngineOutput.maxSpeed.rotation,
                                                theWalkingEngineOutput.energyEfficientWalkSpeed.rotation / theWalkingEngineOutput.maxSpeed.rotation);
        walkingSpeedRatio.translation.x() = mapToRange(timeLeft, ttrpNormal, ttrpSlow,
                                                       theWalkingEngineOutput.noEfficientWalkSpeed.translation.x() / theWalkingEngineOutput.maxSpeed.translation.x(),
                                                       theWalkingEngineOutput.energyEfficientWalkSpeed.translation.x() / theWalkingEngineOutput.maxSpeed.translation.x());
        walkingSpeedRatio.translation.y() = mapToRange(timeLeft, ttrpNormal, ttrpSlow,
                                                       theWalkingEngineOutput.noEfficientWalkSpeed.translation.y() / theWalkingEngineOutput.maxSpeed.translation.y(),
                                                       theWalkingEngineOutput.energyEfficientWalkSpeed.translation.y() / theWalkingEngineOutput.maxSpeed.translation.y());
        break;
      }
      case ReduceWalkSpeedType::distanceBased:
      {
        walkingSpeedRatio.rotation = mapToRange(targetDistance, targetDistanceSpeedReductionRange.min, targetDistanceSpeedReductionRange.max,
                                                theWalkingEngineOutput.energyEfficientWalkSpeed.rotation / theWalkingEngineOutput.maxSpeed.rotation,
                                                theWalkingEngineOutput.noEfficientWalkSpeed.rotation / theWalkingEngineOutput.maxSpeed.rotation);
        walkingSpeedRatio.translation.x() = mapToRange(targetDistance, targetDistanceSpeedReductionRange.min, targetDistanceSpeedReductionRange.max,
                                                       theWalkingEngineOutput.energyEfficientWalkSpeed.translation.x() / theWalkingEngineOutput.maxSpeed.translation.x(),
                                                       theWalkingEngineOutput.noEfficientWalkSpeed.translation.x() / theWalkingEngineOutput.maxSpeed.translation.x());
        walkingSpeedRatio.translation.y() = mapToRange(targetDistance, targetDistanceSpeedReductionRange.min, targetDistanceSpeedReductionRange.max,
                                                       theWalkingEngineOutput.energyEfficientWalkSpeed.translation.y() / theWalkingEngineOutput.maxSpeed.translation.y(),
                                                       theWalkingEngineOutput.noEfficientWalkSpeed.translation.y() / theWalkingEngineOutput.maxSpeed.translation.y());
        break;
      }
      case ReduceWalkSpeedType::noChange:
      default:
        break;
    }
    walkingSpeedRatio.rotation = Rangef::ZeroOneRange().limit(std::min(speed.rotation, walkingSpeedRatio.rotation));
    walkingSpeedRatio.translation.x() = Rangef::ZeroOneRange().limit(std::min(speed.translation.x(), walkingSpeedRatio.translation.x()));
    walkingSpeedRatio.translation.y() = Rangef::ZeroOneRange().limit(std::min(speed.translation.y(), walkingSpeedRatio.translation.y()));
    return walkingSpeedRatio;
  }
}
