/**
 * @file Goalkeeper.h
 *
 * This file declares the goalkeeper role.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Tools/BehaviorControl/Strategy/PositionRole.h"

class Goalkeeper : public PositionRole
{
  Pose2f position(Side side, const Pose2f& basePose, const std::vector<Vector2f>& baseArea, const Agents& teammates) override;

  Pose2f tolerance() const override;

  bool shouldStop(const Pose2f& target) const override;

  /**
   * Interpolate thresholds based on the distance to the ball
   */
  const Rangef ballDistanceInterpolationRange = Rangef(2500.f, 6500.f);
  /**
   * Interpolate rotation threshold between these values
   */
  const Rangea rotationThresholdRange = Rangea(8_deg, 15_deg);
  /**
   * Interpolate translation threshold between these values
   */
  const Rangef translationXThresholdRange = Rangef(200.f, 300.f);
  /**
   * Interpolate translation threshold between these values
   */
  const Rangef translationYThresholdRange = Rangef(150.f, 250.f);
  /**
   * Stop walking if orientation to the target is less than this value
   */
  const Angle shouldStopRotation = 3_deg;
  /**
   * Stop walking is translation to the target is less than this value
   */
  const Vector2f shouldStopTranslation = Vector2f(100.f, 50.f);
};
