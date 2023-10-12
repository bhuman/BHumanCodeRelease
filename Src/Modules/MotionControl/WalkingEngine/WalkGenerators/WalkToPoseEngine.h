/**
 * @file WalkToPoseEngine.h
 *
 * This file declares a module that provides a walk to pose generator.
 *
 * @author Arne Hasselbring
 * @author Philip Reichenberg
 */

#pragma once

#include "Representations/BehaviorControl/Libraries/LibDemo.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/WalkToPoseGenerator.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Framework/Module.h"

MODULE(WalkToPoseEngine,
{,
  REQUIRES(OdometryDataPreview),
  REQUIRES(LibDemo),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  REQUIRES(TorsoMatrix),
  REQUIRES(WalkGenerator),
  REQUIRES(WalkingEngineOutput),
  PROVIDES(WalkToPoseGenerator),
  DEFINES_PARAMETERS(
  {,
    (Angle)(0.3f) controlAheadAngle, /**< The angular part of the arc by which the target is shifted when turning around a circle. */
    (float)(200.f) startTurningBeforeCircleDistance, /**< The distance to the tangent point below which the robot wants to walk on the arc. */
    (float)(0.75f) reduceForwardStepUpperThreshold, /**< If the next forward step size exceeds this much of the forward max speed (in %),
                                                    then slow down. */
    (float)(0.25f) reduceForwardStepLowerThreshold, /**< If after the next step the forward step distance is below this forward step size
                                                    relative to the forward max speed (in %), then slow down. */
    (float)(230.f) useReferenceMaxForwardSpeed,
  }),
});

class WalkToPoseEngine : public WalkToPoseEngineBase
{
  void update(WalkToPoseGenerator& walkToPoseGenerator) override;

  /**
   * Creates a walk phase to get to a target.
   * @param targetInSCS The target pose in the next support coordinate system.
   * @param obstacleAvoidanceInSCS The obstacle avoidance parameters in the next support coordinate system.
   * @param walkSpeed The walk speed ratio.
   * @param isLeftPhase Whether the next phase is a left-swing phase.
   * @param keepTargetRotation Whether the engine must reach the target rotation as soon as possible.
   * @param lastPhase The previous motion phase.
   * @param targetOfInterest If filled, the first entry is used to decide how the body shall be orientated while walking, so the camera can see this target at all times. Target is in relative coordinates.
   * @param sideWalkAllowed If true, no forced 90deg sidewalk will be made
   * @param forceSideWalk If true, force sidewalking
   * @param doNotForceDiagonalWalk If true, the targetOfInterest does not force side/diagonal walk
   * @return The walk phase to the target.
   */
  std::unique_ptr<MotionPhase> createPhase(const Pose2f& targetInSCS, const MotionRequest::ObstacleAvoidance& obstacleAvoidanceInSCS,
                                           const Pose2f& walkSpeed, bool isLeftPhase, bool keepTargetRotation,
                                           const MotionPhase& lastPhase, const bool isFastWalkAllowed,
                                           const std::optional<Vector2f>& targetOfInterest,
                                           const bool forceSideWalk, const bool isSideWalkAllowed, const bool useModTarget) const;
};
