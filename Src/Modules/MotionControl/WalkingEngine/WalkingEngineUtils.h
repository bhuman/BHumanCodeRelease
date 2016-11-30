/**
 * @file WalkingEngineUtils.h
 *
 * @author Colin Graf
 * @author Alexis Tsogias
 */

#pragma once

#include "Representations/MotionControl/WalkKicks.h"
#include "Tools/Motion/LIP3D.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Math/Pose3f.h"

namespace WalkingEngineUtils
{
  /**
   * A parameter set to separate a phase in three arbitrary parts.
   * Would be part of LOADS_PARAMETERS if Microsoft's compiler had
   * allowed it.
   */
  STREAMABLE(SubPhaseParameters,
  {,
    (float)(0) start, /**< The start position of the second sub phase */
    (float)(1) duration, /**< The length of the second sub phase */
  });

  /**
   * A description of the posture of the legs
   */
  STREAMABLE(LegPosture,
  {,
    (Pose3f) leftOriginToFoot, /**< The position of the left foot */
    (Pose3f) rightOriginToFoot, /**< The position of the right foot */
    (Vector3f) leftOriginToCom, /**< The position of the center of mass relative to the origin that was used to describe the position of the left foot */
    (Vector3f) rightOriginToCom, /**< The position of the center of mass relative to the origin that was used to describe the position of the right foot */
  });

  /**
   * A description of the posture of the head and the arms
   */
  struct ArmPosture
  {
    float leftArmJointAngles[4]; /**< left arm joint angles */
    float rightArmJointAngles[4]; /**< right arm joint angles */
  };

  /**
   * A description of the posture of the whole body
   */
  class Posture : public LegPosture, public ArmPosture {};

  /**
   * The size of a single step
   */
  struct StepSize
  {
    Vector3f translation = Vector3f::Zero(); /**< The translational component */
    float rotation = 0.f; /**< The rotational component */

    StepSize() = default;
    StepSize(float rotation, float x, float y) : translation(x, y, 0.f), rotation(rotation) {}
  };

  ENUM(MotionType,
  {,
    stand,
    stepping,
  });

  ENUM(PhaseType,
  {,
    standPhase,
    leftSupportPhase,
    rightSupportPhase,
  });

  struct PendulumPhase
  {
    unsigned id = 0; /**< A phase descriptor */
    LIP3D com;
    Vector2f r = Vector2f::Zero(); /**< The pendulum pivot point  (in Q) used to compute the pendulum motion */
    Vector2f rOpt = Vector2f::Zero(); /**< The initially planned target position of the pendulum pivot point */
    Vector2f rRef = Vector2f::Zero(); /**< The target position of the pendulum pivot point that has been adjusted to achieve the step size */

    PhaseType type = PhaseType::standPhase; /**< What kind of phase is this? */
    float td = 0.f; /**< The time in seconds left till the next pendulum phase */
    float tu = 0.f; /**< The time in seconds passed since the beginning of the pendulum phase */
    StepSize stepSize; /**< The step size used to reach the pendulum pivot point */
    Vector3f lift = Vector3f::Zero(); /**< The height the foot of the swinging foot was lifted to implement the step size */
    Vector3f liftRotation = Vector3f::Zero(); /**< A rotation applied to the foot while lifting it */
    Vector3f comLift = Vector3f::Zero(); /**< An offset added to the desired center of mass position while lifting the foot */
    bool toStand = false; /**< Whether the next phase will be a standPhase */
    bool fromStand = false; /**< Whether the previous phase was a standPhase */
    WalkKicks::Type kickType = WalkKicks::Type::none; /**< The type of kick executed during the phase */

    PendulumPhase(const Array2f& LIPHeights) : com(LIPHeights) {}
  };
}
