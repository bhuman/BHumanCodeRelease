/**
 * @file MotionEngine.h
 *
 * This file declares a module that instructs other modules to create motions.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Configuration/JointLimits.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/StiffnessData.h"
#include "Representations/MotionControl/ArmKeyFrameGenerator.h"
#include "Representations/MotionControl/ArmMotionInfo.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Representations/MotionControl/DribbleGenerator.h"
#include "Representations/MotionControl/FallGenerator.h"
#include "Representations/MotionControl/GetUpGenerator.h"
#include "Representations/MotionControl/HeadAngleRequest.h"
#include "Representations/MotionControl/HeadMotionGenerator.h"
#include "Representations/MotionControl/HeadMotionInfo.h"
#include "Representations/MotionControl/KeyframeMotionGenerator.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/PointAtGenerator.h"
#include "Representations/MotionControl/ReplayWalkRequestGenerator.h"
#include "Representations/MotionControl/StandGenerator.h"
#include "Representations/MotionControl/WalkAtAbsoluteSpeedGenerator.h"
#include "Representations/MotionControl/WalkAtRelativeSpeedGenerator.h"
#include "Representations/MotionControl/WalkToPoseGenerator.h"
#include "Representations/MotionControl/WalkToBallAndKickGenerator.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/InertialData.h"
#include "Tools/Module/Module.h"
#include "Tools/Motion/MotionGenerator.h"
#include "Tools/Motion/MotionPhase.h"

MODULE(MotionEngine,
{,
  REQUIRES(ArmKeyFrameGenerator),
  REQUIRES(ArmMotionRequest),
  REQUIRES(CognitionFrameInfo),
  REQUIRES(DamageConfigurationBody),
  REQUIRES(DribbleGenerator),
  REQUIRES(FallDownState),
  REQUIRES(FallGenerator),
  REQUIRES(FrameInfo),
  REQUIRES(GetUpGenerator),
  REQUIRES(HeadAngleRequest),
  REQUIRES(HeadMotionGenerator),
  REQUIRES(InertialData),
  REQUIRES(JointAngles),
  REQUIRES(JointLimits),
  REQUIRES(KeyframeMotionGenerator),
  REQUIRES(MotionRequest),
  REQUIRES(PointAtGenerator),
  REQUIRES(ReplayWalkRequestGenerator),
  REQUIRES(StandGenerator),
  REQUIRES(StiffnessSettings),
  REQUIRES(WalkAtAbsoluteSpeedGenerator),
  REQUIRES(WalkAtRelativeSpeedGenerator),
  REQUIRES(WalkToPoseGenerator),
  REQUIRES(WalkToBallAndKickGenerator),
  PROVIDES(JointRequest),
  REQUIRES(JointRequest),
  PROVIDES(ArmMotionInfo),
  PROVIDES(HeadMotionInfo),
  PROVIDES(MotionInfo),
  PROVIDES(OdometryData),
  DEFINES_PARAMETERS(
  {,
    (int)(3000) emergencySitDownDelay, /**< If no new data from Cognition arrived for this duration, the robot sits down. */
  }),
});

class MotionEngine : public MotionEngineBase
{
public:
  /** Constructor */
  MotionEngine();

private:
  /**
   * This method is called when the representation provided needs to be updated.
   * @param jointRequest The representation updated.
   */
  void update(JointRequest& jointRequest);

  /**
   * This method is called when the representation provided needs to be updated.
   * @param motionInfo The representation updated.
   */
  void update(MotionInfo& motionInfo)
  {
    motionInfo = this->motionInfo;
  }

  /**
   * This method is called when the representation provided needs to be updated.
   * @param armMotionInfo The representation updated.
   */
  void update(ArmMotionInfo& armMotionInfo)
  {
    armMotionInfo = this->armMotionInfo;
  }

  /**
   * This method is called when the representation provided needs to be updated.
   * @param headMotionInfo The representation updated.
   */
  void update(HeadMotionInfo& headMotionInfo)
  {
    headMotionInfo = this->headMotionInfo;
  }

  /**
   * This method is called when the representation provided needs to be updated.
   * @param odometryData The representation updated.
   */
  void update(OdometryData& odometryData)
  {
    odometryData = this->odometryData;
  }

  /**
   * Calculates the arm joints for an arm.
   * @param arm The arm for which the joints should be calculated.
   * @param setJoints Whether the arm joints may be set at all for this arm.
   * @param jointRequest The joint request that should be filled (only the arm joints for the given arm may be changed).
   */
  void calcArmJoints(Arms::Arm arm, bool setJoints, JointRequest& jointRequest);

  const MotionGenerator* generators[MotionRequest::numOfMotions] = {nullptr}; /**< The generators for each motion type. */
  std::unique_ptr<MotionPhase> phase; /**< The phase that is currently being executed. */
  MotionInfo motionInfo; /**< The current motion info. */
  ArmMotionInfo armMotionInfo; /**< The current arm motion info. */
  HeadMotionInfo headMotionInfo; /**< The current head motion info. */
  OdometryData odometryData; /**< The accumulated odometry of the robot. */
  MotionGenerator playDeadGenerator; /**< The generator for the \c playDead request. */
  unsigned int lastCognitionTime = 0; /**< The timestamp of the last packet received from Cognition. */
  bool forceSitDown = false; /**< Whether the motion request should be overridden with a sit down. */
};

struct PlayDeadPhase : MotionPhase
{
  explicit PlayDeadPhase(const MotionEngine& engine) :
    MotionPhase(MotionPhase::playDead),
    engine(engine)
  {}

private:
  bool isDone(const MotionRequest& motionRequest) const override;
  void calcJoints(const MotionRequest& motionRequest, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo) override;

  const MotionEngine& engine; /**< A reference to the running motion engine. */
};
