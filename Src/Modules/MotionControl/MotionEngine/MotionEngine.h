/**
 * @file MotionEngine.h
 *
 * This file declares a module that instructs other modules to create motions.
 *
 * This is done with the help of so called MotionPhases.
 * In each update execution, the following steps are done:
 * 1. update the current MotionPhase
 * 2.1 check if a FallPhase should start or if the current MotionPhase is done
 * 2.2 create a new MotionPhase based on the MotionRequest
 * 2.3 create a new MotionPhase based on the previous one
 * 2.4 decide which one to use (follow up MotionPhases are prioritized)
 * 3. calculate head angles (if current MotionPhase allows it)
 * 4. calculate the current arm joint angles (if current MotionPhase allows it)
 * 5. calculate the current (other) joint angles
 * 6. some extra handling
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Representations/MotionControl/PhotoModeGenerator.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Configuration/JointLimits.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/StiffnessData.h"
#include "Representations/MotionControl/ArmKeyFrameGenerator.h"
#include "Representations/MotionControl/ArmMotionInfo.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Representations/MotionControl/DiveGenerator.h"
#include "Representations/MotionControl/DribbleGenerator.h"
#include "Representations/MotionControl/FallGenerator.h"
#include "Representations/MotionControl/FreezeGenerator.h"
#include "Representations/MotionControl/GetUpGenerator.h"
#include "Representations/MotionControl/HeadMotionGenerator.h"
#include "Representations/MotionControl/HeadMotionInfo.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/InterceptBallGenerator.h"
#include "Representations/MotionControl/KeyframeMotionGenerator.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/PointAtGenerator.h"
#include "Representations/MotionControl/ReplayWalkRequestGenerator.h"
#include "Representations/MotionControl/SpecialGenerator.h"
#include "Representations/MotionControl/StandGenerator.h"
#include "Representations/MotionControl/WalkAtAbsoluteSpeedGenerator.h"
#include "Representations/MotionControl/WalkAtRelativeSpeedGenerator.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Representations/MotionControl/WalkOutOfBallDirection.h"
#include "Representations/MotionControl/WalkToPoseGenerator.h"
#include "Representations/MotionControl/WalkToBallAndKickGenerator.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GyroOffset.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/JointPlay.h"
#include "Framework/Module.h"
#include "Tools/Motion/MotionGenerator.h"
#include "Tools/Motion/MotionPhase.h"

struct UpperFrameInfo;
struct LowerFrameInfo;

MODULE(MotionEngine,
{,
  REQUIRES(ArmKeyFrameGenerator),
  REQUIRES(ArmMotionRequest),
  REQUIRES(PhotoModeGenerator),
  REQUIRES(CognitionFrameInfo),
  REQUIRES(UpperFrameInfo),
  REQUIRES(LowerFrameInfo),
  REQUIRES(DamageConfigurationBody),
  REQUIRES(DiveGenerator),
  REQUIRES(DribbleGenerator),
  REQUIRES(FallDownState),
  REQUIRES(FallGenerator),
  REQUIRES(FrameInfo),
  REQUIRES(FreezeGenerator),
  REQUIRES(GetUpGenerator),
  REQUIRES(GyroOffset),
  REQUIRES(HeadMotionGenerator),
  REQUIRES(HeadMotionRequest),
  REQUIRES(InertialData),
  REQUIRES(InterceptBallGenerator),
  REQUIRES(JointAngles),
  REQUIRES(JointLimits),
  REQUIRES(JointPlay),
  REQUIRES(KeyframeMotionGenerator),
  REQUIRES(MotionRequest),
  REQUIRES(OdometryDataPreview),
  REQUIRES(OdometryTranslationRequest),
  REQUIRES(PointAtGenerator),
  REQUIRES(ReplayWalkRequestGenerator),
  REQUIRES(SpecialGenerator),
  REQUIRES(StandGenerator),
  REQUIRES(StiffnessSettings),
  REQUIRES(WalkAtAbsoluteSpeedGenerator),
  REQUIRES(WalkAtRelativeSpeedGenerator),
  REQUIRES(WalkGenerator),
  REQUIRES(WalkOutOfBallDirection),
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
    (int)(100) brokenJointAutomaticStiffness, /**< If a broken joint is detected, reduce its stiffness. TODO set it to a lower value after tests on real NAO! */
    (std::vector<Joints::Joint>)({Joints::lAnkleRoll, Joints::rAnkleRoll}) brokenJointsReducesStiffnessList, /**< Only automatically reduce stiffness of those joints. */
    (Vector2a)(20_deg, 30_deg) uprightAngle,
    (int)(10000) uprightWarningTime,
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
  {};
  PlayDeadPhase(const MotionPhase& lastPhase, const MotionEngine& engine);

private:
  bool isDone(const MotionRequest& motionRequest) const override;
  void calcJoints(const MotionRequest& motionRequest, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo) override;
  std::unique_ptr<MotionPhase> createNextPhase(const MotionPhase& defaultNextPhase) const override;
  void update() override;

  const MotionEngine& engine; /**< A reference to the running motion engine. */
  unsigned int uprightWarningTimestamp = 0;
};
