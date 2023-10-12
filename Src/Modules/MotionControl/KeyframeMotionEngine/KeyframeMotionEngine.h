/**
 * @file KeyframeMotionEngine.h
 * A motion engine for standing up and special motions.
 * @author <A href="mailto:s_ksfo6n@uni-bremen.de">Philip Reichenberg</A>
 */
#pragma once

#include <cmath>
#include <functional>
#include <string>
#include "Platform/File.h"

#include "KeyframeMotionLibs/KeyframePhaseBase.h"

#include "Representations/Configuration/FootOffset.h"
#include "Representations/Configuration/JointLimits.h"
#include "Representations/Configuration/StaticJointPoses.h"

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"

#include "Representations/MotionControl/DiveGenerator.h"
#include "Representations/MotionControl/EnergySaving.h"
#include "Representations/MotionControl/GetUpGenerator.h"
#include "Representations/MotionControl/KeyframeMotionGenerator.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/SpecialGenerator.h"
#include "Representations/MotionControl/WalkGenerator.h"

#include "Representations/Sensing/FilteredCurrent.h"
#include "Representations/Sensing/FootSupport.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/GyroOffset.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"

#include "Framework/Module.h"
#include "Math/Range.h"

MODULE(KeyframeMotionEngine,
{,
  REQUIRES(EnergySaving),
  REQUIRES(FilteredCurrent),
  REQUIRES(FrameInfo),
  REQUIRES(FootOffset),
  REQUIRES(FootSupport),
  REQUIRES(GroundContactState),
  REQUIRES(GyroOffset),
  REQUIRES(InertialData),
  REQUIRES(JointAngles),
  USES(JointLimits),
  USES(JointRequest),
  REQUIRES(KeyframeMotionParameters),
  REQUIRES(KeyStates),
  REQUIRES(RobotModel),
  REQUIRES(StaticJointPoses),
  REQUIRES(TorsoMatrix),
  REQUIRES(WalkGenerator),
  PROVIDES(KeyframeMotionGenerator),
  REQUIRES(KeyframeMotionGenerator),
  PROVIDES(GetUpGenerator),
  PROVIDES(DiveGenerator),
  PROVIDES(SpecialGenerator),
  LOADS_PARAMETERS(
  {,
    (int) maxTryCounter, // Number of allowed get up tries
    (bool) motorMalfunctionBreakUp, // Go to helpMeState if a motorMalfunction was detected once
    (int) maxStiffnessDebugMode, // Max allowed stiffness when stepKeyframes is true
    (ENUM_INDEXED_ARRAY(KeyframeMotionList, KeyframeMotionListID::KeyframeMotionListID)) motions, // The different keyframe motions
    (ENUM_INDEXED_ARRAY(KeyframeBlock, KeyframeMotionBlockID::KeyframeMotionBlockID)) keyframeBlock, // Basic keyframed motions
    (Angle) minJointCompensationReduceAngleDiff, // Threshold for joint compensation. Joint must change its position by this value between 2 frames to trigger a reduction in the joint compensation
    (BalanceOutParams) balanceOutParams, // The parameters for the balanceOutMethod
    (Vector3f) supportPolygonOffsets, // Offsets for the support polygon. x Forward, y Backward, z Sideways
    (SafeFallParameters) safeFallParameters, // Parameters for waitForFallen()
    (DirectionValue) safeUprightParameters, // Robot is upright enough to go directly into (sit->)stand
  }),
});

class KeyframeMotionEngine : public KeyframeMotionEngineBase
{
public:
  KeyframeMotionEngine();
  void update(DiveGenerator& output) override;
  void update(GetUpGenerator& output) override;
  void update(KeyframeMotionGenerator& output) override;
  void update(SpecialGenerator& output) override;

  bool calculateDrawing = false;  // Do drawning?
  bool stepKeyframes = false; // Execute every keyframe slow and wait for request
private:

  /**
   * All debugging related stuff
   * @param output The KeyframeMotionGenerator
   */
  void setUpDebugCommands(KeyframeMotionGenerator& output);
};

struct KeyframePhase : KeyframePhaseBase
{
  KeyframePhase(KeyframeMotionEngine& engine, const KeyframeMotionRequest& keyframeMotionRequest, const MotionPhase& lastPhase);

  void update() override;
  bool isDone(const MotionRequest& motionRequest) const override;
  void calcJoints(const MotionRequest& motionRequest, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo) override;
  std::unique_ptr<MotionPhase> createNextPhase(const MotionPhase& defaultPhase) const override;
};
