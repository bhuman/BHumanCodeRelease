/**
 * @file KeyframePhaseBase.h
 * This file declares functions for the KeyframePhase
 * @author Philip Reichenberg
 */

#pragma once
#include "Debugging/Annotation.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/Modify.h"
#include "Streaming/AutoStreamable.h"
#include "Representations/MotionControl/KeyframeMotionRequest.h"
#include "Representations/MotionControl/KeyframeMotionGenerator.h"
#include "Representations/MotionControl/KeyframeMotionParameters.h"
#include <optional>

#include "Math/RingBuffer.h"
#include "Platform/SystemCall.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/MotionControl/KeyframeMotionParameters.h"
#include "Representations/MotionControl/KeyframeMotionRequest.h"
#include "Tools/Motion/EngineState.h"
#include "Tools/Motion/MotionPhase.h"
#include "Tools/Motion/MotionUtilities.h"

#include "Math/Pose2f.h"
#include "Math/Pose3f.h"
#include "Math/Rotation.h"

class KeyframeMotionEngine;
using EngineState = EngineStates::EngineState;
using KeyframeMotionID = KeyframeMotionRequest::KeyframeMotionID;
using Phase = KeyframeMotionPhases::KeyframeMotionPhase;

STREAMABLE(StiffnessPair,
{
  StiffnessPair() = default;
  StiffnessPair(Joints::Joint joint, int s),

  (Joints::Joint)(Joints::headYaw) joint,
  (int)(0) s,
});

inline StiffnessPair::StiffnessPair(Joints::Joint joint, int s) : joint(joint), s(s) {}

// For forcing stuck joints. Object saves all information for the request to let a specific joint get stuck for a given period of time
STREAMABLE(JointFailureParams,
{,
  (KeyframeMotionID)(KeyframeMotionID::decideAutomatic) motionIDFailure,
  (int)(0) lineCounterFailureStart,
  (int)(0) lineCounterFailureEnd,
  (float)(0.f) ratioStart,
  (float)(0.f) ratioEnd,
  (Joints::Joint) joint,
  (float) reduceRatio,
});

// If u want to add variables, then u need to add them in the methods: checkConditions()
ENUM(ConditionVar,
{,
  InertialDataAngleY, // Torso angle forward
  InertialDataAngleAbsoluteX, // Torso angle sidewards without sign
  FluctuationY, // Fluctuation forward
  ShoulderPitchLeft,
  ShoulderPitchRight,
  ShoulderRollLeft,
  ShoulderRollRight,
  WristTranslationYLeft,
  WristTranslationYRight,
  WaitTime, // Wait time of previous keyframe
  FrontHack, //FrontHack
  IsSitting, // Are the knees closed
  LowCom, // How much is the robot actually standing
  HYPDifference, // Different of requested and measured HYP value
  FootSupportVal, // Support foot check
  BreakUp, // Motion would be aborted in this motion frames
  ComOutOfSupportPolygonX, // Negative values -> inside, Positive values -> outside. Distance in mm to the nearest edge
  LyingOnArmsFront,
  TryCounter,
});

STREAMABLE(Condition,
{,
  (ConditionVar)(ConditionVar::InertialDataAngleY) variable, // Condition enum
  (Rangef)(Rangef(0.f, 0.f)) range, // Threshold range
  (bool)(false) isNot, // Negate condition
});

// Defines the possible types of how the interpolation shall be done
ENUM(InterpolationType,
{,
  Default, // Keyframe uses interpolation defined by the motion
  Linear, // Linear interpolation
  SinusMinToMax, // Sinus curve from -1/2 pi to +1/2 pi
  SinusZeroToMax, // Sinus curve from 0 to +1/2 pi
});

STREAMABLE_WITH_BASE(WaitCondition, Condition,
{,
  (int)(1000) maxWaitTime, // Max wait time
});

STREAMABLE(JointPair,
{,
  (Joints::Joint)(Joints::headYaw) joint,  // The joint
  (float)(0.f) scaling, // The scaling factor
});

// List of joints used to balance
STREAMABLE(BalanceOptionJoints,
{,
  (std::vector<JointPair>) jointY,
  (std::vector<JointPair>) jointX,
});

STREAMABLE(JointCompensationParams,
{,
  (Joints::Joint)(Joints::headYaw) jointDelta, // The joint that is stuck and needs compensation
  (bool)(false) hipPitchDifferenceCompensation, // Special case compensation, which ignores the other values
  (Rangea) range, // Scale from minVal to maxVal (even if minVal > maxVal). If both have different signs
  // scale from 0 to minVal and from 0 to maxVal and take the higher percent value
  (std::vector<JointPair>) jointPairs, // The joints used for compensation
  (bool)(false) predictJointDiff, // Predict the angle of jointDelta
});

STREAMABLE(JointCompensation,
{,
  (std::vector<JointCompensationParams>) jointCompensationParams, // List of joints that shall be compensated
  (float)(0.2f) reduceFactorJointCompensation, // reduceFactor to reduce over compensating. Only used if predictJointDiff = true
});

namespace KeyframeMotionListID
{
  ENUM(KeyframeMotionListID,
  {,
    front,
    back,
    recoverGeneric,
    catchBack,
    catchFront,
    recoverFromGenu,
    recoverFromSumo,
    restartFront,
    restartBack,
    stand,
    sit,
    sitDown,
    sitDownKeeper,
    keeperJumpLeft,
    genuflectStand,
    genuflectStandDefender,
    demoBannerWave,
    demoBannerWaveInitial,
  });
}

namespace KeyframeMotionBlockID
{
  ENUM(KeyframeMotionBlockID,
  {,
    sit,
    sitDown,
    stand,
    standHigh,
    front,
    frontCatch,
    frontCatchStart,
    frontUpright,
    back,
    backTipOver,
    backTipOverReset,
    recover,
    recoverBackStaticArms,
    recoverFront,
    recoverBack,
    recoverSlow,
    recoverGenuflect,
    recoverGenuflectWide,
    armSavingFront,
    armSavingBack,
    fromSitting,
    genuflectFromSitting,
    genuflectFromStand,
    genuflect,
    genuflectWide,
    genuflectEnd,
    wait,
    jumpLeftFromSitting,
    jumpLeftFromStand,
    sitKeeper,
    sitKeeperJump,
    waveInitial,
    wave,
  });
}

ENUM(KeyframeLimb,
{,
  leftArm,
  rightArm,
  leftLeg,
  rightLeg,
});

STREAMABLE(KeyframeJointAngles,
{,
  (std::array<Angle, 2>)({{0.f, 0.f}}) head,
  (ENUM_INDEXED_ARRAY(std::array<Angle COMMA 6>, KeyframeLimb)) positions,
});

STREAMABLE(Keyframe3DPose,
{,
  (std::optional<Vector3f>) trans,
  (std::optional<Vector3a>) rot,
});

STREAMABLE(Keyframe3DPoseWrapper,
{,
  (std::optional<Keyframe3DPose>) pose,
});

STREAMABLE(Keyframe3DPoseArray,
{,
  (ENUM_INDEXED_ARRAY(Keyframe3DPoseWrapper, KeyframeLimb)) robotPoses,
});

STREAMABLE(ConditionPair,
{,
  (std::vector<Condition>) conditions,
  (bool)(true) isAnd,
});

STREAMABLE(KeyframeBlockBranch,
{,
  (KeyframeMotionBlockID::KeyframeMotionBlockID) blockID,
  (std::optional<KeyframeMotionListID::KeyframeMotionListID>) motionID, // If set, this whole motion is executed and the old one is aborted
  (bool)(false) initStiffness,
  (bool)(false) removeCurrentBlock,
  (ConditionPair) preCondition,
  (bool)(false) useEarlyEntrance,
  (ConditionPair) earlyEntranceCondition,
  (int)(-1) maxNumberOfLoopsBlockID,
  (int)(-1) maxNumberOfLoopsMotionID,
});

STREAMABLE(Keyframe,
{,
  (Phase)(Phase::UnknownPhase) phase, // name of the current phase
  (float)(2000.f) duration, // duration of the current keyframe
  (Vector2f)(Vector2f::Zero()) goalCom, // reference com
  (bool)(false) setLastCom, // shall last com be overwritten with the current com at the start of the keyframe?
  (bool)(false) forbidWaitBreak, // Wait time is not allowed to break up early if goal angle is not reachable
  (InterpolationType)(InterpolationType::Default) interpolationType, // How shall the interpolation be done
  (std::optional<JointCompensation>) jointCompensation, // list of joints that shall be compensated. Only first entry is used. A list is used because: no compensation -> empty list -> no useless lines in the config file
  (std::vector<WaitCondition>) waitConditions, // conditions, if a wait time shall be executed at the end of a keyframe
  (std::vector<KeyframeBlockBranch>) keyframeBranches,
  (BalanceOptionJoints) balanceWithJoints, // add the balance factor of the pid-controller on top of these joints
  (std::vector<StiffnessPair>) singleMotorStiffnessChange, // from this line on use for joint stiffness
  // Keyframe angles for the joints
  (KeyframeJointAngles) angles,
  (std::optional<Keyframe3DPoseArray>) robotPoses,
  (DirectionValue) torsoAngleBreakUpEnd,
  (std::optional<DirectionValue>) torsoAngleBreakUpStart,
});

namespace EnergySavingType
{
  ENUM(EnergySavingType,
  {,
    none,
    activeInWait,
    activeAlways,
  });
}

STREAMABLE(KeyframeEndRequest,
{,
  (EnergySavingType::EnergySavingType) energySavingLegs,
  (EnergySavingType::EnergySavingType) energySavingArms,
  (bool) isStandHigh,
});

STREAMABLE(KeyframeBlock,
{,
  (std::array<int, 5>)({{0, 0, 0, 0, 0}}) baseLimbStiffness, // Stiffness for head, lArm, rArm, lLeg, rLeg
  (std::vector<Keyframe>) keyframes,
  (InterpolationType)(InterpolationType::Linear) interpolationType,
});

struct KeyframeBlockBranched : public KeyframeBlock
{
  KeyframeBlockBranched() = default;
  KeyframeBlockBranched(const KeyframeBlock& other);

  std::optional<std::array<int, Joints::numOfJoints>> stiffness;
};

inline KeyframeBlockBranched::KeyframeBlockBranched(const KeyframeBlock& other)
  : KeyframeBlock(other) {}

STREAMABLE(FollowUpMotionPair,
{,
  (KeyframeMotionListID::KeyframeMotionListID) startAsNextPhase,
  (std::vector<Condition>) conditions,
  // HardCodedConditions too?
  (bool) isAndCondition,
});

STREAMABLE(KeyframeMotionList,
{,
  (std::vector<KeyframeMotionBlockID::KeyframeMotionBlockID>) keyframes,
  (bool)(false) balanceOut, // Shall the motions get balanced out after finished?
  (KeyframeEndRequest) keyframeEndRequest,
  (bool)(false) isMotionStable,
  (std::vector<FollowUpMotionPair>) startAsNextPhaseMotion, // When the motionRequest changes, this motion must be executed!
});

STREAMABLE(BalanceOutParams,
{,
  (int)(0) maxTime, // max time in balanceOut state (in ms)
  (Angle)(0) minFluctuation, // min fluctuation to be able to leave early
  (Angle)(0) minForwardAngle, //min forward Angle to be able to leave early
  (Angle)(0) minBackwardAngle, //min backward Angle to be able to leave early
  (float)(0) minPIDDValue, // min d value of pid controller
});

STREAMABLE(SafeFallParameters,
{,
  (Angle)(0) sitHYPThreshold, // HYP must be nearly closed, to allow sitting position
  (Angle)(0) head, // Head position
  (int)(0) headStiffness, // Head stiffness at start of fall
  (int)(0) bodyStiffness, // Body stiffness while falling
  (int)(0) lowHeadStiffnessWaitTime, // High head stiffness for this duration
  (int)(0) unstiffWaitTime, // Set Stiffness 0 after this much time has passed
});

struct KeyframePhaseBase : MotionPhase
{
  explicit KeyframePhaseBase(KeyframeMotionEngine& engine, const KeyframeMotionRequest& keyframeMotionRequest);

protected:
  /**
   * The method checks if all joints are working and if there is a problem.
   * Problems are:
   *  * a Joint can't reach a given angle because it is to hot or broken
   *  * a joint is to slow in reaching the desired angle
   * These checks are only done, if the current motion specifically wants to check it.
   */
  void calculateJointDifference();

  /**
   * The method checks, if a recover is needed after the robot is fallen.
   * Possible cases are:
   *  * he is lying on the side
   *  * he is lying on his arm
   *  * one arm is not correct.
   * This needs to be done to prevent that the arms are in wrong positions, because the get up motions
   * assume, that the robot starts in a specific position. Otherwise the first try could fail or the arms could get damaged.
   */
  void selectMotionToBeExecuted();

  /**
   * The method checks, if the current motion shall break up.
   */
  bool isInBreakUpRange();

  /**
   * The method calculates the current ratio for the interpolation.
   * If the current motion line is finished, then it will set the next motion line or if the motion is finished, it will set the state to finished.
   */
  void isCurrentLineOver();

  /**
   * The method initializes everything that needs to be done specifically for this Motion.
   */
  void initKeyframeMotion(const bool overrideMirror);

  /*
   * The method initializes everything that needs to be done specifically for this motion line.
   * @param safeLastKeyframeInfo If true then execute safePreviousKeyframeData()
   */
  void initCurrentLine(const bool safeLastKeyframeInfo);

  /**
   * Update the current values which are needed to calculated the current joints and balancer values
   */
  void updateLineValues();

  /**
   * Init the values that are needed in the method pidWithCom at the start of a keyframe
   * @param calculateCurrentNew If true, reset the current reference values
   */
  void initBalancerValues(bool calculateCurrentNew);

  /**
   * The method checks if the KeyframeMotionEngine still needs to wait until it can start doing stuff. Reason is, if the robot is still
   * falling it would be a really bad idea to start a motion.
   */
  void waitForFallen();

  /**
   * The method does special actions if the current state is helpMeState. This needs to be done, because the robot is unable to get up.
   */
  void doHelpMeStuff();
  /**
   * Sets the current joint goals for MotionLine. This is done once, every time a new MotionLine is started.
   */
  void setNextJoints();

  /**
   * Sets the stiffness values of the joints. Must be between 0 and 100.
   */
  void setJointStiffnessBase();

  /**
   * Sets the stiffness values of the Joints. Must be between 0 and 100.
   */
  void setJointStiffnessKeyframe();

  /**
   * The methods checks the engine state finished. If the motions has the check to balance out, then the KeyframeMotionEngine will balance the standing
   * robot before he is allowed to walk. This is done because the robot could have some momentum left, that would cause him to
   * fall instantly again.
   *
   * If the check is off, then the method will do nothing
   *
   * If there is a connected motion (continueTo), then this motion will get set next.
   */
  void checkFinishedState();

  /**
   * Checks, if the current keyframe should be skipped.
   */
  bool checkOptionalLine();

  bool checkEarlyBranch();

  /**
   * Checks the conditions, if the current keyframe should be skipped.
   * @param conditions Vector of the conditions to be checked
   * @param useAnd If true, concatenate the conditions with AND, otherwise with OR
   */
  template<class C>
  bool checkConditions(const std::vector<C>& conditions, bool useAnd);

  /**
   * The PID-Controller to balance the motion
   */
  void pidWithCom();

  /**
   * Adds the calculated balancer value on top of the joints
   * @param factorY forward / backward balancing (mainly pitch)
   * @param factorX sidewards balancing (mainly roll)
   */
  void addBalanceFactor(const float factorY, const float factorX);

  /**
   * The method applies the joint compensation of the previous keyframe
   */

  void applyJointCompensation();

  /**
   * The method removes the joint compensation of the previous keyframe
   * Reduce jointCompensation of last Keyframe
   * This reduces the joint compensation of the keyframe before, to prevent an overcompensation.
   * Otherwise the compensation of the keyframe before would be reduce not fast enough.
   * Compensation reduction does not work if used with negative values, because every use-case is with positive values.
   */
  void removePreviousKeyframeJointCompensation();

  /**
   * Calculates the difference of the com relative to the ground floor and support polygon middle of the robot.
   */
  void calculateCOMInSupportPolygon();

  /**
   * The method updates the parameters which saves the torso angle of the last few frames and updates the prediction for the next few frames.
   */
  void updateSensorArray();

  /**
   * Balance robot after finished get up motion
   */
  void doBalanceOut();

  /**
   * Fail-Save Method, to cancel the current motion if 2 of 3 head sensors are touched.
   */
  void doManipulateOwnState();

  /**
   * Safe some information from the previous keyframe
   */
  void safePreviousKeyframeData();

  KeyframeMotionEngine& engine;

  EngineState state = EngineState::off; // defines the current engine state

  unsigned int lineStartTimestamp = 0, // timestamp when the interpolation between key frames started
               lastNotActiveTimestamp = 0, // timestamp last time we were in a breakUp
               waitTimestamp = 0, // timestamp for wait times, to stop wait times, that are too long
               helpMeTimestamp = 0, // timestamp for last help me sound
               breakUpTimestamp = 0, // timestamp when break up started
               lastPIDCom = 0, // timestamp last time the PID values were changed
               retryTimestamp = 0, // retry hack, so we can retry keyframes without doing the whole motion over again
               doBalanceOutTimestamp = 0, // timestamp used to decide when we can shutdown the engine
               abortWithHeadSensorTimestamp = 0; // timestamp last time the head sensors where touched

  int breakCounter = 0, // number of normal retried get up tries
      retryTime = 0, // Wait time before a retry is allowed
      tryCounter = 0; // number of tries

  JointAngles startJoints; // measured joint data when the last key frame was passed

  JointAngles jointDiff1Angles, // saved the difference of the request and reached angles with a delay
              jointDiffPredictedAngles, // diff of set and reached joint angles
              lineJointRequestAngles, // current goal angles of the current keyframe
              lastKeyframeLineJointRequest, // lineJointRequest from previous keyframe
              lineJointRequest2Angles, // saves lineJointRequest2
              lastOutputLineJointRequestAngles, // saves the lineJointRequest of the keyframe before

              // for forcing stuck joints
              stuckJointRequest,

              // for using energy saving mode
              preHeatAdjustment; // The executed JointRequest at the end from the current motion frame

  JointRequest lastUnbalanced, // calculated last target joint angles without balance added
               targetJoints, // calculated next target joint angles without balance added
               lineJointRequest, // joint angles from the config for the current keyframe
               jointRequestOutput; // the generated joint request for the current frame

  RingBuffer<JointAngles, 4> pastJointAnglesAngles; // TODO reduce to 3 or even 2? Last JointRequest is added in the current frame. The current JointRequest in the next motion frame.

  // Which joints were used for balancing from the previous keyframe
  std::vector<Joints::Joint> jointsBalanceY,
      jointsBalanceX;

  // Last 2 Gyro values
  RingBuffer<Vector2a, 3> lastTorsoAngle;

  std::array<float, Joints::numOfJoints> jointCompensationReducer; // how much is the last jointCompensation of the keyframe before already reduced? (Value between 0 and 1)

  Vector2f lastGoal = Vector2f::Zero(), // last com goal
           goalGrowth = Vector2f::Zero(), // how much shall com_foot_last - com_foot_current be in an ideal world?
           currentGoal = Vector2f::Zero(), // current keyframe com goal

           comDiff = Vector2f::Zero(), // com relative to support polygon for current frame
           oldCom = Vector2f::Zero(), // com relative to support polygon from last frame
           comDistanceToEdge = Vector2f(-1000.f, -1000.f), // for conditions

           valPID_I = Vector2f::Zero(), // current PID-Values
           valPID_D = Vector2f::Zero(),
           valPID_P = Vector2f::Zero();

  struct BalancerValue
  {
    float pitchValue = 0.f;
    float rollValue = 0.f;
  };

  BalancerValue balanceValue; // output of PID-Controller

  DirectionValue lastComDiffBaseForBalancing, // max com diff, before the balancer value is increased by 100%
                 currentComDiffBaseForBalancing, // Max com to increase balancer values
                 lastTorsoAngleBreakUp, // Max torso angle before we break up the motion
                 currentTorsoAngleBreakUp; // Max Torso angle to break get up try

  Vector2a fluctuation = Vector2a::Zero(); // how much does the robot fluctuates right now?

  float ratio = 1.f, // current ratio for how much the goal joint shall be added as jointRequest
        framesTillEnd = 1.f, // so many frames until the current MotionLine is over
        duration = 0.f, // current keyframe duration
        failedWaitCounter = 0.f, // counter to break wait times early
        pidDForward = 0.f; // used for balanceOut Method. Copy of D part of the PID-Controller

  // bools to check which motion shall be executed when lying on ground
  bool fastRecover = false, // Bring all joints into a default position
       recoverLyingOnArmCheck = false, // Left arm check
       armRightCheck = false, // Right arm check
       errorTriggered = false, //If an error occurred -> true. Should never be true.
       forceBalancerInit = false, // forces initBalancerValues(true)
       tooManyTries = false, // to many get up tries
       wasHelpMe = false, // was in help me
       wasRatio100Percent = false, // was the ratio at least one time 1.0 for the current keyframe
       balancerOn = false, // is the balancer active. Must be a boolean, so balancer can be switched on when calibrating
       isFirstLine = false, // is the first line of the motion. TODO: lineCounter == 0?
       breakUpOn = false, // when calibrating the motion shall not break up
       isMirror = false, // is current motion mirrored
       isInOptionalLine = false, // is current keyframe a optional keyframe
       wasInWaiting = false, // get up engine was in a waiting state
       frontHackTriggered = false, // arms were unable to move in the first keyframe of the front get up
       getUpMirror = false, // should the motion front and back be mirrored
       didFirstGetUp = false, // For the first get up decide if the motion shall be mirrored or not
       finishGettingUp = false, // If the motion request changes from decideAutomatic to another motion and the robot is still doing a recover, he will abort the get up without this flag
       doSlowKeyframeAfterFall = false, // When a non get up motion went into the breakUp state and is upright again, execute the first keyframe slow. We assume the robot is hold by a human
       waitForFallenCheck = true, // Do stuff when breaking up get up motion only once
       forceStandMotionAfterPlaydead = false;

  std::array<int, KeyframeMotionListID::numOfKeyframeMotionListIDs> motionListIDLoops;
  std::array<int, KeyframeMotionBlockID::numOfKeyframeMotionBlockIDs> blockIDLoops;

  Pose2f odometry; // current odometry

  float variableValuesCompare[numOfConditionVars]; // To break up a wait time early

  KeyframeMotionRequest currentKeyframeMotionRequest;
  KeyframeMotionRequest lastKeyframeMotionRequest;

  JointRequest lastJointRequest;

  std::optional<KeyframeMotionList> lastMotion;
  KeyframeMotionList currentMotion;
  std::vector<KeyframeBlockBranched> currentMotionBlock;
  Keyframe currentKeyframe;
  Keyframe lastKeyframe; // dummy parameters if not set

  friend class KeyframeMotionEngine;
};
