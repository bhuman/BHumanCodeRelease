/**
 * @file KeyframeMotionEngine.h
 * A motion engine for standing up.
 * TODO: change the stupid float calculations to Angles...
 * @author <A href="mailto:s_ksfo6n@uni-bremen.de">Philip Reichenberg</A>
 */
#pragma once

#include <cmath>
#include <functional>
#include <string>
#include "Platform/File.h"
#include "Platform/SystemCall.h"

#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Sensing/GyroOffset.h"
#include "Representations/Configuration/JointLimits.h"

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"

#include "Representations/MotionControl/EnergySaving.h"
#include "Representations/MotionControl/GetUpGenerator.h"
#include "Representations/MotionControl/HeadAngleRequest.h"
#include "Representations/MotionControl/KeyframeMotionGenerator.h"
#include "Representations/MotionControl/KeyframeMotionParameters.h"
#include "Representations/MotionControl/KeyframeMotionPhase.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/KeyframeMotionRequest.h"

#include "Representations/Sensing/FootSupport.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/FilteredCurrent.h"

#include "Tools/Debugging/Annotation.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/Modify.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Math/Pose3f.h"
#include "Tools/Math/Rotation.h"
#include "Tools/Module/Module.h"
#include "Tools/Motion/EngineState.h"
#include "Tools/Motion/MotionPhase.h"
#include "Tools/Motion/MotionUtilities.h"
#include "Tools/RingBuffer.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/EnumIndexedArray.h"
#include "Tools/Range.h"

using KeyframeMotionID = KeyframeMotionRequest::KeyframeMotionID;
using EngineState = EngineStates::EngineState;
using Phase = KeyframeMotionPhases::KeyframeMotionPhase;
STREAMABLE(StiffnessPair,
{
  StiffnessPair() = default;
  StiffnessPair(Joints::Joint joint, int s),

  (Joints::Joint) joint,
  (int) s,
});

inline StiffnessPair::StiffnessPair(Joints::Joint joint, int s) : joint(joint), s(s) {}

// Balance factors
STREAMABLE(BalanceFactors,
{,
  (Vector2f)(0.f, 0.f) increaseFactorCapFront,
  (Vector2f)(0.f, 0.f) increaseFactorCapBack,
  (Vector2f)(0.f, 0.f) powDegreeFront,
  (Vector2f)(0.f, 0.f) powDegreeBack,
  (Vector2f)(0.f, 0.f) borderPIDPX,
  (Vector2f)(0.f, 0.f) borderPIDPY,
  (Vector2f)(0.f, 0.f) borderPIDDX,
  (Vector2f)(0.f, 0.f) borderPIDDY,
  (bool)(false) pidDXController,
});

// If u want to add variables, then u need to add them in the methods:
// checkConditions for other usages
ENUM(ConditionVar,
{,
  InertialDataAngleY, // Torso angle forward
  FluctuationY, // Fluctuation forward
  BrokenLeftArm, // Left arm is stuck
  BrokenRightArm, // Right arm is stuck
  WaitTime, // Wait time of previous keyframe
  FrontHack, //FrontHack ;)
  IsSitting, // Are the knees closed
  HYPDifference, // Different of requested and measured HYP value
  FootSupportVal, // Support foot check
});

// Defines the possible types of how the interpolation shall be done
ENUM(InterpolationType,
{,
  Default, // Keyframe uses interpolation defined by the motion
  Linear, // Linear interpolation
  SinusMinToMax, // Sinus curve from -1/2 pi to +1/2 pi
  SinusZeroToMax, // Sinus curve from 0 to +1/2 pi
});

STREAMABLE(Condition,
{,
  (ConditionVar) variable, // Condition enum
  (float) lowerFloat, // Lower threshold
  (float) higherFloat, // Upper threshold
  (bool) isNot, // Negate condition
});

STREAMABLE_WITH_BASE(WaitCondition, Condition,
{,
  (int)(1000) maxWaitTime, // Max wait time
});

// Factor for the used joint to increase or decrease balancer value
STREAMABLE(JointFactor,
{,
  (Joints::Joint) joint, // The joint used for balancing
  (float) factor, // The factor
});

// List of joints used to balance
STREAMABLE(BalanceOptionJoints,
{,
  (std::vector<JointFactor>) jointY,
  (std::vector<JointFactor>) jointX,
});

STREAMABLE(JointPair,
{,
  (Joints::Joint) joint, // The joint used for compensation
  (float) addValue, // Ratio of the joint that is stuck
});

STREAMABLE(JointCompensationParams,
{,
  (Joints::Joint) jointDelta, // The joint that is stuck and needs compensation
  (bool)(false) hipPitchDifferenceCompensation, // Special case compensation, which ignores the other values
  (float) minVal, // If minVal positive: jointDelta must have at least this difference
  // Otherwise: Compensated joint must have this max difference to be fully compensated
  (float) maxVal, // If minVal positive: jointDelta must have this max difference to be fully compensated
  // Otherwise: Compensated joint must have at least this difference
  (std::vector<JointPair>) jointPairs, // The joints used for compensation
  (bool)(false) predictJointDif, // Predict the angle of jointDelta
});

STREAMABLE(JointCompensation,
{,
  (std::vector<JointCompensationParams>) jointCompensationParams, // List of joints that shall be compensated
  (float)(0.2f) reduceFactorJointCompensation, // reduceFactor to reduce over compensating. Only used if predictJointDiff = true
});

STREAMABLE(MofLine,
{,
  (Phase) phase, // name of the current phase
  (float)(2000.f) duration, // duration of the current keyframe
  (Vector2f)(0.f, 0.f) goalCom, // reference com
  (bool)(false) setLastCom, // shall last com be overwritten with the current com at the start of the keyframe?
  (bool)(false) optionalLineAnds, // ands the conditions
  (bool)(false) isElseBlock, // else block of optionale keyframes
  (bool)(false) isPartOfPreviousOptionalLine, // part of a optionalLine
  (bool)(false) forbidWaitBreak, // Wait time is not allowed to break up early if goal angle is not reachable
  (InterpolationType)(InterpolationType::Default) interpolationType, // How shall the interpolation be done
  (std::vector<JointCompensation>) jointCompensation, // list of joints that shall be compensated. Only first entry is used. A list is used because: no compensation -> empty list -> no useless lines in the config file
  (std::vector<WaitCondition>) waitConditions, // conditions, if a wait time shall be executed at the end of a keyframe
  (std::vector<Condition>) conditions, // conditions of the optionale line
  (BalanceOptionJoints) balanceWithJoints, // add the balance factor of the pid-controller on top of these joints
  (bool) balancerActive, // how shall get balanced? (currently only one version implemented / 28.11.18
  (std::vector<StiffnessPair>) singleMotorStiffnessChange, // from this line on use for joint stiffness
  // Keyframe angles for the joints
  (std::array<float, 2>)({{0.f, 0.f}}) head,
  (std::array<float, 6>)({{0.f, 0.f, 0.f, 0.f, 0.f, 0.f}}) leftArm,
  (std::array<float, 6>)({{0.f, 0.f, 0.f, 0.f, 0.f, 0.f}}) rightArm,
  (std::array<float, 6>)({{0.f, 0.f, 0.f, 0.f, 0.f, 0.f}}) leftLeg,
  (std::array<float, 6>)({{0.f, 0.f, 0.f, 0.f, 0.f, 0.f}}) rightLeg,
});

STREAMABLE(Mof,
{,
  (std::array<int, 5>)({{0, 0, 0, 0, 0}}) baseLimbStiffness, // Stiffness for head, lArm, rArm, lLeg, rLeg
  (std::vector<MofLine>) lines, // The Keyframes
  (Pose2f) odometryOffset, // OdometryOffset
  (bool)(false) balanceOut, // Shall the motions get balanced out after finished?
  (KeyframeMotionID)(KeyframeMotionID::decideAutomatic) continueTo, // Self if no motion should follow
  (Angle)(0) clipAngle, // Positive angle -> clip forwardDifAngleBreak, if negativ angle -> clip forwardDifAngleBreak
  (bool)(false) energySavingLegs,
  (bool)(false) energySavingArms,
  (bool)(false) isMotionStable,
  (InterpolationType)(Linear) interpolationType, // Defines the typ of interpolation for the whole motion
});

STREAMABLE(BalanceOutParams,
{,
  (int) maxTime, // max time in balanceOut state (in ms)
  (Angle) minFluctuation, // min fluctuation to be able to leave early
  (Angle) minForwardAngle, //min forward Angle to be able to leave early
  (Angle) minBackwardAngle, //min backward Angle to be able to leave early
  (float) minPIDDValue, // min d value of pid controller
});

STREAMABLE(SafeFallParameters,
{,
  (Angle) sitHYPThreshold, // HYP must be nearly closed, to allow sitting position
  (Angle) head, // Head position
  (int) headStiffness, // Head stiffness at start of fall
  (int) bodyStiffness, // Body stiffness while falling
  (int) lowHeadStiffnessWaitTime, // High head stiffness for this duration
  (int) unstiffWaitTime, // Set Stiffness 0 after this much time has passed
});

// For forcing stuck joints. Object saves all information for the request to let a specific joint get stuck for a given period of time
STREAMABLE(JointFailureParams,
{,
  (KeyframeMotionID)(KeyframeMotionID::decideAutomatic)motionIDFailure,
  (int)(0) lineCounterFailureStart,
  (int)(0) lineCounterFailureEnd,
  (float)(0.f) ratioStart,
  (float)(0.f) ratioEnd,
  (Joints::Joint)joint,
  (float)reduceRatio,
});

MODULE(KeyframeMotionEngine,
{,
  REQUIRES(DamageConfigurationBody),
  REQUIRES(EnergySaving),
  REQUIRES(FilteredCurrent),
  REQUIRES(FrameInfo),
  USES(HeadAngleRequest),
  REQUIRES(FootSupport),
  REQUIRES(GroundContactState),
  REQUIRES(GyroOffset),
  REQUIRES(InertialData),
  REQUIRES(JointAngles),
  USES(JointLimits),
  USES(JointRequest),
  REQUIRES(JointSensorData),
  REQUIRES(KeyframeMotionParameters),
  REQUIRES(KeyStates),
  REQUIRES(RobotModel),
  REQUIRES(StiffnessSettings),
  PROVIDES(KeyframeMotionGenerator),
  REQUIRES(KeyframeMotionGenerator),
  PROVIDES(GetUpGenerator),
  LOADS_PARAMETERS(
  {,
    (int) maxTryCounter, // Number of allowed get up tries
    (int) motionSpecificRetries, // Number of allowed retries
    (int) motionSpecificRetriesFront, // Number of allowed retries to move the arms to the side for the front get up
    (Angle) shoulderPitchThreshold, // Threshold to trigger retry in front get up
    (bool) motorMalfunctionBreakUp, // Go to helpMeState if a motorMalfunction was detected once
    (int) maxStiffnessDebugMode, // Max allowed stiffness when stepKeyframes is true
    (ENUM_INDEXED_ARRAY(Mof, KeyframeMotionRequest::KeyframeMotionID)) mofs, // The different get up motions
    (ENUM_INDEXED_ARRAY(float, Joints::Joint)) brokenJointData, // Thresholds to detect if a joint is too far of the request
    (Angle) minJointCompensationReduceAngleDiff, // Threshold for joint compensation. Joint must change its position by this value between 2 frames to trigger a reduction in the joint compensation
    (float) jointCompensationSimulationFactor, // Value for forcing stuck joints
    (std::vector<Phase>) clipBreakUpAngle, // In these motion phases, the break angles are clipped to 0 degree for one side, depending on which get up motion is executed
    (BalanceOutParams) balanceOutParams, // The parameters for the balanceOutMethod
    (Vector3f) supportPolygonOffsets, // Offsets for the support polygon. x Forward, y Backward, z Sideways
    (SafeFallParameters) safeFallParameters, // Parameters for waitForFallen()
  }),
});

class KeyframeMotionEngine : public KeyframeMotionEngineBase
{
  void update(GetUpGenerator& output) override;
  void update(KeyframeMotionGenerator& output) override;

  /*
   * This method handles the edit mode. This allows for live changes of keyframes while connected with a robot with the simulator via debug requests
   */
  void editMode(KeyframeMotionGenerator& output);

  /*
   * This method checks for the edit mode if the requested edit feature can be executed
   */
  bool checkIfEditIsPossible(int line, KeyframeMotionID motion);

  //Below are debugging methods
  void getMotionLine(KeyframeMotionID motionID, int motionLine);

  void getCurrentAnglesInMof();

  //Add a behavior to let a joint stuck to a specific time
  void addJointFailureBehavior(KeyframeMotionID motionIDFailure, int lineCounterFailureStart,
                               float ratioFailureStart, int lineCounterFailureEnd, float ratioFailureEnd,
                               Joints::Joint joint, float reduceRatio);

  //Remove all joints that shall be stuck to specific times
  void clearJointFailureBehavior();

public:
  KeyframeMotionID motionIDFailure = KeyframeMotionID::decideAutomatic;
  int lineCounterFailureStart = 0;
  int lineCounterFailureEnd = 0;
  float ratioFailureStart = 0.f;
  float ratioFailureEnd = 0.f;
  Joints::Joint joint = Joints::Joint();
  bool isCalibrationState = false; // is KeyframeMotionEngine in a calibration state and is waiting for a request
  bool initPhase = false; // initial state for the calibration state
  bool setCalibrationState = false;
  bool calculateDrawing = false;  // Do drawning?
  bool stepKeyframes = false; // Execute every keyframe slow and wait for request
  Phase calibrationPhase = Phase::ErrorPhase;
  std::vector<JointFailureParams> jointFailureList;
};
struct KeyframePhase : MotionPhase
{
  explicit KeyframePhase(KeyframeMotionEngine& engine, const MotionRequest& motionRequest, const MotionPhase& lastPhase);
private:
  void update() override;
  bool isDone(const MotionRequest& motionRequest) const override;
  void calcJoints(const MotionRequest& motionRequest, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo) override;

  KeyframeMotionEngine& engine;

  STREAMABLE(JointRequestAngles,
  {,
    (ENUM_INDEXED_ARRAY(Angle, Joints::Joint)) angles,
  });

  EngineStates::EngineState state; // defines the current engine state

  KeyframeMotionID motionID;  // the name of the current motion

  unsigned int lineStartTimestamp, // timestamp when a the interpolation between key frames started
           lastNotActiveTimestamp, // timestamp last time we were in a breakUp
           waitTimestamp, // timestamp for wait times, to stop wait times, that are too long
           helpMeTimestamp, // timestamp for last help me sound
           breakUpTimestamp, // timestamp when break up started
           lastPIDCom, // timestamp last time the PID values were changed
           retryTimestamp, // retry hack, so we can retry keyframes without doing the whole motion over again
           doBalanceOutTimestamp, // timestamp used to decide when we can shutdown the engine
           abortWithHeadSensorTimestamp; // timestamp last time the head sensors where touched

  int breakCounter, // number of normal retried get up tries
      retryTime, // Wait time before a retry is allowed
      lineCounter, // current keyframe
      maxCounter, // number of keyframes of current motion
      tryCounter, // number of tries
      retryMotionCounter; // retry whole get up motion counter, so we do not do too many retry. Otherwise we assume a hardware failure and just try our luck.

  JointAngles startJoints; // measured joint data when the last key frame was passed
  JointRequest lastUnbalanced, // calculated last target joint angles without balance added
               targetJoints, // calculated next target joint angles without balance added
               lineJointRequest, // joint angles from the config for the current keyframe
               jointRequestOutput; // the generated joint request for the current frame

  JointRequestAngles jointDiff1Angles, // saved the difference of the request and reached angles with a delay
                     jointDiffPredictedAngles, // diff of set and reached joint angles
                     lineJointRequestAngles, // current goal angles of the current keyframe
                     lineJointRequest2Angles, // saves lineJointRequest2
                     lastLineJointRequestAngles, // saves the lineJointRequest of the keyframe before.

                     // for forcing stuck joints
                     jointFailureRequestAngles,
                     jointFailureRequestCurrentBordersAngles,
                     jointRequestWithoutFailureBehaviorAngles,

                     // for using energy saving mode
                     preHeatAdjustment;

  RingBuffer<JointRequestAngles, 4> pastJointAnglesAngles;

  //Which joints are used for balancing from the previous keyframe
  std::vector<Joints::Joint> jointsBalanceY,
      jointsBalanceX;

  //Last 2 Gyro values
  RingBuffer<Vector2a, 3> lastTorsoAngle;

  std::array<float, Joints::numOfJoints> jointCompensationReducer; // how much is the last jointCompensation of the keyframe before already reduced? (Value between 0 and 1)

  // TODO BackwardAngle and ForwardAngle are not Angles (!), rename them. Also Backward/ForwardBreakDiff should be angles and not floats
  // first values are saved value of the last keyframe
  Vector2f lastBackwardCOMDiff, // max backward com diff, before balancer value is increased
           lastForwardCOMDiff, // max forward com diff, before the balancer value is increased by 100%
           currentBackwardCOMDiff, // Max com backward to increase balancer values
           currentForwardCOMDiff, // Max com forward to increase balancer values

           lastGoal, // last com goal
           goalGrowth, // how much shall com_foot_last - com_foot_current be in an ideal world?
           currentGoal, // current keyframe com goal

           comDif, // com relative to support polygon for current frame
           oldCom, // com relative to support polygon from last frame
           oldCom2, // com relative to support polygon from next-to-last frame

           valPID_I, // current PID-Values
           valPID_D,
           valPID_P,

           balanceValue; // output of PID-Controller

  Vector2a lastForwardAngleBreakUp, // Max forward torso angle before we break up the motion
           lastBackwardAngleBreakUp, // Max backward torso angle before we break up the motion
           currentBackwardAngleBreakUp, // Max Torso angle backward to break get up try
           currentForwardAngleBreakUp, // Max Torso angle forward to break get up try
           fluctuation; // how much does the robot fluctuates right now?

  float ratio, // current ratio for how much the goal joint shall be added as jointRequest
        duration, // current keyframe duration
        framesTillEnd, // so many frames until the current MotionLine is over
        failedWaitCounter, // counter to break wait times early
        pidDForward; // used for balanceOut Method. Copy of D part of the PID-Controller

  // bools to check which motion shall be executed when lying on ground
  bool fastRecover, // Bring all joints into a default position
       nonGetUpMotionCheck, // Check if the KeyframeMotionEngine went active after a non get up motion was executed
       armLeftCheck, // Left arm check
       armRightCheck, // Right arm check
       sideCheck, // Robot is lying on the side
       waitForFallenCheck, // Do stuff when breaking up get up motion only once
       errorTriggered, //If an error occurred -> true. Should never be true.
       isContinueTo, // is this a follow up motion?
       tooManyTries, // to many get up tries
       wasHelpMe, // was in help me
       doBalance, // balance when the current motion finished?
       wasRatio100Percent, // was the ratio at least one time 1.0 for the current keyframe
       balancerOn, // is the balancer active. Must be a boolean, so balancer can be switched on when calibrating
       isFirstLine, // is the first line of the motion. TODO: lineCounter == 0?
       breakUpOn, // when calibrating the motion shall not break up
       isMirror, // is current motion mirrored
       isInOptionalLine, // is current keyframe a optional keyframe
       wasInWaiting, // get up engine was in a waiting state
       frontHackTriggered, // arms were unable to move in the first keyframe of the front get up
       getUpMirror, // should the motion front and back be mirrored
       didFirstGetUp, // For the first get up decide if the motion shall be mirrored or not
       doFastStand, // If the fallEngine detected a fall but robot did not fall, go directly into a stand motion to save time
       finishGettingUp, // If the motion request changes from decideAutomatic to another motion and the robot is still doing a recover, he will abort the get up without this flag
       doSlowKeyframeAfterFall; // When a non get up motion went into the breakUp state and is upright again, execute the first keyframe slow. We assume the robot is hold by a human

  Pose2f odometry; // current odometry

  float variableValuesCompare[numOfConditionVars]; // To break up a wait time early

  MotionRequest currentKeyframeMotionRequest;
  MotionRequest lastKeyframeMotionRequest;

  JointRequest lastJointRequest;

  /**
       * Converts the given angle from degrees to radians, but doesn't change off
       * or ignore
       * @param angle The Angle in degrees
       * @return The Angle in radians
       */
  Angle convertToAngleSpecialCases(float angle);

public:

  /**
   * The method checks if all joints are working and if there is a problem.
   * Problems are:
   *  * if a Joint cant reach a given angle because it is to hot or broken
   *  * a joint is to slow in reaching it given angle
   * These checks are only done, if the current motions specifically wants to check it.
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
  void checkMotion();

  /**
   * The method checks, if the current motion shall break up.
   */
  bool isInBreakUpRange();

  /**
   * The method checks, if a motion want, when in break up, to do a soft reset and restart the motion from a specific keyframe immediately.
   */
  bool checkMotionSpecificActions();

  /*
   * The method checks for the motion front, if both arms did not move correctly at the start.
   * @return true, if both arms did not move to the front
   */
  bool checkRetryForFront();

  /**
   * The method calculates the current ratio for the interpolation.
   * If the current motion line is finished, then it will set the next motion line or if the motion is finished, it will set the state to finished.
   */
  void isCurrentLineOver();

  /**
   * The method initializes everything that needs to be done specifically for this Motion.
   */
  void initGetUpMotion();

  /*
   * The method initializes everything that needs to be done specifically for this motion line.
   */
  void initCurrentLine();

  /**
   * Update the current values which are needed to calculated the current joints and balancer values
   */
  void updateLineValues();

  /**
   * Init the values that are needed in the method pidWithCom at the start of a keyframe
   */
  void initBalancerValues(bool calculateCurrentNew);

  /**
   * The method checks if the KeyframeMotionEngine still needs to wait until it can start doing stuff. Reason is, if the robot is still
   * falling it would be a really bad idea to start a motion.
   */
  void waitForFallen();

  /**
   * Moves the robot into a specific pose. For Debugging stuff.
   */
  void doCalibrationState();

  /**
   * Fail-Save Method, to cancel the current motion if 2 of 3 head sensors are touched.
   */
  void doManipulateOwnState();

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
  void checkOptionalLine();

  /**
   * Checks the conditions, if the current keyframe should be skipped.
   */
  template<class C>
  bool checkConditions(std::vector<C>& conditions, bool useAnd);

  /**
   * The PID-Controller to balance the motion
   */
  void pidWithCom();

  /**
   * Adds the calculated balancer value on top of the joints
   */
  void addBalanceFactor(float factorY, float factorX);

  /**
   * The method calculates for joints, that shall be compensated, with which joints and how much they shall be compensated
   */
  void addJointCompensation();

  /**
   * Calculates the difference of the com relative to the ground floor and support polygon middle of the robot.
   */
  void calculateCOMInSupportPolygon();

  JointRequest calculatedCurrentJointRequestWithoutBalance();

  /**
   * The method updates the parameters which saves the torso angle of the last few frames and updates the prediction for the next few frames.
   */
  void updateSensorArray();

  /**
   * Balance robot after finished get up motion
   */
  void doBalanceOut();

  /*
   * This method is used to force joints to not move for a period of time. It is used to evaluate the get up motions, because it happens often that single
   * joints like the hipYawPitch get stuck for several seconds and causes the get up try to fail.
   */
  void checkJointFailureBehavior();
};
