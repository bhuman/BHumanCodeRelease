/**
 * @file GetUpEngine.h
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
#include "Representations/Configuration/GyroOffset.h"
#include "Representations/Configuration/JointLimits.h"

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"

#include "Representations/MotionControl/HeadAngleRequest.h"
#include "Representations/MotionControl/GetUpEngineOutput.h"
#include "Representations/MotionControl/GetUpEngineOutputLog.h"
#include "Representations/MotionControl/GetUpMotionPhase.h"
#include "Representations/MotionControl/GetUpPhase.h"
#include "Representations/MotionControl/LegMotionSelection.h"
#include "Representations/MotionControl/MotionInfo.h"

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
#include "Tools/Motion/GetUpMotion.h"
#include "Tools/Motion/MotionUtilities.h"
#include "Tools/RingBuffer.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/EnumIndexedArray.h"
#include "Tools/Range.h"

using GetUpMotion = GetUpMotions::GetUpMotion;
using EngineState = EngineStates::EngineState;
using Phase = GetUpMotionPhases::GetUpMotionPhase;
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
  (bool)(false) pidDXcontroller,
});

// If u want to add variables, then u need to add them in the methods:
// checkConditions for other usages
ENUM(ConditionVar,
{,
  InertialDataAngleY, // Torso angle forward
  InertialDataAngleX, // Torso angle sideways
  FluctuationY, // Fluctuation forward
  FluctuationX, // Fluctuation sidewards
  BrokenLeftArm, // Left arm is stuck
  BrokenRightArm, // Right arm is stuck
  OptionalLineFront, // Execute Keyframe/Waitime of value corresponds to value in the damageConfiguration
  OptionalLineBack, // Execute Keyframe/Waitime of value corresponds to value in the damageConfiguration
  WaitTime, // Wait time of previous keyframe
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
  (float)(0.2f) reduceFactorJointCompensation, // reduceFactor to reduce over compensating. Only used if predictJointDif = true
});

STREAMABLE(MofLine,
{,
  (Phase) phase, // name of the current phase
  (float)(0.f) duration, // duration of the current keyframe
  (Vector2f)(0.f, 0.f) goalCom, // refrence com
  (bool)(false) setLastCom, // shall last com be overwritting with the current com at the start of the keyframe?
  (bool)(false) optionalLineAnds, // ands the conditions
  (bool)(false) isElseBlock, // else block of optionale keyframes
  (bool)(false) isPartOfPreviousOptionalLine, // part of a optionalLine
  (bool)(false) forbidWaitBreak, // Wait time is not allowed to break up early if goal angle is not reachable
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
  (GetUpMotion)(GetUpMotions::doNothing) continueTo, // Self if no motion should follow
  (Angle)(0) clipAngle, // Positive angle -> clip forwardDifAngleBreak, if negativ angle -> clip forwardDifAngleBreak.
});

STREAMABLE(BalanceOutParams,
{,
  (int) maxTime, // max time in balanceOut state (in ms)
  (Angle) minFluctuation, // min fluctuation to be able to leave early
  (Angle) minForwardAngle, //min forward Angle to be able to leave early
  (Angle) minBackwardAngle, //min backward Angle to be able to leave early
  (float) minPIDDValue, // min d value of pid controller
});

MODULE(GetUpEngine,
{,
  REQUIRES(DamageConfigurationBody),
  USES(HeadAngleRequest),
  REQUIRES(FilteredCurrent),
  REQUIRES(FrameInfo),
  REQUIRES(GetUpPhase),
  REQUIRES(GyroOffset),
  REQUIRES(InertialData),
  REQUIRES(JointAngles),
  REQUIRES(JointSensorData),
  REQUIRES(KeyStates),
  REQUIRES(LegMotionSelection),
  REQUIRES(MotionRequest),
  REQUIRES(RobotModel),
  USES(JointLimits),
  USES(MotionInfo),
  PROVIDES(GetUpEngineOutput),
  REQUIRES(GetUpEngineOutput),
  PROVIDES(GetUpEngineOutputLog),
  LOADS_PARAMETERS(
  {,
    (int) maxTryCounter, // Number of allowed get up tries
    (int) motionSpecificRetrys, // Number of allowed retries
    (int) motionSpecificRetrysFront, // Number of allowed retries to move the arms to the side for the front get up
    (Angle) shoulderPitchThreshold, // Threshold to trigger retry in front get up
    (bool) motorMalfunctionBreakUp, // Go to helpMeState if a motorMalfunction was detected once
    (ENUM_INDEXED_ARRAY(Mof, GetUpMotions::GetUpMotion)) mofs, // The different get up motions
    (ENUM_INDEXED_ARRAY(float, Joints::Joint)) brokenJointData, // Thresholds to detect if a joint is too far of the request
    (ENUM_INDEXED_ARRAY(float, GetUpMotionPhases::GetUpMotionPhase)) headInterpolation, // Interpolation ratios for head angles from headJointRequest and getUpEngine. This way the head reaches the position it should have when leaving the getUpEngine
    (Angle) minJointCompensationReduceAngleDif, // Threshold for joint compensation. Joint must change its position by this value between 2 frames to trigger a reduction in the joint compensation
    (float) jointCompensationSimulationFactor, // Value for forcing stucked joints
    (std::vector<GetUpMotionPhases::GetUpMotionPhase>) clipBreakUpAngle, // In these motion phases, the break angles are clipped to 0 degree for one side, depending on which get up motion is executed
    (BalanceOutParams) balanceOutParams, // The parameters for the balanceOutMethode
    (Vector3f) supportPolygonOffsets, // Offsets for the support polygon. x Forward, y Backward, z Sideways
  }),
});

class GetUpEngine : public GetUpEngineBase
{
private:

  STREAMABLE(JointRequestAngles,
  {,
    (ENUM_INDEXED_ARRAY(Angle, Joints::Joint)) angles,
  });

  EngineStates::EngineState state; // defines the current engine state

  GetUpMotions::GetUpMotion motionID;  // the name of the current motion

  unsigned int lineStartTimestamp,  // timestamp when a the interpolation between key frames started
           lastNotActiveTimestamp, // timestamp last time we were in a breakUp
           waitTimestamp, // timestamp for wait times, to stop waittimes, that are too long
           getUpFinishedTimStamp, // not used. can be deleted
           helpMeTimestamp, // timestamp for last help me sound
           breakUpTimestamp, // timestamp when break up started
           lastPIDCom, // timestamp last time the PID values were changed
           retryTimestamp, // retry hack, so we can retry keyframes without doing the whole motion over again
           doBalanceOutTimestamp, // timestamp used to decide when we can shutdown the engine
           abortWithHeadSensorTimestamp; // timestamp last time the head sensores where touched

  int breakCounter, // number of normal retried get up tries
      retryTime, // Wait time befor a retry is allowed
      lineCounter, // current keyframe
      maxCounter, // number of keyframes of current motion
      tryCounter, // number of tries
      retryMotionCounter; // retry whole get up motion counter, so we do not do too many retry. Otherwise we assume a hardware failure and just try our luck.

  JointAngles startJoints; // measured joint data when the last key frame was passed
  JointRequest lastUnbalanced, // calculated last target joint angles without balance added
               targetJoints,   // calculated next target joint angles without balance added
               lineJointRequest; // joint angles from the config for the current keyframe

  JointRequestAngles jointDif1Angles, // saved the difference of the request and reached angles with a delay
                     jointDifPredictedAngles, // dif of set and reached joint angles
                     lineJointRequestAngles, // current goal angles of the current keyframe
                     lineJointRequest2Angles, // saves lineJointRequest2
                     lastLineJointRequestAngles; // saves the lineJointRequest of the keyframe before.

  std::array<JointRequestAngles, 4> pastJointAnglesAngles; // saves the last set joint angles

  //Which joints are used for balancing from the previous keyframe
  std::vector<Joints::Joint> jointsBalanceY,
      jointsBalanceX;

  //Last 2 Gyro values
  RingBuffer<Vector2a, 3> lastTorsoAngle;

  std::array<float, Joints::numOfJoints> jointCompensationReducer; // how much is the last jointCompensation of the keyframe before already reduced? (Value between 0 and 1)

  // TODO BackwardAngle and ForwardAngle are not Angles (!), rename them. Also Backward/ForwardBreakDif should be angles and not floats
  // first values are saved value of the last keyframe
  Vector2f lastBackwardCOMDif, // max backward com dif, before balancer value is increased
           lastForwardCOMDif, // max forward com dif, before the balancer value is increaed by 100%
           currentBackwardCOMDif, // Max com backward to increase balancer values
           currentForwardCOMDif, // Max com forward to increase balancer values

           lastGoal, // last com goal
           goalGrowth, // how much shall com_foot_last - com_foot_current be in a ideal world?
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
       specialActionCheck, // Check if the getUpEngine went active after a specialAction was executed
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
       initPhase, // initial state for the celibration state
       balancerOn, // is the balancer active. Must be a boolean, so balancer can be switched on when calibrating
       isFirstLine, // is the first line of the motion. TODO: lineCounter == 0?
       breakUpOn, // when calibrating the motion shall not break up
       isMirror, // is current motion mirrored
       isInOptionalLine, // is current keyframe a optional keyframe
       wasInWaiting, // get up engine was in a waiting state
       getUpMirror, // should the motion front and back be mirrored
       didFirstGetUp, // For the first get up decide if the motion shall be mirrored or not
       stepKeyframes, // Execute every keyframe slow and wait for request
       isLeavingPossible, // Copy of the field in the getUpEngineOutput
       doFastStand; // If the fallEngine detected a fall but robot did not fall, go directly into a stand motion to save time

  Pose2f odometry; // current odometry

  float variableValuesCompare[numOfConditionVars]; // To break up a wait time early

  /**
       * Converts the given angle from degrees to radians, but doesn't change off
       * or ignore
       * @param angle The Angle in degrees
       * @return The Angle in radians
       */
  Angle convertToAngleSpecialCases(float angle);

public:
  GetUpEngine();
  void update(GetUpEngineOutput& output) override;
  void update(GetUpEngineOutputLog& output) override;

  /**
   * The method checks and updateds the state of the GetUpEngine.
   * The states representate, if the Engine is working currently and for what.
   */
  void checkOwnState(GetUpEngineOutput& output);

  /**
   * The method checks if all joints are working and if there is a problem.
   * Problems are:
   *  * if a Joint cant reach a given angle because it is to hot or broken
   *  * a joint is to slow in reaching it given angle
   * These checks are only done, if the current motions specificly wants to check it.
   */
  void calculateJointDifference(GetUpEngineOutput& output);

  /**
   * The method checks, if a recover is needed after the robot is fallen.
   * Possible cases are:
   *  * he is lying on the side
   *  * he is lying on his arm
   *  * one arm is not correct.
   * This needs to be done to prevent that the arms are in wrong positions, because the getupmotions
   * assume, that the robot starts in a specific position. Else the first try could fail or the arms could get damaged.
   */
  void checkRF(GetUpEngineOutput& output);

  /**
   * The method checks, if the current motion shall break up.
   */
  bool isInBreakUpRange(GetUpEngineOutput& output);

  /**
   * The method checks, if a motion want, when in break up, to do a soft reset and restart the motion from a specific keyframe immediately.
   */
  bool checkMotionSpecificActions(GetUpEngineOutput& output);

  /*
   * The method checks for the motion front, if both arms did not move correctly at the start.
   * @return true, if both arms did not move to the front
   */
  bool checkRetryForFront(GetUpEngineOutput& output);

  /**
   * The method calculates the current ratio for the interpolation.
   * If the current motion line is finished, then it will set the next motion line or if the motion is finished, it will set the state to finished.
   */
  void isCurrentLineOver(GetUpEngineOutput& output);

  /**
   * The method initializes everything that needs to be done specifically for this Motion.
   */
  void initGetUpMotion(GetUpEngineOutput& output);

  /*
   * The method initializes everything that needs to be done specifically for this Motionline.
   */
  void initCurrentLine(GetUpEngineOutput& output);

  /**
   * Update the current values which are needed to calculated the current joints and balancer values
   */
  void updateLineValues(GetUpEngineOutput& output);

  /**
   * Init the values that are needed in the method pidWithCom at the start of a keyframe
   */
  void initBalancerValues(GetUpEngineOutput& output, bool calculateCurrentNew);

  /**
   * The method checks if the GetUpEngine still needs to wait until it can start doing stuff. Reason is, if the robot is still
   * falling it would be a really bad idea to start a motion.
   */
  void waitForFallen(GetUpEngineOutput& output);

  /**
   * Fail-Save Method, to cancle the current motion if 2 of 3 head sensors are touched.
   */
  void doManipulateOwnState(GetUpEngineOutput& output);

  /**
   * The method does special actions if the current state is helpMeState. This needs to be done, because the robot is unable to get up.
   */
  void doHelpMeStuff(GetUpEngineOutput& output);
  /**
   * Sets the current Jointgoals for MotionLine. This is done once, everytime a new MotionLine is started.
   */
  void setNextJoints(GetUpEngineOutput& output);

  /**
   * Sets the stiffnessvalues of the Joints. Must be between 0 and 100.
   */
  void setJointStiffness(GetUpEngineOutput& output);

  /**
   * The methods checks the enginestate finished. If the motions has the check to balance out, then the getUpEngine will balance the standing
   * robot befor he is allowed to walk. This is done because the roboter could have some momentum left, that would cause him to
   * fall instantly again.
   *
   * If the check is off, then the method will do nothing
   *
   * If there is a connected motion (continueTo), then this motion will get set next.
   */
  void checkFinishedState(GetUpEngineOutput& output);

  /**
   * Checks, if the current keyframe should be skipped.
   */
  void checkOptionalLine(GetUpEngineOutput& output);

  /**
   * Checks the conditions, if the current keyframe should be skipped.
   */
  template<class C>
  bool checkConditions(std::vector<C>& conditions, bool useAnd, GetUpEngineOutput& output);

  /**
   * The PID-Controller to balance the motion
   */
  void pidWithCom(GetUpEngineOutput& output);

  /**
   * Adds the calculated balancer value on top of the joints
   */
  void addBalanceFactor(GetUpEngineOutput& output, float factorY, float factorX, float addRatio);

  /**
   * The method calculates for joints, that shall be compensated, with which joints and how much they shall be compensated
   */
  void addJointCompensation(GetUpEngineOutput& output);

  /**
   * Calculates the difference of the com releative to the ground floor and support polygon middle of the robot.
   */
  void calculateCOMInSupportPolygon(GetUpEngineOutput& output);

  JointRequest calculatedCurrentJointRequestWithoutBalance(GetUpEngineOutput& output);

  /**
   * The method updates the parameters which saves the torso angle of the last few frames and updates the prediction for the next few frames.
   */
  void updateSensorArray(GetUpEngineOutput& output);

  /**
   * Balance robot after finished get up motion
   */
  void doBalanceOut(GetUpEngineOutput& output);

  /*
   * Sets all values to default
   */
  void shutDownGetUpEngine(GetUpEngineOutput& output);
};
