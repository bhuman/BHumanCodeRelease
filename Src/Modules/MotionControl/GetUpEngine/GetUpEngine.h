/**
 * @file GetUpEngine.h
 * A minimized motion engine for standing up.
 * @author <A href="mailto:judy@tzi.de">Judith Müller</A>
 * @author Marvin Franke
 * @author Philip Reichenberg
 */

#pragma once
#include "Representations/MotionControl/GetUpEngineOutput.h"
#include "Representations/MotionControl/GetUpEngineOutputLog.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/LegMotionSelection.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/MotionControl/Balancer.h"
#include "Representations/Configuration/GetupMotion.h"
#include "Tools/Module/Module.h"
#include "Tools/Streams/AutoStreamable.h"
using GetUpMotion = GetUpMotions::GetUpMotion;

STREAMABLE(StiffnessPair,
{
  StiffnessPair() = default;
  StiffnessPair(Joints::Joint joint, int s),

  (Joints::Joint) joint,
  (int) s,
});

inline StiffnessPair::StiffnessPair(Joints::Joint joint, int s) : joint(joint), s(s) {}

//If u want to add variables, then u need to add them in the methods:
//checkConditions() for other usages
//in the import and export methods (for the parsing)
ENUM(ConditionVar,
{,
  InertialDataAngleY,
  InertialDataAngleX,
  OptionalLineFront,
  OptionalLineBack,
});

//The parameters for the conditions
//these values are needed to the method checkConditions()
STREAMABLE(Condition,
{,
  (ConditionVar) variable,
  (float) lowerFloat,
  (float) higherFloat,
  (bool) isNot,
});

STREAMABLE(MofLine,
{,
  (float[2]) head,
  (float[6]) leftArm,
  (float[6]) rightArm,
  (float[6]) leftLeg,
  (float[6]) rightLeg,
  (std::vector<StiffnessPair>) singleMotorStiffnessChange, //from this line on use for joint stiffness
  (float) duration,
  (bool) critical, //check for current line if torso is too tilted
  (bool) isPartOfPreviousOptionalLine, // part of a optionalLine
  (std::vector<Condition>) conditions,
  (bool) optionalLineAnds, // ends the optionalLine
  (bool) isElseBlock,
  (bool) balanceWithHipAndAnkleY, // balance with HipPitch and AnklePitch
  (bool) balanceWithHipAndAnkleX, // balance with HipRoll and AnkleRoll
  (bool) increaseBalanceY, // increase balanceWithHipAndAnkleY
  (bool) increaseBalanceX, // increase balanceWithHipAndAnkleX
  (float) waitDuration,
  (bool) waitConditionAnds,
  (std::vector<Condition>) waitConditions,
});

STREAMABLE(Mof,
{,
  (int[5]) baseLimbStiffness, //head, lArm, rArm, lLeg, rLeg
  (std::vector<MofLine>) lines,
  (int) balanceStartLine,
  (int) balanceArmStartLine, //must be lower than balancerStartLine
  (int) balanceArmAndLegStartLine, //must be lower than balancerStartLine
  (Pose2f) odometryOffset,
  (GetUpMotion) continueTo, // self if no motion should follow
});

STREAMABLE(GetUpComparisonLine,
{,
  (float) anglesDeviation,
  (bool) criticalIdentical,
  (float) durationDifference,
});

STREAMABLE(GetUpComparisonBaseLine,
{,
  (std::vector<GetUpComparisonLine>) lineComparisons,
  (int) startLineBase,
  (int) startLineCompare,
});

STREAMABLE(GetUpComparison,
{,
  (std::vector<GetUpComparisonBaseLine>) baseComparisonLines,
});

MODULE(GetUpEngine,
{,
  REQUIRES(FrameInfo),
  REQUIRES(InertialData),
  REQUIRES(JointAngles),
  REQUIRES(LegMotionSelection),
  REQUIRES(DamageConfigurationBody),
  REQUIRES(Balancer),
  USES(MotionInfo),
  PROVIDES(GetUpEngineOutput),
  REQUIRES(GetUpEngineOutput),
  PROVIDES(GetUpEngineOutputLog),
  LOADS_PARAMETERS(
  {,
    (ENUM_INDEXED_ARRAY(Mof, GetUpMotions::GetUpMotion)) mofs,
    (BalancingParameter) balancingParams,
    (BalancingParameter) balancingParamsForArms,
    (Vector2f) leftLIPOrigin,
  }),
});

class GetUpEngine : public GetUpEngineBase
{
private:
  ENUM(EngineState,
  {,
    decideAction,
    working,
    breakUp,
    recover,
    pickUp,
    schwalbe,
  });

  EngineState state; // defines the current engine state

  bool wasActive, //if the engine was active in the last frame
       balance,   //toggles balance on or off
       balanceArm,
       balanceArmAndLeg,
       isInOptionalLine,
       mirror; //toggle if motion should be mirrored

  int lineCounter, //the current key frame of the motion sequence
      maxCounter, //the number of key frames of the current motion sequence
      currentBalanceLine;

  GetUpMotions::GetUpMotion motionID;  //the name of the current motion

  float ratio, //0-1 interpolation state between key frame
        upperArmBorder,//maximum positiv angle correction for the arms in the method correctAngleMotion()
        upperLegBorder,//maximum positiv angle correction for the legs in the method correctAngleMotion()
        lowerArmBorder,//maximum negativ angle correction for the arms in the method correctAngleMotion()
        lowerLegBorder,//maximum negativ angle correction for the legs in the method correctAngleMotion()
        gBufferY, //buffer to summarize gyro data
        gLastY, // last InertialData.gyro.y() value
        gBufferX, //buffer to summarize gyro data
        gLastX, // last InertialData.gyro.x() value
        kp, //pid factors
        ki,
        kd,
        balanceHipAngleY;

  //The difference of the requested joint angle (from the cfg) and the actually reached joint angle
  //the 6. value of lArmDif and rArmDif is always 0.f, because we dont use the hand joint.
  std::vector<float> lArmDif = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
                     rArmDif = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
                     lLegDif = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
                     rLegDif = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};

  MotionInfo lastMotionInfo; //saves the last MotionInfo

  unsigned int lineStartTimeStamp,  //time stamp when a the interpolation between key frames started
           lastNotActiveTimeStamp, //time stamp of the last time the engine was not active
           breakUpTimeStamp,  //time stamp when the last unsuccesful try was detected
           soundTimeStamp, //time stamp to coordinate the cry for help (request for pickup)
           recoverTimeStamp; //time stamp so the recover motion get used only once per get up motion

  JointAngles startJoints; //measured joint data when the last key frame was passed
  JointRequest lastUnbalanced, //calculated last target joint angles without balance added
               targetJoints;   //calculated next target joint angles without balance added

  Pose2f internalOdometryOffset; //stores the next odometry offset

  GetUpComparison comparison;

  /**
   * Converts the given angle in degrees to radians, but doesn't change off
   * or ignore
   * @param angle The Angle in degrees
   * @return The Angle in radians
   */
  Angle convertToAngleSpecialCases(float angle);

public:

  void update(GetUpEngineOutput& output) override;
  void update(GetUpEngineOutputLog& output) override;

  GetUpEngine() :
    state(decideAction),
    wasActive(false),
    balance(false),
    balanceArm(false),
    balanceArmAndLeg(false),
    isInOptionalLine(false),
    lineCounter(0),
    maxCounter(-1),
    motionID(GetUpMotion::stand),
    upperArmBorder(10.f),
    upperLegBorder(2.f),
    lowerArmBorder(-10.f),
    lowerLegBorder(-2.f),
    gBufferY(0.f),
    gLastY(0.f),
    gBufferX(0.f),
    gLastX(0.f),
    kp(2.5f),
    ki(0.1f),
    kd(0.01f),
    balanceHipAngleY(0.f),
    lineStartTimeStamp(0),
    lastNotActiveTimeStamp(0),
    breakUpTimeStamp(0),
    soundTimeStamp(0),
    recoverTimeStamp(0),
    internalOdometryOffset(Pose2f())
  {
  }

  /**
   * The method initializes the next motion based on the current engine state,
   * the FallDownState and the measured body angle.
   * @param output the GetUpEngineOutput object for changing joint stiffness
   */
  void pickMotion(GetUpEngineOutput& output);

  /**
   * The method initializes the global variables such as lineCounter
   */
  void initVariables();

  /**
   * The method adds a gyro feedback balance using PID control to the calculated jointRequest for Y direction
   * @param jointRequest joint request the balance should be added to
   */
  float addBalanceY(JointRequest& jointRequest);

  /**
   * The method adds a gyro feedback balance using PID control to the calculated jointRequest for X direction
   * @param jointRequest joint request the balance should be added to
   */
  float addBalanceX(JointRequest& jointRequest);

  /**
   * The method calculates the next JointRequest and counts the key frames up
   * @param output the GetUpEngineOutput object for changing target joint angles
   */
  void setNextJoints(GetUpEngineOutput& output);

  /**
   * The method writes a stiffness to the output
   * @param output the GetUpEngineOutput object the stiffness should be set to
   * @param stiffness 0-100 percent stiffness
   */
  void setStiffnessAllJoints(GetUpEngineOutput& output, int stiffness);

  /**
   * The method writes the base stiffness of the current motion to the targetJoints
   * no modification if current motion is invalid
   * @param output the GetUpEngineOutput object the stiffness should be set to
   */
  void setBaseLimbStiffness();

  /**
   * The method looks up the current motion parameters and intializes maxLineCounter, internalOdometryOffset, and motionID
   * @param current the motion to be initialized
   */
  void setCurrentMotion(GetUpMotion current);

  /**
   * checks if the torso is too tilted. If so, stop current motions and set all stiffness to 0.
   */
  bool verifyUprightTorso(GetUpEngineOutput& output);

  /*
   * This method writes a getUpMotion into getUpEngineDummy.mof for better debugging
   * It needs to be done offline (in simulation) and than deployed to the robot since the
   * "mof"-command (in simulation) would copy the offline file instead of using the online generated one.
   * @param motion motion to be written into specialAction getUpEngineDummy.mof
   */
  void generateMofOfMotion(GetUpMotion motionName);

  /*
   * This method imports a getUpEngineDummy.mof file into this engine, use it only offline in simulation!!!
   * use "save parameters:GetUpEngine" to write getUpEngine.cfg
   * Settings regarding critical lines and balance have to be done manually!!!
   * @param motion to be overwritten by specialAction getUpEngineDummy.mof
   */
  void generateMotionOfMof(GetUpMotion motionName);

  /**
   * Does a correction for the angles of the joints, that will be set in 2 specific cases
   * @param lastAngle the last angle that was set
   * @param nextAngle the next to set angle
   * @param dif substraction of nextAngle Minus lastAngle
   *
   * Cases:
   * next >= last && dif > 0 -> next + dif
   * next > last && dif < 0 -> do no correction
   * next <= last && dif < 0 -> next + dif
   * next < last && dif > 0 -> do no cirrection
   */
  float correctAngleMotion(float nextAngle, float lastAngle, float dif, bool isArm);

  /*
   * Method for comparing two GetUpMotions
   * @param motionIdBade first motion
   * @param motionIdCompare second motion
   */
  void compareMotions(GetUpMotion motionIdBase, GetUpMotion motionIdCompare);

  /*
   * Method to check if lines needs to be skipped because of a condition
   */
  void skipForOptionalLine();

  /*
   * Method to check if the conditions are met
   * @param conditions the list of conditions
   * @useAnd the bool if all conditions are chaned with && or ||
   */
  bool checkConditions(std::vector<Condition> conditions, bool useAnd);
};
