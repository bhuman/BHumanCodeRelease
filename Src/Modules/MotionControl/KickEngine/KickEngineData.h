/**
 * @file Modules/MotionControl/KickEngineData.h
 * This file declares a module that creates the walking motions.
 * @author <A href="mailto:judy@tzi.de">Judith Müller</A>
 * @author Philip Reichenberg
 */

#pragma once

#include "KickEngineParameters.h"
#include "Platform/BHAssert.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Sensing/InertialData.h"
#include "Tools/RingBufferWithSum.h"

#include <vector>

struct JointLimits;

class KickEngineData
{
public:
  bool startComp = false;
  Pose2f odometryOutput;
  int phaseNumber = 0;
  KickEngineParameters currentParameters;
  ENUM_INDEXED_ARRAY(Angle, Joints::Joint) currentTrajetoryOffset,
                     lastTrajetoryOffset;
  unsigned int timestamp = 0;
  float phase = 0.f;

private:
  bool toLeftSupport = false;
  bool formMode = false;
  bool limbOff[Phase::numOfLimbs];

  int motionID = -1;

  float cycleTime = Constants::motionCycleTime;

  int timeSinceTimestamp = 0;

  Vector2f bodyAngle = Vector2f::Zero();
  Vector2f balanceSum = Vector2f::Zero();
  Vector2f gyro = Vector2f::Zero();
  Vector2f lastGyroLeft = Vector2f::Zero();
  Vector2f lastGyroRight = Vector2f::Zero();
  Vector2f gyroErrorLeft = Vector2f::Zero();
  Vector2f gyroErrorRight = Vector2f::Zero();
  Vector2f lastBody = Vector2f::Zero();
  Vector2f bodyError = Vector2f::Zero();
  bool lElbowFront = false,
       rElbowFront = false;

  //Parameter for P, I and D for gyro PID Control
  Vector2f gyroP = Vector2f(3.f, -2.5f);
  Vector2f gyroI = Vector2f::Zero();
  Vector2f gyroD = Vector2f(0.03f, 0.01f);

  Vector2f head = Vector2f::Zero();

  Vector3f origins[Phase::numOfLimbs];

  Vector3f torsoRot = Vector3f::Zero();

  Vector3f lastCom = Vector3f::Zero();
  Vector3f ref = Vector3f::Zero();
  Vector3f actualDiff = Vector3f::Zero();

  RobotModel comRobotModel;

  JointRequest lastBalancedJointRequest;
  JointRequest compenJoints;

  Pose2f lastOdometry;

  bool wasActive = false;

public:
  RobotModel robotModel;
  KickRequest currentKickRequest;
  bool internalIsLeavingPossible = false;
  Vector3f positions[Phase::numOfLimbs];
  bool getMotionIDByName(const KickRequest& kr, const std::vector<KickEngineParameters>& params);
  void calculateOrigins(const KickRequest& kr, const JointAngles& ja);
  bool checkPhaseTime(const FrameInfo& frame, const JointAngles& ja);
  void balanceCOM(JointRequest& joints, const RobotDimensions& rd, const MassCalibration& mc);

  bool calcJoints(const InertialData& inertialData, JointRequest& jointRequest, const RobotDimensions& rd, const DamageConfigurationBody& theDamageConfigurationBody, const bool calcSideCorrection);
  void calcOdometryOffset(const RobotModel& theRobotModel);
  void simpleCalcArmJoints(const Joints::Joint& joint, JointRequest& jointRequest, const RobotDimensions& theRobotDimensions, const Vector3f& armPos, const Vector3f& handRotAng);
  void calcLegJoints(const Joints::Joint& joint, JointRequest& jointRequest, const RobotDimensions& theRobotDimensions, const DamageConfigurationBody& theDamageConfigurationBody);
  void mirrorIfNecessary(JointRequest& joints);
  void addGyroBalance(JointRequest& jointRequest, const JointLimits& jointLimits, const InertialData& id);
  void addDynPoint(const DynPoint& dynPoint);
  void ModifyData(JointRequest& kickEngineOutput);
  void calcPhaseState();
  void calcPositions();
  void initData(const InertialData& inertialData, const FrameInfo& frame, const KickRequest& kr, const std::vector<KickEngineParameters>& params, const JointAngles& ja, JointRequest& jointRequest, const RobotDimensions& rd, const MassCalibration& mc, const DamageConfigurationBody& theDamageConfigurationBody);
  bool activateNewMotion(const KickRequest& br);
  bool sitOutTransitionDisturbance(bool& compensate, bool& compensated, const InertialData& id, JointRequest& jointRequest, const JointRequest& theJointRequest, const FrameInfo& frame);
  void applyTrajetoryAdjustment(JointRequest& jointRequest, const JointLimits& limits);
  Angle interpolate(Angle from, Angle to, float currentTime, KickEngineParameters::BoostAngle::InterpolationMode mode);
  void BOOST(JointRequest& jointRequest, int boostPhase);

  KickEngineData()
  {
    for(int i = 0; i < Phase::numOfLimbs; i++)
      limbOff[i] = false;
  }
};
