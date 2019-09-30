/**
 * @file Modules/MotionControl/KickEngineData.h
 * This file declares a module that creates the walking motions.
 * @author <A href="mailto:judy@tzi.de">Judith Müller</A>
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
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Sensing/InertialData.h"
#include "Tools/RingBufferWithSum.h"

#include <vector>

struct JointLimits;

class KickEngineData
{
private:
  bool toLeftSupport = false;
  bool formMode = false;
  bool limbOff[Phase::numOfLimbs];
  bool startComp = false;

  int phaseNumber = 0;
  int motionID = -1;

  float phase = 0.f;
  float cycletime = Constants::motionCycleTime;
  float lastRatio = 0.f;

  unsigned int timestamp = 0;
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
  Pose2f lastOdometry;
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

  KickEngineParameters currentParameters;

  RobotModel comRobotModel;

  JointRequest lastBalancedJointRequest;
  JointRequest compenJoints;

  bool wasActive = false;
  bool willBeLeft = false;
  float lastXDif = 0.f;
  float lastZDif = 0.f;
  bool fastKickEndAdjusted = false;
  float adjustedXValue = 0.f;
  float adjustedZValue = 0.f;

public:
  RobotModel robotModel;
  KickRequest currentKickRequest;
  bool internalIsLeavingPossible = false;
  Vector3f positions[Phase::numOfLimbs];
  bool getMotionIDByName(const KickRequest& kr, const std::vector<KickEngineParameters>& params);
  void calculateOrigins(const KickRequest& kr, const JointAngles& ja, const TorsoMatrix& to, const RobotDimensions& theRobotDimensions);
  bool checkPhaseTime(const FrameInfo& frame, const JointAngles& ja, const TorsoMatrix& torsoMatrix);
  void balanceCOM(JointRequest& joints, const RobotDimensions& rd, const MassCalibration& mc);

  bool calcJoints(JointRequest& jointRequest, const RobotDimensions& rd, const DamageConfigurationBody& theDamageConfigurationBody);
  void calcOdometryOffset(KickEngineOutput& output, const RobotModel& theRobotModel);
  void calcLegJoints(const Joints::Joint& joint, JointRequest& jointRequest, const RobotDimensions& theRobotDimensions, const DamageConfigurationBody& theDamageConfigurationBody);
  void simpleCalcArmJoints(const Joints::Joint& joint, JointRequest& jointRequest, const RobotDimensions& theRobotDimensions, const Vector3f& armPos, const Vector3f& handRotAng);

  void mirrorIfNecessary(JointRequest& joints);
  void addGyroBalance(JointRequest& jointRequest, const JointLimits& jointLimits, const InertialData& id, const float& ratio);
  void addDynPoint(const DynPoint& dynPoint, const TorsoMatrix& torsoMatrix);
  void ModifyData(const KickRequest& br, JointRequest& kickEngineOutput, std::vector<KickEngineParameters>& params);
  void calcPhaseState();
  void calcPositions(const TorsoMatrix& torsoMatrix);
  void setExecutedKickRequest(KickRequest& br);
  void initData(const FrameInfo& frame, const KickRequest& kr, std::vector<KickEngineParameters>& params, const JointAngles& ja, const TorsoMatrix& torsoMatrix, JointRequest& jointRequest, const RobotDimensions& rd, const MassCalibration& mc, const DamageConfigurationBody& theDamageConfigurationBody);
  void setEngineActivation(const float& ratio);
  bool activateNewMotion(const KickRequest& br, const bool& isLeavingPossible);
  bool sitOutTransitionDisturbance(bool& compensate, bool& compensated, const InertialData& id, KickEngineOutput& kickEngineOutput, const JointRequest& theJointRequest, const FrameInfo& frame);
  void BOOST(JointRequest& jointRequest, int boostPhase);
  bool adjustFastKickHack(const TorsoMatrix& torsoMatrix);

  KickEngineData()
  {
    for(int i = 0; i < Phase::numOfLimbs; i++)
      limbOff[i] = false;
  }
};
