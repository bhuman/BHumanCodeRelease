/**
 * @file Modules/MotionControl/KickEngineData.h
 * This file declares a module that creates the walking motions.
 * @author <A href="mailto:judy@tzi.de">Judith Müller</A>
 */

#pragma once

#include "KickEngineParameters.h"
#include "Platform/BHAssert.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/HeadJointRequest.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Sensing/InertialData.h"
#include "Tools/RingBufferWithSum.h"

#include <vector>

class KickEngineData
{
private:
  bool lSupp = false;
  bool rSupp = false;
  bool toLeftSupport = false;
  bool formMode = false;
  bool limbOff[Phase::numOfLimbs];

  int phaseNumber = 0;
  int motionID = -1;

  float phase = 0.f;
  float cycletime = 0.f;
  float lastRatio = 0.f;

  unsigned int timeStamp = 0;
  int timeSinceTimeStamp = 0;

  RingBufferWithSum<Vector2i, 10> standLegPos;

  Vector2f comOffset = Vector2f::Zero();
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

  //Parameter for P, I and D for gyro PID Contol
  Vector2f gyroP = Vector2f(3.f, -2.5f);
  Vector2f gyroI = Vector2f::Zero();
  Vector2f gyroD = Vector2f(0.03f, 0.01f);

  Vector2f origin = Vector2f::Zero();

  Vector2f head = Vector2f::Zero();

  Vector3f positions[Phase::numOfLimbs];
  Vector3f origins[Phase::numOfLimbs];

  Vector3f torsoRot = Vector3f::Zero();

  Vector3f lastCom = Vector3f::Zero();
  Vector3f ref = Vector3f::Zero();
  Vector3f actualDiff = Vector3f::Zero();

  KickEngineParameters currentParameters;

  RobotModel robotModel;
  RobotModel comRobotModel;

  JointRequest lastBalancedJointRequest;
  JointRequest compenJoints;

  bool wasActive = false;
  bool startComp = false;
  bool willBeLeft = false;

public:
  KickRequest currentKickRequest;
  bool internalIsLeavingPossible = false;

  bool getMotionIDByName(const MotionRequest& motionRequest, const std::vector<KickEngineParameters>& params);
  void calculateOrigins(const KickRequest& kr, const JointAngles& ja, const TorsoMatrix& to);
  bool checkPhaseTime(const FrameInfo& frame, const JointAngles& ja, const TorsoMatrix& torsoMatrix);
  void balanceCOM(JointRequest& joints, const RobotDimensions& rd, const MassCalibration& mc);
  void calculatePreviewCom(Vector3f& ref, Vector2f& origin);
  void setStandLeg(const float& originY);
  bool calcJoints(JointRequest& jointRequest, const RobotDimensions& rd, const HeadJointRequest& hr);
  void calcLegJoints(const Joints::Joint& joint, JointRequest& jointRequest, const RobotDimensions& theRobotDimensions);
  void simpleCalcArmJoints(const Joints::Joint& joint, JointRequest& jointRequest, const RobotDimensions& theRobotDimensions, const Vector3f& armPos, const Vector3f& handRotAng);
  void getCOMReference(const Vector3f& lFootPos, const Vector3f& rFootPos, Vector3f& comRef, Vector2f& origin);
  void setStaticReference();
  void mirrorIfNecessary(JointRequest& joints);
  void addGyroBalance(JointRequest& jointRequest, const JointCalibration& jc, const InertialData& id, const float& ratio);
  void addDynPoint(const DynPoint& dynPoint, const TorsoMatrix& torsoMatrix);
  void ModifyData(const KickRequest& br, JointRequest& kickEngineOutput, std::vector<KickEngineParameters>& params);
  void setCycleTime(float time);
  void calcPhaseState();
  void calcPositions();
  void setRobotModel(const RobotModel& rm);
  bool isMotionAlmostOver();
  void setCurrentKickRequest(const MotionRequest& mr);
  void setExecutedKickRequest(KickRequest& br);
  void initData(const FrameInfo& frame, const MotionRequest& mr, std::vector<KickEngineParameters>& params, const JointAngles& ja, const TorsoMatrix& torsoMatrix);
  void setEngineActivation(const float& ratio);
  bool activateNewMotion(const KickRequest& br, const bool& isLeavingPossible);
  bool sitOutTransitionDisturbance(bool& compensate, bool& compensated, const InertialData& id, KickEngineOutput& kickEngineOutput, const JointAngles& ja, const FrameInfo& frame);


  //Pose3f calcDesBodyAngle(JointRequest& jointRequest, const RobotDimensions& robotDimensions, Joints::Joint joint);

  void transferDynPoint(Vector3f& d, const TorsoMatrix& torsoMatrix);

  KickEngineData() : standLegPos(Vector2i::Zero())
  {
    for(int i = 0; i < Phase::numOfLimbs; i++)
      limbOff[i] = false;
  }
};
