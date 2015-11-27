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
  bool lSupp,
       rSupp,
       toLeftSupport,
       formMode,
       limbOff[Phase::numOfLimbs];

  int phaseNumber,
      motionID;

  float phase,
        cycletime,
        lastRatio;

  unsigned int timeStamp;
  int timeSinceTimeStamp;

  RingBufferWithSum<Vector2i, 10> standLegPos;

  Vector2f comOffset = Vector2f::Zero(),
           balanceSum = Vector2f::Zero(),
           gyro = Vector2f::Zero(),
           lastGyroLeft = Vector2f::Zero(),
           lastGyroRight = Vector2f::Zero(),
           gyroErrorLeft = Vector2f::Zero(),
           gyroErrorRight = Vector2f::Zero(),
           lastBody = Vector2f::Zero(),
           bodyError = Vector2f::Zero(),
           gyroP = Vector2f::Zero(),
           gyroI = Vector2f::Zero(),
           gyroD = Vector2f::Zero(),
           origin = Vector2f::Zero();

  Vector2f head = Vector2f::Zero();

  Vector3f positions[Phase::numOfLimbs], origins[Phase::numOfLimbs];

  Vector3f torsoRot = Vector3f::Zero();

  Vector3f lastCom = Vector3f::Zero(), ref = Vector3f::Zero(), actualDiff = Vector3f::Zero();

  KickEngineParameters currentParameters;

  RobotModel robotModel, comRobotModel;

  JointRequest lastBalancedJointRequest, compenJoints;

  bool wasActive, startComp, willBeLeft;

public:
  KickRequest currentKickRequest;
  bool internalIsLeavingPossible;

  bool getMotionIDByName(const MotionRequest& motionRequest, const std::vector<KickEngineParameters>& params);
  void calculateOrigins(const RobotDimensions& theRobotDimension, const KickRequest& kr);
  bool checkPhaseTime(const FrameInfo& frame, const RobotDimensions& rd, const JointAngles& ja, const TorsoMatrix& torsoMatrix);
  void balanceCOM(JointRequest& joints, const RobotDimensions& rd, const MassCalibration& mc);
  void calculatePreviewCom(Vector3f& ref, Vector2f& origin);
  void setStandLeg(const float& originY);
  bool calcJoints(JointRequest& jointRequest, const RobotDimensions& rd, const HeadJointRequest& hr);
  void calcLegJoints(const Joints::Joint& joint, JointRequest& jointRequest, const RobotDimensions& theRobotDimensions);
  static void simpleCalcArmJoints(const Joints::Joint& joint, JointRequest& jointRequest, const RobotDimensions& theRobotDimensions, const Vector3f& armPos, const Vector3f& handRotAng);
  void getCOMReference(const Vector3f& lFootPos, const Vector3f& rFootPos, Vector3f& comRef, Vector2f& origin);
  void setStaticReference();
  void mirrorIfNecessary(JointRequest& joints);
  void addGyroBalance(JointRequest& jointRequest, const JointCalibration& jc, const InertialData& id, const float& ratio);
  void addDynPoint(const DynPoint& dynPoint, const RobotDimensions& rd, const TorsoMatrix& torsoMatrix);
  void ModifyData(const KickRequest& br, JointRequest& kickEngineOutput, std::vector<KickEngineParameters>& params);
  void setCycleTime(float time);
  void calcPhaseState();
  void calcPositions();
  void setRobotModel(const RobotModel& rm);
  bool isMotionAlmostOver();
  void setCurrentKickRequest(const MotionRequest& mr);
  void setExecutedKickRequest(KickRequest& br);
  void initData(const FrameInfo& frame, const MotionRequest& mr, const RobotDimensions& theRobotDimensions, std::vector<KickEngineParameters>& params, const JointAngles& ja, const TorsoMatrix& torsoMatrix);
  void setEngineActivation(const float& ratio);
  bool activateNewMotion(const KickRequest& br, const bool& isLeavingPossible);
  bool sitOutTransitionDisturbance(bool& compensate, bool& compensated, const InertialData& id, KickEngineOutput& kickEngineOutput, const JointAngles& ja, const FrameInfo& frame);
  static void calcJointsForElbowPos(const Vector3f& elbow, const Vector3f& target, JointAngles& targetJointAngles, int offset, const RobotDimensions& theRobotDimensions);

  Pose3f calcDesBodyAngle(JointRequest& jointRequest, const RobotDimensions& robotDimensions, Joints::Joint joint);

  void transferDynPoint(Vector3f& d, const RobotDimensions& rd, const TorsoMatrix& torsoMatrix);

  KickEngineData() :
    formMode(false),
    motionID(-1),
    cycletime(0.f),
    lastRatio(0.f),
    standLegPos(Vector2i::Zero()),
    //Parameter for P, I and D for gyro PID Contol
    gyroP(3.f, -2.5f),
    gyroI(Vector2f::Zero()),
    gyroD(0.03f, 0.01f),
    internalIsLeavingPossible(false)
  {
    for(int i = 0; i < Phase::numOfLimbs; i++)
    {
      limbOff[i] = false;
    }
  }
};
