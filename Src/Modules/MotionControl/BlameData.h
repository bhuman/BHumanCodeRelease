/**
* @file Modules/MotionControl/BLAMEData.h
* This file declares a module that creates the walking motions.
* @author <A href="mailto:judy@tzi.de">Judith Müller</A>
*/

#pragma once

#include "BIKEParameters.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/Math/Pose3D.h"
#include "Tools/Streams/InStreams.h"
#include "Platform/BHAssert.h"
#include "Representations/Infrastructure/SensorData.h"
#include <vector>
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/RingBufferWithSum.h"
#include "Representations/MotionControl/BikeEngineOutput.h"
#include "Representations/Sensing/TorsoMatrix.h"

class BlameData
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

  RingBufferWithSum <Vector2<int>, 10> standLegPos;

  Vector2<> comOffset,
          balanceSum,
          gyro,
          lastGyroLeft,
          lastGyroRight,
          gyroErrorLeft,
          gyroErrorRight,
          lastBody,
          bodyError,
          gyroP,
          gyroI,
          gyroD,
          origin;

  Vector3<> positions[Phase::numOfLimbs], origins[Phase::numOfLimbs];

  Vector3<> torsoRot;

  Vector3<> lastCom, ref, actualDiff;

  BIKEParameters currentParameters;

  RobotModel robotModel, comRobotModel;

  JointRequest lastBalancedJointRequest, compenJoints;

  bool wasActive, startComp, willBeLeft;
  BikeRequest currentBikeRequest;

public:

  bool getMotionIDByName(const MotionRequest& motionRequest, const std::vector<BIKEParameters>& params);
  void calculateOrigins(const RobotDimensions& theRobotDimension, const FilteredJointData& jd);
  bool checkPhaseTime(const FrameInfo& frame, const RobotDimensions& rd, const FilteredJointData& jd, const TorsoMatrix& torsoMatrix);
  void balanceCOM(JointRequest& joints, const RobotDimensions& rd, const MassCalibration& mc, const FilteredJointData& theFilteredJointData);
  void calculatePreviewCom(Vector3<> &ref, Vector2<> &origin);
  void setStandLeg(const float& originY);
  bool calcJoints(JointRequest& jointRequest, const RobotDimensions& rd, const FilteredJointData& jd);
  void calcLegJoints(const JointData::Joint& joint, JointRequest& jointRequest, const RobotDimensions& theRobotDimensions);
  static void simpleCalcArmJoints(const JointData::Joint& joint, JointRequest& jointRequest, const RobotDimensions& theRobotDimensions, const Vector3<>& armPos, const Vector3<>& handRotAng);
  void getCOMReference(const Vector3<>& lFootPos, const Vector3<>& rFootPos, Vector3<>& comRef, Vector2<>& origin);
  void setStaticReference();
  void mirrorIfNecessary(JointRequest& joints);
  void addGyroBalance(JointRequest& jointRequest, const JointCalibration& jc, const FilteredSensorData& sd, const float& ratio);
  void addDynPoint(const DynPoint& dynPoint, const RobotDimensions& rd, const TorsoMatrix& torsoMatrix);
  void ModifyData(const BikeRequest& br, JointRequest& bikeOutput, std::vector<BIKEParameters>& params);
  void debugFormMode(std::vector<BIKEParameters>& params);
  void setCycleTime(float time);
  void calcPhaseState();
  void calcPositions(JointRequest& joints, const FilteredJointData& theFilteredJointData);
  void setRobotModel(RobotModel rm);
  void setCurrentBikeRequest(const MotionRequest& mr);
  void setExecutedBikeRequest(BikeRequest& br);
  void initData(const bool& compensated, const FrameInfo& frame, const MotionRequest& mr, const RobotDimensions& theRobotDimensions, std::vector<BIKEParameters>& params, const FilteredJointData& jd, const TorsoMatrix& torsoMatrix);
  void setEngineActivation(const float& ratio);
  bool activateNewMotion(const BikeRequest& br, const bool& isLeavingPossible);
  bool sitOutTransitionDisturbance(bool& compensate, bool& compensated, const FilteredSensorData& sd, BikeEngineOutput& blameOutput, const JointData& jd, const FrameInfo& frame);

  Pose3D calcDesBodyAngle(JointRequest& jointData, const RobotDimensions& robotDimensions, JointData::Joint joint);

  void transferDynPoint(Vector3<>& d, const RobotDimensions& rd, const TorsoMatrix& torsoMatrix);

  BlameData():
    formMode(false),
    motionID(-1),
    cycletime(0.f),
    lastRatio(0.f),
    //Parameter for P, I and D for gyro PID Contol
    gyroP(3.f, -2.5f),
    gyroI(0, 0),
    gyroD(0.03f, 0.01f)
  {
    for(int i = 0; i < Phase::numOfLimbs; i++)
    {
      limbOff[i] = false;
    }
  };
};
