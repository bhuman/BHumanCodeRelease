/**
 * @file StableStand.h
 *
 * @author <a href="mailto:roehrig@uni-bremen.de">Enno Roehrig</a>
 */

#pragma once

#include "Tools/Math/Pose2f.h"
#include "Tools/Math/Pose3f.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Function.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Modules/MotionControl/Balancer/LIPStateEstimator.h"
#include "Modules/MotionControl/Balancer/ZmpController.h"

STREAMABLE(BalancingParameter,
{,
  (bool)(true) useIMU,                  //use IMU Data to react to tilting
  (bool)(true) usePreviewController,    //use a preview controller to center of mass control
  (bool)(false) strictInitialTrajectory,//wait to reach zmp values in initial phase
  (float) oneLeggedTorsoRotation,       //torso rotation when standing one-legged
  (float) stabilizationRange,           //maximal com difference from center of standFoot, where reaction to tilting is used

  (float)(0.f) weakAnkleTorso,          //how strongly compensate a weak ankle with torso
  (float)(0.f) weakKneeTorso,           //how strongly compensate a weak knee with torso
  (float)(0.f) counterWeakAnkle,        //how strongly counteract to a weak ankle

  (Vector2f) balanceWithHip,            //how strongly counter tilting with hip
  (Vector2f) balanceWithArms,           //how strongly counter tilting with arms

  (Vector2f) critBalanceWithLeg,        //how strongly counter critical tilting with swing leg
  (Vector2f) critBalanceWithHip,        //how strongly counter critical tilting with hip

  (float)    maxTilt,                   //maximal tilt to react
  (float)    criticalTiltThreshold,     //threashold to use stronger balancing factors

  (Vector2f) tiltFeedback,              //how fast react to tilt
  (Vector2f) fallingTiltFeedback,       //how fast react to tilt, when robot is falling
  (Vector2f) fallingThreshold,          //tilt threashold to consider robot as falling

  (Vector3f) sensorFeedbackX,           //how strongly consider imu values to update intern LIP model (X direction)
  (Vector3f) sensorFeedbackY,           //how strongly consider imu values to update intern LIP model (Y direction)
  (LIPStateEstimatorParameters) lipStateEstimator,  //parameter to estimate current state of lip using sensor data

  (ZmpControllerParameters) previewControllerX, //parameter for preview controller (X direction)
  (ZmpControllerParameters) previewControllerY, //parameter for preview controller (Y direction)

  (Vector2f) tiltCalibrationLeft,       //use only for calibration and safe a value for every robot in damage configuration
  (Vector2f) tiltCalibrationRight,      //use only for calibration and safe a value for every robot in damage configuration
});

STREAMABLE(Balancer,
{
  /**
   * Initializes the generator. Must be called whenever control over stabilization is handed over to this module
   * Must also be called once after creation.
   * @return current zmp with body rotation as rotation matrix. Hand over to balancing methods to avoid jumps. can be slowly set to zero.
   */
  FUNCTION(Pose3f(bool rightIsSupportFoot, const BalancingParameter& param)) init;

  /**
   * Initializes the generator. Must be called whenever control over stabilization is handed over to this module
   * Must also be called once after creation. Reaches targetZMP before reacting to zmpPreviews
   */
  FUNCTION(void(bool rightIsSupportFoot, const Pose3f& targetZMP, float initalizationTime, const BalancingParameter& param)) initWithTarget;

  /**
   * Modifies joint angles to make the robot stand stable
   * @param jointRequest The requested joint angles, which will be modified
   * @param zmpPreviewsX current and future zmpPositions (X-Value)
   * @param zmpPreviewsY current and future zmpPositions (Y-Value)
   * @return returns true when a stable one-legged position is reached
   */
  FUNCTION(bool(JointRequest& jointRequest, const std::vector<float>& zmpPreviewsX, const std::vector<float>& zmpPreviewsY, const BalancingParameter& param)) addBalance;

  /**
   * Balancing the current jointRequest with minimal changes
   * @param jointRequest The requested joint angles, which will be modified
   * @return returns true when a stable one-legged position is reached
   */
  FUNCTION(bool(JointRequest& jointRequest, const BalancingParameter& param)) balanceJointRequest;

  /**
   * Calculates a new set of joint angles based on the swing-foot-position while standing stable
   * @param jointRequest The calculated joint angles are returned here.
   * @param swingFootInSupport swing-foot-position relative to the support-foot
   * @return returns true when a stable one-legged position is reached
   */
  FUNCTION(bool(JointRequest& jointRequest, const Pose3f& swingFootInSupport, float comHeight, const RotationMatrix& bodyRotation, const std::vector<float>& zmpPreviewsX, const std::vector<float>& zmpPreviewsY, const BalancingParameter& param)) calcBalancedJoints;

  ,
  //For Analyzing via Logfiles
  (std::vector<float>) standardParameter,
  (float)(0.f) tiltX,
  (float)(0.f) tiltY,
  (float)(0.f) stabilizationFactor,
  (float)(0.f) comX,
  (float)(0.f) comY,
  (float)(0.f) comXvel,
  (float)(0.f) comYvel,
  (float)(0.f) kneeDiff,
});
