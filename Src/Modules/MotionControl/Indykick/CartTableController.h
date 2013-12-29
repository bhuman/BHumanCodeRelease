/**
 * @file CartTableController.h
 * This file declares the class of the cart table zmp preview controller.
 * @author Felix Wenk
 */

#pragma once

#include "Tools/RingBuffer.h"
#include "Tools/Math/Matrix.h"
#include "Tools/Math/Vector2.h"
#include "Tools/Math/Vector3.h"
#include "Tools/Math/Pose3D.h"
#include "Representations/Configuration/MassCalibration.h"

class JointData;
class OrientationData;
class RobotDimensions;
class RobotModel;

template <int n>
class CartTableZMPPreviewController
{
public:
  CartTableZMPPreviewController(const MassCalibration& theMassCalibration, const RobotDimensions& theRobotDimensions,
                                const OrientationData& theOrientationData, const float motionCycleTime);
  void generateSupportLegAngles(const float yRotationFraction,
                                const float xRotationFraction,
                                const RingBuffer<Vector2f, n>& zmpReference, const Vector3<>& estimatedZmp,
                                const Pose3D& kickFoot,
                                const bool useSupportFootCoordinateSystem,
                                JointData& myJointData,
                                const Vector3<>& delayedComPositionInSupportFoot, const Vector3<>& delayedComVelocityInSupportFoot,
                                const Vector3<>& delayedComAccelerationInSupportFoot,
                                Vector2<>& demandedTorsoAngularVelocity);
  void reset(const bool leftSupport, const Vector3<>& comPositionInSupportFoot, const Vector3<>& comVelocityInSupportFoot,
             const Vector3<>& comAccelerationInSupportFoot, const RobotModel& initialRobotModel);

private:
  /** Position of the center of mass trajectory. */
  Vector3<> comPositionInSupportFoot;
  /** Velocity of the center of mass trajectory. */
  Vector2<> comVelocityInSupportFoot;
  /** Acceleration of the center of mass trajectory. */
  Vector2<> comAccelerationInSupportFoot;
  /** Pose of the support foot relative to the robot's origin. */
  Pose3D supportFootInOrigin;
  /** The support foot. */
  MassCalibration::Limb supportFoot;
  /** Time interval between two motion cycles in milliseconds. */
  const float deltaT;
  /** Solution of the Riccati equation for a certain center of mass height. */
  Matrix4x4f P;
  /** Penalty for the CoM jerk, i.e. the penalty for changing the center of mass's acceleration. Must be consistent with P. */
  float R;
  /** Height of the center of mass in millimeters. Must be consistent with P. */
  const float comHeight;
  /** Gain for the numeric integral of all zmp errors. */
  float integralGain;
  /** Gain for the system state, the motion of the center of mass. */
  Matrix1x3f stateGain;
  /** Gain for each previewed zero moment point. */
  enum {frameDelay = 4};
  float previewGain[frameDelay + n];
  /** Buffer of model based zmps used to calculate the offset to the estimated zmp. */
  RingBuffer<Vector3<>, frameDelay> modelComPositionHistory;
  RingBuffer<Vector2<>, frameDelay> modelComVelocitiesHistory;
  RingBuffer<Vector2<>, frameDelay> modelComAccelerationsHistory;
  float maxComAcceleration;
  float maxComVelocity;
  float maxComPositionError;
  RingBuffer<Vector2<>, frameDelay> modelZmpHistory;
  RingBuffer<Vector2<>, frameDelay> zmpRefHistory;
  RingBuffer<Vector2<>, frameDelay> orientationBuffer;
  RotationMatrix lastTorsoRotation;
  /** Summed up differences between the modeled zmp and its desired trajectory. */
  Vector2<> zmpErrorSum;
  /** Reference to the mass calibration representation. */
  const MassCalibration& theMassCalibration;
  /** Reference to the robot dimensions representation. */
  const RobotDimensions& theRobotDimensions;
  /** Reference to the orientation data. */
  const OrientationData& theOrientationData;

  void calculateNextCom(const RingBuffer<Vector2f, n>& zmpReference, const Vector3<>& estimatedZmp, const Vector3<>& comPositionInSupportFoot,
                        const Vector3<>& comVelocityInSupportFoot, const Vector3<>& comAccelerationInSupportFoot);
  void generateLegAnglesForCenterOfMassOrientation(const float yRotationFraction, const float xRotationFraction, JointData& myJointData, const Pose3D& kickFoot, const bool useSupportFootCoordinateSystem, const Vector3<>& desiredCom, Vector2<>& demandedTorsoAngularVelocity);

  void plotModelError(const Vector3<>& estimatedZmp, const Vector3<>& delayedComPositionInSupportFoot, const Vector3<>& delayedComVelocityInSupportFoot, const Vector3<>& delayedComAccelerationInSupportFoot);
};
