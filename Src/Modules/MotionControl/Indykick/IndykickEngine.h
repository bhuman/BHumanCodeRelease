/**
 * @file IndykickEngine.h
 * This file declares a module that creates the kicking motions.
 * @author Felix Wenk
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/Math/Differentiator.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/BodyDynamics/BodyDynamicsProviderDelegate.h"
#include "Representations/MotionControl/IndykickEngineOutput.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/HeadJointRequest.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Sensing/BodyDynamics.h"
#include "Representations/Sensing/JointDynamics.h"
#include "Representations/Sensing/OrientationData.h"
#include "Representations/Sensing/RobotBalance.h"
#include "Representations/Infrastructure/KeyStates.h"
#include "BSplineGenerator.h"
#include "BKick.h"
#include "FrontContour.h"
#include "ShellLimits.h"
#include "CartTableController.h"

STREAMABLE(VelocityCorrectionGyroParameters,
{,
  (Vector2<>)(0.01f, 0.01f) proportionalGain,
  (Vector2<>)(0.00001f, 0.00001f) derivativeGain,
  (Vector2<>)(0.00001f, 0.00001f) integralGain,
});

MODULE(IndykickEngine)
REQUIRES(RobotDimensions)
REQUIRES(JointCalibration)
REQUIRES(FrameInfo)
REQUIRES(FilteredSensorData)
REQUIRES(MotionSelection)
REQUIRES(MotionRequest)
REQUIRES(MassCalibration)
REQUIRES(RobotModel)
REQUIRES(HeadJointRequest)
REQUIRES(WalkingEngineStandOutput)
REQUIRES(TorsoMatrix)
REQUIRES(JointDynamics)
REQUIRES(FutureJointDynamics)
REQUIRES(KeyStates)
REQUIRES(OrientationData)
REQUIRES(IndykickEngineOutput)
PROVIDES_WITH_MODIFY(IndykickEngineOutput)
PROVIDES_WITH_OUTPUT(RobotBalance)
LOADS_PARAMETER(float, motionCycleTime) /**< Duration in [ms] of one motion cycle. */
LOADS_PARAMETER(Vector2<>, zmp) /**< The zmp that is to be maintained while kicking, in support foot coordinates. (y is mirrored for the left foot.) */
LOADS_PARAMETER(float, radiusZmp) /**< Definition of a circle around the target zmp defining all zmps that allow moving the kick leg. */
LOADS_PARAMETER(float, radiusZmpFinished) /**< Definition of a circle around the target zmp defining all zmps that allow moving the back to stand. */
LOADS_PARAMETER(Vector2f, balanceRegion) /**< Recangle around the desired zmp in which the kicks may move the target zmp for the balance controller. */
LOADS_PARAMETER(float, maxStableZmpSpeed) /**< Maximum movement speed of a ZMP that is considered to be 'stable'. In [mm/ms]. */
LOADS_PARAMETER(Vector2<>, armBack) /**< Extreme back position of the arm balance joints used to balance vertical torque. */
LOADS_PARAMETER(Vector2<>, armFront) /**< Extreme front position of the arm balance joints used to balance vertical torque. */
LOADS_PARAMETER(float, elbowYaw) /**< Angle of the elbow yaw joints while kicking. */
LOADS_PARAMETER(float, shoulderRoll) /**< Angle of the shouler roll joints while kicking. */
LOADS_PARAMETER(float, armBalanceGain) /**< Determines how much of the vertical torque around the support foot should be balanced using arm movements. */
LOADS_PARAMETER(float, comHeight) /**< Height of the center of mass. */
LOADS_PARAMETER(float, torsoRotationYBalance) /**< Determine how much of the COM movement in y-direction should be achieved by rotating the torso when in balance mode. Has to be between 0 and 1. */
LOADS_PARAMETER(float, torsoRotationYNoBalance) /**< Determine how much of the COM movement in y-direction should be achieved by rotating the torso when not in balance mode. Has to be between 0 and 1. */
LOADS_PARAMETER(float, torsoRotationXBalance) /**< Determine how much of the COM movement in x-direction should be achieved by rotating the torso when in balance mode. Has to be between 0 and 1. */
LOADS_PARAMETER(float, torsoRotationXNoBalance) /**< Determine how much of the COM movement in x-direction should be achieved by rotating the torso when not in balance mode. Has to be between 0 and 1. */
LOADS_PARAMETER(bool, useSupportFootReference) /**< True if the support foot is to be used as the coordinate system for the kick foot trajectory. */
LOADS_PARAMETER(VelocityCorrectionGyroParameters, torsoVelocityCorrection) /**< Parameters for the gyro joint velocities controller. */
END_MODULE

class IndykickEngine : public IndykickEngineBase
{
public:
  IndykickEngine() : emergency(false), kickInProgress(false),
                  initialLeftArmCurve(Vector4f(), Vector4f(), Vector4f(), Vector4f()),
                  initialRightArmCurve(Vector4f(), Vector4f(), Vector4f(), Vector4f()),
                  retractLeftArmCurve(Vector4f(), Vector4f(), Vector4f(), Vector4f()),
                  retractRightArmCurve(Vector4f(), Vector4f(), Vector4f(), Vector4f()),
                  comHeightInterpolationDuration(400.0f), comHeightInterpolationTime(0.0f),
                  comHeightCurve(0.0f, 0.0f, 0.0f, 0.0f),
                  originXCurve(0.0f, 0.0f, 0.0f, 0.0f),
                  comHeightReached(false), lastPhaseChange(0),
                  bsplineGenerator(theTorsoMatrix, nonGyroCorrectedRobotModel, theRobotDimensions),
                  futureBodyDynamicsProviderDelegate(theMassCalibration, theTorsoMatrix, theRobotDimensions, theFrameInfo),
                  cartTableController(theMassCalibration, theRobotDimensions, theOrientationData, motionCycleTime),
                  cartTableControllerNeedsReset(true), correctKickLegWithGyro(false)
  {init();}
  void init();
  void update(IndykickEngineOutput& indykickEngineOutput);
  void update(RobotBalance& robotBalance);

private:
  bool isKickRequestFeasible(const IndykickRequest& kickRequest) const;

  void initializeKickEngineFromStand();
  void generateRetractArmCurve();

  /**
   * Interpolate to the center of mass height expected by the controllers.
   * Returns true if the torso height has been reached.
   */
  bool interpolateComHeight();

  /**
   * True if phase completed.
   */
  bool playPhase(const BKickPhase& phase);
  bool phaseFinished(const BKickPhase& phase);

  /**
   * True if the robot is balanced and stable, i.e. ZMP is in a safe
   * position and does not move with more then the maxStableZmpSpeed.
   */
  bool isBalancedAndStable(const float radius, const bool stand) const;

  /**
   * Tries to balance to the robot while maintaining the kick foot position.
   * @param withArms true if the arms should be used for balancing the torque around the z axis.
   */
  void balance(const bool withArms, const bool balanceWithTorsoRotation);

  void correctKickLegJointVelocities(const Vector2<>& demandedTorsoAngularVelocity,
                                     const bool correctKickLeg,
                                     JointData& outputJointData);

  /**
   * Update the estimated zero moment point.
   * This is required for the balance method to work.
   */
  void calculateZmp(Vector3<>& zmp, const BodyDynamics& bodyDynamics) const;

  /**
   * Calculate the partial derivatives of the zmp (and therefore of the
   * difference between the zmp and the reference zmp) by the joint motion
   * variables (i.e. by q, qDot and qDoubleDot for every joint).
   */
  void calculatePartialZmpDerivativesByJointMotion();

  /**
   * Adjusts the leg angles to improve on the zmp objective using an LQR controller
   * with the cart table model of the robot.
   */
  void balanceZmpObjectiveCartTable(const JointDynamics& myFutureJointDynamics,
                                    const bool balanceWithTorsoRotation,
                                    const Vector3<>& zmp,
                                    Vector2<>& demandedTorsoAngularVelocity);
  void generateSupportLegAnglesForCenterOfMass(RobotModel& myFutureRobotModel, JointData& myFutureJointData, const Vector3<>& desiredCom);
  enum {numPreviewFrames = 50};
  /** ZMP reference trajectory preview */
  RingBuffer<Vector2f, numPreviewFrames> zmpReference;
  void updateZmpPreview(const BKick& kick, const Vector2f& defaultZmp);

  Vector3<> zmpDerivativeFromFootForceDerivative(const SpatialVector<>& footForce, const SpatialVector<>& footForceDerivative) const;

  /**
   * Calculates the z-torque gradient
   * from the foot force derivatives in support foot coordinates.
   */
  float zTorqueGradient(const JointData::Joint joint);

  /**
   * Approxiamtes foot front contour.
   */
  void approximateFootContour();

  /**
   * Encapsulate drawings related to the kick foot trajectory.
   */
  void drawKickFootDrawings(const BKickPhase& phase);
  void drawRightFootContour();

  bool emergency; /**< Flags an emergency. If true, all joints are switched off. */
  bool lastMotionStand; /**< True if a stand motion was requested in the last frame. */
  IndykickRequest request; /**< Request currently executed. */
  IndykickEngineOutput output; /**< Currently generated output. */
  bool initialKickPositionReached; /**< True if the initial kick position has been reached, i.e. if moving the kick leg is allowed. */
  bool kickInProgress; /**< True if a kick is currently in progress. */
  FrontContour footFrontContour; /**< Model of the part of the foot which is to be used to kick a ball. */
  Vector3<> standZmp; /**< Point in support foot coordinates which should be the ZMP in the robot's 'stand' pose. */
  Vector3<> wantedZmp; /**< Point in support foot coordinates which should be the ZMP. */
  Vector3<> estimatedZmp; /**< Point in support foot coordinates which is the currently estimated ZMP. */
  BezierCurveTemplate<Vector4f> initialLeftArmCurve; /** The joint trajectory of the left arm to its initial position. */
  BezierCurveTemplate<Vector4f> initialRightArmCurve; /** The joint trajectory of the right arm to its initial position. */
  BezierCurveTemplate<Vector4f> retractLeftArmCurve; /** The joint trajectory of the left arm to its retracted position. */
  BezierCurveTemplate<Vector4f> retractRightArmCurve; /** The joint trajectory of the right arm to its retracted position. */
  Vector2<> zmpVelocity; /**< Velocity of the ZMP (the difference vector, respectively). Only for drawing, only updated by LQR. */
  Vector3<> lastEstimatedZmp; /**< Point in support foot coordinates which was the estimated ZMP in the previous frame. */
  const float comHeightInterpolationDuration;
  float comHeightInterpolationTime;
  BezierCurveTemplate<float> comHeightCurve; /**< Bezier curve used to interpolate the height of the center of mass. */
  float comX; /**< X coordinate of the com that is to be maintained while the origin is moved in the x direction. */
  BezierCurveTemplate<float> originXCurve; /**< Bezier curve used to interpolate the x-position of the torso relative to the feet. */
  bool comHeightReached; /**< True if the com height, which is expected by the balancing controllers, has been reached. */
  BKick kicks[IndykickRequest::numOfMotionIds - 1]; /**< States of the different kick motions. -1 since there's no state for the 'none' motion! */
  unsigned lastPhaseChange; /**< Timestamp of the last change of the phase. [ms] */
  enum {
    numOfArmJoints = 4, /**< 0 is the fist arm joint, i.e. the joint connecting the torso to the shoulder. */
    numOfLegJoints = 6  /**< 0 is the first leg joint (see supportLegJoint0, the joint connecting torso to pelvis). */
  };
  Differentiator kickLegVelocitiesDifferentiator[numOfLegJoints]; /**< Joint angular velocities for the kick leg joints. */
  Differentiator armVelocitiesDifferentiator[4]; /**< Joint angular velocities for the arm joints: LShoulderPitch, LElbowRoll, RShoulderPitch, RElbowRoll. */
  Vector4f leftArm; /**< Joint angles for the left shoulder pitch and left elbow yaw, left elbow roll and left shoulder roll. */
  Vector4f rightArm; /**< Joint angles for the right shoulder pitch and right elbow yaw, right elbow roll and left shoulder roll. */
  float leftArmPosition; /**< Interpolation point between the extreme arm angles. (This is \tau) */
  float deltaTau; /**< Last delta for the arm position. Saved as an attribute for smoothing. */
  float deltaTau1; /**< Last intermediate smoothing value for the arm position. */
  JointData::Joint supportLegJoint0; /**< First joint of the support leg, i.e. one of the yaw-pitch joints. */
  JointData::Joint kickLegJoint0; /**< First joint of the kick leg, i.e. the other one of the yaw-pitch joints. */
  KickFootPose kickFootPose; /**< Pose of the kick foot either in support foot or in origin coordinates. */

  Vector3<> zmpByq[JointData::numOfJoints]; /**< Partial derivatives of the zmp by the joint positions. */
  Vector3<> zmpByqDot[JointData::numOfJoints]; /**< Partial derivatives of the zmp by the joint velocities. */
  Vector3<> zmpByqDoubleDot[JointData::numOfJoints]; /**< Partial derivatives of the zmp by the joint accelerations. */

  RobotModel nonGyroCorrectedRobotModel; /**< Robot model made of the wanted joint angles without the gyro correction. */
  BSplineGenerator bsplineGenerator; /**< Generator to generate a C2-continuous cubic spline in bezier representation. */

  BodyDynamicsProviderDelegate futureBodyDynamicsProviderDelegate;
  FutureBodyDynamics futureBodyDynamics;
  FutureBodyDynamicsDerivatives futureBodyDynamicsDerivatives;

  CartTableZMPPreviewController<numPreviewFrames> cartTableController;
  bool cartTableControllerNeedsReset;

  /* Stuff to correct the joint velocities. */
  enum {frameDelay = 4};
  RingBuffer<Vector2<>, frameDelay> demandedAngularVelocityBuffer; /**< Buffer of demanded angular velocities at the torso. */
  Vector2<> rotationErrorSum; /**< Integral over the error of gyro-measured angular velocities and demanded angular velocities of the torso. */
  Vector2<> lastRotationError; /**< Difference between gyro-measured and demanded angular velocity of the torso from  the last motion frame. */
  Vector2<> gyro; /**< Gyro measurement of the angular velocity of the torso. */
  bool correctKickLegWithGyro; /**< True if the kick leg velocity also is corrected using the gyro measurement. Only do that if the kick leg is on the ground. */
};
