/**
 * @file WalkingEngine.h
 *
 * This file declares a module that generates walking motions.
 *
 * The basic ideas of using the FSRs to determine the support change and the gyroscope
 * for directly proportional feedback to the ankle joints are borrowed from the 2014
 * walking engine from rUNSWift written by Bernhard Hengst.
 *
 * @author Thomas Röfer
 * @author Philip Reichenberg
 * @author Arne Hasselbring
 */

#pragma once

#include "Representations/Configuration/CalibrationRequest.h"
#include "Representations/Configuration/FootOffset.h"
#include "Representations/Configuration/FootSoleRotationCalibration.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/MotionControl/EnergySaving.h"
#include "Representations/MotionControl/KeyframeMotionGenerator.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/StandGenerator.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/WalkKickGenerator.h"
#include "Representations/MotionControl/WalkLearner.h"
#include "Representations/MotionControl/WalkModifier.h"
#include "Representations/MotionControl/WalkStepData.h"
#include "Representations/Sensing/FootSupport.h"
#include "Representations/Sensing/FsrData.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/GyroState.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/JointPlay.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Framework/Module.h"
#include "WalkLibs/WalkPhaseBase.h"

STREAMABLE(WalkingOnOpponentFeetParameters,
{,
  (float) maxObstacleDistance, /**< An obstacle must stand near us to allow the check, if we stepped on another robots foot (in mm). */
  (float) minFeetHeightDifference, /**< Feet height difference sample must be lower than this value (in mm). */
  (float) maxFeetHeightDifferenceVelocity, /**< Feet height difference sample are allowed to change this much from the first to the last one (in mm). */
  (float) maxFeetHeightDifferenceScaling, /**< Scale the allowed change of the samples the higher the first sample was. */
  (float) maxLastBackwardStepAdjustment, /**< Last walking step adjustment must be lower than this value. */
  (unsigned) feetHeightDifferenceNumberOfSamples, /**< Number of samples. */
  (float) stepSizeXAfterWalkingOnOpponentFeet, /**< After a detected step on another robots feet, execute a walking step with this step size in the x-translation. */
  (float) maxSupportSwitchPhaseRangeAfterSteppingOnOpponentFeet, /**< Execute stepSizeXAfterWalkingOnOpponentFeet if the last step duration was below this ratio. */
  (bool) useSteppingOnOpponentFootBehavior, /**< Use detection for walking on other robots feet? */
});

STREAMABLE(BalanceParameters,
{,
  (float) gyroLowPassRatio, /**< To which ratio keep old gyro measurements? */
  (float) gyroForwardBalanceFactor, /**< How much are gyro measurements added to ankle joint angles to compensate falling forwards while walking? */
  (float) gyroBackwardBalanceFactor, /**< How much are gyro measurements added to ankle joint angles to compensate falling backwards while walking? */
  (float) gyroSidewaysBalanceFactor, /**< How much are gyro measurements added to ankle joint angles to compensate falling sideways while standing? */
  (float) gyroBalanceKneeBalanceFactor, /**< How much are gyro measurements added to hip and knee joint angles to compensate falling forwards while walking? */
  (Angle) maxGyroBalanceKneeValue, /**< Max balance value when balancing the with knee and hip pitch. */
  (Angle) gyroBalanceKneeNegativeGyroAbort, /**< Stop knee and hip pitch balancing when gyro measures less than this value. */
  (Vector2f) gyroForwardBalanceFactorHipPitch, /**< Reduce factor of gyroForwardBalanceFactor, to add gyro balancing on the hipPitch of the support foot. x = negative values, y = positiv values. */
  (Angle) slowdownTorsoOffset, /**< Start slowing down the walking speed at this torso rotation. */
  (float) slowdownFactor, /**< Slow down walking speed down to this factor. */
});

STREAMABLE(SpeedParameters,
{,
  (Vector2f) maxAcceleration, /**< Maximum acceleration of forward and sideways speed at each leg change to ratchet up/down in (mm/s/step). */
  (Vector2f) maxDeceleration, /**< (Positive) maximum deceleration of forward and sideways speed at each leg change to ratchet up/down in (mm/s/step). */
  (float) fastFeetAdjustment, /**< Feet can move this fast to adjust to a specific pose. */
  (float) slowFeetAdjustment, /**< Feet shall move this slow to adjust to a specific pose. */
  (float) reduceSwingHeightStartingFactor, /**< Reduce max swing foot height for first walking step. */
  (float) stretchSwingHeightAfterThisOvertime, /**< After so much time after the planned step duration, start to stretch out the swing foot (in s). */
  (float) stretchSwingHeightValue, /**< Stretch swing foot maximal for this much (in mm). */
  (float) stretchSwingHeightSpeed, /**< Stretch swing foot with this speed (in mm/s). */
  (Rangea) soleRotationOffsetSpeed, /**< Sole rotation can return to neutral position with this speed (degree/s). */
  (int) soleRotationOffsetSpeedAfterKickTime, /**< After a kick use a lower soleRotationOffsetSpeed to interpolation the sole rotation. */
  (float) walkSpeedReductionFactor, /**< Reduce the walking speed by this amount (only x translation). */
  (int) reduceWalkingSpeedStepAdjustmentSteps, /**< If the step adjustment adjusted the feet too much two separat times, then reduce the walking speed for this number of walking steps. */
  (float) afterKickFeetHeightAdjustment, /**< Feet height can change this much after a specific kick. */
  (Pose2f) maxSpeed, /**< Maximum speeds in mm/s and degrees/s. */
  (Pose2f) minSpeed, /**< Minimum speeds in mm/s and degrees/s. */
  (float) maxSpeedBackwards, /**< Maximum backwards speed. Positive, in mm/s. */
  (float) minSpeedBackwards, /**< Minimum backwards speed. Positive, in mm/s. */
});

STREAMABLE(StepSizeParameters,
{,
  (float) insideTurnRatio, /**< The ratio of turn that can be done in the inner step of a double-step. */
  (Vector2a) reduceTranslationFromRotation, /**< The rotation from which the translation per step will be reduced. */
  (Vector2a) noTranslationFromRotation, /**< The rotation from which no translation is possible at all. */
  (Angle) noTranslationYFromRotationFastInner, /**< When walking fast with a rotation outwards, the rotation from which no translation is possible at all is higher for the y translation. */
  (Angle) noTranslationYFromRotationFastOuter, /**< When walking fast with a rotation inwards, the rotation from which no translation is possible at all is higher for the y translation. */
  (Angle) reduceTranslationYFromRotationFast, /**< When walking fast, the rotation from which the translation per step will be reduced is higher for the y translation. */
  (float) minXTranslationStep, /**< The forward and backward step size has a minimum. */
  (float) minXForwardTranslationFast, /**< The forward step size has a minimum for the fast translation polygon. */
  (Rangef) minXBackwardTranslationFastRange,/**< The backward step size has a minimum for the fast translation polygon. */
  (Rangef) translationPolygonSafeRange, /**< For the calculation of the translation polygon, the feet are adjusted by this amount in x translation,
                                        to ensure a possible pose even when the arms are on the back or the feet are balanced. */
  (int) noFastTranslationPolygonStepsNumber, /**< Number of steps before big steps near the target are allowed. */
});

STREAMABLE(ArmParameters,
{,
  (Angle) armShoulderRoll, /**< Arm shoulder angle in radians. */
  (float) armShoulderRollIncreaseFactor, /**< Factor between sideways step size (in m) and additional arm roll angles. */
  (float) armShoulderPitchFactor, /**< Factor between forward foot position (in m) and arm pitch angles. */
  (float) comTiltFactor, /**< Factor between the correct torso shift and the one actually used. Used to tilt the torso to compensate the arm positions in high stand. */
  (float) armInterpolationTime, /**< Interpolation time for the arms, when the WalkPhase can set them again. */
});

STREAMABLE(EmergencyStepParameters,
{,
  (float) emergencyStepSize, /**< The size of emergency sideways steps in mm. */
  (float) emergencyStepHeightFactor, /** During an emergency step, the step height is multiplied by this factor. */
  (int) emergencyNotMovingCounter, /** Gyro state measures low deviation and mean values so many times in a row, to start a new step early. */
  (Angle) emergencyMaxGyroMean, /** Gyro state must measure gyro values below this threshold, to increase emergencyNotMovingCounter. */
  (Angle) emergencyMaxGyroDeviation, /** Gyro state must measure gyro value deviations below this threshold, to increase emergencyNotMovingCounter. */
  (Angle) emergencyMaxZGyroDeviation, /** Gyro state must measure z gyro value deviations below this threshold, to increase emergencyNotMovingCounter. */
});

STREAMABLE(KinematicParameters,
{,
  (float) baseWalkPeriod, /**< Duration of a single step, i.e. half of a walk cycle (in ms). */
  (float) sidewaysWalkHeightPeriodIncreaseFactor, /**< Additional duration for the swing height, when walking sideways at maximum speed (in ms). */
  (float) sidewaysHipShiftFactor, /**< Apply the requested sidewards target with this factor on the support foot. */
  (float) walkHipHeight, /**< Walk hip height above ankle joint in mm - seems to work from 200 mm to 235 mm. */
  (float) baseFootLift, /**< Base foot lift in mm. */
  (float) torsoOffset, /**< The base forward offset of the torso relative to the ankles in mm. */
});

STREAMABLE(ParabolicFootHeightParameters,
{,
  (float)(125.f) maxHeightAfterTime,
  (float)(0.5f) maxHeightAfterTimePercent,
});

STREAMABLE(WalkingEngineCommon,
{,
  (Rangef) supportSwitchPhaseRange, /**< In which range of the walk phase can the support foot change? */
  (int) walkStiffness, /**< Joint stiffness while walking in %. */
  (int) standStiffnessDelay, /**< The time in stand before the stiffness is lowered (in ms). */
  (Pose2f) thresholdStopStandTransition, /**< Threshold to be able to switch from walk state stopping to standing. */
  (Pose2f) stoppingThresholds, /**< Increase stopping counter if step size is smaller than this size. */
  (int) stoppingCounterThreshold, /**< Robot must execute this many stopping walk phases to actually stop, if the next phase is a kick phase (KickEngine). */
  (Rangef) clipForward, /**< Clip forward/backward feet movement, to prevent assert in the inverse kinematic calculation (and to prevent damage). */
  (int) lowStiffnessDelay, /**< Low stiffness delay after high stand interpolation started. */
  (float) clipAtBallDistanceX, /**< Step adjustment is not allowed to touch the ball. */
  (float) clipAtBallDistance, /**< Ball is ignored when it further away than this threshold. */
  (Angle) standHighTorsoPitch, /**< The torso pitch angle for standHigh. */
  (Angle) standInterpolationVelocity, /**< The interpolation speed to interpolate to stand (in degree/s). */
  (int) standHighInterpolationDuration, /**< The duration to interpolate from/to stand to/from stand high. */
  (int) lowStiffnessLegs, /**< Low stiffness setting for leg joints (except AnklePitch). */
  (int) lowStiffnessAnklePitch, /** < Low stiffness setting for ankle pitch joints. */
  (int) standHighNotMovingCounter, /**< Robot must be standing still for so many checks in a row, to allow high stand. */

  (float) highDeltaScale, /**< When tilting too much forward or backward, override the last joint request. Interpolate between last request and measured position based on the com delta. */
  (Angle) negativeDeltaJointBaseOffset, /**< Add an offset to the joint request when tilting too much backward, to compensate the motor controls of the joints. */
  (Angle) negativeDeltaKneeScaleOffset, /**< Add an scaling offset to the knee request when tilting too much backward, to ensure the swing foot does not collide with the ground. */
  (float) minTimeForEarlySwitch, /**< An early support foot switch, when the wrong foot is the supporting one, is allowed after this much time (in s) has passed. */

  (Angle) maxWalkDirectionForFootPrediction, /**< The direction of the step target must be lower than angle to allow for foot support switch prediction. */
  (bool) useFootSupportSwitchPrediction, /**< Use predicted support foot switches. */
  (bool) blockStoppingWithStepAdjustment, /**< Step adjustment can not move freely forwards, if the swing foot would collide with the ball. */
  (bool) useJointPlayScaling, /**< Scale the walking speed based on the joint play? */

  (KinematicParameters) kinematicParameters,
  (EmergencyStepParameters) emergencyStep, /**< Parameters for the emergency step. Needed if the robot is stuck at a goal post. */
  (ArmParameters) armParameters, /**< Arm parameters. Normal arm swinging and arm compensation. */
  (StepSizeParameters) stepSizeParameters, /**< Step size parameters. */
  (WalkingOnOpponentFeetParameters) footStepping, /**< Parameters for the behavior when stepping on another robots foot. */
  (BalanceParameters) balanceParameters, /**< Balancing parameters. */
  (WalkStepAdjustmentParams) walkStepAdjustmentParams, /**< Walk step adjustment parameters. */
  (SoleRotationParameter) soleRotationParameter, /**< Sole rotation compensation parameters, to keep the swing foot parallel to the ground. */
  (ParabolicFootHeightParameters) parabolicFootHeightParameters,  /**< Foot height interpolation parameters. */
});

MODULE(WalkingEngine,
{,
  REQUIRES(CalibrationRequest),
  REQUIRES(EnergySaving),
  REQUIRES(FootOffset),
  REQUIRES(FootSoleRotationCalibration),
  REQUIRES(FootSupport),
  REQUIRES(FsrData),
  REQUIRES(FrameInfo),
  REQUIRES(GroundContactState),
  REQUIRES(GyroState),
  REQUIRES(InertialData),
  REQUIRES(JointAngles),
  USES(JointPlay),
  USES(JointRequest),
  USES(KeyframeMotionGenerator),
  REQUIRES(MassCalibration),
  REQUIRES(MotionRequest),
  REQUIRES(OdometryDataPreview),
  REQUIRES(ObstacleModel),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  REQUIRES(StiffnessSettings),
  REQUIRES(TorsoMatrix),
  USES(WalkKickGenerator),
  REQUIRES(WalkLearner),
  REQUIRES(WalkModifier),
  PROVIDES(WalkStepData),
  USES(WalkStepData),
  PROVIDES(WalkGenerator),
  REQUIRES(WalkGenerator),
  PROVIDES(StandGenerator),
  PROVIDES(WalkingEngineOutput),
  LOADS_PARAMETERS(
  {,
    (SpeedParameters) speedParameters,
  }),
});

class WalkingEngine : public WalkingEngineBase, public WalkingEngineCommon
{
  void update(WalkStepData& walkStepData) override;
  void update(StandGenerator& standGenerator) override;
  void update(WalkGenerator& walkGenerator) override;
  void update(WalkingEngineOutput& walkingEngineOutput) override;

public:

  /**
   * Calculate the step duration based on the side translation amount of requested side speed
   * @param sideSpeed Next planned walk translation speed to the side
   */
  float stepDurationSpeedTarget(const float sideSpeed) const;

  /**
   * Calculate the side speed given the side target or the current step
   * @param sideTarget Next planned walk translation to the side
   */
  float getSideSpeed(const float sideTarget) const;

  /**
   * Calculate the current translation polygon
   * @param polygon the translation polygon
   * @param backRight the max allowed back and right translation
   * @param frontLeft the max allowed front and left translation
   */
  void generateTranslationPolygon(std::vector<Vector2f>& polygon, const Vector2f& backRight, const Vector2f& frontLeft);

  /**
   * Filter the polygon to not contain the points that are not needed.
   */
  void filterTranslationPolygon(std::vector<Vector2f>& polygonOut, std::vector<Vector2f>& polygonIn, const std::vector<Vector2f>& polygonOriginal);

  /**
   * Calculates the pose of the feet.
   * @param forwardL x translation position for the left foot, without torso shift
   * @param forwardR x translation position for the right foot, without torso shift
   * @param sideL y translation position for the left foot, without hip shift
   * @param sideR y translation position for the right foot, without hip shift
   * @param footHL foot height
   * @param footHR right foot height
   * @param turn z rotation
   * @param soleRotationYL left foot y rotation
   * @param soleRotationXL left foot x rotation
   * @param soleRotationYR right foot y rotation
   * @param soleRotationXR right foot x rotation
   * @param leftFoot left foot pose
   * @param rightFoot right foot pose
   */
  void calcFeetPoses(const float forwardL, const float forwardR, const float sideL, const float sideR,
                     const float footHL, const float footHR, const Angle turn,
                     const Angle soleRotationYL, const Angle soleRotationXL, const Angle soleRotationYR, const Angle soleRotationXR,
                     Pose3f& leftFoot, Pose3f& rightFoot) const;

  Angle filteredGyroX, /**< low-pass filtered gyro values. */
        filteredGyroY;

  std::vector<Vector2f> translationPolygon; /**< The polygon that defines the max allowed translation for the step size. */
  std::vector<Vector2f> translationPolygonAfterKick; /**< The polygon that defines the max allowed translation after a kick from the KickEngine. */

  MassCalibration lightMassCalibration; /**< MassCalibration without the legs masses. */

  /** The constructor loads the common parameters. */
  WalkingEngine();

  /** Register common parameters. */
  static void reg();

  static constexpr float mmPerM = 1000.f;
};

struct DummyPhase : MotionPhase
{
  using MotionPhase::MotionPhase;

  bool isDone(const MotionRequest&) const override { return true; }
  void calcJoints(const MotionRequest&, JointRequest&, Pose2f&, MotionInfo&) override {}
};

struct WalkPhase : WalkPhaseBase
{
  WalkPhase(const WalkingEngine& engine, const Pose2f& stepTarget, const MotionPhase& lastPhase,
            const WalkGenerator::CreateNextPhaseCallback& createNextPhaseCallback = WalkGenerator::CreateNextPhaseCallback(),
            const WalkKickStep& walkKickStep = WalkKickStep());

private:

  void update() final;

  bool isDone(const MotionRequest& motionRequest) const final;

  void calcJoints(const MotionRequest& motionRequest, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo) final;

  std::unique_ptr<MotionPhase> createNextPhase(const MotionPhase& defaultNextPhase) const final;

  /**
   * Calculate the translation and rotation variables for both feet while walking and standing.
   * @param swingSign Sign of the swing foot. 1 for left foot is the swing foot and -1 for the right foot is the swing foot.
   * @param ratio Current time in the step (in s)
   * @param duration Planned duration of the step (in s)
   * @param heightDuration Used duration for the step height (in s). Side steps use a longer step duration for the height interpolation.
   * @param forwardSwing0 Start value for the forward position of the swing foot.
   * @param forwardSupport0 Start value for the forward position of the support foot.
   * @param forwardSwing Current forward position of the swing foot.
   * @param forwardSupport Current forward position of the support foot.
   * @param sideSwing0 Start value for the side position of the swing foot.
   * @param sideSupport0 Start value for the side position of the support foot.
   * @param sideSwing Current side position of the swing foot.
   * @param sideSupport Current side position of the support foot.
   * @param footHeightSwing0 Start height of the swing foot.
   * @param footHeightSupport0 Start height of the support foot.
   * @param footHeightSwing Current height of the swing foot.
   * @param footHeightSupport Current height of the support foot.
   * @param turnVal Current rotation value of the feet.
   * @param useForwardSpeed Requested forward step size.
   * @param useSideSpeed Requested side step size.
   * @param useTurnSpeed Requested turn step size.
   */
  void calcFootOffsets(const float swingSign, const float ratio, const float ratioBase, const float duration, const float heightDuration,
                       const float forwardSwing0, const float forwardSupport0, float& forwardSwing,
                       float& forwardSupport, const float sideSwing0, const float sideSupport0,
                       float& sideSwing, float& sideSupport, float& footHeightSwing0,
                       float& footHeightSupport0, float& footHeightSwing,
                       float& footHeightSupport, Angle& turnVal,
                       const float useForwardSpeed, const float useSideSpeed, const Angle useTurnSpeed) final;

  /**
   * Calculate the translation and rotation variables for both feet kicking.
   * @param swingSign Sign of the swing foot. 1 for left foot is the swing foot and -1 for the right foot is the swing foot.
   * @param ratio Current time in the step (in s)
   * @param forwardSwing0 Start value for the forward position of the swing foot.
   * @param forwardSupport0 Start value for the forward position of the support foot.
   * @param forwardSwing Current forward position of the swing foot.
   * @param forwardSupport Current forward position of the support foot.
   * @param sideSwing0 Start value for the side position of the swing foot.
   * @param sideSupport0 Start value for the side position of the support foot.
   * @param sideSwing Current side position of the swing foot.
   * @param sideSupport Current side position of the support foot.
   * @param footHeightSwing0 Start height of the swing foot.
   * @param footHeightSupport0 Start height of the support foot.
   * @param footHeightSwing Current height of the swing foot.
   * @param footHeightSupport Current height of the support foot.
   * @param turnRL0 Start rotation value of the feet.
   * @param turnVal Current rotation value of the feet.
   * @param walkKickStep Describes all important parameters for the step, like multiple step sizes and their interpolation
   */
  void calcWalkKickFootOffsets(const float swingSign, const float ratio,
                               float& forwardSwing0, float& forwardSupport0, float& forwardSwing,
                               float& forwardSupport, float& sideSwing0, float& sideSupport0,
                               float& sideSwing, float& sideSupport, float& footHeightSwing0,
                               float& footHeightSupport0, float& footHeightSwing,
                               float& footHeightSupport, Angle& turnVal, WalkKickStep& walkKickStep,
                               Angle& turnRL0) final;

  /**
   * Set the joint request for the arms, or use the given values
   * @param leftFoot Left foot pose.
   * @param rightFoot Right foot pose.
   * @param jointRequest The given arm positions, which may be overriden bei walkArms if they should be ignored.
   * @param walkArms The arm positions based on the current walk.
   */
  void setArms(Pose3f& leftFoot, Pose3f rightFoot, JointRequest& jointRequest, JointRequest& walkArms) final;

  /**
   * Compensates for the changed position of the COM resulting from arm motion.
   * The torso is tilted to move the COM.
   * @param leftFoot The pose of the left foot's sole relative to the torso.
   * @param rightFoot The pose of the right foot's sole relative to the torso.
   * @param jointRequest The joint request as determined by the walk generator.
   *                     The joint request is changed to compensate for the
   *                     effect of external arm movements.
   * @param walkArms The joint request with arms set by the walking engine.
   */
  void compensateArms(Pose3f& leftFoot, Pose3f& rightFoot, JointRequest& jointRequest, const JointRequest& walkArms) final;

  WalkGenerator::CreateNextPhaseCallback createNextPhaseCallback;
};
