/**
 * @file WalkingEngine.h
 *
 * @Author Philip Reichenberg
 */

#pragma once

#include "Framework/Module.h"
#include "Framework/Settings.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/JointLimits.h"
#include "Representations/Configuration/KickInfo.h"
#include "Representations/Configuration/KickLengthPair.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/StiffnessData.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/StandGenerator.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Representations/MotionControl/WalkStepData.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/SoleHeightDifference.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Sensing/VelocityEstimation.h"
#include "Tools/Motion/MotionPhase.h"

#include "Platform/File.h"
#include <CompiledNN2ONNX/CompiledNN.h>

using namespace NeuralNetworkONNX;

STREAMABLE(ArmParameters,
{,
  (Angle) armShoulderRoll, /**< Arm shoulder angle in radians. */
  (float) armShoulderRollIncreaseFactor, /**< Factor between sideways step size (in m) and additional arm roll angles. */
  (float) armShoulderPitchFactor, /**< Factor between forward foot position (in m) and arm pitch angles. */
  (Angle) armShoulderPitch, /**< Arm shoulder pitch angle. */
  (Angle) armElbowYaw, /**< Arm elbow yaw angle. */
  (float) armInterpolationTime, /**< Interpolate the start and target arm positions over this time. */
  (Angle) standInterpolationVelocity, /**< The interpolation speed to interpolate to stand (in degree/s). */
});

STREAMABLE(WalkTargetScaling,
{,
  (Rangef) targetDistance,
  (Rangef) walkSpeed,
});

STREAMABLE(CommonSpeedParameters,
{,
  (Vector2f) maxAcceleration,  /**< Maximum acceleration of forward and sideways speed at each leg change to ratchet up/down in (mm/s/step). */
  (Vector2f) maxDeceleration, /**< (Positive) maximum deceleration of forward and sideways speed at each leg change to ratchet up/down in (mm/s/step). */
  (WalkTargetScaling) xTargetScaling,
  (WalkTargetScaling) yTargetScaling,
  (WalkTargetScaling) rotationTargetScaling,
});

STREAMABLE(StepSizeParameters,
{,
  (Vector2a) reduceTranslationFromRotationBaseOffset, /**< The rotation from which the translation per step will **start** to be reduced. */
  (Vector2a) noTranslationFromRotationThreshold, /**< The **final** rotation from which no translation is possible at all. */
  (Vector2a) reduceTranslationFromRotationBaseFastOffset, /**< When walking fast, the translation starts getting reduced at a higher rotation value. */
  (Vector2a) noTranslationFromRotationFastThreshold, /**< When walking fast, the rotation from which no translation is possible at all is higher. */
  (float) minXTranslationStep, /**< The forward and backward step size has a minimum. */
  (float) minXForwardTranslationFast, /**< The forward step size has a minimum for the fast translation polygon. */
  (Rangef) minXBackwardTranslationFastRange,/**< The backward step size has a minimum for the fast translation polygon. */
});

STREAMABLE(TranslationPolygonParameters,
{,
  (Pose2f) maxSpeed, /**< Maximum speeds in mm/s and degrees/s. */
  (float) maxSpeedBackwards, /**< Maximum backwards speed. Positive, in mm/s. */
});

STREAMABLE(ConfiguredParameters,
{,
  (TranslationPolygonParameters) fastWalkSpeed, /**< Walk speed for normal usage in mm/s and degree/s. */
  (TranslationPolygonParameters) normalWalkSpeed, /**< Walk speed for normal usage in mm/s and degree/s. */
  (TranslationPolygonParameters) slowWalkSpeed, /**< Slow walk speeds in mm/s and degrees/s for everything that does not need to be fast. */
  (TranslationPolygonParameters) maxPossibleSpeed, /**< The maximum speeds to extreme situations, like intercepting. */
  (float) maxSideAtMaxForward, /**< Maximum allowed side speed at max forward speed (in mm/s). */
  (float) maxForwardAtMaxSide, /**< Maximum allowed forward speed at max side speed (in mm/s). */
});

STREAMABLE(FrequencyParameters,
{,
  (float) base, /**< Base frequency. */
  (Rangef) clipRange, /**< Clip frequency offset from neuronal network. */
});

STREAMABLE(BallNetworkParameters,
{,
  (bool) active,
  (unsigned) numInput, /**< Neural Network input size. */
  (float) ballFactor,
  (float) ballVelFactor,
  (float) kickRangeFactor,
  (Rangef) kickVelocityRange,
  (std::string) kickModel,
});

STREAMABLE(WalkNeuronalNetworkParameters,
{,
  (std::string) modelName, /** Neural Network name. Must be .onnx or .hdf5. */
  (float) velocityFactor, /**< Factor for the joint velocities. */
  (Rangef) actionClipRange, /**< Clip action output. */
  (bool) useWaist, /**< Is the waist also part of the action output. */
  (unsigned) numInput, /**< Neural Network input size. */
  (unsigned) numOutput, /**< Neural Network output size. */
  (int) timeLowWalkSpeedForStand, /**< To allow standing, the robot must walk slow for this time. */
  (Pose2f) slowWalkStepSpeed, /**< Speed values to classify slow walking. This is based on the step size. */
});

MODULE(WalkingEngine,
{,
  REQUIRES(BallSpecification),
  REQUIRES(FrameInfo),
  REQUIRES(InertialData),
  REQUIRES(JointAngles),
  REQUIRES(JointLimits),
  USES(JointRequest),
  REQUIRES(KickInfo),
  REQUIRES(KickLengthPair),
  REQUIRES(MassCalibration),
  REQUIRES(MotionRequest),
  REQUIRES(OdometryDataPreview),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  REQUIRES(SoleHeightDifference),
  REQUIRES(TorsoMatrix),
  REQUIRES(VelocityEstimation),
  PROVIDES(WalkStepData),
  USES(WalkStepData),
  PROVIDES(WalkGenerator),
  REQUIRES(WalkGenerator),
  PROVIDES(StandGenerator),
  PROVIDES(WalkingEngineOutput),
  LOADS_PARAMETERS(
  {,
    (ConfiguredParameters) configuredParameters, /**< Speed parameters */
    (StepSizeParameters) stepSizeParameters, /**< Step size parameters. */
    (CommonSpeedParameters) commonSpeedParameters, /**< Acceleration parameters. */
    (ArmParameters) armParameters, /**< Arm parameters. */
    (WalkNeuronalNetworkParameters) walkNeuronalNetworkParameters, /**< Parameters of the walk neuronal network. */
    (FrequencyParameters) frequencyParametersWalk, /**< Frequency parameters. */
    (FrequencyParameters) frequencyParametersKick, /**< Frequency parameters. */
    (BallNetworkParameters) ballNetworkParameters, /**< Ball parameters for the network. */
    (BallNetworkParameters) oldBallNetworkParameters, /**< Ball parameters for the network. */
    (Rangei) standAnkleStiffness,
    (float) ballDistancePolicySwitch,
    (float) ballHysteresisPolicySwitch,
    (bool) shiftBallPosition, /**< If ball is further away to the side, relative to the kick direction, move ball position even further away. */
    (Rangef) shiftBallInterpolationRange,
    (float) shiftBallPositionValue,
    (bool) forceFastKick,
  }),
});

class WalkingEngine : public WalkingEngineBase
{
public:
  /** Constructor */
  WalkingEngine();
  void resetHistoryData();
  void updateHistoryData(const JointAngles& lastAction, const bool init = false);
  const float motionCycleTime = Global::getSettings().motionCycleTime;
  JointAngles lastMeasurement;
  CompiledNN walkPolicy; /**< The compiled neural network for walking. */
  CompiledNN kickPolicy; /**< The compiled neural network for kicking. */
  CompiledNN oldKickPolicy; /**< The compiled neural network for kicking. */
  JointAngles offset;
  unsigned int lastFrameInfo = 0;
  std::vector<Joints::Joint> boosterJoints;
  std::vector<Joints::Joint> boosterWaistJoints;

  struct HistoryData
  {
    Vector3f gravity;
    Vector3a gyro;
    JointAngles measuredAngles;
    JointAngles lastActions;
  };

  RingBuffer<HistoryData, 10> historyBuffer;

private:

  void update(WalkStepData& walkStepData) override;
  void update(StandGenerator& standGenerator) override;
  void update(WalkGenerator& walkGenerator) override;
  void update(WalkingEngineOutput& walkingEngineOutput) override;

  Rangea getMaxRotationToStepFactor(const bool isFastWalk, const Vector2f& stepRatio);
  Vector2f getStepSizeFactor(const Angle rotation, const bool isFastWalk, const Vector2f& walkSpeedRatio);
  void filterTranslationPolygon(std::vector<Vector2f>& polygonOut, std::vector<Vector2f>& polygonIn, const std::vector<Vector2f>& polygonOriginal);
  void generateTranslationPolygon(std::vector<Vector2f>& polygon, const Vector2f& backRight, const Vector2f& frontLeft, const bool useMaxPossibleStepSize);

  /**
   * Compile the model.
   */
  void compile();

  std::vector<Vector2f> translationPolygon; /**< The polygon that defines the max allowed translation for the step size. */
  std::vector<Vector2f> translationPolygonMaxPossible; /**< The polygon that defines the max allowed translation for the actual max limits for the step size. */

  const std::string modelPath = std::string(File::getBHDir()) + "/Config/NeuralNets/Walk/";
};

struct DummyPhase : MotionPhase
{
  using MotionPhase::MotionPhase;

  bool isDone(const MotionRequest&) const override { return true; }
  void calcJoints(const MotionRequest&, JointRequest&, Pose2f&, MotionInfo&) override {}
};

struct WalkPhase : MotionPhase
{
  WalkPhase(WalkingEngine& engine, Pose2f stepTarget, const MotionPhase& lastPhase);

public:
  bool isLeftPhase = false;
  Pose2f step;

  float tBase = 0.f;
  float tWalk = 0.f;
  float frequency = 0.f;
  float rawFrequency = 0.f;
  float lastBallNetBallDistance = 0.f;

  Vector2f oldBall;

private:
  bool isDone(const MotionRequest& motionRequest) const override;
  void calcJoints(const MotionRequest& motionRequest, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo) override;
  std::unique_ptr<MotionPhase> createNextPhase(const MotionPhase& defaultNextPhase) const override;
  void update() override;
  void calcArmJoints(JointRequest& jointRequest);

  void getNextTargetRequest(JointAngles& target);
  std::vector<Joints::Joint> getBoosterLegJointSequence();

  WalkingEngine& engine; /**< A reference to the running motion engine. */
  unsigned lastModelRequest = 0;
  JointAngles nextTarget;
  JointAngles lastTarget;
  JointRequest startTarget;

  unsigned lastWalking = 0;
  unsigned slowWalkStart = 0;
  unsigned armStartInterpolationTimestamp = 0;

  JointRequest leftArm; /**< The last left arm request, that was not set by the WalkPhase. */
  JointRequest rightArm; /**< The last right arm request, that was not set by the WalkPhase. */
  unsigned int leftArmInterpolationStart = 0; /**< The timestamp the last time the left arms where not set by the WalkPhase. */
  unsigned int rightArmInterpolationStart = 0; /**< The timestamp the last time the right arms where not set by the WalkPhase. */
  float leftArmInterpolationTime; /**< The interpolation duration for the left arm. */
  float rightArmInterpolationTime; /**< The interpolation duration for the right arm. */

  bool shouldStop = false;
  bool wasKicking = false;

  friend class WalkingEngine;

protected:
  unsigned freeLimbs() const override
  {
    return bit(MotionPhase::head) | bit(MotionPhase::leftArm) | bit(MotionPhase::rightArm);
  }
};
