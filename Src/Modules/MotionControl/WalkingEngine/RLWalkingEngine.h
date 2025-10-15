/**
 * @file RLWalkingEngine.h
 *
 * @Author Philip Reichenberg
 */

#pragma once

#include "Framework/Module.h"
#include "Framework/Settings.h"
#include "Representations/Configuration/JointLimits.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/StiffnessData.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/StandGenerator.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Representations/MotionControl/WalkStepData.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/FootSupport.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/SolePressureState.h"
#include "Tools/Motion/MotionPhase.h"

#include "Platform/File.h"
#include <CompiledNN2ONNX/CompiledNN.h>

using namespace NeuralNetworkONNX;

STREAMABLE(BoosterArmParameters,
{,
  (Angle) armShoulderRoll, /**< Arm shoulder angle in radians. */
  (float) armShoulderRollIncreaseFactor, /**< Factor between sideways step size (in m) and additional arm roll angles. */
  (float) armShoulderPitchFactor, /**< Factor between forward foot position (in m) and arm pitch angles. */
  (Angle) armShoulderPitch, /**< Arm shoulder pitch angle. */
  (Angle) armElbowYaw, /**< Arm elbow yaw angle. */
});

STREAMABLE(BoosterCommonSpeedParameters,
{,
  (Vector2f) maxAcceleration,  /**< Maximum acceleration of forward and sideways speed at each leg change to ratchet up/down in (mm/s/step). */
  (Vector2f) maxDeceleration, /**< (Positive) maximum deceleration of forward and sideways speed at each leg change to ratchet up/down in (mm/s/step). */
});

STREAMABLE(BoosterStepSizeParameters,
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

STREAMABLE(BoosterConfiguredParameters,
{,
  (Pose2f) maxSpeed, /**< Maximum speeds in mm/s and degrees/s. */
  (float) maxSpeedBackwards, /**< Maximum backwards speed. Positive, in mm/s. */
  (Pose2f) thresholdStopStandTransition, /**< Threshold to be able to switch from walk state stopping to standing. */
  (float) baseWalkPeriod,
});

STREAMABLE(FrequencyParameters,
{,
  (float) base, /**< Base frequency. */
  (Rangef) clipRange, /**< Clip frequency offset from neuronal network. */
  (bool) active, /**< Does the neuronal network output a frequency offset? */
});

STREAMABLE(BallNetworkParameters,
{,
  (bool) active,
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

MODULE(RLWalkingEngine,
{,
  REQUIRES(FootSupport),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(GroundContactState),
  REQUIRES(InertialData),
  REQUIRES(JointAngles),
  USES(JointRequest),
  REQUIRES(MassCalibration),
  REQUIRES(OdometryDataPreview),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  REQUIRES(SolePressureState),
  PROVIDES(WalkStepData),
  USES(WalkStepData),
  PROVIDES(WalkGenerator),
  REQUIRES(WalkGenerator),
  PROVIDES(StandGenerator),
  PROVIDES(WalkingEngineOutput),
  LOADS_PARAMETERS(
  {,
    (BoosterConfiguredParameters) configuredParameters, /**< Speed parameters */
    (BoosterStepSizeParameters) stepSizeParameters, /**< Step size parameters. */
    (BoosterCommonSpeedParameters) commonSpeedParameters, /**< Acceleration parameters. */
    (BoosterArmParameters) armParameters, /**< Arm parameters. */
    (WalkNeuronalNetworkParameters) walkNeuronalNetworkParameters, /**< Parameters of the walk neuroal network. */
    (FrequencyParameters) frequencyParameters, /**< Frequency parameters. */
    (BallNetworkParameters) ballNetworkParameters, /**< Ball parameters for the network. */
  }),
});

class RLWalkingEngine : public RLWalkingEngineBase
{
public:
  /** Constructor */
  RLWalkingEngine();
  const float motionCycleTime = Global::getSettings().motionCycleTime;
  RingBuffer<JointAngles, 10> lastMeasurements;
  CompiledNN network; /**< The compiled neural network. */
  JointAngles offset;

  std::vector<Joints::Joint> boosterJoints;
  std::vector<Joints::Joint> boosterWaistJoints;
  std::vector<Joints::Joint> naoJoints;
private:

  void update(WalkStepData& walkStepData) override;
  void update(StandGenerator& standGenerator) override;
  void update(WalkGenerator& walkGenerator) override;
  void update(WalkingEngineOutput& walkingEngineOutput) override;

  Rangea getMaxRotationToStepFactor(const bool isFastWalk, const bool isLeftPhase, const Angle customReduceOffset, const Vector2f& stepRatio);
  Vector2f getStepSizeFactor(const Angle rotation, const bool isFastWalk, const bool isLeftPhase, const Angle customReduceOffset, const Vector2f& walkSpeedRatio);
  void filterTranslationPolygon(std::vector<Vector2f>& polygonOut, std::vector<Vector2f>& polygonIn, const std::vector<Vector2f>& polygonOriginal);
  void generateTranslationPolygon(std::vector<Vector2f>& polygon, const Vector2f& backRight, const Vector2f& frontLeft, const bool useMaxPossibleStepSize);

  /**
   * Compile the model.
   * @param output Whether to output information.
   */
  void compile(bool output);

  std::vector<Vector2f> translationPolygon; /**< The polygon that defines the max allowed translation for the step size. */

  const std::string modelPath = std::string(File::getBHDir()) + "/Config/NeuralNets/BoosterWalk/";
};

struct DummyPhase : MotionPhase
{
  using MotionPhase::MotionPhase;

  bool isDone(const MotionRequest&) const override { return true; }
  void calcJoints(const MotionRequest&, JointRequest&, Pose2f&, MotionInfo&) override {}
};

struct RLWalkPhase : MotionPhase
{
  RLWalkPhase(RLWalkingEngine& engine, Pose2f stepTarget, const MotionPhase& lastPhase, const bool isDelay = false,
              const WalkGenerator::CreateNextPhaseCallback& createNextPhaseCallback = WalkGenerator::CreateNextPhaseCallback(),
              const WalkKickStep& walkKickStep = WalkKickStep());

public:
  bool isLeftPhase = false;
  Pose2f step;
  WalkKickStep walkKickStep;

  float tBase = 0.f;
  float tWalk = 0.f;
  float frequency = 0.f;
  float rawFrequency = 0.f;
private:
  bool isDone(const MotionRequest& motionRequest) const override;
  void calcJoints(const MotionRequest& motionRequest, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo) override;
  std::unique_ptr<MotionPhase> createNextPhase(const MotionPhase& defaultNextPhase) const override;
  void update() override;

  void getNextTargetRequest(JointAngles& target);
  std::vector<Joints::Joint> getBoosterLegJointSequence();

  RLWalkingEngine& engine; /**< A reference to the running motion engine. */
  unsigned lastModelRequest = 0;
  JointAngles nextTarget;
  JointAngles lastTarget;
  JointAngles requestAtStandStart;
  JointAngles standTarget;
  WalkGenerator::CreateNextPhaseCallback callBack;

  unsigned lastWalking = 0;
  unsigned slowWalkStart = 0;

  friend class WalkingEngine;

protected:
  unsigned freeLimbs() const override
  {
    return bit(MotionPhase::head);
  }
};
