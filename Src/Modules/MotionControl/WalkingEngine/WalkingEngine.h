  /**
* @file WalkingEngine.h
* Declaration a module that creates the walking motions
* @author Colin Graf
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Sensing/InertiaSensorData.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/HeadJointRequest.h"
#include "Representations/MotionControl/ArmMotionEngineOutput.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Tools/Math/Matrix.h"
#include "Tools/Range.h"
#include "Tools/RingBuffer.h"
#include "Tools/RingBufferWithSum.h"
#include "Tools/Optimization/ParticleSwarm.h"
#include "WalkingEngineTools.h"
#include "WalkingEngineKicks.h"
#include "SubPhaseParameters.h"

MODULE(WalkingEngine)
  REQUIRES(MotionSelection)
  REQUIRES(MotionRequest)
  REQUIRES(RobotModel)
  REQUIRES(RobotDimensions)
  REQUIRES(MassCalibration)
  REQUIRES(HeadJointRequest)
  REQUIRES(ArmMotionEngineOutput)
  REQUIRES(FrameInfo)
  REQUIRES(TorsoMatrix)
  REQUIRES(GroundContactState)
  REQUIRES(FallDownState)
  REQUIRES(FilteredJointData)
  REQUIRES(InertiaSensorData)
  REQUIRES(DamageConfiguration)
  PROVIDES_WITH_MODIFY(WalkingEngineOutput)
  REQUIRES(WalkingEngineOutput)
  PROVIDES_WITH_MODIFY(WalkingEngineStandOutput)

  LOADS_PARAMETER(VectorYZ, standComPosition) /**< The position of the center of mass relative to the right foot when standing */
  LOADS_PARAMETER(float, standBodyTilt) /**< The tilt of the torso when standing */
  LOADS_PARAMETER(Vector2<>, standArmJointAngles) /**< The joint angles of the left arm when standing */
  LOADS_PARAMETER(int, standHardnessAnklePitch) /**< The hardness of the ankle pitch joint for standing and walking */
  LOADS_PARAMETER(int, standHardnessAnkleRoll) /**< The hardness of the ankle roll joint for standing and walking */

  LOADS_PARAMETER(Vector2<>, walkRef) /**< The position of the pendulum pivot point in Q */
  LOADS_PARAMETER(Vector2<>, walkRefAtFullSpeedX) /**< The position of the pendulum pivot point when walking forwards with maximum speed */
  LOADS_PARAMETER(Range<>, walkRefXPlanningLimit) /**< The limit for shifting the pendulum pivot point towards the x-axis when planning the next step size */
  LOADS_PARAMETER(Range<>, walkRefXLimit) /**< The limit for shifting the pendulum pivot point towards the x-axis when balancing */
  LOADS_PARAMETER(Range<>, walkRefYLimit) /**< The limit for shifting the pendulum pivot point towards the y-axis when balancing */
  LOADS_PARAMETER(Range<>, walkStepSizeXPlanningLimit) /**< The minimum and maximum step size used to plan the next step size */
  LOADS_PARAMETER(Range<>, walkStepSizeXLimit) /**< The minimum and maximum step size when balancing */
  LOADS_PARAMETER(float, walkStepDuration) /**< the duration of a full step cycle (two half steps) */
  LOADS_PARAMETER(float, walkStepDurationAtFullSpeedX) /**< the duration of a full step cycle when walking forwards with maximum speed */
  LOADS_PARAMETER(float, walkStepDurationAtFullSpeedY) /**< the duration of a full step cycle when walking sidewards with maximum speed */
  LOADS_PARAMETER(Vector2<>, walkHeight) /**< the height of the 3d linear inverted pendulum plane (for the pendulum motion towards the x-axis and the pendulum motion towards the y-axis) */
  LOADS_PARAMETER(float, walkArmRotationAtFullSpeedX) /**< The maximum deflection for the arm swinging motion */
  LOADS_PARAMETER(SubPhaseParameters, walkMovePhase) /**< The beginning and length of the trajectory used to move the swinging foot to its new position */
  LOADS_PARAMETER(SubPhaseParameters, walkLiftPhase) /**< The beginning and length of the trajectory used to lift the swinging foot */
  LOADS_PARAMETER(Vector3<>, walkLiftOffset) /**< The height the swinging foot is lifted */
  LOADS_PARAMETER(Vector3<>, walkLiftOffsetAtFullSpeedX) /**< The height the swinging foot is lifted when walking full speed in x-direction */
  LOADS_PARAMETER(Vector3<>, walkLiftOffsetAtFullSpeedY) /**< The height the swinging foot is lifted when walking full speed in y-direction */
  LOADS_PARAMETER(Vector3<>, walkLiftRotation) /**< The amount the swinging foot is rotated while getting lifted */
  LOADS_PARAMETER(float, walkSupportRotation) /**< A rotation added to the supporting foot to boost the com acceleration */
  LOADS_PARAMETER(Vector3<>, walkComLiftOffset) /**< The height the center of mass is lifted within a single support phase */
  LOADS_PARAMETER(float, walkComBodyRotation) /**< How much the torso is rotated to achieve the center of mass shift along the y-axis */

  LOADS_PARAMETER(Pose2D, speedMax) /**< The maximum walking speed (in "size of two steps") */
  LOADS_PARAMETER(float, speedMaxBackwards) /**< The maximum walking speed for backwards walking (in "size of two steps") */
  LOADS_PARAMETER(Pose2D, speedMaxChange) /**< The maximum walking speed deceleration that is used to avoid overshooting of the walking target */

  LOADS_PARAMETER(bool, balance) /**< Whether sensory feedback should be used or not */
  LOADS_PARAMETER(Vector2<>, balanceBodyRotation) /**< A  torso rotation p-control factor */
  LOADS_PARAMETER(Vector2<>, balanceCom) /**< A measured center of mass position adoption factor */
  LOADS_PARAMETER(Vector2<>, balanceComVelocity)  /**< A measured center of mass velocity adoption factor */
  LOADS_PARAMETER(Vector2<>, balanceRef) /**< A pendulum pivot point p-control factor */
  LOADS_PARAMETER(Vector2<>, balanceNextRef) /**< A pendulum pivot point of the upcoming single support phase p-control factor */
  LOADS_PARAMETER(Vector2<>, balanceStepSize) /**< A step size i-control factor */

  LOADS_PARAMETER(float, observerMeasurementDelay) /**< The delay between setting a joint angle and the ability of measuring the result */
  LOADS_PARAMETER(Vector4f, observerProcessDeviation)  /**< The noise of the filtering process that estimates the position of the center of mass */
  LOADS_PARAMETER(Vector2f, observerMeasurementDeviation) /**< The measurement uncertainty of the computed "measured" center of mass position */

  LOADS_PARAMETER(Pose2D, odometryScale) /**< A scaling factor for computed odometry data */

  /* Parameters to calculate the correction of the torso's angular velocity. */
  LOADS_PARAMETER(float, gyroStateGain) /**< Control weight (P) of the torso's angular velocity error. */
  LOADS_PARAMETER(float, gyroDerivativeGain) /**< Control weight (D) of the approximated rate of change of the angular velocity error. */
  LOADS_PARAMETER(float, gyroSmoothing) /**< Smoothing (between 0 and 1!) to calculate the moving average of the y-axis gyro measurements. */

  LOADS_PARAMETER(float, minRotationToReduceStepSize) /** I have no idea what im doing! Colin pls fix this! **/
END_MODULE

/**
* A module that creates walking motions using a three-dimensional linear inverted pendulum
*/
class WalkingEngine : public WalkingEngineBase
{
public:
  /**
  * Called from a MessageQueue to distribute messages
  * @param message The message that can be read
  * @return true if the message was handled
  */
  static bool handleMessage(InMessage& message);

  /**
  * Default constructor
  */
  WalkingEngine();

  /**
  * Destructor
  */
  ~WalkingEngine() {theInstance = 0;}

private:
  /** Helper for optimizer. */
  class Parameters : public WalkingEngineBase
  {
    void update(WalkingEngineOutput&) {}
    void update(WalkingEngineStandOutput&) {}
  };

  /**
  * The size of a single step
  */
  class StepSize
  {
  public:
    Vector3<> translation; /**< The translational component */
    float rotation; /**< The rotational component */

    StepSize() : rotation(0.f) {}

    StepSize(float rotation, float x, float y) : translation(x, y, 0.f), rotation(rotation) {}
  };

  /**
  * A description of the posture of the legs
  */
  class LegPosture
  {
  public:
    Pose3D leftOriginToFoot; /**< The position of the left foot */
    Pose3D rightOriginToFoot; /**< The position of the right foot */
    Vector3<> leftOriginToCom; /**< The position of the center of mass relative to the origin that was used to describe the position of the left foot */
    Vector3<> rightOriginToCom;  /**< The position of the center of mass relative to the origin that was used to describe the position of the right foot */
  };

  /**
  * A description of the posture of the head and the arms
  */
  class ArmAndHeadPosture
  {
  public:
    float headJointAngles[2]; /**< head joint angles */
    float leftArmJointAngles[4]; /**< left arm joint angles */
    float rightArmJointAngles[4]; /**< right arm joint angles */
  };

  /**
  * A description of the posture of the whole body
  */
  class Posture : public LegPosture, public ArmAndHeadPosture {};

  ENUM(MotionType,
    stand,
    stepping
  );

  ENUM(PhaseType,
    standPhase,
    leftSupportPhase,
    rightSupportPhase
  );

  class PendulumPhase
  {
  public:
    unsigned int id; /**< A phase descriptor */
    PhaseType type; /**< What kind of phase is this? */
    Vector2<> k; /**< The constant of the pendulum motion function sqrt(g/h) */
    Vector2<> r; /**< The pendulum pivot point  (in Q) used to compute the pendulum motion */
    Vector2<> rOpt; /**< The initially planned target position of the pendulum pivot point */
    Vector2<> rRef; /**< The target position of the pendulum pivot point that has been adjusted to achieve the step size */
    Vector2<> x0; /**< The position of the center of mass relative to pendulum pivot point */
    Vector2<> xv0; /**< The velocity of the center of mass */
    float td; /**< The time in seconds left till the next pendulum phase */
    float tu; /**< The time in seconds passed since the beginning of the pendulum phase */
    StepSize s; /**< The step size used to reach the pendulum pivot point */
    Vector3<> l; /**< The height the foot of the swinging foot was lifted to implement the step size */
    Vector3<> lRotation; /**< A rotation applied to the foot while lifting it */
    Vector3<> cl; /**< An offset added to the desired center of mass position while lifting the foot */
    bool toStand; /**< Whether the next phase will be a standPhase */
    bool fromStand; /**< Whether the previous phase was a standPhase */
    WalkRequest::KickType kickType; /**< The type of kick executed during the phase */
  };

  class PendulumPlayer
  {
  public:
    WalkingEngine* engine;

    PendulumPhase phase;
    PendulumPhase nextPhase;

    void seek(float deltaTime);

    void getPosture(LegPosture& posture, float* leftArmAngle, float* rightArmAngle, Pose2D* stepOffset);
    void getPosture(Posture& posture);

  private:
    WalkingEngineKickPlayer kickPlayer;
  };

  static PROCESS_WIDE_STORAGE(WalkingEngine) theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none */

  // computed parameters
  RotationMatrix standBodyRotation; /**< The basic orientation of the torso */
  Vector2<> walkK; /**< The constant of the pendulum motion function sqrt(g/h) */
  float walkPhaseDuration; /**< The basic duration of a single support phase */
  float walkPhaseDurationAtFullSpeedX; /**< The duration of single support phase when walking full with full speed in x-direction */
  float walkPhaseDurationAtFullSpeedY; /**< The duration of single support phase when walking full with full speed in y-direction */
  Range<> walkXvdXPlanningLimit; /**< A limit of the center of mass velocity used to plan the center of mass trajectory */
  Range<> walkXvdXLimit; /**< A limit of the center of mass to protect the walking engine from executing steps that are too large */

  /**
  * Intercept parameter streaming to compute derived paramaters.
  * Note that this does not work during the construction of the module.
  * @param in The stream from which the object is read
  * @param out The stream to which the object is written.
  */
  void serialize(In* in, Out* out);

  /** Initialize derived parameters. */
  void init();

  void reset();

  /**
  * The central update method to generate the walking motion
  * @param walkingEngineOutput The WalkingEngineOutput (mainly the resulting joint angles)
  */
  void update(WalkingEngineOutput& walkingEngineOutput);
  MotionType currentMotionType;
  JointRequest jointRequest;
  PendulumPlayer pendulumPlayer;
  PendulumPlayer predictedPendulumPlayer;
  WalkingEngineKicks kicks;

  /**
  * The update method to generate the standing stance
  * @param standOutput The WalkingEngineStandOutput (mainly the resulting joint angles)
  */
  void update(WalkingEngineStandOutput& standOutput) {(JointRequest&)standOutput = jointRequest;}

  // attributes used by module:WalkingEngine:optimize debug response
  ParticleSwarm optimizeOptimizer;
  RingBufferWithSum<float, 300> optimizeFitness;
  bool optimizeStarted;
  unsigned int optimizeStartTime;
  Parameters optimizeBestParameters;

  void updateMotionRequest();
  MotionType requestedMotionType;
  Pose2D requestedWalkTarget;
  Pose2D lastCopiedWalkTarget;

  void updatePendulumPlayer();
  Vector2<> observedComOffset;

  void computeMeasuredPosture();
  Vector3<> measuredLeftToCom;
  Vector3<> measuredRightToCom;

  void computeExpectedPosture();
  Vector3<> expectedLeftToCom;
  Vector3<> expectedRightToCom;
  Vector2<> expectedComVelocity;
  Pose2D observedStepOffset;

  void computeEstimatedPosture();
  Vector3<> estimatedLeftToCom;
  Vector3<> estimatedRightToCom;
  Vector2<> estimatedComVelocity;
  Vector3<> lastExpectedLeftToCom;
  Vector3<> lastExpectedRightToCom;
  Vector2<> lastExpectedComVelocity;
  Matrix3x3f covX;
  Matrix3x3f covY;

  void computeError();
  Vector3<> errorLeft;
  Vector3<> errorRight;
  Vector2<> errorVelocity;
  RingBufferWithSum<float, 300> instability;

  void correctPendulumPlayer();

  void updatePredictedPendulumPlayer();
  void applyCorrection(PendulumPhase& phase, PendulumPhase& nextPhase);
  Vector2<> measuredPx; /**< The measured com position (in Q) */
  Vector2<> measuredR; /**< The measured "zmp" */

  void generateTargetPosture();
  Posture targetPosture;

  void generateJointRequest();
  Vector3<> bodyToCom;
  Vector3<> lastAverageComToAnkle;
  RotationMatrix lastBodyRotationMatrix;
  RingBuffer<float, 10> relativeRotations;
  float lastSmoothedGyroY; /**< Moving average of the y-axis gyro from the previous motion frame. */
  float lastGyroErrorY; /**< Last y-axis gyro deviation from the commanded angular velocity of the torso. */

  void generateOutput(WalkingEngineOutput& WalkingEngineOutput);
  void generateDummyOutput(WalkingEngineOutput& WalkingEngineOutput);

  void generateFirstPendulumPhase(PendulumPhase& phase);
  void generateNextPendulumPhase(const PendulumPhase& phase, PendulumPhase& nextPhase);
  RingBuffer<PendulumPhase, 5> phaseBuffer;
  void computeNextPendulumParamtersY(PendulumPhase& nextPhase, float walkPhaseDurationX, float walkPhaseDurationY) const;
  void computeNextPendulumParamtersX(PendulumPhase& nextPhase) const;

  void updatePendulumPhase(PendulumPhase& phase, PendulumPhase& nextPhase, bool init) const;
  void repairPendulumParametersY(PendulumPhase& phase, const PendulumPhase& nextPhase) const;
  void updatePendulumParametersY(PendulumPhase& phase, PendulumPhase& nextPhase) const;
  void updatePendulumParametersX(PendulumPhase& phase, PendulumPhase& nextPhase, bool init) const;

  void generateNextStepSize(PhaseType nextSupportLeg, StepSize& stepSize);
  Pose2D lastRequestedSpeedRel;
  Pose2D lastSelectedSpeed;

  void computeOdometryOffset();
  Pose2D odometryOffset;
  Pose3D lastFootLeft;
  Pose3D lastFootRight;
  Vector3<> lastOdometryOrigin;
  Pose2D upcomingOdometryOffset;

  static float asinh(float x);
  static float atanh(float x);

  /**
  * A smoothed blend function with f(0)=0, f'(0)=0, f(1)=1, f'(1)=0, f(2)=0, f'(2)=0
  * @param x The function parameter [0...2]
  * @return The function value
  */
  static float blend(float x);

  // debug drawings
  void declareDrawings(const WalkingEngineOutput& walkingEngineOutput) const;

  // The different coordinate systems
  void drawW() const;
  void drawP() const;
  void drawQ(const WalkingEngineOutput& walkingEngineOutput) const;

  /*
  void drawFootTrajectory(const WalkingEngineOutput& walkingEngineOutput) const;
  void drawPreview(WalkRequest::KickType previewKickType, const WalkingEngineOutput& walkingEngineOutput) const;
  void drawSkeleton(const WalkingEngineOutput& walkingEngineOutput) const;
  void drawSkeleton2(const WalkingEngineOutput& walkingEngineOutput) const;
  void drawJoints(const WalkingEngineOutput& walkingEngineOutput) const;
  void drawJoints2(const WalkingEngineOutput& walkingEngineOutput) const;
  void drawPendulum(const WalkingEngineOutput& walkingEngineOutput) const;
  void drawInvKin1(const WalkingEngineOutput& walkingEngineOutput) const;
  void drawInvKin2(const WalkingEngineOutput& walkingEngineOutput) const;
  void drawInvKin3(const WalkingEngineOutput& walkingEngineOutput) const;
  void drawShowcaseCenterOfMass(const WalkingEngineOutput& walkingEngineOutput) const;
  void drawShowcaseContactAreaLeft() const;
  void drawShowcaseContactAreaRight() const;
  void drawShowcaseSupportAreaLeft() const;
  void drawShowcaseSupportAreaBoth(const WalkingEngineOutput& walkingEngineOutput) const;
  void drawShowcaseIpTrajectory(const WalkingEngineOutput& walkingEngineOutput) const;
  void drawShowcaseIp(const WalkingEngineOutput& walkingEngineOutput) const;
  void drawShowcaseIpSphere(const WalkingEngineOutput& walkingEngineOutput) const;
  void drawShowcase3dlipmPlane(const WalkingEngineOutput& walkingEngineOutput) const;
  void drawShowcaseSimpleIp(const WalkingEngineOutput& walkingEngineOutput) const;
  void drawShowcaseZmp(const WalkingEngineOutput& walkingEngineOutput) const;
  void drawShowcaseFloorGrid(const Pose3D& torsoMatrix) const;
  static const Vector3<> drawFootPoints[];
  static const unsigned int drawNumOfFootPoints;
  void drawComPlot();
  Vector2<> drawComPlotOrigin;
  Vector2<> drawComPlotLastR;
  unsigned int drawComPlotState;
  //unsigned int drawMeasurementDelay;
  */
  static const Vector3<> drawFootPoints[];
  static const unsigned int drawNumOfFootPoints;
  void drawZmp();
};
