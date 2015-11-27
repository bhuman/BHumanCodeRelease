#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Sensing/JointDataPrediction.h"
#include "Representations/MotionControl/DmpKickEngineOutput.h"
#include "Representations/Sensing/Zmp.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Configuration/MotionSettings.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include <string>
#include <vector>
#include "DmpKickState.h"
#include "Tools/Enum.h"
#include <unordered_map>

STREAMABLE(KickTrajectory,
{,
  (Vector3f) startPos,
  (Vector3f) startVel,
  (Vector3f) endPos,
  (Vector3f) endVel,
  (float) executionTime,
  (float)(0.15f) overlap,
  (float)(0.1f) finalPhaseValue,
  (unsigned)(35) numWeights,
  (std::vector<float>) weightsDim0,
  (std::vector<float>) weightsDim1,
  (std::vector<float>) weightsDim2,
});

MODULE(DmpKickEngine,
{,
  USES(JointRequest),
  REQUIRES(JointDataPrediction),
  REQUIRES(Zmp),
  REQUIRES(RobotModel),
  REQUIRES(RobotDimensions),
  REQUIRES(MassCalibration),
  REQUIRES(MotionRequest),
  REQUIRES(FrameInfo),
  REQUIRES(BallModel),
  REQUIRES(TorsoMatrix),
  REQUIRES(FieldDimensions),
  REQUIRES(MotionSettings),
  REQUIRES(BallPercept),
  REQUIRES(JointAngles),
  REQUIRES(InertialSensorData),
  PROVIDES(DmpKickEngineOutput),
  LOADS_PARAMETERS(
  {,
    (float)(0.4f) balanceRotation, /**< Value between 0 and 1 that determines how much of the balancing will be done using the torso rotation */
    (unsigned)(50) numZmpPreviews, /**< Size of the zmp preview vector */
    (double)(0.7f) R,
    (double)(0.1f) Qe,
    (double)(0.1f) Qx,
    (Vector3d)(Vector3d(0.01f, 0.00001f, 0.00005f)) QlDiag,
    (Vector2d)(Vector2d(0.1f, 0.7f)) R0Diag,
    (float)(25.0f) zmpTargetX,//set as if leftSupport was true
    (float)(0.0f) zmpTargetY,//set as if leftSupport was true
    (float)(8.0f) initiallyStableRadius,
    (float)(85.0f) footLength,
    (int)(3) additionalKickSteps, //set to zero when using imitated kicks
    (float)(-75.0f) xStartPos,
    (float)(1.4f) balanceTime, //in seconds
    (float)(0.0f) kickFootH,
    (bool)(true) balanceSensorFeedback, /**<If true sensor feedback will be used for balancing */
    (Vector2f)(Vector2f(30, -30)) comP,
    (float)(1.0f) moveBackTime,
    (float)(5.0f) moveBackEndHeight,
    (float)(20.0f) targetTime, //in mm/s
  }),
});

/**
 * A state machine that controls the kick motion
 */
class DmpKickEngine : public DmpKickEngineBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  DmpKickEngine();
private:
  void update(DmpKickEngineOutput& output);

  /**
   * @param[in, out] torsoInSole the current torsoInSole pose. Will be replaced with the next pose
   */
  void setJoints(Vector3f comInSole, const Vector3f& nextComInSole,
                 Pose3f& torsoInSole,
                 const Vector3f& kickFootInSole) const;

  ENUM(State,
  {,
    Idle,
    InitKick,
    Balance,
    IterateModel,
    InitDynKick,
    InitImitatedKick,
    DynKick,
    ImitatedKick,
    Setup,
  });

  /**Tell the state machine to execute newState immediately*/
  void goTo(State newState);
  /**Tell the state machine to wait execute newState next frame*/
  void goToNext(State newState);

  /**The state functions*/
  void stateIdle();

  /**Initializes the kickState based on the ExpKickRequest */
  void stateInitKick();
  void stateBalance();
  void stateIterateModel();
  void stateInitDynKick();
  void stateInitImitatedKick();
  void stateDynKick();
  void stateSetup();
  void stateImitatedKick();

  /**Reinitializes the zmp parameters if they changed.*/
  void updateZmpParameters();
  //check if some of the parameters changed
  bool parametersChanged();
  /**Initializes the zm controllers.
   * kickState.comInSole should be updated before calling this method */
  void initZmpControllers();

  //goes to the kick state depending on kickState.kickType
  void goToKick();

  /**Calculates the time needed to kick at the specified speed*/
  float calcTimeForSpeed(const float endSpeed, const float startPos, const float endPos) const;

  Vector2f getNextPreview();

  void updateZmpPreviews();

  void checkBalanced();

  /**Returns the kick target relative to kickPoseZero.
     The kick target defines where the kickSole coordinate system should be to
     kick the ball.*/
  Vector3f getKickTarget() const;

  /**writes log data in yaml format*/
  void writeLog(const std::string& filePath) const;

  State state; /**< The current state */
  bool nextFrame; /**<If true the next state will be executed in the next frame. If false the next state will be executed immediately */
  std::function<void()> states[numOfStates];

  DmpKickState kickState;/**<Contains all data relevant for the motion calculation */
  DmpKickEngineOutput* output;

  Vector3f oldKickSoleInSupportSole;//the kick pose from last frame. used for plotting the velocity

  //old parameters
  float oldComHeight;
  unsigned oldNumZmpPreviews;
  double oldR;
  double oldQe;
  double oldQx;
  double oldQlDiagNorm;
  double oldR0DiagNorm;

  std::vector<Vector2f> targets;
  int targetIndex;
  int step;
  int totalSteps;
  int from;
  int to;

  KickTrajectory trajectory;
};
