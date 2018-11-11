#pragma once

#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Configuration/JointLimits.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/MotionControl/Balancer.h"
#include "Representations/Sensing/FootSupport.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/Debugging/ColorRGBA.h"
#include "Tools/Math/Angle.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Module/Module.h"
#include "Tools/Range.h"

#include <vector>
#include <cmath>

MODULE(ZmpBalancer,
{,
  REQUIRES(FootSupport),
  REQUIRES(InertialSensorData),
  REQUIRES(FrameInfo),
  REQUIRES(RobotModel),
  REQUIRES(InertialData),
  REQUIRES(RobotDimensions),
  REQUIRES(KeyStates),
  REQUIRES(JointLimits),
  REQUIRES(MassCalibration),
  REQUIRES(JointAngles),
  REQUIRES(DamageConfigurationBody),
  USES(JointRequest),
  PROVIDES(Balancer),
  LOADS_PARAMETERS(
  {,
    (Vector2f) leftLIPOrigin,
  }),
});

class ZmpBalancer : public ZmpBalancerBase
{
public:
  void update(Balancer& stand) override;
  ZmpBalancer();

  /**
   * Implementation of Balancer::init
   */
  Pose3f init(bool rightIsSupportFoot, const BalancingParameter& param, Balancer& representation);

  /**
   * Implementation of Balancer::initWithTarget
   */
  void initWithTarget(bool rightIsSupportFoot, const Pose3f& targetZMP, float initalizationTime, const BalancingParameter& param, Balancer& representation);

  /**
   * Implementation of Balancer::calcBalancedJoints
   */
  bool calcBalancedJoints(JointRequest& jointRequest, const Pose3f& swingFootInSupport, float comHeight, const RotationMatrix& torsoRotation, const std::vector<float>& zmpPreviewsX, const std::vector<float>& zmpPreviewsY, const BalancingParameter& param, Balancer& representation);

  /**
   * Implementation of Balancer::addBalance
   */
  bool addBalance(JointRequest& jointRequest, const std::vector<float>& zmpPreviewsX, const std::vector<float>& zmpPreviewsY, const BalancingParameter& param, Balancer& representation);

  /**
   * Implementation of Balancer::balanceJointRequest
   */
  bool balanceJointRequest(JointRequest& jointRequest, const BalancingParameter& param, Balancer& representation);

private:

  /** Copies JointAngles from Measurements to JointRequest
   * for all joints, which are switched off, to use it for CenterOfMass calculations
   **/
  void integrateDamageConfig(JointRequest& jointRequest);

  /**
   * Calculates center of mass relative to the center of support foot on ground for given joint angles
   */
  Pose3f calcComInStand(const JointAngles& angles);

  /**
   * Calculates a new set of Joint angles under consideration of a swing foot position, a torso rotation and center of mass relative to a support foot
   */
  void calcJointAngles(const Pose3f& comInStand, const Pose3f& swingInSupoort, const Quaternionf& torsoRotation, const JointAngles& theJointAngles, JointRequest& jointRequest);

  /**
   * draw 3D Coordinate System for debugging
   */
  void drawCoordinateSystem(Pose3f point, ColorRGBA colorX, ColorRGBA colorY, ColorRGBA colorZ);

  //calculates a trajectory under consideration of a maximal acceleration and current celocity to reach a given start position
  std::vector<float> calcZMPTrajectory(const Vector3f& state, float targetPos, float maxAcceleration, float minEndTime);

  ZmpController zmpControllerX;
  ZmpController zmpControllerY;
  LIPStateEstimator estimator;

  Vector3f comInTorso;      // last com In Torso translation for a quick iterative computation of the current comInTorso
  Vector3f stateX;          // current com, velocity and zmp
  Vector3f stateY;
  float comHeight;          // current com height
  bool rightIsSupportFoot;

  std::vector<float> initialZMPTrajectoryX;     //next zmp values for initial phase
  std::vector<float> initialZMPTrajectoryY;
  size_t initializationSteps;                   //total length of initial phase (not remaining)
  float initialComHeight;                       //com height which has to br reached at the end of initial phase
  Vector3f torsoRotationOffset;                 //offset to avoid jumps in initial phase

  Angle lastAnkleRoll;      // saves support ankleRole from last JointRequest to compare it with next JointAngles
  Angle lastKnee;           // saves support Knee from last JointRequest to compare it with next JointAngles

  Vector2f tilt;            // current tilt (filtered)

  Pose3f leftAnkleTorso;    // stored in member variable to make it accessible for debug drawings
  Pose3f rightAnkleTorso;

  void mirrorConstants(bool rightIsSupportFoot); // prepares the following variables depending on support foot

  float sign; // rightIsSupportFoot ? -1.f : 1.f;

  Pose3f standInAnkle;
  Pose3f soleInAnkle;
  Pose3f standInSole;

  Pose3f ankleInStand;
  Pose3f ankleInSole;
  Pose3f soleInStand;

  int sHipPitch;
  int sHipRoll;
  int swingHipRoll;
  int sAnkleRoll;
  int swingAnkleRoll;
  int swingAnklePitch;
  int sKneePitch;
  int sAnklePitch;
  int firstSupportLegJoint;
  int firstSwingLegJoint;
};
