/**
 * @file JointDynamicsProvider.h
 * Declaration of a module which estimates the dynamics of all joints.
 *
 * @author: Felix Wenk
 */

#pragma once

/* Include normal B-Human headers. */
#include "Tools/RingBuffer.h"
#include "Tools/Module/Module.h"
#include "Tools/Math/Differentiator.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Sensing/InertiaSensorData.h"
#include "Representations/Sensing/JointDynamics.h"

/* Include Eigen headers and instantiate template types. */
#include "Tools/MMX.h" // suppress warnings
#include "Eigen/Dense"

MODULE(JointDynamicsProvider)
  USES(JointRequest)
  REQUIRES(FrameInfo)
  REQUIRES(FilteredJointData)
  REQUIRES(InertiaSensorData)
  REQUIRES(MassCalibration)
  REQUIRES(MotionRequest)
  USES(MotionInfo)
  USES(IndykickEngineOutput)
  REQUIRES(RobotDimensions)
  PROVIDES_WITH_MODIFY(JointDynamics)
  REQUIRES(JointDynamics)
  PROVIDES_WITH_MODIFY(FutureJointDynamics)

  DEFINES_PARAMETER(float, positionStddev, fromDegrees(0.001f))
  DEFINES_PARAMETER(float, velocityStddev, fromDegrees(0.00001f))
  DEFINES_PARAMETER(float, accelerationStddev, fromDegrees(0.0000005f))
  DEFINES_PARAMETER(Vector2f, gyroStddev, Vector2f(fromDegrees(0.001f), fromDegrees(0.001f)))
  DEFINES_PARAMETER(float, measurementVariance, 5.82129e-07f)
END_MODULE

namespace Eigen
{
  typedef Eigen::Matrix<float, 18, 1> Vector18f;
  typedef Eigen::Matrix<float, 18, 18> Matrix18f;
  typedef Eigen::Matrix<float, 2, 18> Matrix2x18f;
  typedef Eigen::Matrix<float, 18, 2> Matrix18x2f;
  typedef Eigen::Matrix<float, 6, 18> Matrix6x18f;
  typedef Eigen::Matrix<float, 18, 6> Matrix18x6f;
  typedef Eigen::Matrix<float, 6, 6> Matrix6f;
  typedef Eigen::Matrix<float, 6, 1> Vector6f;
};

class JointDynamicsProvider : public JointDynamicsProviderBase
{
public:
  JointDynamicsProvider();
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:

  enum
  {
    numOfLegJoints = 6,
    numOfNonLegJoints = JointData::numOfJoints - 2 * numOfLegJoints,
    frameDelay = 4,
    maxFrameDelay = 10,
    length = 1000
  };
  void update(JointDynamics& jointDynamics);
  void update(FutureJointDynamics& futureJointDynamics);

  void updateModel(const JointRequest& next, Eigen::Vector3f model[JointData::numOfJoints],
                   Differentiator velocity[JointData::numOfJoints]);
  void predictNoLeg(Eigen::Vector3f stateNoLeg[numOfNonLegJoints],
                    Eigen::Matrix3f covarianceNoLeg[numOfNonLegJoints]);
  void predictLeg(Eigen::Vector18f& stateLeftLeg,
                  Eigen::Matrix18f& covarianceLeftLeg,
                  Eigen::Vector18f& stateRightLeg,
                  Eigen::Matrix18f& covarianceRightLeg);
  void correctNoLeg(const JointRequest& next);
  void correctLeg(const JointRequest& next);
  void correctLeg(Eigen::Vector18f& state, Eigen::Matrix18f& covariance,
                  const Eigen::Vector6f& innovation);
  void mergeStateAndModelToJointDynamics(JointDynamics& jointDynamics,
                                         Eigen::Vector3f model[JointData::numOfJoints]) const;

  void applyGyroMeasurementModel(JointDynamics& jointDynamics);

  void symmetrize3(Eigen::Matrix3f& matrix);
  void symmetrize18(Eigen::Matrix18f& matrix);

  void assembleTimeDependentMatrices();
  void assembleCovariances();
  bool covariancesNeedUpdate;

  /**
   * Intercept parameter streaming to compute derived paramaters.
   * Note that this does not work during the construction of the module.
   * @param in The stream from which the object is read
   * @param out The stream to which the object is written.
   */
  void serialize(In* in, Out* out);

  const Eigen::Vector3f xAxis;
  const Eigen::Vector3f yAxis;
  const Eigen::Vector3f zAxis;
  Eigen::Vector3f leftAxis[numOfLegJoints]; /**< Axes of the joints of the left leg. */
  Eigen::Vector3f rightAxis[numOfLegJoints]; /**< Axes of the joints of the right leg. */

  Eigen::Vector3f model[JointData::numOfJoints];
  Eigen::Vector3f stateNoLeg[numOfNonLegJoints];
  Eigen::Matrix3f covarianceNoLeg[numOfNonLegJoints];
  Eigen::Vector18f stateLeftLeg;
  Eigen::Matrix18f covarianceLeftLeg;
  Eigen::Vector18f stateRightLeg;
  Eigen::Matrix18f covarianceRightLeg;
  Eigen::Matrix2f covarianceGyro;

  Eigen::Matrix3f smallA;
  Eigen::Matrix3f smallAtranspose;

  Eigen::Matrix18f A;
  Eigen::Matrix18f Atranspose;
  Eigen::Matrix6x18f C;
  Eigen::Matrix18x6f Ctranspose;


  Eigen::Matrix3f smallCovarianceProcess;
  Eigen::Matrix18f covarianceProcess;

  /** Stores joint requests to calculate the joint-request-joint-data-variance. */
  RingBuffer<JointRequest, maxFrameDelay> jointRequestsBuffer;

  /** True if the joint dynamics have been initialized. */
  bool jointDynamicsInitialized;

  Differentiator velocity[JointData::numOfJoints];

  Vector2<> angularVelocityInDegreesPerSecond;
  Vector2<> gyroMeasurementInDegreesPerSecond;

  float cycleTime; /**< Motion cycle time in seconds. */
};
