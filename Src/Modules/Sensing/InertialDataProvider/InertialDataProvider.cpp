#include "InertialDataProvider.h"
#include "Debugging/Modify.h"
#include "Debugging/Plot.h"
#include "Math/BHMath.h"
#include "Math/Eigen.h"
#include "Math/Rotation.h"
#include "Platform/SystemCall.h"

#include "Math/RotationMatrix.h"

#include <cmath>
#include <sstream>

MAKE_MODULE(InertialDataProvider);

InertialDataProvider::State InertialDataProvider::State::operator+(const Vector3f& angleAxis) const
{
  return State(*this) += angleAxis;
}

InertialDataProvider::State& InertialDataProvider::State::operator+=(const Vector3f& angleAxis)
{
  orientation = orientation * Rotation::AngleAxis::unpack(angleAxis);
  return *this;
}

Vector3f InertialDataProvider::State::operator-(const State& other) const
{
  return Rotation::AngleAxis::pack(AngleAxisf(other.orientation.inverse() * orientation));
}

void InertialDataProvider::update(InertialData& inertialData)
{
  DECLARE_PLOT("module:InertialDataProvider:diffToAngle");

  DECLARE_PLOT("module:InertialDataProvider:dev1");
  DECLARE_PLOT("module:InertialDataProvider:dev10");
  DECLARE_PLOT("module:InertialDataProvider:dev50");

  bool ignoreAccUpdate = false;
  MODIFY("module:InertialDataProvider:ignoreAccUpdate", ignoreAccUpdate);

  if(SystemCall::getMode() == SystemCall::Mode::simulatedRobot || theGyroOffset.bodyDisconnect || framesSinceStart < 2 || theMotionRobotHealth.frameLostStatus == MotionRobotHealth::bodyDisconnect)
  {
    // This reinitializes the filter after a sudden orientation change in the simulation, e.g. when rotating the robot via commands
    if((lastRawAngle - theInertialSensorData.angle.head<2>()).norm() > pi / 3.f)
    {
      Quaternionf zRot;
      Rotation::splitOffZRotation(ukf.mean.orientation, zRot);
      ukf.init(State(zRot), Matrix3f::Zero());
    }
    //This reinitializes the filter after the real robot lost the connection to its sensory (V6 head flex problem)
    if(theGyroOffset.bodyDisconnect || framesSinceStart < 2 || theMotionRobotHealth.frameLostStatus == MotionRobotHealth::bodyDisconnect)
    {
      Quaternionf rot = Rotation::Euler::fromAngles(theInertialSensorData.angle.x(), theInertialSensorData.angle.y(), 0.f);
      ukf.init(State(rot), Matrix3f::Zero());
    }
    lastRawAngle = theInertialSensorData.angle.head<2>();
  }

  inertialData.acc = theInertialSensorData.acc;
  inertialData.gyro = theInertialSensorData.gyro;

  processGyroscope(inertialData.gyro);

  if(!ignoreAccUpdate && theInertialSensorData.newAccData)
    processAccelerometer(inertialData.acc);

  ASSERT(ukf.mean.orientation.coeffs().allFinite());
  ukf.mean.orientation.normalize();

  DEBUG_RESPONSE_ONCE("module:InertialDataProvider:printCov")
  {
    Eigen::IOFormat octaveFmt(Eigen::FullPrecision, 0, ", ", ";\n", "", "", "[", "]");
    std::stringstream stream;
    stream << ukf.cov.format(octaveFmt);
    OUTPUT_TEXT(stream.str());
  }

  inertialData.orientation2D = Rotation::removeZRotation(ukf.mean.orientation);
  inertialData.orientation3D = ukf.mean.orientation;
  inertialData.angleChange = inertialData.angle.head<2>();
  if(theMotionInfo.executedPhase == MotionPhase::stand && theCalibrationRequest.targetState != CameraCalibrationStatus::State::idle)
    inertialData.angle << theInertialSensorData.angle;
  else
    inertialData.angle << Rotation::AngleAxis::pack(AngleAxisf(inertialData.orientation2D)).head<2>().cast<Angle>(), 0_deg;
  inertialData.angleChange = inertialData.angle.head<2>() - inertialData.angleChange;

  inertialData.orientation3DCov = ukf.cov;

  const Vector3a angleAxisVec = Rotation::AngleAxis::pack(AngleAxisf(ukf.mean.orientation)).cast<Angle>();
  PLOT("module:InertialDataProvider:internalOrientation:x", angleAxisVec.x().toDegrees());
  PLOT("module:InertialDataProvider:internalOrientation:y", angleAxisVec.y().toDegrees());
  PLOT("module:InertialDataProvider:internalOrientation:z", angleAxisVec.z().toDegrees());

  framesSinceStart++;

  const RotationMatrix diff = ukf.mean.orientation.inverse() * RotationMatrix(Rotation::Euler::fromAngles(theInertialSensorData.angle));
  PLOT("module:InertialDataProvider:diffToAngle", diff.getAngleAxis().angle());
}

void InertialDataProvider::processAccelerometer(const Vector3f& acc)
{
  auto measurementModel = [&](const State& state)
  {
    const Vector3f gVec(0.f, 0.f, theIMUValueState.accLength);
    return state.orientation.inverse() * gVec;
  };

  const Vector3f& useBaseAccVariance = theMotionInfo.executedPhase == MotionPhase::walk && theGroundContactState.contact
                                       ? accVarianceWhileWalking
                                       : theMotionInfo.executedPhase == MotionPhase::stand
                                       ? accVarianceWhileStanding
                                       : accVariance;
  Vector3f accLengthNoise = Vector3f::Zero();
  if(theMotionInfo.executedPhase != MotionPhase::walk && theMotionInfo.executedPhase != MotionPhase::stand)
    accLengthNoise = sqr(acc.norm() - theIMUValueState.accLength) * accVarianceFactor;
  const Vector3f measurementNoise = useBaseAccVariance + accLengthNoise;
  ukf.update<3>(acc, measurementModel, measurementNoise.cwiseAbs2().asDiagonal());
}

void InertialDataProvider::processGyroscope(const Vector3a& gyro)
{
  auto dynamicModel = [&](State& state)
  {
    state.orientation = state.orientation * Rotation::AngleAxis::unpack(gyro.cast<float>() * Constants::motionCycleTime);
  };

  const Vector3f dynamicNoise = gyroVariance.cast<float>() * std::sqrt(1.f / Constants::motionCycleTime);
  ukf.predict(dynamicModel, dynamicNoise.cwiseAbs2().asDiagonal());
}
