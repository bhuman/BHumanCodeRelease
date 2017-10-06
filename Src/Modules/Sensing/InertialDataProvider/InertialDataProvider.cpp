#include "InertialDataProvider.h"
#include "Platform/SystemCall.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/Rotation.h"

#include <sstream>

MAKE_MODULE(InertialDataProvider, sensing)

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
  if(SystemCall::getMode() == SystemCall::Mode::simulatedRobot)
  {
    // This reinitializes the filter after a sudden orientation change in the simulation, e.g. when rotating the robot via commands
    if((lastRawAngle - theInertialSensorData.angle).norm() > pi / 3.f)
    {
      Quaternionf zRot;
      Rotation::splitOffZRotation(ukf.mean.orientation, zRot);
      ukf.init(State(zRot), Matrix3f::Zero());
    }
    lastRawAngle = theInertialSensorData.angle;
  }

  const Quaternionf rotation(theIMUCalibration.rotation);
  inertialData.acc = rotation * theInertialSensorData.acc;
  inertialData.gyro = (rotation * theInertialSensorData.gyro.cast<float>()).cast<Angle>();

  const Vector3f& chosenAccDeviation = theMotionInfo.motion == MotionRequest::Motion::walk ? accDeviationWhileWalking : accDeviation;
  estimate(inertialData.gyro, inertialData.acc, Constants::motionCycleTime, gyroDeviation, chosenAccDeviation);

  inertialData.orientation2D = Rotation::removeZRotation(ukf.mean.orientation);
  inertialData.orientation3D = ukf.mean.orientation;
  inertialData.angle = Rotation::AngleAxis::pack(AngleAxisf(inertialData.orientation2D)).head<2>().cast<Angle>();

  const Vector3a angleAxisVec = Rotation::AngleAxis::pack(AngleAxisf(ukf.mean.orientation)).cast<Angle>();
  PLOT("module:InertialDataProvider:internalOrientation:x", angleAxisVec.x().toDegrees());
  PLOT("module:InertialDataProvider:internalOrientation:y", angleAxisVec.y().toDegrees());
  PLOT("module:InertialDataProvider:internalOrientation:z", angleAxisVec.z().toDegrees());
}

void InertialDataProvider::estimate(const Vector3a& gyro, const Vector3f& acc, float timePassed, const Vector3a& gyroDeviation, const Vector3f& accDeviation)
{
  auto dynamicModel = [&](State& state)
  {
    state.orientation = state.orientation * Rotation::AngleAxis::unpack(gyro.cast<float>() * timePassed);
  };
  auto measuremantModelAcc = [&](const State& state)
  {
    const Vector3f gVec = Vector3f(0.f, 0.f, Constants::g_1000);
    return state.orientation.inverse() * gVec;
  };

  const Vector3f dynamicNoise = gyroDeviation.cast<float>() * std::sqrt(1.f / timePassed);
  const Vector3f measurementNoise = accDeviation;
  ukf.predict(dynamicModel, dynamicNoise.asDiagonal());
  ukf.update<3>(acc, measuremantModelAcc, measurementNoise.asDiagonal());

  ASSERT(ukf.mean.orientation.coeffs().allFinite());
  ukf.mean.orientation.normalize();

  DEBUG_RESPONSE_ONCE("module:InertialDataProvider:printCov")
  {
    Eigen::IOFormat octaveFmt(Eigen::FullPrecision, 0, ", ", ";\n", "", "", "[", "]");
    std::stringstream stream;
    stream << ukf.cov.format(octaveFmt);
    OUTPUT_TEXT(stream.str());
  }
}
