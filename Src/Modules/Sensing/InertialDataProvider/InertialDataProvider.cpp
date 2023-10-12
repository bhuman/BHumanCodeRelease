#include "InertialDataProvider.h"
#include "Debugging/Modify.h"
#include "Debugging/Plot.h"
#include "Math/BHMath.h"
#include "Math/Eigen.h"
#include "Math/Rotation.h"
#include "Platform/SystemCall.h"

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
  DECLARE_PLOT("module:InertialDataProvider:dev1");
  DECLARE_PLOT("module:InertialDataProvider:dev10");
  DECLARE_PLOT("module:InertialDataProvider:dev50");

  bool ignoreAccUpdate = false;
  MODIFY("module:InertialDataProvider:ignoreAccUpdate", ignoreAccUpdate);

  if(SystemCall::getMode() == SystemCall::Mode::simulatedRobot || theGyroOffset.bodyDisconnect)
  {
    // This reinitializes the filter after a sudden orientation change in the simulation, e.g. when rotating the robot via commands
    if((lastRawAngle - theInertialSensorData.angle.head<2>()).norm() > pi / 3.f)
    {
      Quaternionf zRot;
      Rotation::splitOffZRotation(ukf.mean.orientation, zRot);
      ukf.init(State(zRot), Matrix3f::Zero());
    }
    //This reinitializes the filter after the real robot lost the connection to its sensory (V6 head flex problem)
    if(theGyroOffset.bodyDisconnect)
    {
      Quaternionf rot = Rotation::Euler::fromAngles(theInertialSensorData.angle.x(), theInertialSensorData.angle.y(), 0.f);
      ukf.init(State(rot), Matrix3f::Zero());
    }
    lastRawAngle = theInertialSensorData.angle.head<2>();
  }

  const Quaternionf rotation(theIMUCalibration.rotation);
  inertialData.acc = rotation * theInertialSensorData.acc;
  inertialData.gyro = (rotation * theInertialSensorData.gyro.cast<float>()).cast<Angle>();
  inertialData.gyro.x() *= theIMUCalibration.gyroFactor.x();
  inertialData.gyro.y() *= theIMUCalibration.gyroFactor.y();
  inertialData.gyro.z() *= theIMUCalibration.gyroFactor.z();

  processGyroscope(inertialData.gyro);

  if(!ignoreAccUpdate && (theInertialSensorData.acc != lastAccelerometerMeasurement || !hadAccelerometerMeasurement))
  {
    processAccelerometer(inertialData.acc);
    lastAccelerometerMeasurement = theInertialSensorData.acc;
    hadAccelerometerMeasurement = true;
  }
  else
    hadAccelerometerMeasurement = false;

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
}

void InertialDataProvider::processAccelerometer(const Vector3f& acc)
{
  if(theMotionInfo.executedPhase == MotionPhase::stand && theGroundContactState.contact)
  {
    accelerometerLengths.push_front(acc.norm());
    if(accelerometerLengths.full())
    {
      const float mean = accelerometerLengths.average();

      float meanSquaredDeviation = sqr(mean - accelerometerLengths[0]);
      float min = accelerometerLengths[0];
      float max = accelerometerLengths[0];
      for(size_t i = 1; i < accelerometerLengths.size(); ++i)
      {
        if(accelerometerLengths[i] < min)
          min = accelerometerLengths[i];
        else if(accelerometerLengths[i] > max)
          max = accelerometerLengths[i];
        meanSquaredDeviation += sqr(mean - accelerometerLengths[i]);
      }
      meanSquaredDeviation /= accelerometerLengths.size();

      if(meanSquaredDeviation < sqr(maxMeanSquaredDeviation) && (max - min) < 2.f * maxMeanSquaredDeviation)
        gravity = mean;
    }
  }
  else
    accelerometerLengths.clear();

  auto measurementModel = [&](const State& state)
  {
    const Vector3f gVec = Vector3f(0.f, 0.f, gravity);
    return state.orientation.inverse() * gVec;
  };

  Vector3f& useBaseAccVariance = theMotionInfo.executedPhase == MotionPhase::walk && theGroundContactState.contact ? accVarianceWhileWalking : (
                                                   theMotionInfo.executedPhase == MotionPhase::stand ? accVarianceWhileStanding : accVariance);
  Vector3f accLengthNoise = Vector3f(0.f, 0.f, 0.f);
  if(theMotionInfo.executedPhase != MotionPhase::walk && theMotionInfo.executedPhase != MotionPhase::stand)
    accLengthNoise = sqr(acc.norm() - gravity) * accVarianceFactor;
  Vector3f measurementNoise = useBaseAccVariance + accLengthNoise;
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
