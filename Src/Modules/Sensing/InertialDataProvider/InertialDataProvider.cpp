#include "InertialDataProvider.h"
#include "Platform/SystemCall.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/Rotation.h"
#include "Tools/Math/Eigen.h"

#include <cmath>
#include <sstream>

MAKE_MODULE(InertialDataProvider, sensing);

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
  bool isAccBasedOnGroundContactPoints = false;
  bool isAccXMeasured = false;
  Angle supportFootRotationY;

  hadXAccUpdate = !(!hadXAccUpdate || theFootSupport.switched);

  if(!hadXAccUpdate &&
     theIMUCalibration.isCalibrated && theFootSoleRotationCalibration.isCalibrated &&
     theMotionInfo.executedPhase == MotionPhase::walk && // is robot walking?
     theMotionRequest.isWalking() &&
     theFootSupport.footPressure[theFootSupport.support > 0.f ? Legs::left : Legs::right].backwardPressure == theFrameInfo.time && theFootSupport.footPressure[theFootSupport.support > 0.f ? Legs::left : Legs::right].backwardPressure == theFrameInfo.time)
  {
    supportFootRotationY = (theFootSupport.support > 0.f ? theRobotModel.soleLeft.inverse().rotation : theRobotModel.soleRight.inverse().rotation).getYAngle() + theFootSoleRotationCalibration.yRotationOffset;
    isAccXMeasured = true;
  }

  // Replace measured acc with modelled acc
  if(theMotionInfo.executedPhase == MotionPhase::walk &&  // is robot walking?
     theMotionRequest.isWalking() &&
     theFootSupport.switched && // support foot switched
     theFootSupport.lastSwitch > minTimeForLastSupportSwitch && // robot had an actual support foot switch
     theFootSupport.lastSwitch < maxTimeForLastSupportSwitch &&
     // In case the robot is tilted far to the front or back (resulting from an unstable walk), we do not want to overwrite the accelerometer measurement
     theFrameInfo.getTimeSince(std::max(theFootSupport.footPressure[Legs::right].forwardPressure, theFootSupport.footPressure[Legs::left].forwardPressure)) < maxTimeSinceForwardAndBackwardPressure &&
     theFrameInfo.getTimeSince(std::max(theFootSupport.footPressure[Legs::left].backwardPressure, theFootSupport.footPressure[Legs::right].backwardPressure)) < maxTimeSinceForwardAndBackwardPressure)
  {
    Vector2f groundPointLeft = (theRobotModel.soleLeft * Vector3f(theRobotModel.soleLeft.rotation.col(2).x() >= 0.f ? theFootOffset.forward : -theFootOffset.backward, theRobotModel.soleLeft.rotation.col(2).y() >= 0.f ? theFootOffset.leftFoot.left : -theFootOffset.leftFoot.right, 0.f)).tail<2>();
    Vector2f groundPointRight = (theRobotModel.soleRight * Vector3f(theRobotModel.soleRight.rotation.col(2).x() >= 0.f ? theFootOffset.forward : -theFootOffset.backward, theRobotModel.soleRight.rotation.col(2).y() >= 0.f ? theFootOffset.rightFoot.left : -theFootOffset.rightFoot.right, 0.f)).tail<2>();

    if(std::abs(groundPointLeft.x() - groundPointRight.x()) > minGroundContactPointsDistance)
    {
      RotationMatrix diff;
      const Angle feetPlaneXRotation = -(groundPointLeft - groundPointRight).angle();
      if(std::abs(feetPlaneXRotation) < maxAllowedFeetPlaneXRotation)
      {
        diff.rotateX(feetPlaneXRotation);
        if(isAccXMeasured)
          diff.rotateY(supportFootRotationY);
        else
          diff.rotateY(inertialData.angle.y());
        Vector3f accGravOnly(diff.col(0).z(), diff.col(1).z(), diff.col(2).z());
        accGravOnly *= Constants::g_1000;
        inertialData.acc = accGravOnly;
        isAccBasedOnGroundContactPoints = true;
        hadAccelerometerMeasurement = false;
      }
    }
  }

  if(!isAccBasedOnGroundContactPoints && isAccXMeasured)
  {
    RotationMatrix usedRotation(theFootSupport.support > 0.f ? theRobotModel.soleLeft.inverse().rotation : theRobotModel.soleRight.inverse().rotation);
    usedRotation.rotateY(theFootSoleRotationCalibration.yRotationOffset);
    Vector3f accGravOnly(usedRotation.col(0).z(), usedRotation.col(1).z(), usedRotation.col(2).z()); // taken from B-Human Coderelease 2016
    accGravOnly *= Constants::g_1000;
    inertialData.acc.x() = accGravOnly.x();
    inertialData.acc.z() = accGravOnly.z();
    hadAccelerometerMeasurement = false;
  }

  processGyroscope(inertialData.gyro);

  if(!ignoreAccUpdate && (theInertialSensorData.acc != lastAccelerometerMeasurement || !hadAccelerometerMeasurement))
  {
    processAccelerometer(inertialData.acc, isAccBasedOnGroundContactPoints, isAccXMeasured);
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
  inertialData.angle << Rotation::AngleAxis::pack(AngleAxisf(inertialData.orientation2D)).head<2>().cast<Angle>(), 0_deg;

  const Vector3a angleAxisVec = Rotation::AngleAxis::pack(AngleAxisf(ukf.mean.orientation)).cast<Angle>();
  PLOT("module:InertialDataProvider:internalOrientation:x", angleAxisVec.x().toDegrees());
  PLOT("module:InertialDataProvider:internalOrientation:y", angleAxisVec.y().toDegrees());
  PLOT("module:InertialDataProvider:internalOrientation:z", angleAxisVec.z().toDegrees());
}

void InertialDataProvider::processAccelerometer(const Vector3f& acc, const bool isAccBasedOnGroundContactPoints, const bool isAccXMeasured)
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

  const Vector3f& useBaseAccDeviation = theMotionInfo.executedPhase == MotionPhase::walk ? accDeviationWhileWalking : (theMotionInfo.executedPhase == MotionPhase::stand ? accDeviationWhileStanding : accDeviation);
  Vector3f measurementNoise = useBaseAccDeviation + sqr(acc.norm() - gravity) * accDeviationFactor;
  if(isAccBasedOnGroundContactPoints)
    measurementNoise.y() = 1.f;
  if(isAccXMeasured)
  {
    measurementNoise.x() = 1.f;
    hadXAccUpdate = true;
  }
  if(isAccBasedOnGroundContactPoints && isAccXMeasured)
    measurementNoise.z() = 1.f;
  ukf.update<3>(acc, measurementModel, measurementNoise.cwiseAbs2().asDiagonal());
}

void InertialDataProvider::processGyroscope(const Vector3a& gyro)
{
  auto dynamicModel = [&](State& state)
  {
    state.orientation = state.orientation * Rotation::AngleAxis::unpack(gyro.cast<float>() * Constants::motionCycleTime);
  };

  const Vector3f dynamicNoise = gyroDeviation.cast<float>() * std::sqrt(1.f / Constants::motionCycleTime);
  ukf.predict(dynamicModel, dynamicNoise.cwiseAbs2().asDiagonal());
}
