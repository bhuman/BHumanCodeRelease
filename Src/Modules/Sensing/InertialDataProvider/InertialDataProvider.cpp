#include "InertialDataProvider.h"
#include "Platform/SystemCall.h"
#include "Debugging/DebugDrawings.h"
#include "Math/BHMath.h"
#include "Math/Rotation.h"
#include "Math/Eigen.h"

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
  DECLARE_PLOT("module:InertialDataProvider:dev1");
  DECLARE_PLOT("module:InertialDataProvider:dev10");
  DECLARE_PLOT("module:InertialDataProvider:dev50");

  bool ignoreAccUpdate = false;
  MODIFY("module:InertialDataProvider:ignoreAccUpdate", ignoreAccUpdate);

  if(soleCalibrationId != theFootSoleRotationCalibration.id)
  {
    soleCalibrationId = theFootSoleRotationCalibration.id;
    soleCalibration[Legs::left] = Quaternionf(Rotation::Euler::fromAngles(theFootSoleRotationCalibration.footCalibration[Legs::left].rotationOffset.x(),
                                              theFootSoleRotationCalibration.footCalibration[Legs::left].rotationOffset.y(),
                                              0.f)).normalized();
    soleCalibration[Legs::right] = Quaternionf(Rotation::Euler::fromAngles(theFootSoleRotationCalibration.footCalibration[Legs::right].rotationOffset.x(),
                                               theFootSoleRotationCalibration.footCalibration[Legs::right].rotationOffset.y(),
                                               0.f)).normalized();
  }

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
  float supportFootRotationY;

  const Legs::Leg leg = theFootSupport.support > 0.f ? Legs::left : Legs::right;
  Quaternionf soleRot;
  hadXAccUpdate = !(!hadXAccUpdate || theFootSupport.switched);
  if(!hadXAccUpdate &&
     theIMUCalibration.isCalibrated && theFootSoleRotationCalibration.footCalibration[leg].isCalibrated &&
     theMotionInfo.executedPhase == MotionPhase::walk && // is robot walking?
     theMotionRequest.isWalking() &&
     theFootSupport.lastSwitch > minTimeForLastSupportSwitch && // robot had an actual support foot switch
     theFootSupport.lastSwitch < maxTimeForLastSupportSwitch &&
     feetPressureRatioSagitalRangeXAcc.isInside(theFsrData.legInfo[leg].sagittalRatio) && // front and back FSRs measured pressure
     theFsrData.legInfo[leg].totals > minSumSolePressure) // the robot has enough weight on the support foot
  {
    soleRot = Quaternionf((leg == Legs::left ? theRobotModel.soleLeft : theRobotModel.soleRight).inverse().rotation).normalized();
    soleRot *= soleCalibration[leg];
    supportFootRotationY = Rotation::AngleAxis::pack(AngleAxisf(soleRot)).y();
    if(supportFootRotationY > 0.5_deg)
      isAccXMeasured = true;
  }

  // TODO clean up ...
  // Replace measured acc with modelled acc
  if(theMotionInfo.executedPhase == MotionPhase::walk &&   // is robot walking?
     theMotionRequest.isWalking() &&
     theFootSupport.switched && // support foot switched
     theFootSupport.lastSwitch > minTimeForLastSupportSwitch && // robot had an actual support foot switch
     theFootSupport.lastSwitch < maxTimeForLastSupportSwitch &&
     // In case the robot is tilted far to the front or back (resulting from an unstable walk), we do not want to overwrite the accelerometer measurement
     feetPressureRatioSagitalRangeYAcc.isInside(theFsrData.legInfo[leg].lateralRatio) &&
     theFsrData.legInfo[leg].totals > 0.4f &&
     theFrameInfo.getTimeSince(std::max(theFsrData.legInfo[Legs::right].forwardPressure, theFsrData.legInfo[Legs::left].forwardPressure)) < maxTimeSinceForwardAndBackwardPressure &&
     theFrameInfo.getTimeSince(std::max(theFsrData.legInfo[Legs::left].backwardPressure, theFsrData.legInfo[Legs::right].backwardPressure)) < maxTimeSinceForwardAndBackwardPressure &&
     theFrameInfo.getTimeSince(std::max(theFsrData.legInfo[Legs::left].hasPressure, theFsrData.legInfo[Legs::right].hasPressure)) < maxTimeSinceForwardAndBackwardPressure)
  {
    const Rangef& range = Rangef::ZeroOneRange();
    const float leftFootRotY = theRobotModel.soleLeft.rotation.getYAngle();
    const float leftFootRotX = theRobotModel.soleLeft.rotation.getXAngle();
    const float rightFootRotY = theRobotModel.soleRight.rotation.getYAngle();
    const float rightFootRotX = theRobotModel.soleRight.rotation.getXAngle();

    const bool breakCalc = std::abs(leftFootRotY - rightFootRotY) > 0.02f || std::max(std::abs(leftFootRotY), std::abs(rightFootRotY)) > 0.03f;

    const float leftYRatio = range.limit((leftFootRotY + soleRotationInterpolation) / (2.f * +soleRotationInterpolation));
    const float leftXRatio = range.limit((leftFootRotX + soleRotationInterpolation) / (2.f * +soleRotationInterpolation));
    const float rightYRatio = range.limit((rightFootRotY + soleRotationInterpolation) / (2.f * +soleRotationInterpolation));
    const float rightXRatio = range.limit((rightFootRotX + soleRotationInterpolation) / (2.f * +soleRotationInterpolation));

    const float forwardLength = theFootOffset.forward + theFootOffset.backward;
    const float sideLengthLeft = theFootOffset.leftFoot.right + theFootOffset.leftFoot.left;
    const float sideLengthRight = theFootOffset.rightFoot.right + theFootOffset.rightFoot.left;

    Vector2f groundPointLeft = ((theRobotModel.soleLeft * soleCalibration[Legs::left]) *
                                Vector3f(-theFootOffset.backward + forwardLength * leftYRatio,
                                         theFootOffset.leftFoot.left - sideLengthLeft * leftXRatio,
                                         0.f)).tail<2>();
    Vector2f groundPointRight = ((theRobotModel.soleRight * soleCalibration[Legs::right]) *
                                 Vector3f(-theFootOffset.backward + forwardLength * rightYRatio,
                                          theFootOffset.rightFoot.left - sideLengthRight * rightXRatio,
                                          0.f)).tail<2>();

    if(!breakCalc && std::abs(groundPointLeft.x() - groundPointRight.x()) > minGroundContactPointsDistance &&
       theRobotModel.soleLeft.translation.y() - theRobotModel.soleRight.translation.y() < 150.f)
    {
      const Angle feetPlaneXRotation = -(groundPointLeft - groundPointRight).angle();
      inertialData.acc.y() = std::tan(-feetPlaneXRotation) * inertialData.acc.z();
      isAccBasedOnGroundContactPoints = true;
      hadAccelerometerMeasurement = false;
    }
  }

  if(isAccXMeasured)
  {
    inertialData.acc.x() = std::tan(-supportFootRotationY) * inertialData.acc.z();
    hadAccelerometerMeasurement = false;

    if(isAccBasedOnGroundContactPoints)
    {
      inertialData.acc.normalize(gravity);
    }
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
  if(useIMUAnglesInCalibration && theMotionInfo.executedPhase == MotionPhase::stand && theCalibrationRequest.targetState != CameraCalibrationStatus::State::idle)
    inertialData.angle << theInertialSensorData.angle;
  else
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

  Vector3f& useBaseAccDeviation = theMotionInfo.executedPhase == MotionPhase::walk && theGroundContactState.contact ? accDeviationWhileWalking : (theMotionInfo.executedPhase == MotionPhase::stand ? accDeviationWhileStanding : accDeviation);
  Vector3f measurementNoise = useBaseAccDeviation + sqr(acc.norm() - gravity) * accDeviationFactor;
  if(isAccBasedOnGroundContactPoints)
    measurementNoise.y() = accSoleDeviation;
  if(isAccXMeasured)
    measurementNoise.x() = accSoleDeviation;
  if(isAccBasedOnGroundContactPoints && isAccXMeasured)
    measurementNoise.z() = accSoleDeviation;
  if(isAccXMeasured)
  {
    hadXAccUpdate = true;
  }

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
