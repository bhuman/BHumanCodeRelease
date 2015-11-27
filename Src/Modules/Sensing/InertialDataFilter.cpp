/**
 * @file InertialDataFilter.cpp
 * Implementation of module InertialDataFilter.
 * @author Colin Graf
 */

#include "InertialDataFilter.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/Modify.h"
#include "Tools/Math/Angle.h"
#include "Tools/Math/Rotation.h"
#include "Tools/Settings.h"
#include <algorithm>

MAKE_MODULE(InertialDataFilter, sensing)

InertialDataFilter::State InertialDataFilter::State::operator+(const Vector2f& value) const
{
  return State(*this) += value;
}

InertialDataFilter::State& InertialDataFilter::State::operator+=(const Vector2f& value)
{
  Vector3f angleAxis = rotation.inverse() * Vector3f(value.x(), value.y(), 0.f);
  rotation *= Rotation::AngleAxis::unpack(angleAxis);
  return *this;
}

Vector2f InertialDataFilter::State::operator-(const State& other) const
{
  const Vector3f rotDiff = other.rotation * (other.rotation.inverse() * rotation).getPackedAngleAxis();
  return rotDiff.head<2>();
}

void InertialDataFilter::update(InertialData& inertialData)
{
  DECLARE_PLOT("module:InertialDataFilter:expectedAccX");
  DECLARE_PLOT("module:InertialDataFilter:accX");
  DECLARE_PLOT("module:InertialDataFilter:expectedAccY");
  DECLARE_PLOT("module:InertialDataFilter:accY");
  DECLARE_PLOT("module:InertialDataFilter:expectedAccZ");
  DECLARE_PLOT("module:InertialDataFilter:accZ");

  // check whether the filter shall be reset
  if(!lastTime || theFrameInfo.time <= lastTime)
  {
    if(theFrameInfo.time == lastTime)
      return; // weird log file replaying?
    reset();
  }

  if(theMotionInfo.motion == MotionRequest::specialAction && theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::playDead)
  {
    reset();
  }

  // get foot positions
  Pose3f leftFoot = theRobotModel.limbs[Limbs::footLeft];
  Pose3f rightFoot = theRobotModel.limbs[Limbs::footRight];
  leftFoot.translate(0.f, 0.f, -theRobotDimensions.footHeight);
  rightFoot.translate(0.f, 0.f, -theRobotDimensions.footHeight);
  const Pose3f leftFootInvert(leftFoot.inverse());
  const Pose3f rightFootInvert(rightFoot.inverse());

  // calculate rotation and position offset using the robot model (joint data)
  const Pose3f leftOffset(lastLeftFoot.translation.z() != 0.f ? Pose3f(lastLeftFoot).conc(leftFootInvert) : Pose3f());
  const Pose3f rightOffset(lastRightFoot.translation.z() != 0.f ? Pose3f(lastRightFoot).conc(rightFootInvert) : Pose3f());

  // detect the foot that is on ground
  bool useLeft = true;
  if(theMotionInfo.motion == MotionRequest::walk && theWalkingEngineOutput.speed.translation.x() != 0)
  {
    useLeft = theWalkingEngineOutput.speed.translation.x() > 0 ?
              (leftOffset.translation.x() > rightOffset.translation.x()) :
              (leftOffset.translation.x() < rightOffset.translation.x());
  }
  else
  {
    Pose3f left(mean.rotation);
    Pose3f right(mean.rotation);
    left.conc(leftFoot);
    right.conc(rightFoot);
    useLeft = left.translation.z() < right.translation.z();
  }

  // update the filter
  const float timeScale = theFrameInfo.cycleTime;
  predict(RotationMatrix::fromEulerAngles(theInertialSensorData.gyro.x() * timeScale,
                                          theInertialSensorData.gyro.y() * timeScale, 0));

  // insert calculated rotation
  safeRawAngle = theInertialSensorData.angle.head<2>().cast<float>();
  bool useFeet = true;
  MODIFY("module:InertialDataFilter:useFeet", useFeet);
  if(useFeet &&
     (theMotionInfo.motion == MotionRequest::walk || theMotionInfo.motion == MotionRequest::stand ||
      (theMotionInfo.motion == MotionRequest::specialAction && theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::standHigh)) &&
     std::abs(safeRawAngle.x()) < calculatedAccLimit.x() && std::abs(safeRawAngle.y()) < calculatedAccLimit.y())
  {
    const RotationMatrix& usedRotation(useLeft ? leftFootInvert.rotation : rightFootInvert.rotation);
    Vector3f accGravOnly(usedRotation.col(0).z(), usedRotation.col(1).z(), usedRotation.col(2).z());
    accGravOnly *= Constants::g_1000;
    readingUpdate(accGravOnly);
  }
  else // insert acceleration sensor values
    readingUpdate(theInertialSensorData.acc);

  // fill the representation
  inertialData.angle = Vector2a(std::atan2(mean.rotation.col(1).z(), mean.rotation.col(2).z()), std::atan2(-mean.rotation.col(0).z(), mean.rotation.col(2).z()));

  inertialData.acc = theInertialSensorData.acc;
  inertialData.gyro = theInertialSensorData.gyro;

  inertialData.orientation = Rotation::removeZRotation(Quaternionf(mean.rotation));

  // this  keeps the rotation matrix orthogonal
  mean.rotation.normalize();

  // store some data for the next iteration
  lastLeftFoot = leftFoot;
  lastRightFoot = rightFoot;
  lastTime = theFrameInfo.time;

  // plots
  Vector3f angleAxisVec = Rotation::AngleAxis::pack(AngleAxisf(inertialData.orientation));
  PLOT("module:InertialDataFilter:angleX", toDegrees(angleAxisVec.x()));
  PLOT("module:InertialDataFilter:angleY", toDegrees(angleAxisVec.y()));
  PLOT("module:InertialDataFilter:angleZ", toDegrees(angleAxisVec.z()));
  PLOT("module:InertialDataFilter:unfilteredAngleX", theInertialSensorData.angle.x().toDegrees());
  PLOT("module:InertialDataFilter:unfilteredAngleY", theInertialSensorData.angle.y().toDegrees());

  angleAxisVec = Rotation::AngleAxis::pack(AngleAxisf(mean.rotation));
  PLOT("module:InertialDataFilter:interlanAngleX", toDegrees(angleAxisVec.x()));
  PLOT("module:InertialDataFilter:interlanAngleY", toDegrees(angleAxisVec.y()));
  PLOT("module:InertialDataFilter:interlanAngleZ", toDegrees(angleAxisVec.z()));
}

void InertialDataFilter::reset()
{
  mean = State();
  cov = processNoise.array().square().matrix().asDiagonal();

  lastLeftFoot = lastRightFoot = Pose3f();
  lastTime = theFrameInfo.time - static_cast<unsigned int>(theFrameInfo.cycleTime * 1000.f);
}

void InertialDataFilter::predict(const RotationMatrix& rotationOffset)
{
  generateSigmaPoints();

  // update sigma points
  for(int i = 0; i < 5; ++i)
    sigmaPoints[i].rotation *= rotationOffset;

  // get new mean and cov
  meanOfSigmaPoints();
  covOfSigmaPoints();

  // add process noise
  cov += processNoise.array().square().matrix().asDiagonal();
}

void InertialDataFilter::readingModel(const State& sigmaPoint, Vector3f& reading)
{
  reading = Vector3f(sigmaPoint.rotation.col(0).z(), sigmaPoint.rotation.col(1).z(), sigmaPoint.rotation.col(2).z());
  reading *= Constants::g_1000;
}

void InertialDataFilter::readingUpdate(const Vector3f& reading)
{
  generateSigmaPoints();

  for(int i = 0; i < 5; ++i)
    readingModel(sigmaPoints[i], sigmaReadings[i]);

  meanOfSigmaReadings();

  PLOT("module:InertialDataFilter:expectedAccX", readingMean.x());
  PLOT("module:InertialDataFilter:accX", reading.x());
  PLOT("module:InertialDataFilter:expectedAccY", readingMean.y());
  PLOT("module:InertialDataFilter:accY", reading.y());
  PLOT("module:InertialDataFilter:expectedAccZ", readingMean.z());
  PLOT("module:InertialDataFilter:accZ", reading.z());

  const Matrix3x2f readingsSigmaPointsCov = covOfSigmaReadingsAndSigmaPoints();
  const Matrix3f readingsCov = covOfSigmaReadings();

  const Matrix3f sensorCov = accNoise.array().square().matrix().asDiagonal();
  const Matrix2x3f kalmanGain = readingsSigmaPointsCov.transpose() * (readingsCov + sensorCov).inverse();
  mean += kalmanGain * (reading - readingMean);
  cov -= kalmanGain * readingsSigmaPointsCov;
}

void InertialDataFilter::generateSigmaPoints()
{
  cholOfCov();
  sigmaPoints[0] = mean;
  sigmaPoints[1] = mean + l.col(0);
  sigmaPoints[2] = mean + l.col(1);
  sigmaPoints[3] = mean + (-l.col(0));
  sigmaPoints[4] = mean + (-l.col(1));
}

void InertialDataFilter::meanOfSigmaPoints()
{
  mean = sigmaPoints[0];
  //for(int i = 0; i < 5; ++i) // ~= 0 .. inf
  for(int i = 0; i < 1; ++i)
  {
    Vector2f chunk((sigmaPoints[0] - mean) +
                   (sigmaPoints[1] - mean) +
                   (sigmaPoints[2] - mean) +
                   (sigmaPoints[3] - mean) +
                   (sigmaPoints[4] - mean));
    chunk *= 1.f / 5.f;
    mean += chunk;
  }
}

void InertialDataFilter::covOfSigmaPoints()
{
  cov = tensor(sigmaPoints[0] - mean) +
    tensor(sigmaPoints[1] - mean) +
    tensor(sigmaPoints[2] - mean) +
    tensor(sigmaPoints[3] - mean) +
    tensor(sigmaPoints[4] - mean);
  cov *= 0.5f;
}

void InertialDataFilter::cholOfCov()
{
  // improved symmetry
  const float a11 = cov(0, 0);
  const float a21 = (cov(1, 0) + cov(0, 1)) * 0.5f;

  const float a22 = cov(1, 1);

  float& l11 = l(0, 0);
  float& l21 = l(1, 0);

  float& l22 = l(1, 1);

  //ASSERT(a11 >= 0.f);
  l11 = std::sqrt(std::max(a11, 0.f));
  if(l11 == 0.f)
    l11 = 0.0000000001f;
  l21 = a21 / l11;

  //ASSERT(a22 - l21 * l21 >= 0.f);
  l22 = std::sqrt(std::max(a22 - l21 * l21, 0.f));
  if(l22 == 0.f)
    l22 = 0.0000000001f;
}

void InertialDataFilter::meanOfSigmaReadings()
{
  readingMean = sigmaReadings[0];
  //for(int i = 0; i < 5; ++i) // ~= 0 .. inf
  for(int i = 0; i < 1; ++i)
  {
    Vector3f chunk((sigmaReadings[0] - readingMean) +
                   (sigmaReadings[1] - readingMean) +
                   (sigmaReadings[2] - readingMean) +
                   (sigmaReadings[3] - readingMean) +
                   (sigmaReadings[4] - readingMean));
    chunk *= 1.f / 5.f;
    readingMean += chunk;
  }
}

Matrix3x2f InertialDataFilter::covOfSigmaReadingsAndSigmaPoints()
{
  Matrix3x2f readingsSigmaPointsCov =
    tensor(sigmaReadings[1] - readingMean, l.col(0)) +
    tensor(sigmaReadings[2] - readingMean, l.col(1)) +
    tensor(sigmaReadings[3] - readingMean, -l.col(0)) +
    tensor(sigmaReadings[4] - readingMean, -l.col(1));
  readingsSigmaPointsCov *= 0.5f;
  return readingsSigmaPointsCov;
}

Matrix3f InertialDataFilter::covOfSigmaReadings()
{
  Matrix3f readingsCov = 
    tensor(Vector3f(sigmaReadings[0] - readingMean)) +
    tensor(Vector3f(sigmaReadings[1] - readingMean)) +
    tensor(Vector3f(sigmaReadings[2] - readingMean)) +
    tensor(Vector3f(sigmaReadings[3] - readingMean)) +
    tensor(Vector3f(sigmaReadings[4] - readingMean));
  readingsCov *= 0.5f;
  return readingsCov;
}
