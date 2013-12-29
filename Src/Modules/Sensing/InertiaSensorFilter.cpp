/**
* @file InertiaSensorFilter.cpp
* Implementation of module InertiaSensorFilter.
* @author Colin Graf
*/

#include "InertiaSensorFilter.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include <algorithm>

MAKE_MODULE(InertiaSensorFilter, Sensing)

InertiaSensorFilter::InertiaSensorFilter() : lastTime(0)
{
}

void InertiaSensorFilter::calculateConstants()
{
  processCov.c[0].x = sqr(processNoise.x);
  processCov.c[1].y = sqr(processNoise.y);

  sensorCov.c[0].x = sqr(accNoise.x);
  sensorCov.c[1].y = sqr(accNoise.y);
  sensorCov.c[2].z = sqr(accNoise.z);
}

void InertiaSensorFilter::reset()
{
  x = State();
  cov = processCov;

  lastLeftFoot = lastRightFoot = Pose3D();
  lastTime = theFrameInfo.time - (unsigned int)(theFrameInfo.cycleTime * 1000.f);
}

void InertiaSensorFilter::update(OrientationData& orientationData)
{
  calculateConstants();

  DECLARE_PLOT("module:InertiaSensorFilter:expectedAccX");
  DECLARE_PLOT("module:InertiaSensorFilter:accX");
  DECLARE_PLOT("module:InertiaSensorFilter:expectedAccY");
  DECLARE_PLOT("module:InertiaSensorFilter:accY");
  DECLARE_PLOT("module:InertiaSensorFilter:expectedAccZ");
  DECLARE_PLOT("module:InertiaSensorFilter:accZ");

  DECLARE_PLOT("module:InertiaSensorFilter:orientationX");
  DECLARE_PLOT("module:InertiaSensorFilter:orientationY");
  DECLARE_PLOT("module:InertiaSensorFilter:velocityX");
  DECLARE_PLOT("module:InertiaSensorFilter:velocityY");
  DECLARE_PLOT("module:InertiaSensorFilter:velocityZ");

  // check whether the filter shall be reset
  if(!lastTime || theFrameInfo.time <= lastTime)
  {
    if(theFrameInfo.time == lastTime)
      return; // weird log file replaying?
    reset();
  }

  // get foot positions
  Pose3D leftFoot(theRobotModel.limbs[MassCalibration::footLeft]);
  Pose3D rightFoot(theRobotModel.limbs[MassCalibration::footRight]);
  leftFoot.translate(0.f, 0.f, -theRobotDimensions.heightLeg5Joint);
  rightFoot.translate(0.f, 0.f, -theRobotDimensions.heightLeg5Joint);
  const Pose3D leftFootInvert(leftFoot.invert());
  const Pose3D rightFootInvert(rightFoot.invert());

  // calculate rotation and position offset using the robot model (joint data)
  const Pose3D leftOffset(lastLeftFoot.translation.z != 0.f ? Pose3D(lastLeftFoot).conc(leftFootInvert) : Pose3D());
  const Pose3D rightOffset(lastRightFoot.translation.z != 0.f ? Pose3D(lastRightFoot).conc(rightFootInvert) : Pose3D());

  // detect the foot that is on ground
  bool useLeft = true;
  if(theMotionInfo.motion == MotionRequest::walk && theWalkingEngineOutput.speed.translation.x != 0)
    useLeft = theWalkingEngineOutput.speed.translation.x > 0 ?
              (leftOffset.translation.x > rightOffset.translation.x) :
              (leftOffset.translation.x < rightOffset.translation.x);
  else
  {
    Pose3D left(x.rotation);
    Pose3D right(x.rotation);
    left.conc(leftFoot);
    right.conc(rightFoot);
    useLeft = left.translation.z < right.translation.z;
  }

  // calculate velocity
  float timeScale = 1.f / theFrameInfo.cycleTime;
  Vector3<> velocity = useLeft ? leftOffset.translation : rightOffset.translation;
  velocity *= timeScale * 0.001f; // => m/s

  // update the filter
  timeScale = theFrameInfo.cycleTime;
  predict(theInertiaSensorData.gyro.x != InertiaSensorData::off ?
          RotationMatrix(Vector3<>(theInertiaSensorData.gyro.x * timeScale, theInertiaSensorData.gyro.y * timeScale, 0)) :
          (useLeft ? leftOffset.rotation :  rightOffset.rotation));

  // insert calculated rotation
  if(theInertiaSensorData.acc.x != InertiaSensorData::off)
    safeRawAngle = Vector2f(theSensorData.data[SensorData::angleX], theSensorData.data[SensorData::angleY]);
  if((theMotionInfo.motion == MotionRequest::walk || theMotionInfo.motion == MotionRequest::stand) &&
     std::abs(safeRawAngle.x) < calculatedAccLimit.x && std::abs(safeRawAngle.y) < calculatedAccLimit.y)
  {
    const RotationMatrix& usedRotation(useLeft ? leftFootInvert.rotation : rightFootInvert.rotation);
    Vector3f accGravOnly(usedRotation.c0.z, usedRotation.c1.z, usedRotation.c2.z);
    accGravOnly *= -9.80665f;
    readingUpdate(accGravOnly);
  }
  else // insert acceleration sensor values
  {
    if(theInertiaSensorData.acc.x != InertiaSensorData::off)
      readingUpdate(Vector3f(theInertiaSensorData.acc.x, theInertiaSensorData.acc.y, theInertiaSensorData.acc.z));
  }

  // fill the representation
  orientationData.rotation = x.rotation;
  orientationData.velocity = velocity;

  // this  keeps the rotation matrix orthogonal
  x.rotation = RotationMatrix(x.rotation.getAngleAxis());

  // store some data for the next iteration
  lastLeftFoot = leftFoot;
  lastRightFoot = rightFoot;
  lastTime = theFrameInfo.time;

  // plots
  PLOT("module:InertiaSensorFilter:orientationX", toDegrees(atan2(x.rotation.c1.z, x.rotation.c2.z)));
  PLOT("module:InertiaSensorFilter:orientationY", toDegrees(atan2(-x.rotation.c0.z, x.rotation.c2.z)));
  PLOT("module:InertiaSensorFilter:velocityX", orientationData.velocity.x);
  PLOT("module:InertiaSensorFilter:velocityY", orientationData.velocity.y);
  PLOT("module:InertiaSensorFilter:velocityZ", orientationData.velocity.z);
}

void InertiaSensorFilter::predict(const RotationMatrix& rotationOffset)
{
  generateSigmaPoints();

  // update sigma points
  for(int i = 0; i < 5; ++i)
    sigmaPoints[i].rotation *= rotationOffset;

  // get new mean and cov
  meanOfSigmaPoints();
  covOfSigmaPoints();

  // add process noise
  cov.c[0].x += processCov.c[0].x;
  cov.c[1].y += processCov.c[1].y;
}

void InertiaSensorFilter::readingModel(const State& sigmaPoint, Vector3f& reading)
{
  reading = Vector3f(sigmaPoint.rotation.c0.z, sigmaPoint.rotation.c1.z, sigmaPoint.rotation.c2.z);
  reading *= -9.80665f;
}

void InertiaSensorFilter::readingUpdate(const Vector3f& reading)
{
  generateSigmaPoints();

  for(int i = 0; i < 5; ++i)
    readingModel(sigmaPoints[i], sigmaReadings[i]);

  meanOfSigmaReadings();

  PLOT("module:InertiaSensorFilter:expectedAccX", readingMean.x);
  PLOT("module:InertiaSensorFilter:accX", reading.x);
  PLOT("module:InertiaSensorFilter:expectedAccY", readingMean.y);
  PLOT("module:InertiaSensorFilter:accY", reading.y);
  PLOT("module:InertiaSensorFilter:expectedAccZ", readingMean.z);
  PLOT("module:InertiaSensorFilter:accZ", reading.z);

  covOfSigmaReadingsAndSigmaPoints();
  covOfSigmaReadings();

  const Matrix2x3f& kalmanGain = readingsSigmaPointsCov.transpose() * (readingsCov + sensorCov).invert();
  const Vector2f& innovation = kalmanGain * (reading - readingMean);
  x += innovation;
  cov -= kalmanGain * readingsSigmaPointsCov;
}

void InertiaSensorFilter::generateSigmaPoints()
{
  cholOfCov();
  sigmaPoints[0] = x;
  sigmaPoints[1] = x + l.c[0];
  sigmaPoints[2] = x + l.c[1];
  sigmaPoints[3] = x + (-l.c[0]);
  sigmaPoints[4] = x + (-l.c[1]);
}

void InertiaSensorFilter::meanOfSigmaPoints()
{
  x = sigmaPoints[0];
  //for(int i = 0; i < 5; ++i) // ~= 0 .. inf
  for(int i = 0; i < 1; ++i)
  {
    Vector2f chunk((sigmaPoints[0] - x) +
                    ((sigmaPoints[1] - x) + (sigmaPoints[2] - x)) +
                    ((sigmaPoints[3] - x) + (sigmaPoints[4] - x)));
    chunk *= 1.f / 5.f;
    x += chunk;
  }
}

void InertiaSensorFilter::covOfSigmaPoints()
{
  cov = tensor(sigmaPoints[0] - x) +
        (tensor(sigmaPoints[1] - x) + tensor(sigmaPoints[2] - x)) +
        (tensor(sigmaPoints[3] - x) + tensor(sigmaPoints[4] - x));
  cov *= 0.5f;
}

InertiaSensorFilter::State InertiaSensorFilter::State::operator+(const Vector2f& value) const
{
  return State(*this) += value;
}

InertiaSensorFilter::State& InertiaSensorFilter::State::operator+=(const Vector2f& value)
{
  rotation *= RotationMatrix(rotation.invert() * Vector3<>(value.x, value.y, 0.f));
  return *this;
}

Vector2f InertiaSensorFilter::State::operator-(const State& other) const
{
  const Vector3<>& rotDiff(other.rotation * ((const RotationMatrix&)(other.rotation.invert() * rotation)).getAngleAxis());
  return Vector2f(rotDiff.x, rotDiff.y);
}

void InertiaSensorFilter::cholOfCov()
{
  // improved symmetry
  const float a11 = cov.c[0].x;
  const float a21 = (cov.c[0].y + cov.c[1].x) * 0.5f;

  const float a22 = cov.c[1].y;

  float& l11(l.c[0].x);
  float& l21(l.c[0].y);

  float& l22(l.c[1].y);

  //ASSERT(a11 >= 0.f);
  l11 = std::sqrt(std::max<>(a11, 0.f));
  if(l11 == 0.f) l11 = 0.0000000001f;
  l21 = a21 / l11;

  //ASSERT(a22 - l21 * l21 >= 0.f);
  l22 = std::sqrt(std::max<>(a22 - l21 * l21, 0.f));
  if(l22 == 0.f) l22 = 0.0000000001f;
}

void InertiaSensorFilter::meanOfSigmaReadings()
{
  readingMean = sigmaReadings[0];
  //for(int i = 0; i < 5; ++i) // ~= 0 .. inf
  for(int i = 0; i < 1; ++i)
  {
    Vector3f chunk((sigmaReadings[0] - readingMean) +
                    ((sigmaReadings[1] - readingMean) + (sigmaReadings[2] - readingMean)) +
                    ((sigmaReadings[3] - readingMean) + (sigmaReadings[4] - readingMean)));
    chunk *= 1.f / 5.f;
    readingMean += chunk;
  }
}

void InertiaSensorFilter::covOfSigmaReadingsAndSigmaPoints()
{
  readingsSigmaPointsCov = (
    (tensor(sigmaReadings[1] - readingMean, l.c[0]) + tensor(sigmaReadings[2] - readingMean, l.c[1])) +
    (tensor(sigmaReadings[3] - readingMean, -l.c[0]) + tensor(sigmaReadings[4] - readingMean, -l.c[1])));
  readingsSigmaPointsCov *= 0.5f;
}

void InertiaSensorFilter::covOfSigmaReadings()
{
  readingsCov = (tensor(sigmaReadings[0] - readingMean) +
                 (tensor(sigmaReadings[1] - readingMean) + tensor(sigmaReadings[2] - readingMean)) +
                 (tensor(sigmaReadings[3] - readingMean) + tensor(sigmaReadings[4] - readingMean)));
  readingsCov *= 0.5f;
}
