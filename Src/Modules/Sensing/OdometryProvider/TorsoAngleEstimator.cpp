#include "TorsoAngleEstimator.h"
#include "Tools/Math/Rotation.h"

TorsoAngleEstimator::State TorsoAngleEstimator::State::operator+(const Vector2f& value) const
{
  return State(*this) += value;
}

TorsoAngleEstimator::State& TorsoAngleEstimator::State::operator+=(const Vector2f& value)
{
  const Vector3f angleAxis = rotation.inverse() * Vector3f(value.x(), value.y(), 0.f);
  rotation *= Rotation::AngleAxis::unpack(angleAxis);
  return *this;
}

Vector2f TorsoAngleEstimator::State::operator-(const State& other) const
{
  const Vector3f rotDiff = other.rotation * (other.rotation.inverse() * rotation).getPackedAngleAxis();
  return rotDiff.head<2>();
}

Vector2a TorsoAngleEstimator::update(float walkSpeedX)
{
  // get foot positions
  const Pose3f& leftFoot = theRobotModel.soleLeft;
  const Pose3f& rightFoot = theRobotModel.soleRight;
  const Pose3f leftFootInvert = leftFoot.inverse();
  const Pose3f rightFootInvert = rightFoot.inverse();

  // calculate rotation and position offset using the robot model (joint data)
  const Pose3f leftOffset(lastLeftFoot.translation.z() != 0.f ? lastLeftFoot* leftFootInvert : Pose3f());
  const Pose3f rightOffset(lastRightFoot.translation.z() != 0.f ? lastRightFoot* rightFootInvert : Pose3f());
  lastLeftFoot = leftFoot;
  lastRightFoot = rightFoot;

  // detect the foot that is on ground
  bool useLeft = true;
  if(theMotionRequest.motion == MotionRequest::walk && walkSpeedX != 0)
  {
    useLeft = walkSpeedX > 0 ?
              (leftOffset.translation.x() > rightOffset.translation.x()) :
              (leftOffset.translation.x() < rightOffset.translation.x());
  }
  else
  {
    const Pose3f left = Pose3f(mean.rotation) *= leftFoot;
    const Pose3f right = Pose3f(mean.rotation) *= rightFoot;
    useLeft = left.translation.z() < right.translation.z();
  }

  // update the filter
  Vector3f rotationOffset;
  rotationOffset << theInertialSensorData.gyro.head<2>().cast<float>(), 0.f;
  predict(rotationOffset * Constants::motionCycleTime);

  // insert calculated rotation
  const Vector2f safeRawAngle = theInertialSensorData.angle.head<2>().cast<float>();
  if((theMotionRequest.motion == MotionRequest::walk || theMotionRequest.motion == MotionRequest::stand ||
      (theMotionRequest.motion == MotionRequest::specialAction && theMotionRequest.specialActionRequest.specialAction == SpecialActionRequest::standHigh)) &&
     std::abs(safeRawAngle.x()) < calculatedAccLimit.x() && std::abs(safeRawAngle.y()) < calculatedAccLimit.y())
  {
    const RotationMatrix& usedRotation(useLeft ? leftFootInvert.rotation : rightFootInvert.rotation);
    Vector3f accGravOnly(usedRotation.col(0).z(), usedRotation.col(1).z(), usedRotation.col(2).z());
    accGravOnly *= Constants::g_1000;
    update(accGravOnly);
  }
  else // insert acceleration sensor values
    update(theInertialSensorData.acc);

  mean.rotation.normalize();
  return Vector2a(std::atan2(mean.rotation(2, 1), mean.rotation(2, 2)), std::atan2(-mean.rotation(2, 0), mean.rotation(2, 2)));
}

void TorsoAngleEstimator::reset()
{
  mean = State();
  cov = processNoise.array().square().matrix().asDiagonal();

  lastLeftFoot = lastRightFoot = Pose3f();
}

void TorsoAngleEstimator::dynamicModel(State& state, const Vector3f& rotationOffset)
{
  state.rotation *= Rotation::AngleAxis::unpack(rotationOffset).toRotationMatrix();
}

void TorsoAngleEstimator::predict(const Vector3f& rotationOffset)
{
  updateSigmaPoints();

  // update sigma points
  for(State& sigmaPoint : sigmaPoints)
    dynamicModel(sigmaPoint, rotationOffset);

  // get new mean and cov
  meanOfSigmaPoints();

  // covOfSigmaPoints
  cov = Matrix2f::Zero();
  for(State& sigmaPoint : sigmaPoints)
  {
    const Vector2f dist = sigmaPoint - mean;
    cov += dist * dist.transpose();
  }
  cov *= 0.5f;
  cov += processNoise.array().square().matrix().asDiagonal();
}

Vector3f TorsoAngleEstimator::readingModel(const State& sigmaPoint)
{
  return Vector3f(sigmaPoint.rotation.row(2)) * Constants::g_1000;
}

void TorsoAngleEstimator::update(const Vector3f& reading)
{
  updateSigmaPoints();

  std::array<Vector3f, 5> Z;
  for(size_t i = 0; i < sigmaPoints.size(); ++i)
    Z[i] = readingModel(sigmaPoints[i]);

  Vector3f z = Vector3f::Zero();
  for(Vector3f& Zi : Z)
    z += Zi;
  z /= static_cast<float>(sigmaPoints.size());

  Matrix3f sigmaz = Matrix3f::Zero();
  for(Vector3f& Zi : Z)
  {
    const Vector3f dist = Zi - z;
    sigmaz += dist * dist.transpose();
  }
  sigmaz *= 0.5f;
  sigmaz += accNoise.array().square().matrix().asDiagonal();

  Matrix2x3f simgaxz = Matrix2x3f::Zero();
  for(size_t i = 0; i < sigmaPoints.size(); ++i)
  {
    const State& Xi = sigmaPoints[i];
    const Vector3f& Zi = Z[i];
    simgaxz += (Xi - mean) * (Zi - z).transpose();
  }
  simgaxz *= 0.5f;

  const Matrix2x3f K = simgaxz * sigmaz.inverse();
  mean += K * (reading - z);
  cov -= K * sigmaz * K.transpose();
}

void TorsoAngleEstimator::updateSigmaPoints()
{
  Matrix2f l = cholOfCov();
  sigmaPoints[0] = mean;
  sigmaPoints[1] = mean + l.col(0);
  sigmaPoints[2] = mean + l.col(1);
  sigmaPoints[3] = mean + (-l.col(0));
  sigmaPoints[4] = mean + (-l.col(1));
}

void TorsoAngleEstimator::meanOfSigmaPoints()
{
  mean = sigmaPoints[0];
  State lastMean;
  unsigned iterations = 0;
  do
  {
    Vector2f sum = Vector2f::Zero();
    for(const State& sigmaPoint : sigmaPoints)
    {
      sum += sigmaPoint - mean;
    }
    lastMean = mean;
    mean += sum / static_cast<float>(sum.size()); // sum.size() == 2 * n + 1
    ++iterations;
  }
  while(!Approx::isZero((lastMean - mean).norm()) && iterations < 20);
}

Matrix2f TorsoAngleEstimator::cholOfCov()
{
  Matrix2f l = Matrix2f::Zero();
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

  return l;
}
