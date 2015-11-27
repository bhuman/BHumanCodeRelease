/**
* @file UKFSample.cpp
*
* Implementation of Unscented Kalman Filter for robot pose estimation
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
* @author Colin Graf
*/

#include "UKFSample.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Probabilistics.h"

using namespace std;

void UKFSample::init(const Pose2f& pose, const SelfLocatorBase::Parameters& parameters, int id)
{
  this->id = id;
  mean << pose.translation.x(), pose.translation.y(), pose.rotation;
  cov = Matrix3f::Zero();
  cov(0, 0) = sqr(parameters.defaultPoseDeviation.translation.x());
  cov(1, 1) = sqr(parameters.defaultPoseDeviation.translation.y());
  cov(2, 2) = sqr(parameters.defaultPoseDeviation.rotation);
  // Buffer is always initially filled with average value
  for(size_t i = 0; i < validityBuffer.capacity(); ++i)
    validityBuffer.push_front(0.5f);
  validity = 0.5f;
  for(int i = 0; i < 7; ++i)
    sigmaPoints[i] = Vector3f::Zero();
}

void UKFSample::motionUpdate(const Pose2f& odometryOffset, const SelfLocatorBase::Parameters& parameters)
{
  generateSigmaPoints();
  performOdometryUpdate(odometryOffset, parameters);
}

Pose2f UKFSample::getPose() const
{
  return Pose2f(mean.z(), mean.head<2>());
}

void UKFSample::mirror()
{
  const Pose2f newPose = Pose2f(pi) + getPose();
  mean.x() = newPose.translation.x();
  mean.y() = newPose.translation.y();
  mean.z() = newPose.rotation;
}

float UKFSample::computeValidity(const FieldDimensions& fieldDimensions)
{
  const Vector2f pos = getPose().translation;
  if(fieldDimensions.isInsideCarpet(pos))
    return validity = validityBuffer.average();
  else
    return validity = 0.f;
}

void UKFSample::invalidate()
{
  for(size_t i = 0; i < validityBuffer.capacity(); ++i)
    validityBuffer.push_front(0.f);
  validity = 0.f;
}

float UKFSample::getVarianceWeighting() const
{
  return std::max(cov(0, 0), cov(1, 1)) * cov(2, 2);
}

void UKFSample::twist()
{
  mean.z() += pi;
  mean.z() = Angle::normalize(mean.z());
}

void UKFSample::generateSigmaPoints()
{
  // Cholesky decomposition
  const float a11 = cov(0, 0);
  const float a21 = (cov(1, 0) + cov(0, 1)) * 0.5f;
  const float a31 = (cov(2, 0) + cov(0, 2)) * 0.5f;
  const float a22 = cov(1, 1);
  const float a32 = (cov(2, 1) + cov(1, 2)) * 0.5f;
  const float a33 = cov(2, 2);

  float& l11(l(0, 0));
  float& l21(l(1, 0));
  float& l31(l(2, 0));
  float& l22(l(1, 1));
  float& l32(l(2, 1));
  float& l33(l(2, 2));

  //ASSERT(a11 >= 0.f);
  l11 = sqrt(std::max<>(a11, 0.f));
  if(l11 == 0.f) l11 = 0.0000000001f;
  l21 = a21 / l11;
  l31 = a31 / l11;
  //ASSERT(a22 - l21 * l21 >= 0.f);
  l22 = sqrt(std::max<>(a22 - l21 * l21, 0.f));
  if(l22 == 0.f) l22 = 0.0000000001f;
  l32 = (a32 - l31 * l21) / l22;
  //ASSERT(a33 - l31 * l31 - l32 * l32 >= 0.f);
  l33 = sqrt(std::max<>(a33 - l31 * l31 - l32 * l32, 0.f));

  sigmaPoints[0] = mean;
  sigmaPoints[1] = mean + l.col(0);
  sigmaPoints[2] = mean - l.col(0);
  sigmaPoints[3] = mean + l.col(1);
  sigmaPoints[4] = mean - l.col(1);
  sigmaPoints[5] = mean + l.col(2);
  sigmaPoints[6] = mean - l.col(2);
}

void UKFSample::performOdometryUpdate(const Pose2f& odometryOffset, const SelfLocatorBase::Parameters& parameters)
{
  // addOdometryToSigmaPoints
  for(int i = 0; i < 7; ++i)
  {
    Vector2f odo(odometryOffset.translation);
    odo.rotate(sigmaPoints[i].z());
    sigmaPoints[i] += Vector3f(odo.x(), odo.y(), odometryOffset.rotation);
  }

  // computeMeanOfSigmaPoints
  mean = sigmaPoints[0];
  for(int i = 1; i < 7; ++i)
    mean += sigmaPoints[i];
  mean *= 1.f / 7.f;

  // computeCovOfSigmaPoints
  Vector3f d = sigmaPoints[0] - mean;
  cov << d * d.x(), d * d.y(), d * d.z();
  for(int i = 1; i < 7; ++i)
  {
    Vector3f d = sigmaPoints[i] - mean;
    cov += (Matrix3f() << d * d.x(), d * d.y(), d * d.z()).finished();
  }
  cov *= 0.5f;

  // addProcessNoise
  cov(0, 0) += sqr(parameters.filterProcessDeviation.translation.x());
  cov(1, 1) += sqr(parameters.filterProcessDeviation.translation.y());
  cov(2, 2) += sqr(parameters.filterProcessDeviation.rotation);

  Vector2f odo(odometryOffset.translation);
  odo.rotate(mean.z());
  cov(0, 0) += sqr(odo.x() * parameters.odometryDeviation.translation.x());
  cov(1, 1) += sqr(odo.y() * parameters.odometryDeviation.translation.y());
  cov(2, 2) += sqr(odometryOffset.rotation * parameters.odometryDeviation.rotation);
  cov(2, 2) += sqr(odo.x() * parameters.odometryRotationDeviation.x());
  cov(2, 2) += sqr(odo.y() * parameters.odometryRotationDeviation.y());

  mean.z() = Angle::normalize(mean.z());
}

void UKFSample::updateByGoalPercept(const GoalPercept& goalPercept, const FieldModel& fieldModel, const SelfLocatorBase::Parameters& parameters,
                                    const MotionInfo& motionInfo, const CameraMatrix& cameraMatrix)
{
  const Pose2f robotPose = getPose();
  for(vector<GoalPost>::const_iterator goalPost = goalPercept.goalPosts.begin(); goalPost != goalPercept.goalPosts.end(); goalPost++)
  {
    // Data association:
    Vector2f associatedPost;
    bool postFound(false);
    if(goalPost->position == GoalPost::IS_UNKNOWN)
      postFound = fieldModel.getAssociatedUnknownGoalPost(robotPose, goalPost->positionOnField, associatedPost);
    else // goalPost->IS_LEFT || goalPost->IS_RIGHT
      postFound = fieldModel.getAssociatedKnownGoalPost(robotPose, goalPost->positionOnField, goalPost->position == GoalPost::IS_LEFT, associatedPost);

    // Filter update:
    if(postFound)
    {
      Matrix2f cov = getCovOfPointInWorld(goalPost->positionOnField, 0.f, motionInfo, cameraMatrix, parameters);
      landmarkSensorUpdate(associatedPost, Vector2f(goalPost->positionOnField.x(), goalPost->positionOnField.y()), cov);
      validityBuffer.push_front(1.f);
    }
    else
    {
      validityBuffer.push_front(0.f);
    }
  }
}

void UKFSample::updateByPenaltyMarkPercept(const PenaltyMarkPercept& penaltyMarkPercept, const FieldModel& fieldModel, const SelfLocatorBase::Parameters& parameters,
                                           const MotionInfo& motionInfo, const CameraMatrix& cameraMatrix)
{
  const Pose2f robotPose = getPose();
  Vector2f associatedPenaltyMark;
  if(fieldModel.getAssociatedPenaltyMark(robotPose, penaltyMarkPercept.positionOnField, associatedPenaltyMark))
  {
    Matrix2f cov = getCovOfPointInWorld(penaltyMarkPercept.positionOnField, 0.f, motionInfo, cameraMatrix, parameters);
    landmarkSensorUpdate(associatedPenaltyMark, penaltyMarkPercept.positionOnField, cov);
  }
}

void UKFSample::updateByLinePercept(const LinePercept& linePercept, const FieldModel& fieldModel, const SelfLocatorBase::Parameters& parameters,
                                    const FieldDimensions& fieldDimensions, const MotionInfo& motionInfo, const CameraMatrix& cameraMatrix)
{
  // Check for center circle:
  bool centerCircleCanBeUsed = false;
  if(linePercept.circle.found)
  {
    const Pose2f robotPose = getPose();
    const Vector2f circleInWorld = robotPose * linePercept.circle.pos;
    centerCircleCanBeUsed = circleInWorld.norm() <= parameters.centerCircleAssociationDistance;
    validityBuffer.push_front(centerCircleCanBeUsed ? 1.f : 0.f);
  }
  // Iterate over all observed lines:
  for(std::vector<LinePercept::Line>::const_iterator it = linePercept.lines.begin(); it != linePercept.lines.end(); ++it)
  {
    const Pose2f robotPose = getPose();
    int index = fieldModel.getIndexOfAssociatedLine(robotPose, it->first, it->last);
    if(index != -1)
    {
      const FieldModel::FieldLine& fieldLine = fieldModel.fieldLines[index];
      Vector2f center = (it->first + it->last) * 0.5f;
      Vector2f dir = it->last - it->first;
      dir.normalize();
      Matrix2f cov = getCovOfPointInWorld(center, 0.f, motionInfo, cameraMatrix, parameters);
      Vector2f orthogonalProjectiona = getOrthogonalProjection(it->first, dir, Vector2f::Zero());
      float measuredAngle = -atan2(orthogonalProjectiona.y(), orthogonalProjectiona.x());
      measuredAngle = Angle::normalize(measuredAngle + (fieldLine.vertical ? pi_2 : 0));
      float possibleAngle2 = Angle::normalize(measuredAngle - pi);
      if(abs(Angle::normalize(possibleAngle2 - robotPose.rotation)) < abs(Angle::normalize(measuredAngle - robotPose.rotation)))
        measuredAngle = possibleAngle2;
      float c = cos(measuredAngle), s = sin(measuredAngle);
      Matrix2f angleRotationMatrix = (Matrix2f() << c, -s, s, c).finished();
      Vector2f orthogonalProjection = angleRotationMatrix * Vector2f(orthogonalProjectiona.x(), orthogonalProjectiona.y());

      // Check for special object (center circle + center line):
      if(it->midLine && centerCircleCanBeUsed)
      {
        Matrix2f circleCov = getCovOfCircle(linePercept.circle.pos, fieldDimensions.centerCircleRadius, motionInfo, cameraMatrix, parameters);
        Matrix2f covXR = angleRotationMatrix * cov * angleRotationMatrix.transpose();
        Matrix2f covY = angleRotationMatrix * circleCov * angleRotationMatrix.transpose();
        const float measuredX = -orthogonalProjection.x();
        const float xVariance = covXR(0, 0);
        const float angleVariance = sqr(atan(sqrt(4.f * xVariance / (it->first - it->last).squaredNorm())));
        const float measueredY = -(Pose2f(measuredAngle, measuredX, 0.f) * linePercept.circle.pos).y();

        poseSensorUpdate(Vector3f(measuredX, measueredY, measuredAngle), (Matrix3f() << xVariance, 0.f, 0.f, 0.f, covY(1, 1), 0.f, 0.f, 0.f, angleVariance).finished());
        centerCircleCanBeUsed = false;
      }
      // Integrate "normal" lines:
      else
      {
        cov = angleRotationMatrix * cov * angleRotationMatrix.transpose();
        if(fieldLine.vertical)
        {
          const float measuredY = fieldLine.start.y() - orthogonalProjection.y();
          const float yVariance = cov(1, 1);
          const float angleVariance = sqr(atan(sqrt(4.f * yVariance / (it->first - it->last).squaredNorm())));
          Matrix2f cov;
          cov << yVariance, 0.f, 0.f, angleVariance;
          lineSensorUpdate(true, Vector2f(measuredY, measuredAngle), cov);
        }
        else
        {
          const float measuredX = fieldLine.start.x() - orthogonalProjection.x();
          const float xVariance = cov(0, 0);
          const float angleVariance = sqr(atan(sqrt(4.f * xVariance / (it->first - it->last).squaredNorm())));
          Matrix2f cov;
          cov << xVariance, 0.f, 0.f, angleVariance;
          lineSensorUpdate(false, Vector2f(measuredX, measuredAngle), cov);
        }
      }
    }
  }
  // There is a valid center circle that has not been fused with the center line:
  if(centerCircleCanBeUsed)
  {
    Matrix2f cov = getCovOfCircle(linePercept.circle.pos, fieldDimensions.centerCircleRadius, motionInfo, cameraMatrix, parameters);
    landmarkSensorUpdate(Vector2f::Zero(), Vector2f(linePercept.circle.pos.x(), linePercept.circle.pos.y()), cov);
  }
  // If there has been a circle, we can stop now:
  if(linePercept.circle.found)
    return;
  // Otherwise, we try to integrate the line intersections:
  for(std::vector<LinePercept::Intersection>::const_iterator intersection = linePercept.intersections.begin();
      intersection != linePercept.intersections.end(); ++intersection)
  {
    const Pose2f robotPose = getPose();
    Vector2f associatedCorner;
    if(fieldModel.getAssociatedCorner(robotPose, *intersection, associatedCorner))
    {
      Matrix2f cov = getCovOfPointInWorld(intersection->pos, 0.f, motionInfo, cameraMatrix, parameters);
      landmarkSensorUpdate(associatedCorner, Vector2f(intersection->pos.x(), intersection->pos.y()), cov);
    }
  }
}

Matrix2f UKFSample::getCovOfPointInWorld(const Vector2f& pointInWorld2, float pointZInWorld, const MotionInfo& motionInfo,
    const CameraMatrix& cameraMatrix, const SelfLocatorBase::Parameters& parameters) const
{
  Vector3f unscaledVectorToPoint = cameraMatrix.inverse() * Vector3f(pointInWorld2.x(), pointInWorld2.y(), pointZInWorld);
  const Vector3f unscaledWorld = cameraMatrix.rotation * unscaledVectorToPoint;
  const float h = cameraMatrix.translation.z() - pointZInWorld;
  const float scale = h / -unscaledWorld.z();
  Vector2f pointInWorld(unscaledWorld.x() * scale, unscaledWorld.y() * scale);
  const float distance = pointInWorld.norm();
  Matrix2f rot;
  Vector2f cossin = pointInWorld * (1.f / distance);
  if(distance == 0.f)
    rot = Matrix2f::Identity();
  else
    rot << cossin.x(), -cossin.y(), cossin.y(), cossin.x();
  const Vector2f& robotRotationDeviation = motionInfo.motion == MotionRequest::stand ||
      (motionInfo.motion == MotionRequest::specialAction && motionInfo.specialActionRequest.specialAction == SpecialActionRequest::standHigh)
      ? parameters.robotRotationDeviationInStand : parameters.robotRotationDeviation;
  Matrix2f cov;
  cov << sqr(h / tan((distance == 0.f ? pi_2 : atan(h / distance)) - robotRotationDeviation.x()) - distance), 0.f,
      0.f, sqr(tan(robotRotationDeviation.y()) * distance);
  return rot * cov * rot.transpose();
}

Matrix2f UKFSample::getCovOfCircle(const Vector2f& circlePos, float centerCircleRadius, const MotionInfo& motionInfo,
                                     const CameraMatrix& cameraMatrix, const SelfLocatorBase::Parameters& parameters) const
{
  float circleDistance = circlePos.norm();
  Vector2f increasedCirclePos = circlePos;
  if(circleDistance < centerCircleRadius * 2.f)
  {
    if(circleDistance < 10.f)
      increasedCirclePos = Vector2f(centerCircleRadius * 2, 0.f);
    else
      increasedCirclePos *= centerCircleRadius * 2.f / circleDistance;
  }
  return getCovOfPointInWorld(increasedCirclePos, 0.f, motionInfo, cameraMatrix, parameters);
}

void UKFSample::landmarkSensorUpdate(const Vector2f& landmarkPosition, const Vector2f& reading, const Matrix2f& readingCov)
{
  generateSigmaPoints();

  // computeLandmarkReadings
  Vector2f landmarkReadings[7];
  for(int i = 0; i < 7; ++i)
  {
    Pose2f pose(sigmaPoints[i].z(), sigmaPoints[i].head<2>());
    Vector2f landmarkPosRel = pose.invert() * landmarkPosition; // TODO: optimize this
    landmarkReadings[i] = Vector2f(landmarkPosRel.x(), landmarkPosRel.y());
  }

  // computeMeanOfLandmarkReadings
  Vector2f landmarkReadingMean = landmarkReadings[0];
  for(int i = 1; i < 7; ++i)
    landmarkReadingMean += landmarkReadings[i];
  landmarkReadingMean *= 1.f / 7.f;

  // computeCovOfLandmarkReadingsAndSigmaPoints
  Matrix2x3f landmarkReadingAndMeanCov = Matrix2x3f::Zero();
  for(int i = 0; i < 3; ++i)
  {
    Vector2f d1 = landmarkReadings[i * 2 + 1] - landmarkReadingMean;
    landmarkReadingAndMeanCov += (Matrix2x3f() << d1 * l(0, i), d1 * l(1, i), d1 * l(2, i)).finished();
    Vector2f d2 = landmarkReadings[i * 2 + 2] - landmarkReadingMean;
    landmarkReadingAndMeanCov += (Matrix2x3f() << d2 * -l(0, i), d2 * -l(1, i), d2 * -l(2, i)).finished();
  }
  landmarkReadingAndMeanCov *= 0.5f;

  // computeCovOfLandmarkReadingsReadings
  Vector2f d = landmarkReadings[0] - landmarkReadingMean;
  Matrix2f landmarkReadingCov = Matrix2f::Zero();
  landmarkReadingCov << d * d.x(), d * d.y();
  for(int i = 1; i < 7; ++i)
  {
    Vector2f d = landmarkReadings[i] - landmarkReadingMean;
    landmarkReadingCov += (Matrix2f() << d * d.x(), d * d.y()).finished();
  }
  landmarkReadingCov *= 0.5f;

  const Matrix3x2f kalmanGain = landmarkReadingAndMeanCov.transpose() * (landmarkReadingCov + readingCov).inverse();
  Vector2f innovation = reading - landmarkReadingMean;
  const Vector3f correction = kalmanGain * innovation;
  mean += correction;
  mean.z() = Angle::normalize(mean.z());
  cov -= kalmanGain * landmarkReadingAndMeanCov;
}

void UKFSample::lineSensorUpdate(bool vertical, const Vector2f& reading, const Matrix2f& readingCov)
{
  generateSigmaPoints();

  // computeLineReadings
  Vector2f lineReadings[7];
  if(vertical)
    for(int i = 0; i < 7; ++i)
      lineReadings[i] = Vector2f(sigmaPoints[i].y(), sigmaPoints[i].z());
  else
    for(int i = 0; i < 7; ++i)
      lineReadings[i] = Vector2f(sigmaPoints[i].x(), sigmaPoints[i].z());

  // computeMeanOfLineReadings
  Vector2f lineReadingMean = lineReadings[0];
  for(int i = 1; i < 7; ++i)
    lineReadingMean += lineReadings[i];
  lineReadingMean *= 1.f / 7.f;

  // computeCovOfLineReadingsAndSigmaPoints
  Matrix2x3f lineReadingAndMeanCov = Matrix2x3f::Zero();
  for(int i = 0; i < 3; ++i)
  {
    Vector2f d1 = lineReadings[i * 2 + 1] - lineReadingMean;
    lineReadingAndMeanCov += (Matrix2x3f() << d1 * l(0, i), d1 * l(1, i), d1 * l(2, i)).finished();
    Vector2f d2 = lineReadings[i * 2 + 2] - lineReadingMean;
    lineReadingAndMeanCov += (Matrix2x3f() << d2 * -l(0, i), d2 * -l(1, i), d2 * -l(2, i)).finished();
  }
  lineReadingAndMeanCov *= 0.5f;

  // computeCovOfLineReadingsReadings
  Vector2f d = lineReadings[0] - lineReadingMean;
  Matrix2f lineReadingCov = (Matrix2f() << d * d.x(), d * d.y()).finished();
  for(int i = 1; i < 7; ++i)
  {
    Vector2f d = lineReadings[i] - lineReadingMean;
    lineReadingCov += (Matrix2f() << d * d.x(), d * d.y()).finished();
  }
  lineReadingCov *= 0.5f;

  lineReadingMean.y() = Angle::normalize(lineReadingMean.y());
  const Matrix3x2f kalmanGain = lineReadingAndMeanCov.transpose() * (lineReadingCov + readingCov).inverse();
  Vector2f innovation = reading - lineReadingMean;
  innovation.y() = Angle::normalize(innovation.y());
  const Vector3f correction = kalmanGain * innovation;
  mean += correction;
  mean.z() = Angle::normalize(mean.z());
  cov -= kalmanGain * lineReadingAndMeanCov;
}

void UKFSample::poseSensorUpdate(const Vector3f& reading, const Matrix3f& readingCov)
{
  generateSigmaPoints();

  // computePoseReadings
  Vector3f poseReadings[7];
  for(int i = 0; i < 7; ++i)
    poseReadings[i] = sigmaPoints[i];

  // computeMeanOfPoseReadings
  Vector3f poseReadingMean = poseReadings[0];
  for(int i = 1; i < 7; ++i)
    poseReadingMean += poseReadings[i];
  poseReadingMean *= 1.f / 7.f;

  // computeCovOfPoseReadingsAndSigmaPoints
  Matrix3f poseReadingAndMeanCov = Matrix3f::Zero();
  for(int i = 0; i < 3; ++i)
  {
    Vector3f d1 = poseReadings[i * 2 + 1] - poseReadingMean;
    poseReadingAndMeanCov += (Matrix3f() << d1 * l(0, i), d1 * l(1, i), d1 * l(2, i)).finished();
    Vector3f d2 = poseReadings[i * 2 + 2] - poseReadingMean;
    poseReadingAndMeanCov += (Matrix3f() << d2 * -l(0, i), d2 * -l(1, i), d2 * -l(2, i)).finished();
  }
  poseReadingAndMeanCov *= 0.5f;

  // computeCovOfPoseReadingsReadings
  Matrix3f poseReadingCov;
  Vector3f d = poseReadings[0] - poseReadingMean;
  poseReadingCov << d * d.x(), d * d.y(), d * d.z();
  for(int i = 1; i < 7; ++i)
  {
    Vector3f d = poseReadings[i] - poseReadingMean;
    poseReadingCov += (Matrix3f() << d * d.x(), d * d.y(), d * d.z()).finished();
  }
  poseReadingCov *= 0.5f;

  poseReadingMean.z() = Angle::normalize(poseReadingMean.z());
  const Matrix3f kalmanGain = poseReadingAndMeanCov.transpose() * (poseReadingCov + readingCov).inverse();
  Vector3f innovation = reading - poseReadingMean;
  innovation.z() = Angle::normalize(innovation.z());
  const Vector3f correction = kalmanGain * innovation;
  mean += correction;
  mean.z() = Angle::normalize(mean.z());
  cov -= kalmanGain * poseReadingAndMeanCov;
}

void UKFSample::computeWeightingBasedOnValidity(const FieldDimensions& fieldDimensions, const SelfLocatorBase::Parameters& parameters)
{
  float factor = computeValidity(fieldDimensions);
  weighting = parameters.baseValidityWeighting + (1.f - parameters.baseValidityWeighting) * factor;
}

float UKFSample::computeAngleWeighting(float measuredAngle, const Vector2f& modelPosition,
    const Pose2f& robotPose, float standardDeviation) const
{
  const float modelAngle = Geometry::angleTo(robotPose, modelPosition);
  return gaussianProbability(abs(modelAngle - measuredAngle), standardDeviation);
}

float UKFSample::computeDistanceWeighting(float measuredDistanceAsAngle, const Vector2f& modelPosition,
    const Pose2f& robotPose, float cameraZ, float standardDeviation) const
{
  const float modelDistance = (robotPose.translation - modelPosition).norm();
  const float modelDistanceAsAngle = (pi_2 - atan2(cameraZ, modelDistance));
  return gaussianProbability(abs(modelDistanceAsAngle - measuredDistanceAsAngle), standardDeviation);
}

Vector2f UKFSample::getOrthogonalProjection(const Vector2f& base, const Vector2f& dir, const Vector2f& point) const
{
  const float l = (point.x() - base.x()) * dir.x() + (point.y() - base.y()) * dir.y();
  return base + dir * l;
}

void UKFSample::draw(bool simple)
{
  if(simple)
  {
    Pose2f pose = getPose();
    Vector2f headPos(30, 0);
    headPos = pose * headPos;
    ColorRGBA sampleFillColor = ColorRGBA(200, 200, 0);
    RECTANGLE2("module:SelfLocator:simples", Vector2f(pose.translation - Vector2f(55.f, 90.f)), 110, 180, pose.rotation, 0, Drawings::solidPen,
               ColorRGBA(180, 180, 180),
               Drawings::solidBrush,
               sampleFillColor);
    CIRCLE("module:SelfLocator:simples",
           headPos.x(),
           headPos.y(),
           30,
           0, // pen width
           Drawings::solidPen,
           ColorRGBA(180, 180, 180),
           Drawings::solidBrush,
           ColorRGBA(180, 180, 180));
  }
  else
  {
    const float factor = 1.f;
    const float cov102 = cov(1, 0) * cov(1, 0);
    const float varianceDiff = cov(0, 0) - cov(1, 1);
    const float varianceDiff2 = varianceDiff * varianceDiff;
    const float varianceSum = cov(0, 0) + cov(1, 1);
    const float root = sqrt(varianceDiff2 + 4.0f * cov102);
    const float eigenValue1 = 0.5f * (varianceSum + root);
    const float eigenValue2 = 0.5f * (varianceSum - root);
    const float axis1 = 2.0f * sqrt(factor * eigenValue1);
    const float axis2 = 2.0f * sqrt(factor * eigenValue2);
    const float angle = 0.5f * atan2(2.0f * cov(1, 0), varianceDiff);
    ELLIPSE("module:SelfLocator:samples", mean, sqrt(3.0f) * axis1, sqrt(3.0f) * axis2, angle,
            10, Drawings::solidPen, ColorRGBA(100, 100, 255, 100), Drawings::solidBrush, ColorRGBA(100, 100, 255, 100));
    ELLIPSE("module:SelfLocator:samples", mean, sqrt(2.0f) * axis1, sqrt(2.0f) * axis2, angle,
            10, Drawings::solidPen, ColorRGBA(150, 150, 100, 100), Drawings::solidBrush, ColorRGBA(150, 150, 100, 100));
    ELLIPSE("module:SelfLocator:samples", mean, axis1, axis2, angle,
            10, Drawings::solidPen, ColorRGBA(255, 100, 100, 100), Drawings::solidBrush, ColorRGBA(255, 100, 100, 100));
  }
}
