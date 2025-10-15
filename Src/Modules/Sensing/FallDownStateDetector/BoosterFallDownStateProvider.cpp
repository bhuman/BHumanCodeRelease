/**
 * @File BoosterFallDownStateProvider.cpp
 * This file implements a module that computes the current body state from sensor data
 * @author Daniel Krause
 * @author Philip Reichenberg
 */

#include "BoosterFallDownStateProvider.h"
#include "Debugging/Annotation.h"
#include "Debugging/DebugDrawings3D.h"
#include "Framework/Settings.h"
#include "Math/Constants.h"
#include "Math/Pose3f.h"
#include "Math/Rotation.h"
#include "Platform/SystemCall.h"
#include "RobotParts/FsrSensors.h"
#include "Streaming/Global.h"

MAKE_MODULE(BoosterFallDownStateProvider);

BoosterFallDownStateProvider::BoosterFallDownStateProvider() : lastTimeSoundPlayed(theFrameInfo.time)
{
  theSensorData = useInertiaData ? &theInertialData : &theInertialSensorData;
  getSupportPolygon();
  lastTiltingEdge = tiltingEdge = getTiltingEdge();
  initUKF(measure());
  disablePickUp = SystemCall::getMode() == SystemCall::simulatedRobot;
}

void BoosterFallDownStateProvider::update(FallDownState& fallDownState)
{
  DECLARE_DEBUG_DRAWING3D("module:BoosterFallDownStateProvider:ukf:predict", "field");
  DECLARE_DEBUG_DRAWING3D("module:BoosterFallDownStateProvider:fall", "field");

  dynamicNoise = (Vector5f() << positionProcessDeviation.cwiseAbs2(), velocityProcessDeviation.cwiseAbs2().cast<float>()).finished() * Global::getSettings().motionCycleTime;
  measurementNoise = (Vector5f() << positionMeasurementDeviation.cwiseAbs2(), velocityMeasurementDeviation.cwiseAbs2().cast<float>()).finished();
  theSensorData = useInertiaData ? &theInertialData : &theInertialSensorData;
  theFallDownState = &fallDownState;
  theFallDownState->predictedCom = Vector3f::Zero();

  getSupportPolygon();
  tiltingEdge = getTiltingEdge();

  if(std::isnan(tiltingEdge.translation.x()) || std::isnan(tiltingEdge.translation.y()) || std::isnan(tiltingEdge.translation.z()))
    tiltingEdge = supportFoot == Legs::left ? theRobotModel.soleLeft : theRobotModel.soleRight;

  // transform support foot and centroid to the tilting edge coordinate system
  const Pose3f torsoToOrigin = tiltingEdge.inverse();
  for(Vector3f& point : supportPolygon)
    point = torsoToOrigin * point;
  supportFootCenter = torsoToOrigin * supportFootCenter;

  // predict and measure the next state
  if((fallDownState.state == FallDownState::upright || fallDownState.state == FallDownState::staggering) && !resetFilter)
  {
    convertToNewOrigin(lastTiltingEdge, tiltingEdge, ukf.mean);
    updateUKF();
  }
  else
  {
    const Vector5f measurement = measure();
    initUKF(measurement);
    resetFilter = false;
  }
  const Vector3f& comInOrigin = ukf.mean.head<3>();
  torsoUpright = (comInOrigin.head<2>() - supportFootCenter.head<2>()).norm() <= 0.7f * (-supportFootCenter.head<2>()).norm();
  torsoAboveGround = comInOrigin.z();
  falling = isFalling();

  stable = std::abs(theSensorData->gyro.x()) <= maxGyroToRegainStableState
           && std::abs(theSensorData->gyro.y()) <= maxGyroToRegainStableState;
  toUpright = torsoUpright && torsoAboveGround >= maxTorsoHeightToKeepSquatting && theGroundContactState.contact;
  toSquatting = torsoUpright && torsoAboveGround < minTorsoHeightToKeepUpright && theGroundContactState.contact;
  isPickedUp = torsoUpright && torsoAboveGround >= maxTorsoHeightToKeepSquatting && !theGroundContactState.contact;
  useTorsoOrientation = std::abs(theSensorData->angle.x()) >= minTorsoOrientationToDetermineDirection.x()
                        || std::abs(theSensorData->angle.y()) >= minTorsoOrientationToDetermineDirection.y();
  direction = getFallDirection();
  lastTiltingEdge = tiltingEdge;

  // Execute behavior
  beginFrame(theFrameInfo.time);
  execute("Root");
  endFrame();

  // debug stuff
  draw();
}

void BoosterFallDownStateProvider::updatePredictedCom()
{
  if(theFallDownState->state == FallDownState::upright || theFallDownState->state == FallDownState::staggering)
  {
    Vector2f v = ukf.mean.tail<2>();
    v = v.norm() > maxVelForPrediction ? maxVelForPrediction * v.normalized() : v;
    UKF<5> ukfPredict = UKF<5>((Vector5f() << ukf.mean.head<3>(), v).finished());
    ukfPredict.cov = Matrix5f(ukf.cov);

    const Matrix3f I = calcInertiaTensor(tiltingEdge);
    for(float timePast = 0.f; timePast < forwardingTime; timePast += Global::getSettings().motionCycleTime)
    {
      auto dynamic = [&](Vector5f& state)
      {
        dynamicModel(tiltingEdge, I, state);
      };
      ukfPredict.predict(dynamic, dynamicNoise.asDiagonal());
      ukfPredict.mean.tail<2>() = velocityDiscountFactor * ukfPredict.mean.tail<2>();
    }
    theFallDownState->predictedCom = tiltingEdge * ukfPredict.mean.head<3>();
  }
}

bool BoosterFallDownStateProvider::isFalling()
{
  //if the com is within the subSupportPolygon the robot is in a stable position
  if(torsoUpright)
  {
    updatePredictedCom();
    return false;
  }

  const Vector3f& com = ukf.mean.head<3>();
  Vector2f vel = (Vector2f() << ukf.mean.tail<2>().y(), -ukf.mean.tail<2>().x()).finished();
  Vector2f direction = (com.head<2>() - supportFootCenter.head<2>()).normalized();

  Vector3f comSigma(std::sqrt(ukf.cov(0, 0)) * sigmaArea, std::sqrt(ukf.cov(1, 1)) * sigmaArea, 0.01f);
  Vector3f bestCaseCom = (Vector3f() << com.head<2>() - (comSigma.x() * direction), 0.f).finished();

  // robot is falling, if the com is not in the support polygon
  if(!isPointInsidePolygon(bestCaseCom, supportPolygon)
     && (!restrictiveFallDetection || bestCaseCom.norm() > minComDistanceToDetectFall)
     && ((direction.dot(vel) > minFallVectorScalar && direction.dot(vel.normalized()) > minNormalizedFallVectorScalar)
         || bestCaseCom.norm() > minComDistanceToDetectFall))
  {
    updatePredictedCom();
    return true;
  }

  //if the robot is not falling and not stable, predict the next positions of the com
  bool canBreakUp = false;
  if(theFallDownState->state == FallDownState::upright || theFallDownState->state == FallDownState::staggering)
  {
    Vector2f v = ukf.mean.tail<2>();
    v = v.norm() > maxVelForPrediction ? maxVelForPrediction * v.normalized() : v;
    UKF<5> ukfPredict = UKF<5>((Vector5f() << ukf.mean.head<3>(), v).finished());
    ukfPredict.cov = Matrix5f(ukf.cov);
    const Matrix3f I = calcInertiaTensor(tiltingEdge);
    for(float timePast = 0.f; timePast < forwardingTime; timePast += Global::getSettings().motionCycleTime)
    {
      auto dynamic = [&](Vector5f& state)
      {
        dynamicModel(tiltingEdge, I, state);
      };
      ukfPredict.predict(dynamic, dynamicNoise.asDiagonal());
      ukfPredict.mean.tail<2>() = velocityDiscountFactor * ukfPredict.mean.tail<2>();
      vel << ukfPredict.mean.tail<2>().y(), -ukfPredict.mean.tail<2>().x();
      direction << (ukfPredict.mean.head<2>() - supportFootCenter.head<2>()).normalized();

      if(canBreakUp || direction.dot(vel) < minPredictFallVectorScalar || direction.dot(vel.normalized()) < minPredictNormalizedFallVectorScalar)
        canBreakUp = true;
      if(!canBreakUp) // keep predicting for predicted com
      {
        const Vector2f& futureCom = ukfPredict.mean.head<2>();
        comSigma << std::sqrt(ukfPredict.cov(0, 0)) * sigmaArea, std::sqrt(ukfPredict.cov(1, 1))* sigmaArea, 0.01f;
        bestCaseCom << futureCom - (comSigma.x() * direction), 0.f;

        DEBUG_DRAWING3D("module:BoosterFallDownStateProvider:fall", "field")
        {
          Vector3f drawCom = tiltingEdge.translation + Vector3f(futureCom.x(), futureCom.y(), 0.f);
          drawCom.z() = 0.f;
          Vector3f drawBestCom = tiltingEdge.translation + bestCaseCom;
          POINT3D("module:BoosterFallDownStateProvider:fall", drawBestCom.x(), drawBestCom.y(), 0.f, 5.f, ColorRGBA::blue);
          ELLIPSOID3D("module:BoosterFallDownStateProvider:fall", Pose3f(drawCom), Vector3f(comSigma), ColorRGBA(100, 100, 255, 50));
        }
        if(!isPointInsidePolygon(bestCaseCom, supportPolygon) && (!restrictiveFallDetection || bestCaseCom.norm() > minComDistanceToDetectFall))
        {
          return true;
        }
      }
    }
    theFallDownState->predictedCom = tiltingEdge * ukfPredict.mean.head<3>();
  }
  return false;
}

FallDownState::Direction BoosterFallDownStateProvider::getFallDirection() const
{
  Vector2f direction = (ukf.mean.head<2>() - supportFootCenter.head<2>());
  Angle fallDirection = std::atan2(direction.x(), direction.y());

  if(-pi_4 + pi_8 <= fallDirection && fallDirection <= pi_8) return FallDownState::left;
  if(-pi3_4 - pi_8 >= fallDirection || fallDirection >= pi3_4 + pi_8) return FallDownState::right;
  if(pi_8 < fallDirection && fallDirection < pi3_4 + pi_8) return FallDownState::front;
  return FallDownState::back;
}

void BoosterFallDownStateProvider::initUKF(const Vector5f& measurement)
{
  const Vector5f initMean = measurement;
  const Matrix5f noise = Matrix5f::Identity();
  ukf.init(initMean, noise);
}

void BoosterFallDownStateProvider::updateUKF()
{
  const Matrix3f I = calcInertiaTensor(tiltingEdge);
  auto dynamic = [&](Vector5f& state)
  {
    dynamicModel(tiltingEdge, I, state);
  };
  auto measurementModel = [](const Vector5f& state)
  {
    return state;
  };
  const Vector5f measurement = measure();
  ukf.predict(dynamic, dynamicNoise.asDiagonal());
  ukf.update<5>(measurement, measurementModel, measurementNoise.asDiagonal());
}

Vector5f BoosterFallDownStateProvider::measure() const
{
  const RotationMatrix torsoRotation(Rotation::Euler::fromAngles(theSensorData->angle.x(), theSensorData->angle.y(), 0.f));
  const Vector3f com = tiltingEdge.inverse() * theRobotModel.centerOfMass;
  const Vector2f vel = (torsoRotation * theSensorData->gyro.cast<float>()).head<2>();
  return (Vector5f() << com, vel).finished();
}

void BoosterFallDownStateProvider::dynamicModel(const Pose3f&, const Matrix3f& I, Vector5f& state, float dt) const
{
  const Vector3f weightForce = (Vector3f() << 0.f, 0.f, -theMassCalibration.totalMass * Constants::g).finished();
  const Vector3f standWidth = (Vector3f() << state.head<2>(), 0.f).finished();
  const Vector3f stabilityMoment = standWidth.cross(weightForce);
  const Vector3f acc = I.inverse() * stabilityMoment;

  const Vector2f velocity = state.tail<2>() + acc.head<2>() * dt;
  const Vector2f angleOffset = velocity * dt;
  const RotationMatrix rotation(Rotation::Euler::fromAngles(angleOffset.x(), angleOffset.y(), 0.f));
  const Vector3f com = rotation * state.head<3>();
  state << com, velocity;
}

void BoosterFallDownStateProvider::convertToNewOrigin(const Pose3f& originToTorso, const Pose3f& newOriginToTorso, Vector5f& state) const
{
  const Vector3f& comToOrigin = state.head<3>();
  const Pose3f originToNewOrigin = newOriginToTorso.inverse() * originToTorso;
  state.head<3>() = originToNewOrigin.translation + comToOrigin;
}

void BoosterFallDownStateProvider::getSupportPolygon()
{
  supportPolygon.clear();
  supportFoot = theSolePressureState.legInfo[Legs::left].totals > theSolePressureState.legInfo[Legs::right].totals ? Legs::left : Legs::right;
  const Pose3f& leftSoleToTorso = theRobotModel.soleLeft;
  const Pose3f& rightSoleToTorso = theRobotModel.soleRight;
  const Pose3f& supportedFootSoleToTorso = supportFoot == Legs::left ? leftSoleToTorso : rightSoleToTorso;
  const Pose3f& otherSupportedFootSoleToTorso = supportFoot == Legs::left ? rightSoleToTorso : leftSoleToTorso;
  const Vector3f com = tiltingEdge.translation + ukf.mean.head<3>();

  const bool useSupportFootOnly = std::abs(com.y()) > std::abs(supportedFootSoleToTorso.translation.y()) - minComDistanceToFootCenter
                                  && sgn(com.y()) == sgn(supportedFootSoleToTorso.translation.y())
                                  && std::abs(theSolePressureState.legInfo[Legs::left].totals - theSolePressureState.legInfo[Legs::right].totals) > 0.3f
                                  && !((supportedFootSoleToTorso.translation.x() < com.x() && supportedFootSoleToTorso.translation.x() < otherSupportedFootSoleToTorso.translation.x())
                                       || (supportedFootSoleToTorso.translation.x() > com.x() && supportedFootSoleToTorso.translation.x() > otherSupportedFootSoleToTorso.translation.x()));

  if(useSupportFootOnly)
  {
    const float sign = supportFoot == Legs::left ? 1.f : -1.f;
    for(const Vector2f& point : theRobotDimensions.soleShape)
      supportPolygon.push_back(supportedFootSoleToTorso * Vector3f(point.x(), sign * point.y(), 0.f));
  }
  else
  {
    for(const Vector2f& point : theRobotDimensions.soleShape)
    {
      supportPolygon.push_back(leftSoleToTorso * Vector3f(point.x(), point.y(), 0.f));
      supportPolygon.push_back(rightSoleToTorso * Vector3f(point.x(), -point.y(), 0.f));
    }
  }
  supportPolygon = getConvexHull(supportPolygon);

  // calculate centroid of the convex, counter-clockwise ordered polygon
  const size_t size = supportPolygon.size();
  float area = 0.f, x = 0.f, y = 0.f;
  for(size_t i = 0, j = size - 1; i < size; j = i++)
  {
    const Vector3f& p1 = supportPolygon[i];
    const Vector3f& p2 = supportPolygon[j];
    float f = p1.x() * p2.y() - p2.x() * p1.y();
    area += f;
    x += (p1.x() + p2.x()) * f;
    y += (p1.y() + p2.y()) * f;
  }
  area *= 3.f;
  supportFootCenter << x / area, y / area, supportedFootSoleToTorso.translation.z();
}

Pose3f BoosterFallDownStateProvider::getTiltingEdge()
{
  if(std::isnan(tiltingEdge.translation.x()) || std::isnan(tiltingEdge.translation.y()) || std::isnan(tiltingEdge.translation.z()))
  {
    return supportFoot == Legs::left ? theRobotModel.soleLeft : theRobotModel.soleRight;
  }
  const Vector2f direction = (tiltingEdge.translation + ukf.mean.head<3>() - supportFootCenter).head<2>();
  const Geometry::Line fallDirectionLine = Geometry::Line(Vector2f(supportFootCenter.head<2>()), direction.normalized());

  // search for the intersection with the support foot polygon and the line
  Vector3f intersection3D;
  for(size_t i = 0; i < supportPolygon.size(); ++i)
  {
    // check if the line intersect a line from the two points of the polygon
    Vector2f intersection2D;
    const Vector3f& p1 = supportPolygon[i];
    const Vector3f& p2 = supportPolygon[(i + 1) % supportPolygon.size()];
    const Vector2f& base = p1.head<2>();
    const Vector2f dir = p2.head<2>() - base;
    const Geometry::Line polygonLine(base, dir.normalized());

    if(Geometry::isPointLeftOfLine(fallDirectionLine.base, fallDirectionLine.base + fallDirectionLine.direction, base)
       != Geometry::isPointLeftOfLine(fallDirectionLine.base, fallDirectionLine.base + fallDirectionLine.direction, p2.head<2>())
       && Geometry::getIntersectionOfLines(fallDirectionLine, polygonLine, intersection2D)
       && (supportFootCenter.head<2>() - intersection2D).norm() > (supportFootCenter.head<2>() + fallDirectionLine.direction - intersection2D).norm())
    {
      float scalar = (intersection2D - base).norm() / dir.norm();
      intersection3D = p1 + scalar * (p2 - p1);
      break;
    }
  }
  const RotationMatrix torsoRotation(Rotation::Euler::fromAngles(theSensorData->angle.x(), theSensorData->angle.y(), 0.f));
  return Pose3f(torsoRotation.inverse(), intersection3D);
}

bool BoosterFallDownStateProvider::isPointInsidePolygon(const Vector3f& point, const std::vector<Vector3f>& polygon) const
{
  for(size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++)
  {
    if(cross(polygon[j], point, polygon[i]) > 0)
      return false;
  }
  return true;
}

float BoosterFallDownStateProvider::cross(const Vector3f& O, const Vector3f& A, const Vector3f& B) const
{
  return (A.x() - O.x()) * (B.y() - O.y()) - (A.y() - O.y()) * (B.x() - O.x());
}

std::vector<Vector3f> BoosterFallDownStateProvider::getConvexHull(std::vector<Vector3f>& polygon) const
{
  int n = static_cast<int>(polygon.size()),
      k = 0;
  if(n == 1) return polygon;
  std::vector<Vector3f> hull(2 * n);

  // Sort points lexicographically
  std::sort(polygon.begin(), polygon.end(), [](const Vector3f& p1, const Vector3f& p2) {return p1.x() < p2.x() || (p1.x() == p2.x() && p1.y() < p2.y()); });

  // Build lower hull
  for(int i = 0; i < n; ++i)
  {
    while(k >= 2 && cross(hull[k - 2], hull[k - 1], polygon[i]) <= 0) k--;
    hull[k++] = polygon[i];
  }
  // Build upper hull
  for(int i = n - 2, t = k + 1; i >= 0; i--)
  {
    while(k >= t && cross(hull[k - 2], hull[k - 1], polygon[i]) <= 0) k--;
    hull[k++] = polygon[i];
  }
  hull.resize(k - 1);
  return hull;
}

Matrix3f BoosterFallDownStateProvider::calcInertiaTensor(const Pose3f& originToTorso) const
{
  Matrix3f I = Matrix3f::Zero();
  const Pose3f torsoToOrigin = originToTorso.inverse();
  for(int i = 0; i < Limbs::numOfLimbs; ++i)
  {
    const Pose3f& limbToTorso = theRobotModel.limbs[i];
    const Vector3f& massPointToLimb = theMassCalibration.masses[i].offset;
    const Vector3f massPointToOrigin = torsoToOrigin * limbToTorso * massPointToLimb;
    const float x = massPointToOrigin.x(), y = massPointToOrigin.y(), z = massPointToOrigin.z();
    Matrix3f partial = (Matrix3f() << y * y + z * z, -x * y, -x * z,
                        -y * x, x * x + z * z, -y * z,
                        -z * x, -z * y, x * x + y * y).finished();
    partial *= theMassCalibration.masses[i].mass;
    I += partial;
  }
  return I;
}

void BoosterFallDownStateProvider::setState(FallDownState::State state, FallDownState::Direction direction,
                                            Angle odometryRotationOffset)
{
  if(theFallDownState->state != state)
    theFallDownState->timestampSinceStateSwitch = theFrameInfo.time;
  theFallDownState->state = state;
  theFallDownState->direction = direction;
  theFallDownState->odometryRotationOffset = odometryRotationOffset;
}

void BoosterFallDownStateProvider::setStateWithPossibleDirectionChange(FallDownState::State state)
{
  if(theFallDownState->direction == FallDownState::left || theFallDownState->direction == FallDownState::right)
  {
    if(theFallDownState->direction % 4 + 1 == direction)
      setState(state, direction, -pi_2);
    else if((theFallDownState->direction + 2) % 4 + 1 == direction)
      setState(state, direction, pi_2);
    else
      setState(state, theFallDownState->direction);
  }
  else
    setState(state, theFallDownState->direction);
}

void BoosterFallDownStateProvider::say(const char* text)
{
  ANNOTATION("BoosterFallDownStateProvider", text);
  if(playSounds && theFrameInfo.getTimeSince(lastTimeSoundPlayed) > minTimeBetweenSound)
  {
    SystemCall::say(text);
    lastTimeSoundPlayed = theFrameInfo.time;
  }
}

void BoosterFallDownStateProvider::draw() const
{
  Vector2a vel = (Vector2a() << ukf.mean.tail<2>().y(), -ukf.mean.tail<2>().x()).finished();
  Angle velNorm = vel.norm();
  Vector2f direction = (ukf.mean.head<2>() - supportFootCenter.head<2>()).normalized();
  float scalar = direction.dot(vel.cast<float>());
  float normScalar = direction.dot(vel.cast<float>().normalized());
  MODIFY("module:BoosterFallDownStateProvider:scalar", scalar);
  MODIFY("module:BoosterFallDownStateProvider:normScalar", normScalar);
  MODIFY("module:BoosterFallDownStateProvider:vel", vel);
  MODIFY("module:BoosterFallDownStateProvider:normVel", velNorm);

  DEBUG_DRAWING3D("module:BoosterFallDownStateProvider:supportPolygon:field", "field")
  {
    POINT3D("module:BoosterFallDownStateProvider:supportPolygon:field", ukf.mean.x(), ukf.mean.y(), 0.f, 5.f, ColorRGBA::red);

    for(size_t i = 0; i < supportPolygon.size(); ++i)
    {
      const Vector3f p1 = tiltingEdge.translation + supportPolygon[i];
      const Vector3f p2 = tiltingEdge.translation + supportPolygon[(i + 1) % supportPolygon.size()];
      LINE3D("module:BoosterFallDownStateProvider:supportPolygon:field", p1.x(), p1.y(), 0.1f, p2.x(), p2.y(), 0.1f, 5.f, ColorRGBA::orange);
    }
  }
  DEBUG_DRAWING3D("module:BoosterFallDownStateProvider:tiltingEdge", "field")
  {
    Vector2f center = (tiltingEdge.translation + supportFootCenter).head<2>();
    const Vector2f fallDirection = ukf.mean.head<2>() - supportFootCenter.head<2>();
    const Geometry::Line line(center, fallDirection.normalized());
    SPHERE3D("module:BoosterFallDownStateProvider:tiltingEdge", center.x(), center.y(), 0.f, 2.f, ColorRGBA::black);
    Vector2f to = center + 30.f * line.direction;
    LINE3D("module:BoosterFallDownStateProvider:tiltingEdge", center.x(), center.y(), 0.f, to.x(), to.y(), 0.f, 3.f, ColorRGBA::black);
    SPHERE3D("module:BoosterFallDownStateProvider:tiltingEdge", tiltingEdge.translation.x(), tiltingEdge.translation.y(), 0.f, 3.f, ColorRGBA::red);
  }

  DEBUG_DRAWING3D("module:BoosterFallDownStateProvider:measurement", "field")
  {
    Vector3f comInOrigin = measure().head<3>();
    Vector3f com = tiltingEdge.translation + comInOrigin;
    SPHERE3D("module:BoosterFallDownStateProvider:measurement", com.x(), com.y(), 0.f, 1.f, ColorRGBA::black);
  }

  DEBUG_DRAWING3D("module:BoosterFallDownStateProvider:ukf:field", "field")
  {
    const Vector3f& point = supportFoot == Legs::left ? theRobotModel.soleLeft.translation : theRobotModel.soleRight.translation;
    POINT3D("module:BoosterFallDownStateProvider:ukf:field", point.x(), point.y(), 0.f, 5.f, ColorRGBA::red);
    SPHERE3D("module:BoosterFallDownStateProvider:ukf:field", ukf.mean.x(), ukf.mean.y(), 0.f, 1.f, ColorRGBA::violet);

    Vector3f com = (Vector3f() << (tiltingEdge.translation + ukf.mean.head<3>()).head<2>(), 0.f).finished();
    Vector3f vel = (Vector3f() << ukf.mean.tail<2>().y(), -ukf.mean.tail<2>().x(), 0.f).finished();
    Vector3f velDirection = com + 10.f * vel;
    LINE3D("module:BoosterFallDownStateProvider:ukf:field", com.x(), com.y(), 0.f, velDirection.x(), velDirection.y(), 0.f, 3.f, ColorRGBA::blue);

    SPHERE3D("module:BoosterFallDownStateProvider:ukf:field", com.x(), com.y(), com.z(), 1.f, ColorRGBA::gray);
    Vector3f comSigma(std::sqrt(ukf.cov(0, 0)), std::sqrt(ukf.cov(1, 1)), Global::getSettings().motionCycleTime);
    ELLIPSOID3D("module:BoosterFallDownStateProvider:ukf:field", Pose3f(com), comSigma, ColorRGBA(255, 100, 100, 255));
    ELLIPSOID3D("module:BoosterFallDownStateProvider:ukf:field", Pose3f(com), Vector3f(comSigma * 2.f), ColorRGBA(150, 150, 100, 100));
    ELLIPSOID3D("module:BoosterFallDownStateProvider:ukf:field", Pose3f(com), Vector3f(comSigma * 3.f), ColorRGBA(100, 100, 255, 50));

    comSigma *= sigmaArea;
    Vector2f direction = (com.head<2>() - supportFootCenter.head<2>()).normalized();
    Vector3f bestCaseCom = (Vector3f() << com.head<2>() - (comSigma.x() * direction), 0.f).finished();
    SPHERE3D("module:BoosterFallDownStateProvider:ukf:field", bestCaseCom.x(), bestCaseCom.y(), bestCaseCom.z(), 0.5f, ColorRGBA::black);
  }

  DEBUG_DRAWING3D("module:BoosterFallDownStateProvider:ukf:predict", "field")
  {
    if(theFallDownState->state == FallDownState::upright || theFallDownState->state == FallDownState::staggering)
    {
      UKF<5> ukfPredict = UKF<5>(Vector5f(ukf.mean));
      ukfPredict.cov = Matrix5f(ukf.cov);
      const Pose3f& originToTorso = tiltingEdge;
      const Matrix3f I = calcInertiaTensor(tiltingEdge);
      for(float timePast = 0.f; timePast < forwardingTime; timePast += Global::getSettings().motionCycleTime)
      {
        auto dynamic = [&](Vector5f& state)
        {
          dynamicModel(tiltingEdge, I, state);
        };
        ukfPredict.predict(dynamic, dynamicNoise.asDiagonal());
        Vector3f futureCom = originToTorso.translation + ukfPredict.mean.head<3>();
        const Vector3f comSigma(std::sqrt(ukfPredict.cov(0, 0)), std::sqrt(ukfPredict.cov(1, 1)), 0.1f);

        POINT3D("module:BoosterFallDownStateProvider:ukf:predict", futureCom.x(), futureCom.y(), 0.f, 3.f, ColorRGBA::blue);
        //ELLIPSOID3D("module:BoosterFallDownStateProvider:ukf:predict", Pose3f(futureCom), comSigma, ColorRGBA(255, 100, 100, 255));
        //ELLIPSOID3D("module:BoosterFallDownStateProvider:ukf:predict", Pose3f(futureCom), Vector3f(comSigma * 2.f), ColorRGBA(150, 150, 100, 100));
        //ELLIPSOID3D("module:BoosterFallDownStateProvider:ukf:predict", Pose3f(futureCom), Vector3f(comSigma * 3.f), ColorRGBA(100, 100, 255, 50));
      }
    }
  }

  DEBUG_DRAWING3D("module:BoosterFallDownStateProvider:robot", "robot")
  {
    const Vector3f& com = tiltingEdge.translation + ukf.mean.head<3>();
    SPHERE3D("module:BoosterFallDownStateProvider:robot", com.x(), com.y(), com.z(), 3.f, ColorRGBA::gray);
    SPHERE3D("module:BoosterFallDownStateProvider:robot", theRobotModel.centerOfMass.x(), theRobotModel.centerOfMass.y(), theRobotModel.centerOfMass.z(), 3.f, ColorRGBA::gray);
    SUBCOORDINATES3D("module:BoosterFallDownStateProvider:robot", tiltingEdge, 50.f, 1.f);

    for(size_t i = 0; i < supportPolygon.size(); ++i)
      LINE3D("module:BoosterFallDownStateProvider:robot", supportPolygon[i].x(), supportPolygon[i].y(), supportPolygon[i].z(), supportPolygon[(i + 1) % supportPolygon.size()].x(), supportPolygon[(i + 1) % supportPolygon.size()].y(), supportPolygon[(i + 1) % supportPolygon.size()].z(), 5.f, ColorRGBA::orange);
  }
}
