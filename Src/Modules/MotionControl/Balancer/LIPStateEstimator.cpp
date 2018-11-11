#include "LIPStateEstimator.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Rotation.h"

void LIPStateEstimatorParameters::onRead()
{
  positionProcessDeviation.cwiseAbs();
  velocityProcessDeviation.cwiseAbs();
  zmpProcessDeviation.cwiseAbs();
  positionMeasurementDeviation.cwiseAbs();
  zmpMeasurementDeviation.cwiseAbs();
}

LIPStateEstimator::LIPStateEstimator(const InertialData& theInertialData, const RobotModel& theRobotModel):
  theInertialData(theInertialData), theRobotModel(theRobotModel)
{}

void LIPStateEstimator::init(const Array2f& LIPHeights, const Vector2f& leftOrigin, LIPStateEstimatorParameters params)
{
  this->params = params;
  supportFoot = guessSupportFoot(leftOrigin);
  this->LIPHeights = LIPHeights;
  origin = leftOrigin;
  if(supportFoot != SupportFoot::left)
    origin.y() *= -1;
  const Vector4f measurement = measure(supportFoot, origin);
  const Vector6f initMean = (Vector6f() << measurement.head<2>(), Vector2f::Zero(), measurement.tail<2>()).finished();
  const Vector6f initNoise = (Vector6f() << params.positionProcessDeviation, params.velocityProcessDeviation, params.zmpProcessDeviation).finished();
  ukf.init(initMean, initNoise.cwiseAbs2().asDiagonal());
}

void LIPStateEstimator::update(float timePassed, const Array2f& LIPHeights, const Vector2f& leftOrigin)
{
  update(timePassed, LIPHeights, leftOrigin, guessSupportFoot(leftOrigin));
}

void LIPStateEstimator::update(float timePassed, const Array2f& LIPHeights, const Vector2f& leftOrigin, const SupportFoot newSupportFoot)
{
  this->LIPHeights = LIPHeights;
  Vector2f newOrigin = leftOrigin;
  if(newSupportFoot != SupportFoot::left)
    newOrigin.y() *= -1;
  const Vector4f measurement = measure(newSupportFoot, newOrigin);
  if(supportFoot != newSupportFoot)
  {
    EstimatedState est = getEstimate();
    est = convertToOtherFoot(est);
    ukf.mean.head<2>() = est.com.position;
    ukf.mean.segment<2>(2) = est.com.velocity;
    ukf.mean.tail<2>() = est.zmp;
  }
  else
  {
    ukf.mean.head<2>() = origin + ukf.mean.head<2>() - newOrigin;
    ukf.mean.tail<2>() = origin + ukf.mean.tail<2>() - newOrigin;
  }
  supportFoot = newSupportFoot;
  origin = newOrigin;

  auto dynamicMoldel = [&](Vector6f& state)
  {
    const Vector2f zmp = state.tail<2>();
    LIP3D lipState(state.head<2>(), state.segment<2>(2), LIPHeights);
    lipState.update(timePassed, zmp);
    state << lipState.position, lipState.velocity, zmp;
  };

  auto measurementModel = [&](const Vector6f& state)
  {
    return (Vector4f() << state.head<2>(), state.tail<2>()).finished();
  };

  const Vector6f dynamicNoise = (Vector6f() << params.positionProcessDeviation, params.velocityProcessDeviation,
                                 params.zmpProcessDeviation).finished() * timePassed;
  const Vector4f measurementNoise = (Vector4f() << params.positionMeasurementDeviation, params.zmpMeasurementDeviation).finished() * timePassed;
  ukf.predict(dynamicMoldel, dynamicNoise.cwiseAbs2().asDiagonal());
  ukf.update<4>(measurement, measurementModel, measurementNoise.cwiseAbs2().asDiagonal());
}

LIPStateEstimator::EstimatedState LIPStateEstimator::getEstimate() const
{
  EstimatedState state(LIPHeights);
  state.com.position << ukf.mean.head<2>();
  state.com.velocity << ukf.mean.segment<2>(2);
  state.zmp = ukf.mean.tail<2>();
  state.supportFoot = supportFoot;
  state.origin = origin;
  return state;
}

LIPStateEstimator::EstimatedState LIPStateEstimator::convertToOtherFoot(const EstimatedState& state) const
{
  EstimatedState otherState(state);
  otherState.supportFoot = state.supportFoot == SupportFoot::left ? SupportFoot::right : SupportFoot::left;
  otherState.origin.y() *= -1.f;

  const Pose3f& soleToTorso = state.supportFoot == SupportFoot::left ? theRobotModel.soleLeft : theRobotModel.soleRight;
  const Pose3f& otherSoleToTorso = state.supportFoot == SupportFoot::left ? theRobotModel.soleRight : theRobotModel.soleLeft;
  const Pose3f originToTorso = soleToTorso + Vector3f(state.origin.x(), state.origin.y(), 0.f);
  const Pose3f otherOriginToTorso = otherSoleToTorso + Vector3f(otherState.origin.x(), otherState.origin.y(), 0.f);

  //const Quaternionf& torsoToWorld = theInertiaData.orientation;
  //Pose3f oritinToOtherOrigin = Pose3f(torsoToWorld) * otherOriginToTorso.rotation * otherOriginToTorso.inverse() * originToTorso * originToTorso.rotation.inverse() *= torsoToWorld.inverse();

  const Pose3f oritinToOtherOrigin = otherOriginToTorso.inverse() * originToTorso;

  otherState.com.position = (oritinToOtherOrigin * Vector3f(state.com.position.x(), state.com.position.y(), 0.f)).head<2>();
  otherState.com.velocity = (oritinToOtherOrigin.rotation * Vector3f(state.com.velocity.x(), state.com.velocity.y(), 0.f)).head<2>();
  otherState.zmp = (oritinToOtherOrigin * Vector3f(state.zmp.x(), state.zmp.y(), 0.f)).head<2>();
  return otherState;
}

LIPStateEstimator::SupportFoot LIPStateEstimator::guessSupportFoot(const Vector2f& leftOrigin) const
{
  const Pose3f leftOriginToTorso = theRobotModel.soleLeft + Vector3f(leftOrigin.x(), leftOrigin.y(), 0.f);
  const Pose3f rightOriginToTorso = theRobotModel.soleRight + Vector3f(leftOrigin.x(), -leftOrigin.y(), 0.f);
  const Pose3f torsoToWorld(theInertialData.orientation2D);
  const Pose3f leftOriginToWorld = torsoToWorld * leftOriginToTorso;
  const Pose3f rightOriginToWorld = torsoToWorld * rightOriginToTorso;

  if(leftOriginToWorld.translation.z() < rightOriginToWorld.translation.z())
    return SupportFoot::left;
  else
    return SupportFoot::right;
}

Vector4f LIPStateEstimator::measure(SupportFoot supportFoot, const Vector2f& LIPOrigin) const
{
  const Pose3f& supportFootToTorso = supportFoot == SupportFoot::left ? theRobotModel.soleLeft : theRobotModel.soleRight;
  const Quaternionf& torsoToWorld = theInertialData.orientation2D;
  const Pose3f originToTorso = supportFootToTorso + Vector3f(LIPOrigin.x(), LIPOrigin.y(), 0.f);
  const Vector3f comInOrigin = originToTorso.inverse() * theRobotModel.centerOfMass;
  const Vector3f com = (torsoToWorld * originToTorso.rotation) * comInOrigin;

  PLOT("module:ZmpWalkingEngine:LIPStateEstimator:Estimate:measuredComHeight", com.z());

  const Vector2f accInWorld = (torsoToWorld * theInertialData.acc * 1000.f).head<2>();
  const Vector2f zmp = com.head<2>().array() - (accInWorld.array() / LIP3D(LIPHeights).getK().square());

  return (Vector4f() << com.head<2>(), zmp).finished();
}

void LIPStateEstimator::draw(float footOffset, float forwardingTime) const
{
  DEBUG_DRAWING3D("module:ZmpWalkingEngine:LIPStateEstimator:measurement", "field")
  {
    const Vector4f measurement = measure(supportFoot, origin);
    const float supportFootSign = static_cast<float>(supportFoot);
    const Vector3f comToOrigin = (Vector3f() << measurement.head<2>(), LIPHeights.x()).finished();
    const Vector3f originToSupport = (Vector3f() << origin, 0.f).finished();
    const Vector3f supportToWorld = (Vector3f() << -origin.x(), supportFootSign * footOffset, 0.f).finished();
    const Vector3f comInWorld = supportToWorld + originToSupport + comToOrigin;
    const Vector3f zmp = supportToWorld + originToSupport + (Vector3f() << measurement.tail<2>(), 0.f).finished();

    SUBCOORDINATES3D("module:ZmpWalkingEngine:LIPStateEstimator:measurement", Pose3f(comInWorld), 50, 1);
    SPHERE3D("module:ZmpWalkingEngine:LIPStateEstimator:measurement", comInWorld.x(), comInWorld.y(), comInWorld.z(), 3, ColorRGBA::yellow);
    CROSS3D("module:ZmpWalkingEngine:LIPStateEstimator:measurement", zmp.x(), zmp.y(), zmp.z(), 3, 5, ColorRGBA::yellow);
  }

  DEBUG_DRAWING3D("module:ZmpWalkingEngine:LIPStateEstimator:estimate", "field")
  {
    const float supportFootSign = static_cast<float>(supportFoot);
    const Vector3f comToOrigin = (Vector3f() << ukf.mean.head<2>(), LIPHeights.x()).finished();
    const Vector3f originToSupport = (Vector3f() << origin, 0.f).finished();
    const Pose3f supportToWorld((Vector3f() << -origin.x(), supportFootSign * footOffset, 0.f).finished());
    const Vector3f comInWorld = supportToWorld * (originToSupport + comToOrigin);

    SUBCOORDINATES3D("module:ZmpWalkingEngine:LIPStateEstimator:estimate", Pose3f(comInWorld), 50, 1);
    FOOT3D("module:ZmpWalkingEngine:LIPStateEstimator:estimate", supportToWorld, supportFoot == SupportFoot::left, ColorRGBA::orange);
    const Vector3f comSigma(std::sqrt(ukf.cov(0, 0)), std::sqrt(ukf.cov(1, 1)), 0.001f);
    ELLIPSOID3D("module:ZmpWalkingEngine:LIPStateEstimator:estimate", Pose3f(comInWorld), comSigma, ColorRGBA(255, 100, 100, 255));
    ELLIPSOID3D("module:ZmpWalkingEngine:LIPStateEstimator:estimate", Pose3f(comInWorld), Vector3f(comSigma * 2), ColorRGBA(150, 150, 100, 100));
    ELLIPSOID3D("module:ZmpWalkingEngine:LIPStateEstimator:estimate", Pose3f(comInWorld), Vector3f(comSigma * 3), ColorRGBA(100, 100, 255, 50));

    Vector3f originToWorld = supportToWorld * originToSupport;
    const Vector2f zmp = ukf.mean.tail<2>();
    const Vector3f zmpInWorld = originToWorld + (Vector3f() << zmp, 0.f).finished();
    LINE3D("module:ZmpWalkingEngine:LIPStateEstimator:estimate", originToWorld.x(), originToWorld.y(), originToWorld.z(), zmpInWorld.x(), zmpInWorld.y(), zmpInWorld.z(), 2, ColorRGBA::orange);
    LINE3D("module:ZmpWalkingEngine:LIPStateEstimator:estimate", zmpInWorld.x(), zmpInWorld.y(), zmpInWorld.z(), comInWorld.x(), comInWorld.y(), comInWorld.z(), 2, ColorRGBA::orange);
    const Vector3f zmpSigma(std::sqrt(ukf.cov(4, 4)), std::sqrt(ukf.cov(5, 5)), 0.001f);
    ELLIPSOID3D("module:ZmpWalkingEngine:LIPStateEstimator:estimate", Pose3f(zmpInWorld), zmpSigma, ColorRGBA(255, 100, 100, 255));
    ELLIPSOID3D("module:ZmpWalkingEngine:LIPStateEstimator:estimate", Pose3f(zmpInWorld), Vector3f(zmpSigma * 2), ColorRGBA(150, 150, 100, 100));
    ELLIPSOID3D("module:ZmpWalkingEngine:LIPStateEstimator:estimate", Pose3f(zmpInWorld), Vector3f(zmpSigma * 3), ColorRGBA(100, 100, 255, 50));
    {
      const float dt = 0.01f;
      LIP3D comEstimate(ukf.mean.head<2>() - zmp, ukf.mean.segment<2>(2), LIPHeights);
      Vector3f futureCom;
      for(float timeRemaining = 0.3f; timeRemaining >= 0; timeRemaining -= dt)
      {
        comEstimate.update(dt);
        futureCom << comEstimate.position + zmp, LIPHeights.x();
        const Vector3f futureComInWorld = (supportToWorld * Pose3f(originToSupport + futureCom)).translation;
        POINT3D("module:ZmpWalkingEngine:LIPStateEstimator:estimate", futureComInWorld.x(), futureComInWorld.y(), futureComInWorld.z(), 2, ColorRGBA::black);
      }
    }
    {
      const float dt = 0.01f;
      LIP3D comEstimate(ukf.mean.head<2>() - zmp, ukf.mean.segment<2>(2), LIPHeights);
      Vector3f futureCom;
      for(float timeRemaining = forwardingTime; timeRemaining >= 0; timeRemaining -= dt)
      {
        comEstimate.update(dt);
        futureCom << comEstimate.position + zmp, LIPHeights.x();
        const Vector3f futureComInWorld = (supportToWorld * Pose3f(originToSupport + futureCom)).translation;
        POINT3D("module:ZmpWalkingEngine:LIPStateEstimator:estimate", futureComInWorld.x(), futureComInWorld.y(), futureComInWorld.z(), 2, ColorRGBA::orange);
      }
    }
  }
}

void LIPStateEstimator::plot() const
{
  PLOT("module:ZmpWalkingEngine:LIPStateEstimator:estimate:posX", ukf.mean(0));
  PLOT("module:ZmpWalkingEngine:LIPStateEstimator:estimate:posY", ukf.mean(1));
  PLOT("module:ZmpWalkingEngine:LIPStateEstimator:estimate:velX", ukf.mean(2));
  PLOT("module:ZmpWalkingEngine:LIPStateEstimator:estimate:velY", ukf.mean(3));
  PLOT("module:ZmpWalkingEngine:LIPStateEstimator:estimate:zmpX", ukf.mean(4));
  PLOT("module:ZmpWalkingEngine:LIPStateEstimator:estimate:zmpY", ukf.mean(5));

  Vector4f measurement = measure(supportFoot, origin);
  PLOT("module:ZmpWalkingEngine:LIPStateEstimator:measurement:posX", measurement(0));
  PLOT("module:ZmpWalkingEngine:LIPStateEstimator:measurement:posY", measurement(1));
  PLOT("module:ZmpWalkingEngine:LIPStateEstimator:measurement:zmpX", measurement(2));
  PLOT("module:ZmpWalkingEngine:LIPStateEstimator:measurement:zmpY", measurement(3));
}
