/**
 * @file RobotStableStateProvider.h
 * TODO
 * @author Philip Reichenberg
 */

#include "RobotStableStateProvider.h"
#include "Debugging/DebugDrawings3D.h"
#include "Debugging/Plot.h"
#include "Math/Rotation.h"

MAKE_MODULE(RobotStableStateProvider);

RobotStableStateProvider::RobotStableStateProvider()
{
  // Set the masses of all leg joints to 0
  lightMassCalibration = theMassCalibration;
  for(size_t i = Limbs::firstLeftLegLimb; i < Limbs::torso; i++)
    lightMassCalibration.masses[i].mass = 0.f;
}

void RobotStableStateProvider::update(RobotStableState& theRobotStableState)
{
  DECLARE_DEBUG_DRAWING3D("module:RobotStableStateProvider:footMid", "robot");
  DECLARE_PLOT("module:RobotStableState:prediction:x");
  DECLARE_PLOT("module:RobotStableState:prediction:y");
  DECLARE_DEBUG_RESPONSE("module:RobotStableState:prediction");

  theRobotStableState.predictRotation = [this, &theRobotStableState](const bool isLeftPhase, const bool prediction, const bool findTurnPoint)
  {
    if(lastUpdate == theFrameInfo.time && lastIsLeftPhase == isLeftPhase && !findTurnPoint)
      return;

    lastUpdate = theFrameInfo.time;
    lastIsLeftPhase = isLeftPhase;

    RobotModel lightModel = theRobotModel;
    lightModel.updateCenterOfMass(lightMassCalibration);
    theRobotStableState.lightCenterOfMass = lightModel.centerOfMass;

    const RotationMatrix tiltInTorso(Rotation::AngleAxis::unpack(Vector3f(-theInertialData.angle.x(), -theInertialData.angle.y(), 0.f)));
    predictRotation(theRobotStableState, tiltInTorso, lightModel, isLeftPhase, prediction, findTurnPoint);
    calculateCoMInPositionPercent(theRobotStableState);
    theRobotStableState.lastUpdate = theFrameInfo.time;
  };

  theRobotStableState.getTurnPoint = [this, &theRobotStableState](const Legs::Leg leg)
  {
    // Do not use this function is no prediction happened before!
    ASSERT(theFrameInfo.time == lastTurnPointMatrixUpdate);
    RobotStableState newState = theRobotStableState; // For CoM position
    newState.predictedTorsoRotation = turnPointMatrix;
    calculateCoMInPositionPercent(newState);
    return newState.comInFeet[leg];
  };

  if(!theMotionInfo.isMotion(MotionPhase::walk) &&
     !theMotionInfo.isMotion(MotionPhase::stand) &&
     !theMotionInfo.isMotion(MotionPhase::kick))
    theRobotStableState.predictRotation(theFootSupport.support > 0.f, false, false);
}

void RobotStableStateProvider::calculateCoMInPositionPercent(RobotStableState& theRobotStableState)
{
  const Vector3f soleLeft = theRobotStableState.predictedTorsoRotation.inverse() * theRobotModel.soleLeft.translation;
  const Vector3f soleRight = theRobotStableState.predictedTorsoRotation.inverse() * theRobotModel.soleRight.translation;
  const float supportheight = std::min(soleRight.z(), soleLeft.z());
  const Vector3f comInTorso = theRobotStableState.predictedTorsoRotation * (Vector3f() << (theRobotStableState.predictedTorsoRotation.inverse() * theRobotStableState.lightCenterOfMass).head<2>(), supportheight).finished();

  float heelFeetInTorso[Legs::numOfLegs];
  float toeFeetInTorso[Legs::numOfLegs];
  float innerEdgeFeetInTorso[Legs::numOfLegs];
  float outerEdgeFeetInTorso[Legs::numOfLegs];
  float middleOfFeetInTorso[Legs::numOfLegs];

  heelFeetInTorso[Legs::left] = theRobotModel.soleLeft.translation.x() - theFootOffset.backward;
  heelFeetInTorso[Legs::right] = theRobotModel.soleRight.translation.x() - theFootOffset.backward;
  toeFeetInTorso[Legs::left] = theRobotModel.soleLeft.translation.x() + theFootOffset.forward;
  toeFeetInTorso[Legs::right] = theRobotModel.soleRight.translation.x() + theFootOffset.forward;

  innerEdgeFeetInTorso[Legs::left] = theRobotModel.soleLeft.translation.y() - theFootOffset.leftFoot.right;
  outerEdgeFeetInTorso[Legs::left] = theRobotModel.soleLeft.translation.y() + theFootOffset.leftFoot.left;
  middleOfFeetInTorso[Legs::left] = theRobotModel.soleLeft.translation.y();
  innerEdgeFeetInTorso[Legs::right] = theRobotModel.soleRight.translation.y() + theFootOffset.rightFoot.left;
  outerEdgeFeetInTorso[Legs::right] = theRobotModel.soleRight.translation.y() - theFootOffset.rightFoot.right;
  middleOfFeetInTorso[Legs::right] = theRobotModel.soleRight.translation.y();

  FOREACH_ENUM(Legs::Leg, leg)
  {
    const float sign = leg == Legs::right ? -1.f : 1.f;
    theRobotStableState.comInTorso[leg].forward = Rangef::ZeroOneRange().limit(
                                                    (comInTorso.x() - heelFeetInTorso[leg]) /
                                                    (toeFeetInTorso[leg] - heelFeetInTorso[leg]));

    theRobotStableState.comInTorso[leg].outerSide =
      calcPercentInFeet(sign * comInTorso.y(), 0.f, sign * innerEdgeFeetInTorso[leg], sign * middleOfFeetInTorso[leg], sign * outerEdgeFeetInTorso[leg]);

    const Pose3f& sole = leg == Legs::left ? theRobotModel.soleLeft : theRobotModel.soleRight;
    const Vector2f comInFeetRotated = (comInTorso.head<2>() - sole.translation.head<2>()).rotated(-sole.rotation.getZAngle());
    const Vector2f zeroPointInFeet = (-sole.translation.head<2>()).rotated(-sole.rotation.getZAngle());

    // To be honest: the difference between torso and feet space is not even that much (on the outer edge like 0.1 points difference).
    // And in such state the robot would probably fall anyway
    theRobotStableState.comInFeet[leg].forward = Rangef::ZeroOneRange().limit(
                                                   (comInFeetRotated.x() + theFootOffset.backward) /
                                                   (theFootOffset.backward + theFootOffset.forward));

    const float useInner = leg == Legs::left ? -theFootOffset.leftFoot.right : -theFootOffset.rightFoot.left;
    const float useOuter = leg == Legs::left ? theFootOffset.leftFoot.left : theFootOffset.rightFoot.right;
    theRobotStableState.comInFeet[leg].outerSide =
      calcPercentInFeet(sign * comInFeetRotated.y(), sign * zeroPointInFeet.y(), useInner, 0.f, useOuter);
  }
}

float RobotStableStateProvider::calcPercentInFeet(const float refPoint, const float p0, const float p05, const float p075, const float p1)
{
  if(refPoint < p0)
    return 0;
  if(refPoint < p05)
    return (refPoint - p0) / (p05 - p0) * 0.5f;
  if(refPoint < p075)
    return (refPoint - p05) / (p075 - p05) * 0.25f + 0.5f;
  else if(refPoint > p1)
    return 1.f;
  return (refPoint - p075) / (p1 - p075) * 0.25f + 0.75f;
}

void RobotStableStateProvider::predictRotation(RobotStableState& state, RotationMatrix rotationMatrix, const RobotModel& model, const bool isLeftPhase, const bool prediction, const bool findTurnPoint)
{
  calcSupportPolygon(isLeftPhase);
  Vector2a velocity(-theInertialData.angleChange.x(), -theInertialData.angleChange.y()); // the usage is not correct because of 3D rotation. But this is just an approximation!
  const Vector2a startVelocity = velocity;

  auto predictStep = [this, &startVelocity](RotationMatrix& useMatrixForUpdate, const RobotModel& model, Vector2a& velocity, const bool isLeftPhase, const RobotModel& theRobotModel)
  {
    // 1. Calculate current com and approx next com, to find tilting edge
    const Vector3f currentComInRobot = useMatrixForUpdate.inverse() * model.centerOfMass;
    RotationMatrix nextApproxRotationMatrix = useMatrixForUpdate;
    nextApproxRotationMatrix.rotateY(velocity.y());
    nextApproxRotationMatrix.rotateX(velocity.x());
    const Vector3f nextApproxComInRobot = nextApproxRotationMatrix.inverse() * model.centerOfMass;

    const Vector3f currentComInFoot = useMatrixForUpdate * (Vector3f() << currentComInRobot.head<2>(), (useMatrixForUpdate.inverse() * (isLeftPhase ? theRobotModel.soleRight : theRobotModel.soleLeft).translation).z()).finished();
    const Vector3f nextApproxComInFoot = nextApproxRotationMatrix * (Vector3f() << nextApproxComInRobot.head<2>(), (nextApproxRotationMatrix.inverse() * (isLeftPhase ? theRobotModel.soleRight : theRobotModel.soleLeft).translation).z()).finished();

    // 2. calc tilting edge
    Vector3f tiltingEdge;
    if(!getTiltingPoint(nextApproxComInFoot, currentComInFoot, tiltingEdge))
      return PredictReturnType::noIntersection;

    // 3. convert into robot coordinates
    const Vector3f tiltingEdgeInCurrentRobot = useMatrixForUpdate.inverse() * tiltingEdge;

    // TODO do an approximation for the support foot. In case the tilting edge has no ground contact yet,
    // the real tiltingEdge will in the other direction of the support foot

    // 4. calc current angle
    const Vector3f currentDiff = currentComInRobot - tiltingEdgeInCurrentRobot;
    const Vector2a angle(-sgnPos(currentDiff.y()) * std::acos(currentDiff.z() / currentDiff.tail<2>().norm()),
                         sgnPos(currentDiff.x()) * std::acos(currentDiff.z() / Vector2f(currentDiff.x(), currentDiff.z()).norm()));

    // 5. calc acc
    const float length = currentDiff.tail<2>().norm();

    //
    const Vector2a acc(Angle::fromDegrees(-Constants::g / length * angle.x() * Constants::motionCycleTime),
                       Angle::fromDegrees(-Constants::g / length * angle.y() * Constants::motionCycleTime));

    // 6. calc velocity
    velocity = velocity + acc;

    // 7. update rotationMatrix
    useMatrixForUpdate.rotateY(velocity.y());
    useMatrixForUpdate.rotateX(velocity.x());

    if(startVelocity.x() * velocity.x() < 0.f)
      return PredictReturnType::velocityChanged;
    return PredictReturnType::none;
  };

  const unsigned int numberOfSteps = prediction ? predictionSteps : 1;
  PredictReturnType returnType = PredictReturnType::none;
  for(std::size_t i = 0; i < numberOfSteps; i++)
  {
    returnType = predictStep(rotationMatrix, model, velocity, isLeftPhase, theRobotModel);
    if(returnType == PredictReturnType::noIntersection)
      break;
  }
  turnPointMatrix = rotationMatrix;
  if(findTurnPoint)
  {
    if(returnType != PredictReturnType::noIntersection) // ignore velocity change if it happened here already too
    {
      ASSERT(turnPointSteps - predictionSteps > 0);
      for(std::size_t i = 0; i < turnPointSteps - predictionSteps; i++)
      {
        returnType = predictStep(turnPointMatrix, model, velocity, isLeftPhase, theRobotModel);
        if(returnType != PredictReturnType::none)
          break;
      }
    }
    lastTurnPointMatrixUpdate = theFrameInfo.time;
  }

  DEBUG_RESPONSE("module:RobotStableState:prediction")
  {
    PLOT("module:RobotStableState:prediction:x", -Angle(rotationMatrix.getXAngle()).toDegrees());
    PLOT("module:RobotStableState:prediction:y", -Angle(rotationMatrix.getYAngle()).toDegrees());
  }

  state.predictedTorsoRotation = rotationMatrix;
}

bool RobotStableStateProvider::getTiltingPoint(const Vector3f& currentCom, const Vector3f& lastCom, Vector3f& intersection3D)
{
  //code copied from FallDownStateProvider. direction is calculated different, rest is the same.
  const Vector2f direction = (currentCom - lastCom).head<2>();
  const Geometry::Line fallDirectionLine = Geometry::Line(supportFootCenter, direction.normalized());

  CROSS3D("module:RobotStableStateProvider:footMid", currentCom.x(), currentCom.y(), currentCom.z(), 3, 3, ColorRGBA::violet);
  LINE3D("module:RobotStableStateProvider:footMid", currentCom.x(), currentCom.y(), currentCom.z(), currentCom.x() + 10.f * direction.x(), currentCom.y() + 10.f * direction.y(), currentCom.z(), 3, ColorRGBA::violet);

  // search for the intersection with the support foot polygon and the line
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
       && (supportFootCenter - intersection2D).norm() > (supportFootCenter + fallDirectionLine.direction - intersection2D).norm())
    {
      LINE3D("module:RobotStableStateProvider:footMid", p1.x(), p1.y(), p1.z(), p2.x(), p2.y(), p2.z(), 3, ColorRGBA::blue);
      const float scalar = (intersection2D - base).norm() / dir.norm();
      intersection3D = p1 + scalar * (p2 - p1);
      CROSS3D("module:RobotStableStateProvider:footMid", intersection3D.x(), intersection3D.y(), intersection3D.z(), 3, 3, ColorRGBA::orange);

      return true;
    }
    LINE3D("module:RobotStableStateProvider:footMid", p1.x(), p1.y(), p1.z(), p2.x(), p2.y(), p2.z(), 3, ColorRGBA::green);
  }
  return false;
}

void RobotStableStateProvider::calcSupportPolygon(const bool isLeftPhase)
{
  supportPolygon = std::vector<Vector3f>();
  const Pose3f& useFoot = !isLeftPhase ? theRobotModel.soleLeft : theRobotModel.soleRight;
  const float left = !isLeftPhase ? theFootOffset.leftFoot.left : theFootOffset.rightFoot.left;
  const float right = !isLeftPhase ? theFootOffset.leftFoot.right : theFootOffset.rightFoot.right;
  supportPolygon.push_back(((useFoot + Vector3f(theFootOffset.forward, left, 0.f))).translation);
  supportPolygon.push_back(((useFoot + Vector3f(-theFootOffset.backward, left, 0.f))).translation);
  supportPolygon.push_back(((useFoot + Vector3f(-theFootOffset.backward, -right, 0.f))).translation);
  supportPolygon.push_back(((useFoot + Vector3f(theFootOffset.forward, -right, 0.f))).translation);

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
  supportFootCenter << x / area, y / area;
}
