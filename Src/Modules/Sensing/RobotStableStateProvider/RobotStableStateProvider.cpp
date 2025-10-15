/**
 * @file RobotStableStateProvider.h
 * Provides information about the predicted center of mass in the soles
 * @author Philip Reichenberg
 */

#include "RobotStableStateProvider.h"
#include "Debugging/DebugDrawings3D.h"
#include "Debugging/Plot.h"
#include "Framework/Settings.h"
#include "Math/Rotation.h"
#include "Streaming/Global.h"

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

  theRobotStableState.predictRotation = [this, &theRobotStableState](const bool isLeftPhase, const bool prediction, const bool findTurnPoint)
  {
    if(lastUpdate == theFrameInfo.time && lastIsLeftPhase == isLeftPhase && !findTurnPoint)
      return;

    lastUpdate = theFrameInfo.time;
    lastIsLeftPhase = isLeftPhase;

    RobotModel lightModel = theRobotModel;
    if(useLightModel)
      lightModel.updateCenterOfMass(lightMassCalibration);
    theRobotStableState.lightCenterOfMass = lightModel.centerOfMass;

    const RotationMatrix tiltInTorso(Rotation::AngleAxis::unpack(Vector3f(-theInertialData.angle.x(), -theInertialData.angle.y(), 0.f)));
    predictRotation(theRobotStableState, tiltInTorso, lightModel, isLeftPhase, prediction, findTurnPoint);
    calculateCoMInPositionPercent(theRobotStableState);
    theRobotStableState.lastUpdate = theFrameInfo.time;
  };

  theRobotStableState.getTurnPoint = [this, &theRobotStableState](const Legs::Leg leg)
  {
    // Do not use this function when no prediction happened before!
    ASSERT(theFrameInfo.time == lastTurnPointMatrixUpdate);
    RobotStableState newState = theRobotStableState; // For CoM position
    newState.predictedTorsoRotationMatrix = turnPointMatrix;
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
  const Vector3f soleLeft = theRobotStableState.predictedTorsoRotationMatrix.inverse() * theRobotModel.soleLeft.translation;
  const Vector3f soleRight = theRobotStableState.predictedTorsoRotationMatrix.inverse() * theRobotModel.soleRight.translation;
  const float supportheight = std::min(soleRight.z(), soleLeft.z());
  const Vector3f comInTorso = theRobotStableState.predictedTorsoRotationMatrix * (Vector3f() << (theRobotStableState.predictedTorsoRotationMatrix.inverse() * theRobotStableState.lightCenterOfMass).head<2>(), supportheight).finished();

  float heelFeetInTorso[Legs::numOfLegs];
  float toeFeetInTorso[Legs::numOfLegs];
  float innerEdgeFeetInTorso[Legs::numOfLegs];
  float outerEdgeFeetInTorso[Legs::numOfLegs];
  float middleOfFeetInTorso[Legs::numOfLegs];

  heelFeetInTorso[Legs::left] = theRobotModel.soleLeft.translation.x() - theRobotDimensions.soleToBackEdgeLength;
  heelFeetInTorso[Legs::right] = theRobotModel.soleRight.translation.x() - theRobotDimensions.soleToBackEdgeLength;
  toeFeetInTorso[Legs::left] = theRobotModel.soleLeft.translation.x() + theRobotDimensions.soleToFrontEdgeLength;
  toeFeetInTorso[Legs::right] = theRobotModel.soleRight.translation.x() + theRobotDimensions.soleToFrontEdgeLength;

  innerEdgeFeetInTorso[Legs::left] = theRobotModel.soleLeft.translation.y() - theRobotDimensions.soleToInnerEdgeLength;
  outerEdgeFeetInTorso[Legs::left] = theRobotModel.soleLeft.translation.y() + theRobotDimensions.soleToOuterEdgeLength;
  middleOfFeetInTorso[Legs::left] = theRobotModel.soleLeft.translation.y();
  innerEdgeFeetInTorso[Legs::right] = theRobotModel.soleRight.translation.y() + theRobotDimensions.soleToInnerEdgeLength;
  outerEdgeFeetInTorso[Legs::right] = theRobotModel.soleRight.translation.y() - theRobotDimensions.soleToOuterEdgeLength;
  middleOfFeetInTorso[Legs::right] = theRobotModel.soleRight.translation.y();

  FOREACH_ENUM(Legs::Leg, leg)
  {
    const float sign = leg == Legs::right ? -1.f : 1.f;
    theRobotStableState.comInTorso[leg].forward = Rangef::ZeroOneRange().limit(
                                                    (comInTorso.x() - heelFeetInTorso[leg]) /
                                                    (toeFeetInTorso[leg] - heelFeetInTorso[leg]));

    theRobotStableState.comInTorso[leg].outerSide =
      calcPercentInFeet(sign * comInTorso.y(), 0.f, sign * innerEdgeFeetInTorso[leg], sign * middleOfFeetInTorso[leg], sign * outerEdgeFeetInTorso[leg]);

    theRobotStableState.comInTorso[leg].outerSideAbsolute = sign * comInTorso.y();

    const Pose3f& sole = leg == Legs::left ? theRobotModel.soleLeft : theRobotModel.soleRight;
    const Vector2f comInFeetRotated = (comInTorso.head<2>() - sole.translation.head<2>()).rotated(-sole.rotation.getZAngle());
    const Vector2f zeroPointInFeet = (-sole.translation.head<2>()).rotated(-sole.rotation.getZAngle());

    // To be honest: the difference between torso and feet space is not even that much (on the outer edge like 0.1 points difference).
    // And in such state the robot would probably fall anyway
    theRobotStableState.comInFeet[leg].forward = Rangef::ZeroOneRange().limit(
                                                   (comInFeetRotated.x() + theRobotDimensions.soleToBackEdgeLength) /
                                                   (theRobotDimensions.soleToBackEdgeLength + theRobotDimensions.soleToFrontEdgeLength));

    const float useInner = leg == Legs::left ? -theRobotDimensions.soleToInnerEdgeLength : -theRobotDimensions.soleToInnerEdgeLength;
    const float useOuter = leg == Legs::left ? theRobotDimensions.soleToOuterEdgeLength : theRobotDimensions.soleToOuterEdgeLength;
    theRobotStableState.comInFeet[leg].outerSide =
      calcPercentInFeet(sign * comInFeetRotated.y(), sign * zeroPointInFeet.y(), useInner, 0.f, useOuter);
    theRobotStableState.comInFeet[leg].outerSideAbsolute = sign * comInFeetRotated.y();
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
  Vector2a velocity(-theInertialData.gyro.x() * Global::getSettings().motionCycleTime, -theInertialData.gyro.y() * Global::getSettings().motionCycleTime); // the usage is not correct because of 3D rotation. But this is just an approximation!
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
    const Vector2a acc(Angle::fromDegrees(-Constants::g / length * angle.x() * Global::getSettings().motionCycleTime),
                       Angle::fromDegrees(-Constants::g / length * angle.y() * Global::getSettings().motionCycleTime));

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
    if(returnType != PredictReturnType::none)
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

  state.predictedTorsoRotationMatrix = rotationMatrix;
  state.predictedTorsoRotation = Vector2a(-rotationMatrix.getXAngle(), -rotationMatrix.getYAngle());
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
  const float left = !isLeftPhase ? theRobotDimensions.soleToOuterEdgeLength : theRobotDimensions.soleToInnerEdgeLength;
  const float right = !isLeftPhase ? theRobotDimensions.soleToInnerEdgeLength : theRobotDimensions.soleToOuterEdgeLength;
  supportPolygon.push_back(((useFoot + Vector3f(theRobotDimensions.soleToFrontEdgeLength, left, 0.f))).translation);
  supportPolygon.push_back(((useFoot + Vector3f(-theRobotDimensions.soleToBackEdgeLength, left, 0.f))).translation);
  supportPolygon.push_back(((useFoot + Vector3f(-theRobotDimensions.soleToBackEdgeLength, -right, 0.f))).translation);
  supportPolygon.push_back(((useFoot + Vector3f(theRobotDimensions.soleToFrontEdgeLength, -right, 0.f))).translation);

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
