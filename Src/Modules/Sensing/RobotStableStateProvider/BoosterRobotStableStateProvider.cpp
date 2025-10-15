/**
 * @file BoosterRobotStableStateProvider.h
 * Provides information about the predicted center of mass in the soles
 * @author Philip Reichenberg
 */

#include "BoosterRobotStableStateProvider.h"
#include "Debugging/DebugDrawings3D.h"
#include "Debugging/Plot.h"
#include "Framework/Settings.h"
#include "Math/Rotation.h"
#include "Streaming/Global.h"

MAKE_MODULE(BoosterRobotStableStateProvider);

void BoosterRobotStableStateProvider::update(RobotStableState& theRobotStableState)
{
  DECLARE_DEBUG_DRAWING3D("module:BoosterRobotStableStateProvider:footMid", "robot");

  const Vector3f soleLeft = theTorsoMatrix * theRobotModel.soleLeft.translation;
  const Vector3f soleRight = theTorsoMatrix * theRobotModel.soleRight.translation;
  const float supportheight = std::min(soleRight.z(), soleLeft.z());
  const Vector3f comInRobot = theTorsoMatrix * theFallDownState.predictedCom;
  const Vector3f comInTorso = theTorsoMatrix.inverse() * (Vector3f() << comInRobot.head<2>(), supportheight).finished();

  theRobotStableState.comInFloor = comInTorso;

  const Vector3f currentComInRobot = theTorsoMatrix * theRobotModel.centerOfMass;
  const Vector3f currentComInTorso = theTorsoMatrix.inverse() * (Vector3f() << currentComInRobot.head<2>(), supportheight).finished();

  CROSS3D("module:BoosterRobotStableStateProvider:footMid", comInTorso.x(), comInTorso.y(), comInTorso.z(), 3, 3, ColorRGBA::orange);

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

  const Angle yRotationChange = Vector2f(-currentComInTorso.z(), comInTorso.x() - currentComInTorso.x()).angle();
  const Angle xRotationChange = Vector2f(-currentComInTorso.z(), comInTorso.y() - currentComInTorso.y()).angle();
  theRobotStableState.predictedTorsoRotation.x() = theInertialData.angle.x() + xRotationChange;
  theRobotStableState.predictedTorsoRotation.y() = theInertialData.angle.y() + yRotationChange;

  theRobotStableState.predictRotation = [](const bool, const bool, const bool)
  {
  };

  theRobotStableState.getTurnPoint = [&theRobotStableState](const Legs::Leg leg)
  {
    return theRobotStableState.comInFeet[leg];
  };
}

float BoosterRobotStableStateProvider::calcPercentInFeet(const float refPoint, const float p0, const float p05, const float p075, const float p1)
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
