/*
 * @file CartTableController.cpp
 * This file implements the methods of the cart table zmp preview controller class.
 * @author Felix Wenk
 */

#include "CartTableController.h"
#include <algorithm>
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Sensing/OrientationData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/InverseKinematic.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/Modify.h"

template <int n>
CartTableZMPPreviewController<n>::CartTableZMPPreviewController(const MassCalibration& theMassCalibration,
                                                                const RobotDimensions& theRobotDimensions,
                                                                const OrientationData& theOrientationData,
                                                                const float motionCycleTime)
: deltaT(motionCycleTime),
  comHeight(200.0f),
  zmpErrorSum(0.0f, 0.0f),
  theMassCalibration(theMassCalibration), theRobotDimensions(theRobotDimensions), theOrientationData(theOrientationData)
{
  InTextFile stream("carttable-riccatisolution.cfg");
  stream >> P;
  stream >> R;

  /* COM height and motion cycle time used to calculate the Riccati equation
   * must match the actual motion cycle time and COM height. */
  float comHeightTest;
  stream >> comHeightTest;
  ASSERT(comHeightTest == comHeight);
  float motionCycleTimeTest;
  stream >> motionCycleTimeTest;
  ASSERT(motionCycleTimeTest == motionCycleTime);

  /* Assemble matrices. */
  const float zg = comHeight / 9.81e-3f;
  Matrix4x1f Bt;
  Bt.c[0][0] = sqr(deltaT) * deltaT / 6.0f - zg * deltaT;
  Bt.c[0][1] = sqr(deltaT) * deltaT / 6.0f;
  Bt.c[0][2] = sqr(deltaT) / 2.0f;
  Bt.c[0][3] = deltaT;
  const Matrix1x4f BtTrans = Bt.transpose();

  Matrix4x1f It;
  It[0][0] = 1.0f;
  It[0][1] = 0.0f;
  It[0][2] = 0.0f;
  It[0][3] = 0.0f;

  Matrix4x3f Ft;
  Ft[0][0] = 1.0f; Ft[1][0] = deltaT; Ft[2][0] = sqr(deltaT) / 2.0f - zg;
  Ft[0][1] = 1.0f; Ft[1][1] = deltaT; Ft[2][1] = sqr(deltaT) / 2.0f;
  Ft[0][2] = 0.0f; Ft[1][2] = 1.0f;   Ft[2][2] = deltaT;
  Ft[0][3] = 0.0f; Ft[1][3] = 0.0f;   Ft[2][3] = 1.0f;

  const Matrix4x4f At(It.c[0], Ft.c[0], Ft.c[1], Ft.c[2]);

  /* Calculate integral and state gain matrices. */
  const Matrix1x1f bpb = BtTrans * P * Bt;
  const float denum = R + bpb[0][0];
  const Matrix1x4f gainBase = (BtTrans * P) / denum;
  const Matrix1x1f integralGainMatrix = gainBase * It;
  integralGain = integralGainMatrix[0][0];
  stateGain = gainBase * Ft;

  /* Calculate preview gain matrices */
  const Matrix4x4f Atc = At - Bt * gainBase * At;
  const Matrix4x4f AtcTranspose = Atc.transpose();
  Matrix4x1f Xt = AtcTranspose * P * (It * (-1.0f));

  previewGain[0] = 0.0f; // Index 0 contains no valid preview gain!
  previewGain[1] = -integralGain;
  for(int l = 2; l < n + frameDelay; ++l)
  {
    const Matrix1x1f previewGainMatrix = BtTrans / denum * Xt;
    previewGain[l] = previewGainMatrix[0][0];
    Xt = AtcTranspose * Xt;
  }
}

template <int n> void CartTableZMPPreviewController<n>::generateSupportLegAngles(const float yRotationFraction,
                                                                                 const float xRotationFraction,
                                                                                 const RingBuffer<Vector2f, n>& zmpReference,
                                                                                 const Vector3<>& estimatedZmp,
                                                                                 const Pose3D& kickFoot,
                                                                                 const bool useSupportFootCoordinateSystem,
                                                                                 JointData& myJointData,
                                                                                 const Vector3<>& delayedComPositionInSupportFoot,
                                                                                 const Vector3<>& delayedComVelocityInSupportFoot,
                                                                                 const Vector3<>& delayedComAccelerationInSupportFoot,
                                                                                 Vector2<>& demandedTorsoAngularVelocity)
{
  calculateNextCom(zmpReference, estimatedZmp, delayedComPositionInSupportFoot, delayedComVelocityInSupportFoot, delayedComAccelerationInSupportFoot);
  generateLegAnglesForCenterOfMassOrientation(yRotationFraction, xRotationFraction, myJointData,
                                       kickFoot, useSupportFootCoordinateSystem, comPositionInSupportFoot, demandedTorsoAngularVelocity);
}

template <int n> void CartTableZMPPreviewController<n>::calculateNextCom(const RingBuffer<Vector2f, n>& zmpReference, const Vector3<>& estimatedZmp,
                                                                         const Vector3<>& delayedComPositionInSupportFoot,
                                                                         const Vector3<>& delayedComVelocityInSupportFoot,
                                                                         const Vector3<>& delayedComAccelerationInSupportFoot)
{
  static const float zg = comHeight / 9.81e-3f;
  const bool bufferFull = zmpRefHistory.getNumberOfEntries() == frameDelay;

  zmpErrorSum.x = estimatedZmp.x - zmpReference[n - 1].x + 0.5f * zmpErrorSum.x;
  zmpErrorSum.y = estimatedZmp.x - zmpReference[n - 1].y + 0.5f * zmpErrorSum.y;

  float actionX = -integralGain * zmpErrorSum.x;
  actionX -= (stateGain[0][0] * comPositionInSupportFoot.x + stateGain[1][0] * comVelocityInSupportFoot.x + stateGain[2][0] * comAccelerationInSupportFoot.x);
  float actionY = -integralGain * zmpErrorSum.y;
  actionY -= (stateGain[0][0] * comPositionInSupportFoot.y + stateGain[1][0] * comVelocityInSupportFoot.y + stateGain[2][0] * comAccelerationInSupportFoot.y);

  for(int j = 1; j < n; ++j)
  {
    actionX -= previewGain[j] * zmpReference[n - 1 - j].x;
    actionY -= previewGain[j] * zmpReference[n - 1 - j].y;
  }

  comPositionInSupportFoot.x += comVelocityInSupportFoot.x * deltaT + comAccelerationInSupportFoot.x * sqr(deltaT) / 2.0f
  + actionX * sqr(deltaT) * deltaT / 6.0f;
  comPositionInSupportFoot.y += comVelocityInSupportFoot.y * deltaT + comAccelerationInSupportFoot.y * sqr(deltaT) / 2.0f
  + actionY * sqr(deltaT) * deltaT / 6.0f;
  comVelocityInSupportFoot.x += comAccelerationInSupportFoot.x * deltaT + actionX * sqr(deltaT) / 2.0f;
  comVelocityInSupportFoot.y += comAccelerationInSupportFoot.y * deltaT + actionY * sqr(deltaT) / 2.0f;
  comAccelerationInSupportFoot.x += actionX * deltaT;
  comAccelerationInSupportFoot.y += actionY * deltaT;

  if(bufferFull)
    plotModelError(estimatedZmp, delayedComPositionInSupportFoot, delayedComVelocityInSupportFoot, delayedComAccelerationInSupportFoot);

  zmpRefHistory.add(Vector2<>(zmpReference[n - 1].x, zmpReference[n - 1].y));
  modelZmpHistory.add(Vector2<>(comPositionInSupportFoot.x - zg * comAccelerationInSupportFoot.x,
                                comPositionInSupportFoot.y - zg * comAccelerationInSupportFoot.y));
  modelComPositionHistory.add(comPositionInSupportFoot);
  modelComVelocitiesHistory.add(comVelocityInSupportFoot);
  modelComAccelerationsHistory.add(comAccelerationInSupportFoot);
}

template <int n> void CartTableZMPPreviewController<n>::generateLegAnglesForCenterOfMassOrientation(const float yRotationFraction,
                                                                                         const float xRotationFraction,
                                                                                         JointData& myJointData,
                                                                                         const Pose3D& kickFoot2,
                                                                                         const bool useSupportFootCoordinateSystem,
                                                                                         const Vector3<>& desiredCom,
                                                                                         Vector2<>& demandedTorsoAngularVelocity)
{
  bool reachable;
  const bool left = supportFoot == MassCalibration::footLeft;
  const float ratio = left ? 1.0f : 0.0f;

  PLOT("module:IndykickEngine:carttable:kickfoot:rotation:x", toDegrees(kickFoot2.rotation.getXAngle()));
  PLOT("module:IndykickEngine:carttable:kickfoot:rotation:y", toDegrees(kickFoot2.rotation.getYAngle()));
  PLOT("module:IndykickEngine:carttable:kickfoot:rotation:z", toDegrees(kickFoot2.rotation.getZAngle()));

  Pose3D originInSupportFoot = supportFootInOrigin.invert();

  /* Calculate torso orientation error and remove that from kick foot orientation. TODO: Move to kick engine. */
  RotationMatrix kickFootOrientationCorrection(Vector3<>(1.0f, 0.0f, 0.0f), Vector3<>(0.0f, 1.0f, 0.0f), Vector3<>(0.0f, 0.0f, 1.0f));
  const float orientationBalanceTranslation = 7.0f;
  if(orientationBuffer.getNumberOfEntries() == frameDelay)
  {
    const Vector2<>& delayedTorsoOrientation = orientationBuffer[frameDelay - 1];

    const Vector2<> measuredTorsoOrientation(std::atan2(theOrientationData.rotation.c1.z, theOrientationData.rotation.c2.z),
                                             std::atan2(-theOrientationData.rotation.c0.z, theOrientationData.rotation.c2.z));

    Vector2<> torsoOrientationError = delayedTorsoOrientation - measuredTorsoOrientation;
    torsoOrientationError.x = normalize(torsoOrientationError.x);
    torsoOrientationError.y = normalize(torsoOrientationError.y);
    float kickFootZ = kickFoot2.translation.z;
    if(!useSupportFootCoordinateSystem)
    {
      Pose3D kickFootInSupportFoot(originInSupportFoot);
      kickFootInSupportFoot.conc(kickFoot2);
      kickFootZ = kickFootInSupportFoot.translation.z;
    }
    const float correctionFactor = std::min(kickFootZ / orientationBalanceTranslation, 1.0f);

    PLOT("module:IndykickEngine:carttable:measuredxzrotation", toDegrees(measuredTorsoOrientation.y));
    PLOT("module:IndykickEngine:carttable:demandedxzrotation", toDegrees(delayedTorsoOrientation.y));
    PLOT("module:IndykickEngine:carttable:orientationerror:y", toDegrees(torsoOrientationError.y));
    kickFootOrientationCorrection.rotateY(torsoOrientationError.y * correctionFactor);
  }

  const Pose3D kickFootPose(useSupportFootCoordinateSystem ? kickFootOrientationCorrection * kickFoot2.rotation : kickFoot2.rotation * kickFootOrientationCorrection, kickFoot2.translation);

  PLOT("module:IndykickEngine:carttable:correctedkickfoot:rotation:x", toDegrees(kickFootPose.rotation.getXAngle()));
  PLOT("module:IndykickEngine:carttable:correctedkickfoot:rotation:y", toDegrees(kickFootPose.rotation.getYAngle()));
  PLOT("module:IndykickEngine:carttable:correctedkickfoot:rotation:z", toDegrees(kickFootPose.rotation.getZAngle()));

  Pose3D kickFootInOrigin;
  if(useSupportFootCoordinateSystem)
    kickFootInOrigin = supportFootInOrigin;
  kickFootInOrigin.conc(kickFootPose);
  Pose3D *poseLeft = left ? &supportFootInOrigin : &kickFootInOrigin;
  Pose3D *poseRight = left ? &kickFootInOrigin : &supportFootInOrigin;

  reachable = InverseKinematic::calcLegJoints(*poseLeft, *poseRight, myJointData, theRobotDimensions, ratio);
  if(!reachable)
  {
    printf("Rotation X: %f\tRotation Y: %f\t Rotation Z: %f\n", toDegrees(originInSupportFoot.rotation.getXAngle()),
           toDegrees(originInSupportFoot.rotation.getYAngle()), toDegrees(originInSupportFoot.rotation.getZAngle()));
    OUTPUT(idText, text, "Cart table: Unreachable kick leg position.");
  }
  RobotModel myRobotModel(myJointData, theRobotDimensions, theMassCalibration);
  Vector3<> com = originInSupportFoot * myRobotModel.centerOfMass;

  const float y = yRotationFraction * com.y + (1.0f - yRotationFraction) * desiredCom.y;
  const float x = xRotationFraction * com.x + (1.0f - xRotationFraction) * desiredCom.x;
  for(int i = 0; i < 7; ++i)
  {
    const float deltaY = y - com.y;
    const float deltaX = x - com.x;
    originInSupportFoot.translation.x += deltaX;
    originInSupportFoot.translation.y += deltaY;
    supportFootInOrigin = originInSupportFoot.invert();
    if(useSupportFootCoordinateSystem)
    {
      kickFootInOrigin = supportFootInOrigin;
      kickFootInOrigin.conc(kickFootPose);
    }
    reachable = InverseKinematic::calcLegJoints(*poseLeft, *poseRight, myJointData, theRobotDimensions, ratio);
    if(!reachable)
      OUTPUT(idText, text, "Cart table: Unreachable kick leg position.");
    myRobotModel.setJointData(myJointData, theRobotDimensions, theMassCalibration);
    com = originInSupportFoot * myRobotModel.centerOfMass;
  }

  float z2 = std::sqrt(sqr(com.y) + sqr(com.z) - sqr(desiredCom.y));
  float angle = std::atan2(com.y, com.z) - std::atan2(desiredCom.y, z2);
  //  while(abs(angle) > 0.0005f)
  for(int i = 0; (i < 10) && (std::abs(angle) > 0.0005f); ++i)
  {
    originInSupportFoot.rotation = RotationMatrix::fromRotationX(angle) * originInSupportFoot.rotation;
    supportFootInOrigin = originInSupportFoot.invert();
    if(useSupportFootCoordinateSystem)
    {
      kickFootInOrigin = supportFootInOrigin;
      kickFootInOrigin.conc(kickFootPose);
    }
    reachable = InverseKinematic::calcLegJoints(*poseLeft, *poseRight, myJointData, theRobotDimensions, ratio);
    if(!reachable)
      OUTPUT(idText, text, "Cart table: Unreachable kick leg position.");
    myRobotModel.setJointData(myJointData, theRobotDimensions, theMassCalibration);
    com = originInSupportFoot * myRobotModel.centerOfMass;
    z2 = std::sqrt(sqr(com.y) + sqr(com.z) - sqr(desiredCom.y));
    angle = std::atan2(com.y, com.z) - std::atan2(desiredCom.y, z2);
  }

  z2 = std::sqrt(sqr(com.x) + sqr(com.z) - sqr(desiredCom.x));
  angle = std::atan2(com.x, com.z) - std::atan2(desiredCom.x, z2);
  for(int i = 0; (i < 10) && (std::abs(angle) > 0.0005f); ++i)
  {
    originInSupportFoot.rotation = RotationMatrix::fromRotationY(-angle) * originInSupportFoot.rotation;
    supportFootInOrigin = originInSupportFoot.invert();
    if(useSupportFootCoordinateSystem)
    {
      kickFootInOrigin = supportFootInOrigin;
      kickFootInOrigin.conc(kickFootPose);
    }
    reachable = InverseKinematic::calcLegJoints(*poseLeft, *poseRight, myJointData, theRobotDimensions, ratio);
    if(!reachable)
      OUTPUT(idText, text, "Cart table: Unreachable kick leg position.");
    myRobotModel.setJointData(myJointData, theRobotDimensions, theMassCalibration);
    com = originInSupportFoot * myRobotModel.centerOfMass;
    z2 = std::sqrt(sqr(com.x) + sqr(com.z) - sqr(desiredCom.x));
    angle = std::atan2(com.x, com.z) - std::atan2(desiredCom.x, z2);
  }

  for(int i = 0; i < 7; ++i)
  {
    const Vector3<> deltaCom = desiredCom - com;
    originInSupportFoot.translation += deltaCom;
    supportFootInOrigin = originInSupportFoot.invert();
    if(useSupportFootCoordinateSystem)
    {
      kickFootInOrigin = supportFootInOrigin;
      kickFootInOrigin.conc(kickFootPose);
    }
    reachable = InverseKinematic::calcLegJoints(*poseLeft, *poseRight, myJointData, theRobotDimensions, ratio);
    if(!reachable)
      OUTPUT(idText, text, "Cart table: Unreachable kick leg position.");
    myRobotModel.setJointData(myJointData, theRobotDimensions, theMassCalibration);
    com = originInSupportFoot * myRobotModel.centerOfMass;
  }

  /* Save demanded torso orientation. */
  orientationBuffer.add(Vector2<>(std::atan2(originInSupportFoot.rotation.c1.z, originInSupportFoot.rotation.c2.z),
                                  std::atan2(-originInSupportFoot.rotation.c0.z, originInSupportFoot.rotation.c2.z)));

  /* Calculate demanded angular velocities of the torso. */
  const RotationMatrix rotationDelta = originInSupportFoot.rotation.invert() * lastTorsoRotation;
  const Vector3<> rotationDeltaPole = rotationDelta * Vector3<>(0.0f, 0.0f, 1.0f);
  demandedTorsoAngularVelocity.x = std::atan2(rotationDeltaPole.y, rotationDeltaPole.z) / 0.01f;
  demandedTorsoAngularVelocity.y = std::atan2(rotationDeltaPole.x, rotationDeltaPole.z) / 0.01f;

  lastTorsoRotation = originInSupportFoot.rotation;
}

template <int n> void CartTableZMPPreviewController<n>::reset(const bool leftSupport,
                                                              const Vector3<>& comPositionInSupportFoot,
                                                              const Vector3<>& comVelocityInSupportFoot,
                                                              const Vector3<>& comAccelerationInSupportFoot,
                                                              const RobotModel& initialRobotModel)
{
  zmpErrorSum.x = 0.0f;
  zmpErrorSum.y = 0.0f;
  this->comPositionInSupportFoot = comPositionInSupportFoot;
  this->comVelocityInSupportFoot = Vector2<>(comVelocityInSupportFoot.x, comVelocityInSupportFoot.y);
  this->comAccelerationInSupportFoot = Vector2<>(comAccelerationInSupportFoot.x, comAccelerationInSupportFoot.y);
  supportFoot = leftSupport ? MassCalibration::footLeft : MassCalibration::footRight;
  supportFootInOrigin = initialRobotModel.limbs[supportFoot];
  lastTorsoRotation = supportFootInOrigin.rotation.invert();
  zmpRefHistory.init();
  maxComAcceleration = 0.0f;
  maxComVelocity = 0.0f;
  maxComPositionError = 0.0f;
  orientationBuffer.init();
}

template <int n> void CartTableZMPPreviewController<n>::plotModelError(const Vector3<>& estimatedZmp,
                                                                       const Vector3<>& delayedComPositionInSupportFoot,
                                                                       const Vector3<>& delayedComVelocityInSupportFoot,
                                                                       const Vector3<>& delayedComAccelerationInSupportFoot)
{
  const Vector2<>& modelZmp = modelZmpHistory[frameDelay - 1];
  const Vector3<>& modelComPosition = modelComPositionHistory[frameDelay - 1];
  const Vector2<>& modelComVelocity = modelComVelocitiesHistory[frameDelay - 1];
  const Vector2<>& modelComAcceleration = modelComAccelerationsHistory[frameDelay - 1];

  if(modelComAcceleration.abs() > maxComAcceleration)
    maxComAcceleration = modelComAcceleration.abs();
  if(modelComVelocity.abs() > maxComVelocity)
    maxComVelocity = modelComVelocity.abs();


  const Vector2<> zmpError(estimatedZmp.x - modelZmp.x, estimatedZmp.y - modelZmp.y);
  const Vector2<> comPositionError = Vector2<>(delayedComPositionInSupportFoot.x - modelComPosition.x,
                                               delayedComPositionInSupportFoot.y - modelComPosition.y);
  const Vector2<> comVelocityError = Vector2<>(delayedComVelocityInSupportFoot.x, delayedComVelocityInSupportFoot.y) - modelComVelocity;
  const Vector2<> comAccelerationError = Vector2<>(delayedComAccelerationInSupportFoot.x, delayedComAccelerationInSupportFoot.y) - modelComAcceleration;

  if(comPositionError.abs() > maxComPositionError)
    maxComPositionError = comPositionError.abs();

  PLOT("module:IndykickEngine:carttable:zmpError", zmpError.abs());
  PLOT("module:IndykickEngine:carttable:comPositionError", comPositionError.abs());
  PLOT("module:IndykickEngine:carttable:comVelocityError", comVelocityError.abs());
  PLOT("module:IndykickEngine:carttable:comAccelerationError", comAccelerationError.abs());
}

template class CartTableZMPPreviewController<50>;
