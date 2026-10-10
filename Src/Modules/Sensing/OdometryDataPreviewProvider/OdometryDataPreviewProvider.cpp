/**
 * @file OdometryDataPreviewProvider
 * This file implements the provider for the preview of the odometryData
 */

#include "OdometryDataPreviewProvider.h"
#include "Math/Rotation.h"
#include "Framework/Settings.h"
#include "Streaming/Global.h"
#include <filesystem>

MAKE_MODULE(OdometryDataPreviewProvider);

using namespace NeuralNetworkONNX;

void OdometryDataPreviewProvider::update(OdometryDataPreview& odometryDataPreview)
{
  static_cast<OdometryData&>(odometryDataPreview) = theOdometryData;
  const Pose2f odometryOffset = getOdometryOffset(odometryDataPreview, theRobotModel, theOdometryData, true, lastSoleLeft, lastSoleRight);
  odometryDataPreview += odometryOffset;
  odometryDataPreview.odometryChange = odometryOffset;
}

Pose2f OdometryDataPreviewProvider::getOdometryOffset(OdometryDataPreview& odometryDataPreview, const RobotModel& theRobotModel, const Pose2f& lastOdometry, const bool overrideRotation, Vector2f& lastLeftSole, Vector2f& lastRightSole)
{
  Pose2f odometryOffset;

  bool skipOdoUpdate = false;

  const Pose3f currentLeftSole3D = theTorsoMatrix * theRobotModel.soleLeft;
  const Pose3f currentRightSole3D = theTorsoMatrix * theRobotModel.soleRight;
  const Pose2f currentSoleLeft(currentLeftSole3D.rotation.getZAngle(), currentLeftSole3D.translation.head<2>());
  const Pose2f currentSoleRight(currentRightSole3D.rotation.getZAngle(), currentRightSole3D.translation.head<2>());

  const bool useIsLeftPhase = theSoleHeightDifference.difference > 0.f;
  skipOdoUpdate = std::abs(theSoleHeightDifference.difference) < (theMotionInfo.executedPhase == MotionPhase::walk ? 1.f : 5.f);

  const Pose2f& lastSupportFoot = useIsLeftPhase ? lastRightSole : lastLeftSole;
  const Pose2f& lastSwingFoot = useIsLeftPhase ? lastLeftSole : lastRightSole;

  const Pose2f& currentSupportFoot = useIsLeftPhase ? currentSoleRight : currentSoleLeft;
  const Pose2f& currentSwingFoot = useIsLeftPhase ? currentSoleLeft : currentSoleRight;

  const Vector2f lastRef = lastSwingFoot.translation - lastSupportFoot.translation;
  const Vector2f currentRef = currentSwingFoot.translation - currentSupportFoot.translation;

  if(!skipOdoUpdate)
  {
    odometryOffset.translation = currentRef - lastRef;
    odometryOffset.translation *= 0.5f;

    odometryOffset.translation.x() *= odometryWalkScaling.x();
    odometryOffset.translation.y() *= odometryWalkScaling.y();
  }
  else
    odometryOffset = Pose2f();

  lastLeftSole = currentSoleLeft.translation;
  lastRightSole = currentSoleRight.translation;

  if(!receivedFirstData && theFrameInfo.time != 0)
  {
    odometryDataPreview.rotationOffsetAtStart = Rotation::Euler::getZAngle(theInertialData.orientation3D);
    receivedFirstData = true;
  }

  // Construct odometry update for this frame.
  if(theFallDownState.state == FallDownState::falling || theFallDownState.state == FallDownState::fallen)
    odometryOffset.rotation = 0_deg; // Postpone rotation change until being upright again to avoid huge variances in the self locator.
  else if(overrideRotation)
    odometryOffset.rotation = Angle::normalize(Rotation::Euler::getZAngle(theInertialData.orientation3D) - odometryDataPreview.rotationOffsetAtStart - lastOdometry.rotation);

  return odometryOffset;
}
