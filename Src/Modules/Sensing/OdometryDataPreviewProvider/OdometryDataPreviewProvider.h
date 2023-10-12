/**
 * @file OdometryDataPreviewProvider
 * This file implements the provider for the preview of the odometryData
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/FootSupport.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/JointPlay.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"

MODULE(OdometryDataPreviewProvider,
{,
  USES(OdometryData),
  USES(MotionInfo),
  REQUIRES(FallDownState),
  REQUIRES(FootSupport),
  REQUIRES(FrameInfo),
  REQUIRES(InertialData),
  REQUIRES(JointPlay),
  REQUIRES(MassCalibration),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  REQUIRES(TorsoMatrix),
  PROVIDES(OdometryDataPreview),
  REQUIRES(OdometryDataPreview),
  PROVIDES(OdometryTranslationRequest),
  LOADS_PARAMETERS(
  {,
    (float) useMeasuredSwingAfterStepStartedTime, /**< Use the measured swing foot and not the planned one when a new walk phase started for this time. */
    (unsigned int) usePlannedIsLeftPhaseAfterThisSwitchTime, /**< If last support foot switch took more than this time, use the planned left phase flag. */
    (Vector2f) odometryWalkScaling, /**< While walking scale the odometry with these factors. */
  }),
});

class OdometryDataPreviewProvider : public OdometryDataPreviewProviderBase
{
private:
  /**
   * Updates the OdometryDataPreview representation.
   * @param odometryDataPreview The preview of the odometry
   */
  void update(OdometryDataPreview& odometryDataPreview) override;

  void update(OdometryTranslationRequest& theOdometryTranslationRequest) override { theOdometryTranslationRequest = internalOdometryTranslationRequest; };

  Pose2f getOdometryOffset(const RobotModel& theRobotModel, const Pose2f& lastOdometry, const bool overrideRotation, Pose2f& lastLeftSole, Pose2f& lastRightSole);

  Pose2f lastSoleLeft; /**< Last frame left sole in last frame torso. */
  Pose2f lastSoleRight; /**< Last frame right sole in last frame torso. */

  Pose2f lastRequestedSoleLeft; /**< Last frame left sole in last frame torso. */
  Pose2f lastRequestedSoleRight; /**< Last frame right sole in last frame torso. */

  OdometryTranslationRequest internalOdometryTranslationRequest;
};
