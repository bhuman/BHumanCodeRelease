/**
 * @file OdometryDataPreviewProvider
 * This file implements the provider for the preview of the odometryData
 */

#pragma once

#include "Framework/Module.h"
#include "Framework/Settings.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/SoleHeightDifference.h"
#include "Representations/Sensing/TorsoMatrix.h"

#include "Platform/File.h"
#include <CompiledNN2ONNX/CompiledNN.h>

MODULE(OdometryDataPreviewProvider,
{,
  USES(OdometryData),
  USES(MotionInfo),
  REQUIRES(FallDownState),
  REQUIRES(FrameInfo),
  REQUIRES(InertialData),
  REQUIRES(RobotModel),
  REQUIRES(SoleHeightDifference),
  REQUIRES(TorsoMatrix),
  PROVIDES(OdometryDataPreview),
  LOADS_PARAMETERS(
  {,
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

  Pose2f getOdometryOffset(OdometryDataPreview& odometryDataPreview, const RobotModel& theRobotModel, const Pose2f& lastOdometry, const bool overrideRotation, Vector2f& lastLeftSole, Vector2f& lastRightSole);

  Vector2f lastSoleLeft = Vector2f::Zero(); /**< Last frame left sole in last frame torso. */
  Vector2f lastSoleRight = Vector2f::Zero(); /**< Last frame right sole in last frame torso. */

  Vector2f lastRequestedSoleLeft = Vector2f::Zero(); /**< Last frame left sole in last frame torso. */
  Vector2f lastRequestedSoleRight = Vector2f::Zero(); /**< Last frame right sole in last frame torso. */

  bool receivedFirstData = false;

  const int motionCycleTimeMS = static_cast<int>(Global::getSettings().motionCycleTime * 1000.f);
};
