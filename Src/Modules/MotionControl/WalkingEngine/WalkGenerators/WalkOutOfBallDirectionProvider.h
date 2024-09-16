/**
 * @file WalkOutOfBallDirectionProvider.h
 *
 * This file declares a module that intentionally moves out of the ball direction
 *
 * @author Sina Schreiber
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/WalkOutOfBallDirection.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Sensing/TorsoMatrix.h"

MODULE(WalkOutOfBallDirectionProvider,
{,
  REQUIRES(BallSpecification),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(OdometryDataPreview),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  REQUIRES(RobotPose),
  REQUIRES(TorsoMatrix),
  REQUIRES(WalkGenerator),
  PROVIDES(WalkOutOfBallDirection),

  DEFINES_PARAMETERS(
  {,
    (float)(180.f) robotWidthThreshold,
    (int)(2000) annotationTime,
  }),
});

class WalkOutOfBallDirectionProvider : public WalkOutOfBallDirectionProviderBase
{
  /**
   * This method is called when the representation provided needs to be updated.
   * @param theInterceptBallGenerator The representation updated.
   */
  void update(WalkOutOfBallDirection& theWalkOutOfBallDirection) override;

  unsigned int annotationTimestamp = 0;
};
