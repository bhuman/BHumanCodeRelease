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
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/WalkOutOfBallDirection.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Representations/Modeling/RobotPose.h"

MODULE(WalkOutOfBallDirectionProvider,
{,
  REQUIRES(BallSpecification),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(OdometryDataPreview),
  REQUIRES(RobotPose),
  REQUIRES(WalkGenerator),
  PROVIDES(WalkOutOfBallDirection),

  LOADS_PARAMETERS(
  {,
    (float) robotWidthThreshold, /**< Assumed width of the robot. */
    (int) annotationTime,
    (float) ballLineDistance, /**< Distance to keep away from the ball roll path. */
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
