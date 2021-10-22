/**
 * @file FootOffsetProvider.h
 *
 * This file declares a module that provides information for the distance between
 * the sole origin and to the four possible foot edges (forward, backward, left, right).
 *
 * Calibrates the positions of the soles of the feet. With this the robots know the real position of their feet relative to their model.
 * This model is based on the joint angles and the IMU. If one or both are wrong, at least the robot knows the correct feet position relative.
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Configuration/FootOffset.h"
#include "Representations/Sensing/FootSupport.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"

#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/Modify.h"
#include "Tools/Motion/InverseKinematic.h"
#include "Tools/Module/Module.h"
#include "Tools/Motion/MotionUtilities.h"
#include "Tools/Math/Rotation.h"

MODULE(FootOffsetProvider,
{,
  REQUIRES(FootSupport),
  REQUIRES(FrameInfo),
  REQUIRES(GroundContactState),
  REQUIRES(InertialData),
  REQUIRES(JointAngles),
  REQUIRES(KeyStates),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  REQUIRES(TorsoMatrix),
  USES(FootOffset),
  PROVIDES(FootOffset),
  PROVIDES(JointRequest),
  DEFINES_PARAMETERS(
  {,
    (float)(5.f) footSpeed, // how fast shall the body move (in mm/s)
    (float)(2.f) slowFootSpeedFactor, // how fast shall the body move when near the border (in mm/s)
    (Angle)(25_deg) slowThreshold, // when knees are below this, set slow speed
    (Angle)(0.1_deg) fastThreshold, // when knees are above this, set fast speed again
    (int)(100) legStiffness, // Leg stiffness
    (int)(20) nonLegStiffness, // Non leg stiffness
    (float)(2000.f) interpolationTime, // After a translation direction finished, move back to standing position
    (Angle)(5.5_deg) gyroThresholdX, // if the measured gyro is above this threshold, then the robot is starting to fall
    (Angle)(5.5_deg) gyroThresholdY, // if the measured gyro is above this threshold, then the robot is starting to fall
    (float)(-230.f) standHeight, // Stand height
    (float)(50) footOffsetY, // Side feet offset to the side
    (int)(200) timeUntilReturnStand, // Gyro value must be above the thresholds for at least this time
  }),
});

class FootOffsetProvider : public FootOffsetProviderBase
{
private:
  ENUM(ModuleState,
  {,
    none, // module went active, but no request yet
    moving, // feet are moving
    waitForFall, //
    returnStand, // return into a stand
    wait, // wait for request
  });
  ModuleState state; // State of the calibration
  JointRequest startJoints, // start joints
               targetJoints; // target joints
  FootOffset newFootOffsets; // new calibrated foot offsets
  std::vector<Vector2f> trajectory; // movement command for the legs for each measurement cycle
  Vector2f currentTrajectoryLeft, // translation (x and y) for the left foot
           currentTrajectoryRight;  // translation (x and y) for the right foot
  int trajectoryIndex; // calibration state index (0 == feet move forward, 1 == feet move backward, 2 == feet move to the left, 3 == feet move to the right)
  unsigned int timestamp; // timestamp used to calculate interpolation back into standing position
  unsigned int returnStandTimestamp; // timestamp when falling was expected
  Vector2f difLeftCopy; // Left foot difference when gyros exceeded the thresholds
  Vector2f difRightCopy; // Right foot difference when gyros exceeded the thresholds
  void update(FootOffset& footOffset) override;
  void update(JointRequest& jointRequest) override;

public:
  FootOffsetProvider();
};
