/**
 * @file Tools/Motion/Transformation.h
 *
 * This file implements transformations for motion.
 *
 * @author Philip Reichenberg
 */

#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/MotionControl/OdometryData.h"

namespace Motion::Transformation
{
  /**
   * This function provides a 2D transformation from cognition coordinates relative to the robot
   * TO robot relative coordinates
   * IF the robot would execute a walking step of the size 0 rotation and 0 translation.
   *
   * @param torsoMatrix The torso matrix
   * @param robotModel The robot model
   * @param yHipOffset The y-hip offset of the legs
   * @param startOdometry The cognition odometry
   * @param targetOdometry The motion odometry
   * @param isLeftSwing Is the left foot the swing foot?
   */
  Pose2f getTransformationToZeroStep(const TorsoMatrix& torsoMatrix, const RobotModel& robotModel,
                                     const float yHipOffset, const Pose2f& startOdometry,
                                     const Pose2f& targetOdometry, const bool isLeftSwing);
}
