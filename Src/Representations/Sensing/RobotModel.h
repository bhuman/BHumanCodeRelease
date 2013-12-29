/**
* @file RobotModel.h
*
* Declaration of class RobotModel
*
* @author <A href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</A>
*/

#pragma once

#include "Tools/Math/Pose3D.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/MassCalibration.h"

/**
 * @class RobotModel
 *
 * Contains information about extremities.
 */
STREAMABLE(RobotModel,
{
public:
  /**
   * Constructs the RobotModel from given joint data.
   * @param joints The joint data.
   * @param robotDimensions The dimensions of the robot.
   * @param massCalibration The mass calibration of the robot.
   */
  RobotModel(const JointData& joints, const RobotDimensions& robotDimensions, const MassCalibration& massCalibration);

  /**
   * Recalculates the RobotModel from given joint data.
   * @param joints The joint data.
   * @param robotDimensions The dimensions of the robot.
   * @param massCalibration The mass calibration of the robot.
   */
  void setJointData(const JointData& joints, const RobotDimensions& robotDimensions, const MassCalibration& massCalibration);

  /** Creates a 3-D drawing of the robot model. */
  void draw() const,

  (Pose3D[MassCalibration::numOfLimbs]) limbs, /**< Coordinate frame of the limbs of the robot relative to the robot's origin. */
  (Vector3<>) centerOfMass, /**< Position of the center of mass (center of gravity) relative to the robot's origin. */
  (float)(0) totalMass, /**< The mass of the robot. */
});
