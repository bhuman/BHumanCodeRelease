#pragma once

#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/Math/Eigen.h"

/**
 * @note predictions will only be calculated for the leg joints.
 *       Arm joints do not have a prediction
 */
STREAMABLE_WITH_BASE(JointDataPrediction, JointAngles,
{
  void draw() const;
  ,
  (bool) valid, /**< Prediction might be invalid for a few frames when starting bhuman */
  (float[Joints::numOfJoints]) velocities, // (in rad/s)
  (float[Joints::numOfJoints]) accelerations, // (in rad/s^2)
  (Vector3f)(Vector3f::Zero()) com, // Center of mass in torso coordinates
  (Vector3f)(Vector3f::Zero()) comVelocity, // Center of mass
  (Vector3f)(Vector3f::Zero()) comAcceleration, // Acceleration of the com
  (RobotModel) predictedRobotModel, // A RobotModel based on the predicted joint data
});
