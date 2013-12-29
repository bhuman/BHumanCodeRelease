/**
 * @file JointDynamics.h
 * Declaration of class JointDynamics.
 * @author Felix Wenk
 */

#pragma once

#include "Tools/Math/Vector.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/MotionControl/IndykickRequest.h"

/**
 * @class JointDynamics
 * Representation of the dynamics: angle, angular velocity and angular acceleration for each joint.
 */
STREAMABLE(JointDynamics,
{
public:
  JointData asJointData() const
  {
    JointData jd;
    for(int i = 0; i < JointData::numOfJoints; ++i)
      jd.angles[i] = joint[i][0];
    return jd;
  },

  /**
   * x: angle, y: angular velocity: z: angular acceleration
   * Units: angle: rad
   *        angular velocity: rad / ms
   *        angular acceleration: rad / (ms)^2
   *        time: ms
   */
  (Vector3f[JointData::numOfJoints]) joint,
  (IndykickRequest, SupportLeg)(unspecified) supportLeg,
});

class FutureJointDynamics : public JointDynamics {};
