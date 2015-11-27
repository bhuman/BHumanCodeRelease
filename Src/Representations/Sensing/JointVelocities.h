/**
 * @author Arne Böckmann
 */

#pragma once

#include "Tools/Joints.h"
#include "Tools/Math/Angle.h"
#include "Tools/Streams/AutoStreamable.h"

/**
 * @struct JointVelocities
 * Contains approximated joint velocities
 */
STREAMABLE(JointVelocities,
{
public:
  void draw() const;
  ,
  /**approximated velocities of the joints in rad/s*/
  (Angle[Joints::numOfJoints]) velocities,
  (Angle[Joints::numOfJoints]) filteredVelocities,

  /**
   * Approximated accelerations of the joints in rad/s^2
   * @note The acceleration is not very exact because the cycleTime on the NAO is too high
   */
  (Angle[Joints::numOfJoints]) accelerations,
});
