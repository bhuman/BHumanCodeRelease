/**
 * @file Modules/MotionControl/ArmMotionEngine/PointAtEngine.h
 *
 * This file declares a module to provide the PointAtEngineOutput.
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once
#include "Tools/Module/Module.h"
#include "Tools/Math/Pose3f.h"
#include "Tools/Math/BHMath.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/MotionControl/PointAtEngineOutput.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Representations/MotionControl/ArmMotionSelection.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"

/**
 * A module that handles a point at request (part of ArmMotionRequest) and
 * provides the PointAtEngineOutput. The aim is to calculate arm joints in
 * a way that the selected arm is smoothly pointing at a specified 3d position
 * with the lower arm. The arm is always fully stretched.
 */
MODULE(PointAtEngine,
{,
  USES(MotionInfo),
  REQUIRES(ArmMotionSelection),
  REQUIRES(ArmMotionRequest),
  REQUIRES(JointAngles),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  REQUIRES(TorsoMatrix),
  PROVIDES(PointAtEngineOutput),
  DEFINES_PARAMETERS(
  {,
    (float)(sqr(100.f)) maxSquardDisToIgnorePoints, /**< The max distance of a point to the robot z-axis that will be ignored */
    (int)(100) stiffness, /**< The stiffness that will be set for all used joints */
    (float)(15.f) smoothnessValue, /**< The higher this value the slower the arm moves*/
  }),
});

class PointAtEngine : public PointAtEngineBase
{
public:
  /**
     * The update method that will be called by the motion process to fill
     * the representation PointAtEngineOutput.
     */
  void update(PointAtEngineOutput& pointAtEngineOutput) override;

private:
  /**
   * This method will be called by the update(PointAtEngineOutput) method
   * for each arm.
   *
   * If the arm is selected (ArmMotionSelection) for pointAt, this method
   * will calculate the right joint angles to point at a position specified
   * by theArmMotionRequest.pointToPointAt.
   */
  void update(Arms::Arm arm, PointAtEngineOutput& pointAtEngineOutput);

  /**
   * Calculates the inverse kinematic for the shoulder joints
   * to point at a specified position with the lower arm.
   * IMPORTANT:
   * This is NOT the same as InverseKinematic::calcShoulderJoints(...) !
   *
   * @param arm, the arm to calulate for
   * @param pointPosition, the 3d position to point at (relative to torso)
   * @param robotDimensions, theRobotDimensions
   * @param targetJointAngles, the JointAngles to save the results
   */
  void calcShoulderJoints(const Arms::Arm arm, const Vector3f& pointPosition, const RobotDimensions& robotDimensions,
                          JointAngles& targetJointAngles) const;
};
