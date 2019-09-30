#pragma once

#include "ArmKeyFrameMotion.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/StiffnessData.h"
#include "Representations/MotionControl/ArmKeyFrameEngineOutput.h"
#include "Representations/MotionControl/ArmMotionSelection.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Tools/Module/Module.h"

MODULE(ArmKeyFrameEngine,
{,
  USES(MotionInfo),
  REQUIRES(ArmMotionSelection),
  REQUIRES(JointAngles),
  REQUIRES(RobotInfo),
  REQUIRES(StiffnessSettings),
  PROVIDES(ArmKeyFrameEngineOutput),
  LOADS_PARAMETERS(
  {,
    (std::vector<ArmKeyFrameMotion>) allMotions,  /**< contains the existing arm motions */
  }),
});

/**
 * Module which provides SpecialAction style arm motions which can be
 * dynamically configured using the configuration file belonging to this
 * module. Please see B-Human's 2013 Code release for details on how this
 * module works and how to use it.
 * @author <a href="mailto:simont@tzi.de">Simon Taddiken</a>
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */
class ArmKeyFrameEngine : public ArmKeyFrameEngineBase
{
public:
  void update(ArmKeyFrameEngineOutput& armKeyFrameEngineOutput) override;

  ArmKeyFrameEngine();

private:
  const ArmKeyFrameRequest& theArmKeyFrameRequest = theArmMotionSelection.armKeyFrameRequest;
  /**
   * Class to encapsulate state information about one arm
   */
  class Arm
  {
  public:
    Arms::Arm id;  /**< ID of the motion this arm currently executes */
    int firstJoint;            /**< Index of the first joint belonging to this arm within the JointAngles' array */
    unsigned stateIndex;       /**< Index to identify the current set of angles the arm should head to within its currentMotion */
    int interpolationTime;     /**< Time in motion frames how long current motion is active */
    bool isLastMotionFinished; /**< Whether this arm currently performs a motion */
    bool fast;                 /**< If set to true, current motion will be performed without interpolation */
    bool wasActive;            /**< If the arm key frame engine was active last frame */
    ArmKeyFrameMotion::ArmAngles interpolationStart; /**< Set of angles at which interpolation towards the next state began */
    ArmKeyFrameMotion currentMotion;   /**< The motion which is currently performed by this arm. Contains the actual target states in order they should be reached */
    Arm(Arms::Arm id = Arms::left, int firstJoint = 2) :
      id(id), firstJoint(firstJoint), stateIndex(0), interpolationTime(0), isLastMotionFinished(true), wasActive(false)
    {}

    /**
     * Resets fields in this instance to start a new motion. The current joint information
     * are needed as starting point for the interpolation.
     * @param motion The actual motion to start.
     * @param fast Whether interpolation should be disabled for this motion.
     * @param currentJoints Current joint data.
     */
    void startMotion(const ArmKeyFrameMotion& motion, bool fast, const JointAngles& currentJoints)
    {
      stateIndex = 0;
      interpolationTime = 1;
      isLastMotionFinished = false;
      currentMotion = motion;
      this->fast = fast;

      for(unsigned i = 0; i < interpolationStart.angles.size(); ++i)
      {
        interpolationStart.angles[i] = currentJoints.angles[firstJoint + i];
        if(id == Arms::right &&
           (i + Joints::rShoulderPitch == Joints::rShoulderRoll ||
            i + Joints::rShoulderPitch == Joints::rElbowYaw ||
            i + Joints::rShoulderPitch == Joints::rElbowRoll ||
            i + Joints::rShoulderPitch == Joints::rWristYaw))
          interpolationStart.angles[i] *= -1.f;
      }
    }
  };

  Arm arms[2];                     /**< There are two arms :) */
  ArmKeyFrameMotion::ArmAngles defaultPos; /**< Default position of an arm. Will be read from configuration */

  /**
   * Performs the update step for a single arm. Checks whether a new motion for the provided arm is
   * possible or continues the currently active motion. If a new motion is possible, the desired
   * ArmKeyFrameMotion to perform is identified and then executed.
   *
   * @param arm For which arm current update step is performed
   * @param armKeyFrameEngineOutput Output representation to be filled
   */
  void updateArm(Arm& arm, ArmKeyFrameEngineOutput& armKeyFrameEngineOutput);

  /**
   * Performs the next interpolation step for the provided arm.
   * @param arm The arm
   * @param target Target angles for the interpolation. Starting point for interpolation is retrieved
   *      from the Arm instance itself.
   * @param time Current interpolation time int motion steps
   * @param result Will contain the four interpolated result angles+stiffness to be set.
   */
  void createOutput(Arm& arm, ArmKeyFrameMotion::ArmAngles target, int& time, ArmKeyFrameMotion::ArmAngles& result);

  /**
   * For a given arm, updates the engine's current output with the calculated target angles.
   * @param arm The arm
   * @param armKeyFrameEngineOutput Output representation to be filled
   * @param values Angles+stiffness to be set for this arm during this motion frame
   */
  void updateOutput(Arm& arm, ArmKeyFrameEngineOutput& armKeyFrameEngineOutput, ArmKeyFrameMotion::ArmAngles& values);
};
