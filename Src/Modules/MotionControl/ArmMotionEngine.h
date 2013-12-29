#pragma once

#include "Tools/Module/Module.h"
#include "Representations/MotionControl/ArmMotionEngineOutput.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Sensing/ArmContactModel.h"
#include "Representations/Sensing/FallDownState.h"
#include "Tools/MessageQueue/InMessage.h"
#include "ArmMotion.h"

MODULE(ArmMotionEngine)
  USES(MotionInfo)
  REQUIRES(ArmMotionRequest)
  REQUIRES(FrameInfo)
  REQUIRES(GameInfo)
  REQUIRES(RobotInfo)
  REQUIRES(ArmContactModel)
  REQUIRES(FilteredJointData)
  REQUIRES(HardnessSettings)
  REQUIRES(FallDownState)
  REQUIRES(GroundContactState)
  PROVIDES_WITH_MODIFY(ArmMotionEngineOutput)
  LOADS_PARAMETER(int, actionDelay);                    /**< delay in ms after which arm contact triggered motions can be started again. */
  LOADS_PARAMETER(int, targetTime);                     /**< time of how long arm stays in target position for contact triggered motions */
  LOADS_PARAMETER(std::vector<ArmMotion>, allMotions);  /**< contains the existing arm motions */
END_MODULE

/**
* Module which provides SpecialAction style arm motions which can be
* dynamically configured using the configuration file belongig to this
* module. Please see B-Human's 2013 Code release for details on how this
* module works and how to use it.
* @author <a href="mailto:simont@tzi.de">Simon Taddiken</a>
*/
class ArmMotionEngine : public ArmMotionEngineBase
{
public:
  void update(ArmMotionEngineOutput& armMotionEngineOutput);

  ArmMotionEngine();

private:

  /**
   * Class to encapsulate state information about one arm
   */
  class Arm
  {
  public:
    ArmMotionRequest::Arm id;  /**< ID of the motion this arm currently executes */
    int firstJoint;            /**< Index of the first joint belonging to this arm within the JointData's angle array */
    unsigned stateIndex;       /**< Index to identify the current set of angles the arm should head to within its currentMotion */
    int interpolationTime;     /**< Time in motion frames how long current motion is active */
    int targetTime;            /**< Time in motion frames how long this arm already stayed in its target position */
    int lastContactAction;     /**< Timestamp of when this arm was last moved due to arm contact detection */
    bool isMotionActive;       /**< Whether this arm currently performs a motion */
    bool fast;                 /**< If set to true, current motion will be performed without interpolation */
    bool autoReverse;          /**< If set to true, the arm will be moved to its default position once it stayed long enough in its target state */
    int autoReverseTime;       /**< Time in motion frames after which the arm should be automatically moved back to default position */
    bool contactTriggered;     /**< Whether the current arm motion was triggered by an arm contact */
    ArmMotion::ArmAngles interpolationStart; /**< Set of angles at which interpolation towards the next state began */
    ArmMotion currentMotion;   /**< The motion which is currently performed by this arm. Contains the actual target states in order they should be reached */

    Arm(ArmMotionRequest::Arm id = ArmMotionRequest::left, int firstJoint = 2) :
      id(id), firstJoint(firstJoint) , stateIndex(0), interpolationTime(0), isMotionActive(false), targetTime(0),
      lastContactAction(0) {}

    /**
     * Resets fields in this instance to start a new motion. The current joint information
     * are needed as starting point for the interpolation.
     * @param motion The actual motion to start.
     * @param fast Whether interpolation should be disabled for this motion.
     * @param autoReverse Whether this motion should be reversed automatically
     * @param autoReverseTime Time in motion frames after which the motion should be reversed.
     * @param currentJoints Current joint data.
     */
    void startMotion(ArmMotion motion, bool fast, bool autoReverse, int autoReverseTime, const FilteredJointData& currentJoints)
    {
      stateIndex = 0;
      interpolationTime = 1;
      targetTime = 0;
      isMotionActive = true;
      currentMotion = motion;
      this->fast = fast;
      this->autoReverse = autoReverse;
      this->autoReverseTime = autoReverseTime;

      for(unsigned i = 0; i < interpolationStart.angles.size(); ++i)
      {
        interpolationStart.angles[i] = currentJoints.angles[firstJoint + i];
      }
    }


  };

  Arm arms[2];                     /**< There are two arms :) */
  ArmMotion::ArmAngles defaultPos; /**< Default position of an arm. Will be read from configuration */

  /**
   * Performs the update step for a single arm. Checks whether a new motion for the provided arm is
   * possible or continues the currently active motion. If a new motion is possible, the desired
   * ArmMotion to perform is identified and then executed.
   *
   * @param arm For which arm current update step is performed
   * @param armMotionEngineOutput Output representation to be filled
   */
  void updateArm(Arm& arm, ArmMotionEngineOutput& armMotionEngineOutput);

  /**
   * Performs the next interpolation step for the provided arm.
   * @param arm The arm
   * @param target Target angles for the interpolation. Starting point for interpolation is retrieved
   *      from the Arm instance itself.
   * @param time Current interpolation time int motion steps
   * @param result Will contain the four interpolated result angles+hardness to be set.
   */
  void createOutput(Arm& arm, ArmMotion::ArmAngles target, int& time, ArmMotion::ArmAngles& result);

  /**
   * For a given arm, updates the engine's current output with the calculated target angles.
   * @param arm The arm
   * @param armMotionEngineOutput Output representation to be filled
   * @param values Angles+hardness to be set for this arm during this motion frame
   */
  void updateOutput(Arm& arm, ArmMotionEngineOutput& armMotionEngineOutput, ArmMotion::ArmAngles& values);

  /**
   * Determines whether a new motion can be started for the provided arm. That is the case if there
   * is currently no motion active, the game state is either PLAYING or READY, (the robot is walking or
   * standing) and he has ground contact. All of these single conditions must hold in order for a new
   * motion to be started.
   * @param arm Arm to check.
   */
  bool newMotionPossible(Arm& arm);
};