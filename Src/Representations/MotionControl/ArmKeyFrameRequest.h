/**
 * Request for the key frame engine.
 * @author <a href="mailto:simont@tzi.de>Simon Taddiken</a>
 */
#pragma once
#include "RobotParts/Arms.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/EnumIndexedArray.h"

/**
 * Class that represents the possible arm motions that can be requested from
 * the robot.
 */
STREAMABLE(ArmKeyFrameRequest,
{
  /** Existing arm motions. Ordering must match ordering in armMotionEngine.cfg */
  ENUM(ArmKeyFrameId,
  {,
    // IF U TOUCH THIS MAKE SURE U TAKE CARE TO SAVE THE ARMS CORRECTLY IN A FALLING CASE
    useDefault,  /**< No explicit arm motion, so WalkingEngine's arm angles will be used */
    back,
    raiseArm,    /**< Raise the arm. */
    keeperStand, /**< Arm position for the keeper when guarding the goal */
    waving1,
    waving2,
    waving,      /**< Arm position for anyPlaceDemo */
    wavingInitial, /**< initial Arm Position for waving in anyPlaceDemo*/
    arm45degreeUpSideways, /**< Move the Arm in a position 45 Degrees up sideways to the body of the robot*/
    arm45degreeDownSideways, /**< Move the Arm in a position 45 Degrees down sideways to the body of the robot*/
    armHorizontalSideways, /**< Move the Arm in a position horizontal sideways to the body of the robot*/
    armHandToChest, /**< Bent the arm at the ellbow, so the hand of the robot is in front of the chest while the shoulder doesn't move*/
    arm45degreeUpFront, /**< Move the arm in a position 45 Degree down in front of the body of the robot*/
    dynamicRefereePosition, /**< both arms in a horizontal sideways position, to a hands in front of the chest position two times*/
    reverse,     /**< Reverse current arm keyframe motion */
  });

  STREAMABLE(Arm,
  {,
    (ArmKeyFrameRequest::ArmKeyFrameId)(useDefault) motion, /**< Motion to execute */
    (bool)(false) fast, /**< Whether states should not be interpolated */
  }),

  (ENUM_INDEXED_ARRAY(Arm, Arms::Arm)) arms,
});
