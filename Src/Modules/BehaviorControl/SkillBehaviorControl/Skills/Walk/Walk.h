/**
 * @file Walk.h
 *
 * This file declares complex skills related to walking.
 *
 * @author Thomas Röfer
 */

/**
 * This skill observes a point by walking next to it and look at it.
 * @param target The point to observe.
 */
option(ObservePoint, args((const Vector2f&) target));

/**
 * This skill turns the robot on the spot by a specified angle.
 * @param angle The angle relative to the rotation that the robot had when the skill started
 * @param margin The tolerance for the skill to be done
 */
option(TurnAngle, args((Angle) angle,
                       (Angle)(5_deg) margin));

/**
 * This skill turns the robot to look forward at a target.
 * @param target The position in robot relative coordinates to turn to
 * @param margin The tolerance for the skill to be done
 */
option(TurnToPoint, args((const Vector2f&) target,
                         (Angle)(5_deg) margin));

/**
 * This skill walks to a target that is modified by a potential field.
 * @param target The target pose in absolute field coordinates
 * @param straight Walk straight to the target (ballFactor, useRotation and rotation are ignored)
 * @param ballFactor How much should the robot turn to the ball when moving to its target in [0, 1]
 * @param useRotation Use \c rotation when close the target instead of the rotation to the ball
 * @param rotation The rotation relative to the robot if \c useRotation is true
 * @param targetOfInterest If set, it is used to decide how the body shall be orientated while walking, so the camera can see this target at all times. Target is in relative coordinates. (Only applies for side walk)
 */
option(WalkPotentialField, args((const Vector2f&) target,
                                (bool)(false) straight,
                                (float)(0.5f) ballFactor,
                                (bool)(false) useRotation,
                                (float)(0.f) rotation,
                                (const std::optional<Vector2f>&)({}) targetOfInterest));

/**
 * This skill walks to a target using intelligent path planning.
 * @param target The target pose in robot-relative coordinates
 * @param speed The walking speed as ratio of the maximum speed in [0, 1]
 * @param reduceWalkingSpeed How shall the walking speed be reduced?
 * @param rough Ignore obstacles if they prevent the robot from reaching its target
 * @param disableObstacleAvoidance Disables avoiding dynamic obstacles like robots and referees (keeps goal posts, own penalty area and field border avoidance enabled)
 * @param disableAligning Disables rotation to the walk target
 * @param disableStanding Disables standing at the target (i.e. if set to true, the robot will walk on the target spot)
 * @param disableAvoidFieldBorder Do not avoid the field border when walking.
 * @param targetOfInterest If set, it is used to decide how the body shall be orientated while walking, so the camera can see this target at all times. Target is in relative coordinates. (Only applies for side walk)
 * @param forceSideWalking If true, force sidewalking
 */
option(WalkToPoint, args((const Pose2f&) target,
                         (const Pose2f&)({1.f, 1.f, 1.f}) speed,
                         (ReduceWalkSpeedType)(ReduceWalkSpeedType::slow) reduceWalkingSpeed,
                         (bool)(false) rough,
                         (bool)(false) disableObstacleAvoidance,
                         (bool)(false) disableAligning,
                         (bool)(false) disableStanding,
                         (bool)(false) disableAvoidFieldBorder,
                         (const std::optional<Vector2f>&)({}) targetOfInterest,
                         (bool)(false) forceSideWalking));

/**
 * This skill walks to a target using intelligent path planning and shift the target pose to avoid obstacles
 * @param target The target pose in robot-relative coordinates
 * @param speed The walking speed as ratio of the maximum speed in [0, 1]
 * @param reduceWalkingSpeed How shall the walking speed be reduced?
 * @param rough Ignore obstacles if they prevent the robot from reaching its target
 * @param disableObstacleAvoidance Disables avoiding dynamic obstacles like robots and referees (keeps goal posts, own penalty area and field border avoidance enabled)
 * @param disableAligning Disables rotation to the walk target
 * @param disableStanding Disables standing at the target (i.e. if set to true, the robot will walk on the target spot)
 * @param disableAvoidFieldBorder Do not avoid the field border when walking.
 * @param targetOfInterest If set, it is used to decide how the body shall be orientated while walking, so the camera can see this target at all times. Target is in relative coordinates. (Only applies for side walk)
 */
option(WalkToPointObstacle,
       args((const Pose2f&) target,
            (const Pose2f&)({1.f, 1.f, 1.f}) speed,
            (ReduceWalkSpeedType)(ReduceWalkSpeedType::slow) reduceWalkingSpeed,
            (bool)(false) rough,
            (bool)(false) disableObstacleAvoidance,
            (bool)(false) disableAligning,
            (bool)(false) disableStanding,
            (bool)(false) disableAvoidFieldBorder,
            (const std::optional<Vector2f>&)({}) targetOfInterest,
            (bool)(false) forceSideWalking));

/**
 * This skill walks to a kick-off or penalty kick pose (i.e. in the ready state).
 * @param target The target pose in absolute field coordinates.
 * @param speed The walking speed as ratio of the maximum speed in [0, 1].
 * @param reduceWalkingSpeed How shall the walking speed be reduced?
 * @param rotateHead If true, rotates the head before walking.
 */
option(WalkToPointReady, args((const Pose2f&) target,
                              (const Pose2f&)({1.f, 1.f, 1.f}) speed,
                              (ReduceWalkSpeedType)(ReduceWalkSpeedType::slow) reduceWalkingSpeed));
