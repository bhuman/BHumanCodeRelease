/**
 * @file Output.h
 *
 * This file declares skills that directly set the behavior's output.
 *
 * @author Thomas Röfer
 */

/**
 * This skill adds an annotation if it differs from the one that has been added in the last frame.
 * @param annotation The annotation message
 */
option(Annotation, args((const std::string&) annotation));

/** This skill sets the calibrationFinished member of the BehaviorStatus. */
option(CalibrationFinished);

/**
 * This skill lets the robot execute a diving motion.
 * @param request The dive request
 */
option(Dive, args((MotionRequest::Dive::Request) request));

/**
 * This skill dribbles the ball.
 * @param targetDirection The (robot-relative) direction in which the ball should go
 * @param speed The walking speed as ratio of the maximum speed in [0, 1]
 * @param obstacleAvoidance The obstacle avoidance request
 * @param alignPrecisely Whether the robot should align more precisely than usual
 * @param kickLength The distance the ball shall roll (in mm)
 * @param preStepType Is a prestep for the InWalkKick allowed?
 * @param turnKickAllowed Does the forward kick not need to align with the kick direction?
 * @param directionPrecision The allowed deviation of the direction in which the ball should go. If default, the WalkToBallAndKickEngine uses its own precision.
 */
option(Dribble, args((Angle) targetDirection,
                     (const Pose2f&)({1.f, 1.f, 1.f}) speed,
                     (const MotionRequest::ObstacleAvoidance&)({}) obstacleAvoidance,
                     (KickPrecision)(KickPrecision::notPrecise) alignPrecisely,
                     (float)(0.f) kickLength,
                     (PreStepType)(PreStepType::allowed) preStepType,
                     (bool)(true) turnKickAllowed,
                     (const Rangea&)({0_deg, 0_deg}) directionPrecision));

/**
 * This skill executes a key frame motion with the arms.
 * @param motion The motion that should be executed.
 * @param arm The arm that should execute the motion. The default
 *            is do use both arms.
 * @param fast Whether states should not be interpolated.
 */
option(KeyFrameArms, args((ArmKeyFrameRequest::ArmKeyFrameId) motion,
                          (Arms::Arm)(Arms::numOfArms) arm,
                          (bool)(false) fast));

/**
 * This skill executes a key frame motion with the left arm.
 * @param motion The motion that the arm should execute
 * @param fast Whether states should not be interpolated
 */
option(KeyFrameLeftArm, args((ArmKeyFrameRequest::ArmKeyFrameId) motion,
                             (bool)(false) fast));

/**
 * This skill executes a key frame motion with the right arm.
 * @param motion The motion that the arm should execute
 * @param fast Whether states should not be interpolated
 */
option(KeyFrameRightArm, args((ArmKeyFrameRequest::ArmKeyFrameId) motion,
                              (bool)(false) fast));

/**
 * This skill moves the head to look at interesting points
 * @param withBall Whether the ball must be in the image
 * @param ignoreBall Whether the ball should be completely ignored
 * @param onlyOwnBall Whether to use only the own ball model and not the team ball model
 * @param fixTilt Whether to use a fix tilt or to allow the head control to calculate it
 * @param slowdownFactor A factor to slow down the speed of the head movement
 */
option(LookActive, args((bool)(false) withBall,
                        (bool)(false) ignoreBall,
                        (bool)(false) onlyOwnBall,
                        (bool)(false) fixTilt,
                        (float)(1.f) slowdownFactor));

/**
 * This skill moves the head so that a camera looks at specified angles.
 * @param pan The target pan angle
 * @param tilt The target tilt angle
 * @param speed The speed with which to move the head
 * @param camera The camera which should have the specified angles
 * @param calibrationMode Whether to set the mode to calibrationMode instead of panAndTiltMode, which disables clipping and interpolation of angles.
 */
option(LookAtAngles, args((Angle) pan,
                          (Angle) tilt,
                          (float)(180_deg) speed,
                          (HeadMotionRequest::CameraControlMode)(HeadMotionRequest::autoCamera) camera,
                          (bool)(false) calibrationMode));

/** This skill moves the head so that the ball is focused by one camera. */
option(LookAtBall);

/**
 * This skill moves the head such that a specified (robot-relative) point is focused by one camera.
 * @param target The point to look at in robot-relative coordinates
 * @param camera The camera which should look at the point
 * @param speed The speed with which to move the head
 */
option(LookAtPoint, args((const Vector3f&) target,
                         (HeadMotionRequest::CameraControlMode)(HeadMotionRequest::autoCamera) camera,
                         (Angle)(180_deg) speed));

/** This skill moves the head so that it looks forward.  */
option(LookForward);

/**
 * This skill moves the head alternately to the left and to the right
 * @param startLeft Whether to look left first (otherwise right)
 * @param maxPan The maximum pan angle
 * @param tilt The constant tilt angle during the motion
 * @param speed The speed with which to move the head
 */
option(LookLeftAndRight, args((bool)(true) startLeft,
                              (Angle)(50_deg) maxPan,
                              (Angle)(23_deg) tilt,
                              (Angle)(100_deg) speed));

/**
 * This skill moves the head while walking, to look at the ball and the target.
 * @param startBall Start looking first at the ball
 * @param tilt The constant tilt angle during the motion
 * @param maxPan The maximum pan angle
 */
option(LookAtBallAndTarget, args((bool)(true) startBall,
                                 (Angle)(23_deg) tilt,
                                 (Angle)(10_deg) thresholdAngle,
                                 (Vector2f)(Vector2f::Zero()) walkingDirection));

/**
 * This skill sets the passTarget member of the BehaviorStatus.
 * Caution: If the passTarget is set to -1, the ballTarget Vector is often used to communicate good angles instead of a position
 * @param passTarget The passTarget to set
 * @param ballTarget Optionally, the position where the ball should end up in robot-relative coordinates
 */
option(PassTarget, args((int) passTarget,
                        (const Vector2f&)(Vector2f::Zero()) ballTarget));

/** Sets the motion request to photo mode. */
option(PhotoMode);

/** This skill turns off the joints of the robot. */
option(PlayDead);

/**
 * This skill plays a sound file if it differs from the one that has been played in the last frame.
 * @param force Play sound even if muted.
 * @param name The name of the sound file
 */
option(PlaySound, args((const std::string&) name,
                       (bool)(false) force));

/**
 * This skill lets one arm point at some point.
 * @param localPoint The point in robot-relative coordinates.
 * @param arm The arm that shall be used for pointing. By default,
 *            the arm is selected automatically.
 */
option(PointAt, args((const Vector3f&) localPoint,
                     (Arms::Arm)(Arms::numOfArms) arm));

/**
 * This skill records the walkingTo target and desired speed in the BehaviorStatus
 * to be published to teammates.
 * @param target The target the robot is walking towards.
 * @param speed The desired relative speed.
 */
option(PublishMotion, args((const Vector2f) target,
                           (const Pose2f&)({1.f, 1.f, 1.f}) speed));

/** This skill replays walk phases. */
option(ReplayWalk);

/**
 * This skill makes the Nao say something if it differs from what was said in the last frame.
 * @param name The text to be synthesized and pronounced
 * @param force Talk even if muted.
 * @param speed Use speed < 1 to talk slower and speed > 1 to talk faster.
 */
option(Say, args((const std::string&) text,
                 (bool)(false) force,
                 (float)(1.f) speed));

/**
 * This skill lets the robot execute a special motion.
 * @param request The special request
 */
option(Special, args((MotionRequest::Special::Request) request));

/**
 * This skill makes the robot stand.
 * @param high Whether the knees should be stretched
 */
option(Stand, args((bool)(false) high));

/**
 * This skill walks with a specified speed.
 * @param speed The walking speed in radians/s for the rotation and mm/s for the translation
 */
option(WalkAtAbsoluteSpeed, args((const Pose2f&) speed));

/**
 * This skill walks with a specified speed relative to the configured maximum.
 * @param speed The walking speed as ratio of the maximum speed in [0, 1]
 */
option(WalkAtRelativeSpeed, args((const Pose2f&) speed));

/**
 * This skill walks to the ball and kicks it.
 * @param targetDirection The (robot-relative) direction in which the ball should go
 * @param kickType The type of kick that should be used
 * @param alignPrecisely Whether the robot should align more precisely than usual
 * @param kickLength The distance the ball shall roll (in mm)
 * @param speed The walking speed as ratio of the maximum speed in [0, 1]
 * @param obstacleAvoidance The obstacle avoidance request
 * @param preStepType Is a prestep for the InWalkKick allowed?
 * @param turnKickAllowed Does the forward kick not need to align with the kick direction?
 * @param directionPrecision The allowed deviation of the direction in which the ball should go. If default, the WalkToBallAndKickEngine uses its own precision.
 */
option(WalkToBallAndKick, args((Angle) targetDirection,
                               (KickInfo::KickType) kickType,
                               (KickPrecision)(KickPrecision::notPrecise) alignPrecisely,
                               (float)(std::numeric_limits<float>::max()) kickLength,
                               (const Pose2f&)({1.f, 1.f, 1.f}) speed,
                               (const MotionRequest::ObstacleAvoidance&)({}) obstacleAvoidance,
                               (PreStepType)(PreStepType::allowed) preStepType,
                               (bool)(true) turnKickAllowed,
                               (bool)(false) shiftTurnKickPose,
                               (const Rangea&)({0_deg, 0_deg}) directionPrecision));

/**
 * This skill walks to a (relative) target.
 * @param target The target pose in robot-relative coordinates
 * @param speed The walking speed as ratio of the maximum speed in [0, 1]
 * @param obstacleAvoidance The obstacle avoidance request
 * @param keepTargetRotation Whether the target rotation should be headed for all the time (instead of allowing motion to plan it)
 * @param targetOfInterest If set, it is used to decide how the body shall be orientated while walking, so the camera can see this target at all times. Target is in relative coordinates. (Only applies for side walk)
 * @param forceSideWalking If true, force sidewalking
 */
option(WalkToPose, args((const Pose2f&) target,
                        (const Pose2f&)({1.f, 1.f, 1.f}) speed,
                        (const MotionRequest::ObstacleAvoidance&)({}) obstacleAvoidance,
                        (bool)(false) keepTargetRotation,
                        (const std::optional<Vector2f>&)({}) targetOfInterest,
                        (bool)(false) forceSideWalking));
