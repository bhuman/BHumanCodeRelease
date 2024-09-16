/**
 * @file Ball.h
 *
 * This file declares complex skills related to handling the ball.
 *
 * @author Thomas Röfer
 */

/** This skill clears the ball when no pass target is available */
option(ClearBall);

/** This skill does just a kick-off into the opponent's half. */
option(DirectKickOff);

/** This skill dribbles the ball to the goal. */
option(DribbleToGoal);

/**
 * This skill walks to the ball and dribbles it from there.
 * @param targetDirection The direction to which the ball should be dribbled in robot-relative coordinates
 * @param alignPrecisely Whether the robot should align more precisely than usual
 * @param kickLength The distance the ball shall roll
 * @param lookActiveWithBall If true, use LookActive but with the flag withBall = true
 * @param preStepType Is a prestep for the InWalkKick allowed?
 * @param turnKickAllowed Does the forward kick not need to align with the kick direction?
 * @param directionPrecision The allowed deviation of the direction in which the ball should go. If default, the WalkToBallAndKickEngine uses its own precision.
 */
option(GoToBallAndDribble, args((Angle) targetDirection,
                                (KickPrecision)(KickPrecision::notPrecise) alignPrecisely,
                                (float)(750.f) kickLength,
                                (bool)(false) lookActiveWithBall,
                                (PreStepType)(PreStepType::allowed) preStepType,
                                (bool)(true) turnKickAllowed,
                                (const Rangea&)({0_deg, 0_deg}) directionPrecision));

/**
 * This skill walks to the ball and executes a kick there.
 * @param targetDirection The direction to which the ball should be kicked in robot-relative coordinates
 * @param kickType The kick type that should be executed there
 * @param lookActiveWithBall If true, use LookActive but with the flag withBall = true
 * @param alignPrecisely Whether the robot should align more precisely than usual
 * @param length The desired length of the kick (works only for certain types of kicks)
 * @param preStepType Is a prestep for the InWalkKick allowed?
 * @param turnKickAllowed Does the forward kick not need to align with the kick direction?
 * @param speed The walking speed
 * @param reduceWalkSpeedType Reduce the walking speed with specific modes
 * @param directionPrecision The allowed deviation of the direction in which the ball should go. If default, the WalkToBallAndKickEngine uses its own precision.
 */
option(GoToBallAndKick, args((Angle) targetDirection,
                             (KickInfo::KickType) kickType,
                             (bool)(false) lookActiveWithBall,
                             (KickPrecision)(KickPrecision::notPrecise) alignPrecisely,
                             (float)(std::numeric_limits<float>::max()) length,
                             (PreStepType)(PreStepType::allowed) preStepType,
                             (bool)(true) turnKickAllowed,
                             (bool)(false) shiftTurnKickPose,
                             (const Pose2f&)({1.f, 1.f, 1.f}) speed,
                             (ReduceWalkSpeedType)(ReduceWalkSpeedType::noChange) reduceWalkSpeedType,
                             (const Rangea&)({0_deg, 0_deg}) directionPrecision));

/** This skill plays the ball when the own goal posts constrain the possible kick poses. */
option(HandleBallAtOwnGoalPost);

/**
 * This skill intercepts a rolling ball with the goal that it does not pass the y axis of this robot.
 * @param interceptionMethods A bit set of methods that may be used to intercept the ball (from Interception::Method).
 * @param allowGetUp Whether the robot is allowed to get up afterwards.
 * @param allowDive Whether the robot is allowed to actually dive. Otherwise, only sounds are played to indicate what it would do.
 */
option(InterceptBall, args((unsigned) interceptionMethods,
                           (bool)(true) allowGetUp,
                           (bool)(true) allowDive));

/** This skill kicks the ball at the (opponent's) goal. It may revert to dribbling if the goal is unreachable. */
option(KickAtGoal);

/**
 * This skill passes the ball to a teammate.
 * @param playerNumber The player number of the pass target.
 */
option(PassToTeammate, args((int) playerNumber));

/**
 * This skill walks very carefully to a kick pose and executes a kick there.
 * @param kickPose The pose at which the kick should be executed in robot-relative coordinates
 * @param kickType The kick type that should be executed there
 * @param walkSpeed The walking speed as ratio of the maximum speed in [0, 1]
 */
option(PenaltyStrikerGoToBallAndKick, args((const Pose2f&) kickPose,
                                           (KickInfo::KickType) kickType,
                                           (float) walkSpeed));

/** This skill plays the ball under consideration of the skill request. */
option(PlayBall);

/**  Skill for dueling an opponent. */
option(Zweikampf);
