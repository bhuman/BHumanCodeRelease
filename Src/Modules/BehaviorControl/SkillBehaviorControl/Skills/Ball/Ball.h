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
option(DribbleToGoal, args((KickPrecision)(KickPrecision::normal) alignPrecisely));

/**
 * This skill walks to the ball and dribbles it from there.
 * @param targetDirection The direction to which the ball should be dribbled in robot-relative coordinates
 * @param alignPrecisely Whether the robot should align more precisely than usual
 * @param kickLength The distance the ball shall roll
 * @param lookActiveWithBall If true, use LookActive but with the flag withBall = true
 */
option(GoToBallAndDribble, args((Angle) targetDirection,
                                (KickPrecision)(KickPrecision::justHitTheBall) alignPrecisely,
                                (float)(750.f) kickLength,
                                (bool)(false) lookActiveWithBall));

/**
 * This skill walks to the ball and executes a kick there.
 * @param targetDirection The direction to which the ball should be kicked in robot-relative coordinates
 * @param kickType The kick type that should be executed there
 * @param lookActiveWithBall If true, use LookActive but with the flag withBall = true
 * @param alignPrecisely Whether the robot should align more precisely than usual
 * @param length The desired length of the kick (works only for certain types of kicks)
 * @param speed The walking speed
 * @param reduceWalkSpeedType Reduce the walking speed with specific modes
   */
option(GoToBallAndKick, args((Angle) targetDirection,
                             (KickInfo::KickType) kickType,
                             (bool)(false) lookActiveWithBall,
                             (KickPrecision)(KickPrecision::normal) alignPrecisely,
                             (float)(std::numeric_limits<float>::max()) length,
                             (const Pose2f&)({1.f, 1.f, 1.f}) speed,
                             (ReduceWalkSpeedType::ReduceWalkSpeedType)(ReduceWalkSpeedType::noChange) reduceWalkSpeedType));

/** This skill walks to the ball, waits a bit, and then decides whether to pass or to kick at  the goal*/
option(GoToBallWaitAndKick);

/** This skill plays the ball when the own goal posts constrain the possible kick poses. */
option(HandleBallAtOwnGoalPost);

/** This skill kicks the ball at the (opponent's) goal. It may revert to dribbling if the goal is unreachable. */
option(KickAtGoal);

/**
 * This skill passes the ball to a teammate.
 * @param playerNumber The player number of the pass target.
 * @param passedWaitingTime The amount of time that was already spent waiting
 */
option(PassToTeammate, args((int) playerNumber,
                            (int)(0) passedWaitingTime));

/**
 * This skill walks very carefully to a kick pose and executes a kick there.
 * @param kickPose The pose at which the kick should be executed in robot-relative coordinates
 * @param kickType The kick type that should be executed there
 * @param walkSpeed The walking speed as ratio of the maximum speed in [0, 1]
 */
option(PenaltyStrikerGoToBallAndKick, args((const Pose2f&) kickPose,
                                           (KickInfo::KickType) kickType,
                                           (Pose2f) walkSpeed));

/** This skill plays the ball under consideration of the skill request. */
option(PlayBall);

/**  Skill for dueling an opponent. */
option(Zweikampf);
