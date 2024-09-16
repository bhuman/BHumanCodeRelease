/**
 * @file Head.h
 *
 * This file declares complex skills related to head control.
 *
 * @author Thomas Röfer
 */

/**
 * This skill controls the head for a robot that walks to the ball.
 * @param distanceToTarget The distance to the kick pose (from which the ball can be kicked).
 * @param lookAtKickTarget Whether the robot is allowed to look at the kick target.
 * @param kickTargetRelative The kick target (only needed if the previous parameter is true).
 */
option(GoToBallHeadControl, args((float) distanceToTarget,
                                 (bool)(false) lookAtKickTarget,
                                 (Vector2f)(Vector2f::Zero()) kickTargetRelative));

/**
 * This skill moves the head in steps between the maximum pan and tilt, relative to the current orientation.
 * @param original The original orientation
 * @param maximum The maximum absolute (left and right) deviation from the original orientation.
 * @param panStep
 * @param tiltStep
 * @param waitInPosition
 * @param speed
 */
option(PanAndTiltGrid, args((const HeadOrientation&) original,
                            (const HeadOrientation&) maximum,
                            (const Angle&) panStep,
                            (const Angle&) tiltStep,
                            (int) waitInPosition,
                            (Angle)(100_deg) speed));
