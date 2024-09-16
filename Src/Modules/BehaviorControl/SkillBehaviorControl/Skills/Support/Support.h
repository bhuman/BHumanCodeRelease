/**
 * @file Support.h
 *
 * This file declares complex skills related to supporters.
 *
 * @author Thomas Röfer
 */

/**
 * This skill blocks another robot (i.e. preventing it from moving, especially towards the ball).
 * This robot will be facing the other robot and thus probably not see the ball.
 * @param target The position of the blocked robot in robot-relative coordinates
 * @param useAlternativeBall Whether the alternative ball position should be used (instead of something taken directly from a model)
 * @param alternativeBall The alternative ball position in robot-relative coordinates
 */
option(Block, args((const Vector2f&) target,
                   (bool)(false) useAlternativeBall,
                   (Vector2f)(Vector2f::Zero()) alternativeBall));

/**
 * This skill marks another robot (i.e. positioning between it and the ball to make it unattractive as pass target).
 * This robot will be facing the ball and thus probably not see the marked robot.
 * @param target The position of the marked robot in robot-relative coordinates
 */
option(Mark, args((const Vector2f&) target));

/**
 * This skill receives the ball passed from a teammate.
 * @param playerNumber The player number passing the ball.
 */
option(ReceivePass, args((int) playerNumber));
