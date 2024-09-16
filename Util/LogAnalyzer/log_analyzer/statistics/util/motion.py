from __future__ import annotations

from enum import Enum


class Motion(Enum):
    """Mirros the Motion enum from Representations/MotionControl/MotionRequest.h.

    ENUM(Motion,
    {,
        playDead, /**< Set all joints to zero stiffness. */
        stand, /**< Stand. */
        walkAtAbsoluteSpeed, /**< Walk at a given speed. */
        walkAtRelativeSpeed, /**< Walk at a given speed, given as a ratio of the maximum. */
        walkToPose, /**< Walk to a target pose. */
        walkToBallAndKick, /**< Walk to the ball and kick it. */
        dribble, /**< Dribble the ball. */
        dive, /**< Execute a diving motion. */
        special, /**< Execute a special motion. */
        replayWalk, /**< Replay a recorded walkPhase history. */
        calibration, /**< Calibration module controls the arms. */
    });
    """

    PLAY_DEAD = 0
    STAND = 1
    WALK_AT_ABSOLUTE_SPEED = 2
    WALK_AT_RELATIVE_SPEED = 3
    WALK_TO_POSE = 4
    WALK_TO_BALL_AND_KICK = 5
    DRIBBLE = 6
    DIVE = 7
    SPECIAL = 8
    REPLAY_WALK = 9
    CALIBRATION = 10
