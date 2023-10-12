#ifndef VISUALREFEREECHALLENGE_H
#define VISUALREFEREECHALLENGE_H

/*
 * In the In-game Visual Referee Challenge, robots must communicate a detected gesture to the
 * GameController. This is done via the same channel on which RoboCupGameControlReturnData is sent.
 * Specifically, messages must be sent via UDP unicast directly to the GameController host on
 * the port GAMECONTROLLER_RETURN_PORT (as given in RoboCupGameControlData.h) and using the
 * structure RoboCupGameControlReturnData with the following properties:
 * - header, playerNum and teamNum set as usual
 * - version set to GAMECONTROLLER_RETURN_STRUCT_VRC_VERSION (defined below)
 * - fallen set to one of the GAMECONTROLLER_RETURN_STRUCT_VRC_GESTURE_* constants (defined below)
 * - ballAge set to the time (in seconds) since the corresponding whistle was heard
 * The other fields (pose and ball) are ignored.
 */

#define GAMECONTROLLER_RETURN_STRUCT_VRC_VERSION 255

#define GAMECONTROLLER_RETURN_STRUCT_VRC_GESTURE_KICK_IN_BLUE_TEAM 1
#define GAMECONTROLLER_RETURN_STRUCT_VRC_GESTURE_KICK_IN_RED_TEAM 2
#define GAMECONTROLLER_RETURN_STRUCT_VRC_GESTURE_GOAL_KICK_BLUE_TEAM 3
#define GAMECONTROLLER_RETURN_STRUCT_VRC_GESTURE_GOAL_KICK_RED_TEAM 4
#define GAMECONTROLLER_RETURN_STRUCT_VRC_GESTURE_CORNER_KICK_BLUE_TEAM 5
#define GAMECONTROLLER_RETURN_STRUCT_VRC_GESTURE_CORNER_KICK_RED_TEAM 6
#define GAMECONTROLLER_RETURN_STRUCT_VRC_GESTURE_GOAL_BLUE_TEAM 7
#define GAMECONTROLLER_RETURN_STRUCT_VRC_GESTURE_GOAL_RED_TEAM 8
#define GAMECONTROLLER_RETURN_STRUCT_VRC_GESTURE_PUSHING_FREE_KICK_BLUE_TEAM 9
#define GAMECONTROLLER_RETURN_STRUCT_VRC_GESTURE_PUSHING_FREE_KICK_RED_TEAM 10
#define GAMECONTROLLER_RETURN_STRUCT_VRC_GESTURE_FULL_TIME 11
#define GAMECONTROLLER_RETURN_STRUCT_VRC_GESTURE_SUBSTITUTION_BLUE_TEAM 12
#define GAMECONTROLLER_RETURN_STRUCT_VRC_GESTURE_SUBSTITUTION_RED_TEAM 13

#endif // VISUALREFEREECHALLENGE_H
