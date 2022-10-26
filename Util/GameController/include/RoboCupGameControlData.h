#ifndef ROBOCUPGAMECONTROLDATA_H
#define ROBOCUPGAMECONTROLDATA_H

#include <stdint.h>

#define GAMECONTROLLER_DATA_PORT   3838
#define GAMECONTROLLER_RETURN_PORT 3939

#define GAMECONTROLLER_STRUCT_HEADER  "RGme"
#define GAMECONTROLLER_STRUCT_VERSION 14

#define MAX_NUM_PLAYERS 7

// SPL
#define TEAM_BLUE   0 // blue, cyan
#define TEAM_RED    1 // red, magenta, pink
#define TEAM_YELLOW 2 // yellow
#define TEAM_BLACK  3 // black, dark gray
#define TEAM_WHITE  4 // white
#define TEAM_GREEN  5 // green
#define TEAM_ORANGE 6 // orange
#define TEAM_PURPLE 7 // purple, violet
#define TEAM_BROWN  8 // brown
#define TEAM_GRAY   9 // lighter gray

#define COMPETITION_PHASE_ROUNDROBIN 0
#define COMPETITION_PHASE_PLAYOFF    1

#define COMPETITION_TYPE_NORMAL                0
#define COMPETITION_TYPE_CHALLENGE_SHIELD      1
#define COMPETITION_TYPE_7V7                   2
#define COMPETITION_TYPE_DYNAMIC_BALL_HANDLING 3

#define GAME_PHASE_NORMAL       0
#define GAME_PHASE_PENALTYSHOOT 1
#define GAME_PHASE_OVERTIME     2
#define GAME_PHASE_TIMEOUT      3

#define STATE_INITIAL  0
#define STATE_READY    1
#define STATE_SET      2
#define STATE_PLAYING  3
#define STATE_FINISHED 4

#define SET_PLAY_NONE              0
#define SET_PLAY_GOAL_KICK         1
#define SET_PLAY_PUSHING_FREE_KICK 2
#define SET_PLAY_CORNER_KICK       3
#define SET_PLAY_KICK_IN           4
#define SET_PLAY_PENALTY_KICK      5

#define PENALTY_NONE                          0
// SPL
#define PENALTY_SPL_ILLEGAL_BALL_CONTACT      1 // ball holding / playing with hands
#define PENALTY_SPL_PLAYER_PUSHING            2
#define PENALTY_SPL_ILLEGAL_MOTION_IN_SET     3 // heard whistle too early?
#define PENALTY_SPL_INACTIVE_PLAYER           4 // fallen, inactive
#define PENALTY_SPL_ILLEGAL_POSITION          5
#define PENALTY_SPL_LEAVING_THE_FIELD         6
#define PENALTY_SPL_REQUEST_FOR_PICKUP        7
#define PENALTY_SPL_LOCAL_GAME_STUCK          8
#define PENALTY_SPL_ILLEGAL_POSITION_IN_SET   9

#define PENALTY_SUBSTITUTE                    14
#define PENALTY_MANUAL                        15

struct RobotInfo
{
  uint8_t penalty;             // penalty state of the player
  uint8_t secsTillUnpenalised; // estimate of time till unpenalised
};

struct TeamInfo
{
  uint8_t teamNumber;                 // unique team number
  uint8_t teamColour;                 // colour of the team
  uint8_t score;                      // team's score
  uint8_t penaltyShot;                // penalty shot counter
  uint16_t singleShots;               // bits represent penalty shot success
  uint16_t messageBudget;             // number of team messages the team is allowed to send for the remainder of the game
  RobotInfo players[MAX_NUM_PLAYERS]; // the team's players
};

struct RoboCupGameControlData
{
  char header[4];           // header to identify the structure
  uint8_t version;          // version of the data structure
  uint8_t packetNumber;     // number incremented with each packet sent (with wraparound)
  uint8_t playersPerTeam;   // the number of players on a team
  uint8_t competitionPhase; // phase of the competition (COMPETITION_PHASE_ROUNDROBIN, COMPETITION_PHASE_PLAYOFF)
  uint8_t competitionType;  // type of the competition (COMPETITION_TYPE_NORMAL, COMPETITION_TYPE_CHALLENGE_SHIELD, etc)
  uint8_t gamePhase;        // phase of the game (GAME_PHASE_NORMAL, GAME_PHASE_PENALTYSHOOT, etc)
  uint8_t state;            // state of the game (STATE_READY, STATE_PLAYING, etc)
  uint8_t setPlay;          // active set play (SET_PLAY_NONE, SET_PLAY_GOAL_KICK, etc)
  uint8_t firstHalf;        // 1 = game in first half, 0 otherwise
  uint8_t kickingTeam;      // the team number of the next team to kick off, free kick etc
  int16_t secsRemaining;    // estimate of number of seconds remaining in the half
  int16_t secondaryTime;    // number of seconds shown as secondary time (remaining ready, until free ball, etc)
  TeamInfo teams[2];
};

// data structure header
#define GAMECONTROLLER_RETURN_STRUCT_HEADER      "RGrt"
#define GAMECONTROLLER_RETURN_STRUCT_VERSION     4

struct RoboCupGameControlReturnData
{
  char header[4];     // "RGrt"
  uint8_t version;    // has to be set to GAMECONTROLLER_RETURN_STRUCT_VERSION
  uint8_t playerNum;  // player number starts with 1
  uint8_t teamNum;    // team number
  uint8_t fallen;     // 1 means that the robot is fallen, 0 means that the robot can play

  // position and orientation of robot
  // coordinates in millimeters
  // 0,0 is in center of field
  // +ve x-axis points towards the goal we are attempting to score on
  // +ve y-axis is 90 degrees counter clockwise from the +ve x-axis
  // angle in radians, 0 along the +x axis, increasing counter clockwise
  float pose[3];         // x,y,theta

  // ball information
  float ballAge;         // seconds since this robot last saw the ball. -1.f if we haven't seen it

  // position of ball relative to the robot
  // coordinates in millimeters
  // 0,0 is in center of the robot
  // +ve x-axis points forward from the robot
  // +ve y-axis is 90 degrees counter clockwise from the +ve x-axis
  float ball[2];

#ifdef __cplusplus
  // constructor
  RoboCupGameControlReturnData() :
    version(GAMECONTROLLER_RETURN_STRUCT_VERSION),
    playerNum(0),
    teamNum(0),
    fallen(255),
    ballAge(-1.f)
  {
    const char* init = GAMECONTROLLER_RETURN_STRUCT_HEADER;
    for(unsigned int i = 0; i < sizeof(header); ++i)
      header[i] = init[i];
    pose[0] = 0.f;
    pose[1] = 0.f;
    pose[2] = 0.f;
    ball[0] = 0.f;
    ball[1] = 0.f;
  }
#endif
};

#endif // ROBOCUPGAMECONTROLDATA_H
