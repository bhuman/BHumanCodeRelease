/**
 * @file BHULKsStandardMessage.h
 *
 * The file declares a struct BHULKsStandardMessage.
 * This message is intended to be used in between the SPLStandardMessage and the team's
 * private additional data.
 * The BHULKsStandardMessage is designed for the RoboCup SPL Mixed-Teams Competition 2017,
 * to specify the communication between the robots of team HULKs and the ones of team
 * B-Human that are building the Mixed-Team "B-Hulks".
 * In fact all data that is needed for an "inter team communication" must be declared
 * here, if it is not part of the SPLStandardMessage.
 *
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 */

#pragma once

#include <stdint.h>
#include <limits>
#include <vector>
#include <cassert>

namespace RoboCup
{
#include <RoboCupGameControlData.h>
}

namespace B_HULKs
{
#define BHULKS_STANDARD_MESSAGE_STRUCT_HEADER  "BHLK"
#define BHULKS_STANDARD_MESSAGE_STRUCT_VERSION 8        //< this should be incremented with each change
#define BHULKS_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS 10   //< max teammembers for Mixed-Teams 2017
#define BHULKS_STANDARD_MESSAGE_MAX_NUM_OF_OBSTACLES 7  //< max send obstacles per messages

#define HULKS_MEMBER 0
#define B_HUMAN_MEMBER 1

  /*
   Important remarks about units:

   For each parameter, the respective comments describe its unit.
   The following units are used:

     - Distances:  Millimeters (mm)
     - Angles:     Radian
     - Time:       Milliseconds (ms)
     - Speed:      Millimeters per second (mm/s)
     - Timestamp:  Milliseconds since system / software start (ms)
   */

  /*
    Important remarks about streaming sizes:
    NOT all values are streamed as a whole!
    If a unit is not streamed in its natural range, a commentary will indicate
    the range as it is interpreted.
    The general comment pattern is "[rangeFrom..rangeTo (in precision)]".

    E.g.
     uint32_t value0;    //-- no comment     (This will be streamed as full 4 Byte)
     bool     value1;    //-- no comment     (This may be streamed as 1 Bit)
     uint32_t value2;    // < [3..12]        (This will be streamed with minumum value 3
                                                                   and maximum value 12)
     uint16_t value3;    // < [3..12] bla    (Same as above)
     uint32_t value2;    // < [4..10 (2)]    (This will be streamed with a precision of 2, a minumum
                                               value of 4 and a maximum value of 10.
                                               This means after streaming it can hold 2,4,6,8 and 10)
     uint32_t time1      // < [delta 0..-10] (This will be streamed in relation to the timestamp of
                                               the message in range of 0 to -10.)
   */

  /**
   * Definition of all shared behavior roles
   */
  enum class Role : uint8_t
  {
    // This is chess, because if it is chess than it is more exact! (Cheers Tim)
    King,         //< Keeper         -> short range doing what he wants
    Rook,         //< Defender       -> mainly covering with "horizontal" movement
    Queen,        //< Striker        -> running wherever she wants
    Knight,       //< Supporter      -> jumping alongside the Queen and helping her out
    Bishop,       //< Pass Station   -> diagonal movement in front of the Queen
    beatenPieces  //< not perceived numbers -> robots staying outside the field carpet
  };
  constexpr Role numOfRoles = Role(6);

  /**
   * Definition of possible confidences based on teh robots hearing capability
   */
  enum class HearingConfidence : uint8_t
  {
    iAmDeaf = (unsigned char)-1,
    heardOnOneEarButThinkingBothAreOk = 33,
    oneEarIsBroken = 66,
    allEarsAreOk = 100
  };
  constexpr HearingConfidence numOfHearingConfidences = HearingConfidence(4);

  /**
   * The type of an obstacle.
   */
  enum class ObstacleType : uint8_t
  {
    goalpost,
    unknown,
    someRobot,
    opponent,
    teammate,
    fallenSomeRobot,
    fallenOpponent,
    fallenTeammate
  };
  constexpr ObstacleType numOfObstacleTypes = ObstacleType(8);

  /**
   * The definintion of an Obstacle as it is shared between B-HULKs-robots.
   */
  struct Obstacle
  {
    // the obstacle center in robot (self) centered coordinates
    // - x goes to front
    // - y goes to left
    float center[2]; // < [short (4mm)]

    uint32_t timestampLastSeen; //< [delta 0..-16384 (64) ] The name says it
    ObstacleType type;          //< [0..numOfObstacleTypes] The name says it

    // returns the size of this struct when it is written
    static int sizeOfObstacle();

    // Method to convert this struct for communication usage
    // @param data point to dataspace,
    // @param timestamp the reference timestamp of the writimg message
    //  -asserts: writing sizeOfObstacle() bytes
    void write(void*& data, uint32_t timestamp) const;

    // Method to reads the Obstacle from data.
    // @param data the Obstacle
    // @param timestamp the reference timestamp of reading message
    void read(const void*& data, uint32_t timestamp);
  };

  /**
   * The definintion of an ntp message we send - in response to a previously received request
   */
  struct BNTPMessage
  {
    uint32_t requestOrigination;  //<                        The timestamp of the generation of the request
    uint32_t requestReceipt;      //< [delta 0..-4096]       The timestamp of the receipt of the request
    uint8_t receiver;             //< [#_MAX_NUM_OF_PLAYERS] The robot, to which this message should be sent
  };

  /**
   * A struct that holds the main (useful) data of RoboCupGameControlData.
   * This is used to to spread important data via BHULKsStandardMessage for
   *   partly compensation in case of game controller package loss.
   */
  struct OwnTeamInfo
  {
    static_assert(GAMECONTROLLER_STRUCT_VERSION == 10,
                  "Please adjust this struct to the newer game controller struct version");

    OwnTeamInfo();

    // timestamp when RoboCupGameControlData (RoboCup::) was reveived
    uint32_t timestampWhenReceived; // [delta 0..-1.09 minutes (256ms)]

    //values of RoboCup::RoboCupGameControlData
    uint8_t packetNumber;
    uint8_t gameType;       // < [0..3]
    uint8_t state;          // < [0..7]
    uint8_t firstHalf;      // < [0..1]
    uint8_t kickOffTeam;
    uint8_t secondaryState; // < [0..3]
    uint8_t dropInTeam;
    uint16_t dropInTime;    // < [0..62 (2)]
    uint16_t secsRemaining; // < [0..1023]
    uint16_t secondaryTime; // < [0..511]

    //values of RoboCup::RoboCupGameControlData::TeamInfo
    uint8_t score;
    bool playersArePenalized[BHULKS_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS];

    // returns the size of this struct when it is written
    int sizeOfOwnTeamInfo() const;

    // Method to convert this struct for communication usage
    // @param data point to dataspace
    // @param timestamp the reference timestamp of the writimg message
    //  -asserts: writing sizeOfOwnTeamInfo() bytes
    void write(void*& data, uint32_t timestamp) const;

    // Method to reads the OwnTeamInfo from data.
    // @param data the OwnTeamInfo
    // @param timestamp the reference timestamp of reading message
    void read(const void*& data, uint32_t timestamp);
  };

  /**
   * The BHULKsStandardMessage, for clear team communication between B-HULKs-robots
   * This message is intended to be used in between the SPLStandardMessage and the team's
   * private additional data.
   */
  struct BHULKsStandardMessage
  {
    char header[4];      //<        "BHLK"
    uint8_t version;     //<        Has to be set to BHULKS_STANDARD_MESSAGE_STRUCT_VERSION
    int8_t member;       //< [0..1] Use HULKS_MEMBER or B_HUMAN_MEMBER
    uint32_t timestamp;  //<        The timestamp of this message

    bool     isUpright;               //< The name says it all
    bool     hasGroundContact;        //< The name says it all
    uint32_t timeOfLastGroundContact; //< The name says it all

    // is the robot penalized?
    // Theoretically the game controller say it too, but this is for better information
    // spreading in case of bad WLAN quality in combination with PENALTY_MANUAL.
    bool isPenalized;

    // last reveived RoboCupGameControlData
    // Hence package leakage at RoboCup events is a thing, the redundany of main game
    // controle data should help compensate as much as possible.
    OwnTeamInfo gameControlData;

    // the current meassurement of head joint: HeadYaw
    float headYawAngle; // < [-127..127 (1°)]

    // the role this robot is currently performing 
    Role currentlyPerfomingRole;

    // the calculated role per robot
    // e.g. the role that is calculated for robot 2 is at position
    //      roleAssignments[<robot id> - 1] => roleAssignments[1]
    Role roleAssignments[BHULKS_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS];

    // the King is playing the ball => the Queen must not play the ball
    // - !!! for safety reasons this should always be combined with 
    //        (sPLStandardMessage.playerNum == 0)
    bool kingIsPlayingBall;

    // the pass target's player number, filled by the current Queen
    //    robot if necessary
    // - normaly it is -1 (no target)
    // - for safety reasons this should always be combined with check on
    //     Queen role
    int8_t passTarget;

    // does/means what it says
    uint32_t timeWhenReachBall;
    uint32_t timeWhenReachBallQueen;

    // timestamp, when the ball was recognized
    // this is theoretically equal to SPLStandardMessage::ballAge, BUT it allows us to "ntp" it.
    // Also this is a better format for old messages. 
    uint32_t ballTimeWhenLastSeen;
    
    // timestamp of "last jumped"
    // - "last jumped" describes a situation, when the robots self localisation
    //   corrects for an bigger update than normal
    uint32_t timestampLastJumped;

    // whistle recognition stuff 
    HearingConfidence confidenceOfLastWhistleDetection; //< confidence based on hearing capability
    uint32_t lastTimeWhistleDetected; //< timestamp

    // the obstacles from the private obstacle model
    std::vector<Obstacle> obstacles;

    // is this robot requesting an ntp message?
    bool requestsNTPMessage;
    // all ntp-message this robot sends to his teammates in response to their requests
    std::vector<BNTPMessage> ntpMessages;

    // returns the size of this struct when it is written
    int sizeOfBHULKsMessage() const;

    // Method to convert this struct for communication usage
    // @param data point to dataspace,
    //        THIS SHOULD BE AT LEAST AS BIG AS this->sizeOfBHMessage()
    // -asserts: writing sizeOfBHMessage() bytes 
    void write(void* data) const;

    // Method to reads the message from data.
    // @param data the message
    // @return the header and the versions are convertible
    bool read(const void* data);

    // constructor
    BHULKsStandardMessage();
  };
}
