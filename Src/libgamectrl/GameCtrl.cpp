/**
 * @file GameCtrl.cpp
 * Implementation of a NAOqi library that communicates with the GameController.
 * It provides the data received in ALMemory.
 * It also implements the official button interface and sets the LEDs as
 * specified in the rules.
 *
 * @author Thomas Röfer
 */

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wconversion"
#endif
#include <alcore/altypes.h>
#include <alcore/alerror.h>
#include <alcommon/albroker.h>
#include <alcommon/alproxy.h>
#include <alproxies/dcmproxy.h>
#include <alproxies/almemoryproxy.h>
#ifdef __clang__
#pragma clang diagnostic pop
#endif

#include "RoboCupGameControlData.h"
#include "UdpComm.h"

static const int BUTTON_DELAY = 30; /**< Button state changes are ignored when happening in less than 30 ms. */
static const int GAMECONTROLLER_TIMEOUT = 2000; /**< Connected to GameController when packet was received within the last 2000 ms. */
static const int ALIVE_DELAY = 500; /**< Send an alive signal every 500 ms. */

enum Button
{
  chest,
  leftFootLeft,
  leftFootRight,
  rightFootLeft,
  rightFootRight,
  numOfButtons
};

static const char* buttonNames[] =
{
  "Device/SubDeviceList/ChestBoard/Button/Sensor/Value",
  "Device/SubDeviceList/LFoot/Bumper/Left/Sensor/Value",
  "Device/SubDeviceList/LFoot/Bumper/Right/Sensor/Value",
  "Device/SubDeviceList/RFoot/Bumper/Left/Sensor/Value",
  "Device/SubDeviceList/RFoot/Bumper/Right/Sensor/Value"
};

enum LED
{
  chestRed,
  chestGreen,
  chestBlue,
  leftFootRed,
  leftFootGreen,
  leftFootBlue,
  rightFootRed,
  rightFootGreen,
  rightFootBlue,
  numOfLEDs
};

static const char* ledNames[] =
{
  "ChestBoard/Led/Red/Actuator/Value",
  "ChestBoard/Led/Green/Actuator/Value",
  "ChestBoard/Led/Blue/Actuator/Value",
  "LFoot/Led/Red/Actuator/Value",
  "LFoot/Led/Green/Actuator/Value",
  "LFoot/Led/Blue/Actuator/Value",
  "RFoot/Led/Red/Actuator/Value",
  "RFoot/Led/Green/Actuator/Value",
  "RFoot/Led/Blue/Actuator/Value"
};

class GameCtrl : public AL::ALModule
{
private:
  static GameCtrl* theInstance; /**< The only instance of this class. */

  AL::DCMProxy* proxy; /**< Gives access to the DCM. */
  AL::ALMemoryProxy* memory; /**< Give access to ALMemory. */
  AL::ALValue ledRequest; /**< Prepared request to set the LEDs. */
  UdpComm* udp; /**< The socket used to communicate. */
  const float* buttons[numOfButtons]; /**< Pointers to where ALMemory stores the current button states. */
  const int* playerNumber; /** Points to where ALMemory stores the player number. */
  const int* teamNumberPtr; /** Points to where ALMemory stores the team number. The number be set to 0 after it was read. */
  const int* defaultTeamColour; /** Points to where ALMemory stores the default team color. */
  int teamNumber; /**< The team number. */
  RoboCupGameControlData gameCtrlData; /**< The local copy of the GameController packet. */
  uint8 previousState; /**< The game state during the previous cycle. Used to detect when LEDs have to be updated. */
  uint8 previousSecondaryState; /**< The secondary game state during the previous cycle. Used to detect when LEDs have to be updated. */
  uint8 previousKickOffTeam; /**< The kick-off team during the previous cycle. Used to detect when LEDs have to be updated. */
  uint8 previousTeamColour; /**< The team colour during the previous cycle. Used to detect when LEDs have to be updated. */
  uint16 previousPenalty; /**< The penalty set during the previous cycle. Used to detect when LEDs have to be updated. */
  bool previousChestButtonPressed; /**< Whether the chest button was pressed during the previous cycle. */
  bool previousLeftFootButtonPressed; /**< Whether the left foot bumper was pressed during the previous cycle. */
  bool previousRightFootButtonPressed; /**< Whether the right foot bumper was pressed during the previous cycle. */
  unsigned whenChestButtonStateChanged; /**< When last state change of the chest button occured (DCM time). */
  unsigned whenLeftFootButtonStateChanged; /**< When last state change of the left foot bumper occured (DCM time). */
  unsigned whenRightFootButtonStateChanged; /**< When last state change of the right foot bumper occured (DCM time). */
  unsigned whenPacketWasReceived; /**< When the last GameController packet was received (DCM time). */
  unsigned whenPacketWasSent; /**< When the last return packet was sent to the GameController (DCM time). */

  /**
   * Resets the internal state when an application was just started.
   */
  void init()
  {
    previousState = (uint8) -1;
    previousSecondaryState = (uint8) -1;
    previousKickOffTeam = (uint8) -1;
    previousTeamColour = (uint8) -1;
    previousPenalty = (uint16) -1;
    previousChestButtonPressed = false;
    previousLeftFootButtonPressed = false;
    previousRightFootButtonPressed = false;
    whenChestButtonStateChanged = 0;
    whenLeftFootButtonStateChanged = 0;
    whenRightFootButtonStateChanged = 0;
    whenPacketWasReceived = 0;
    whenPacketWasSent = 0;
    memset(&gameCtrlData, 0, sizeof(gameCtrlData));
  }

  /**
   * Sets the LEDs whenever the state they visualize changes.
   * Regularily sends the return packet to the GameController.
   */
  void handleOutput()
  {
    unsigned now = (unsigned) proxy->getTime(0);

    if(teamNumber && *playerNumber &&
       *playerNumber <= gameCtrlData.playersPerTeam &&
       (gameCtrlData.teams[0].teamNumber == teamNumber ||
        gameCtrlData.teams[1].teamNumber == teamNumber))
    {
      const TeamInfo& team = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == teamNumber ? 0 : 1];
      if(gameCtrlData.state != previousState ||
         gameCtrlData.secondaryState != previousSecondaryState ||
         gameCtrlData.kickOffTeam != previousKickOffTeam ||
         team.teamColour != previousTeamColour ||
         team.players[*playerNumber - 1].penalty != previousPenalty)
      {
        if(team.teamColour == TEAM_BLUE)
          setLED(leftFootRed, 0.f, 0.f, 1.f);
        else
          setLED(leftFootRed, 1.f, 0.f, 0.f);

        if(gameCtrlData.state == STATE_INITIAL &&
           gameCtrlData.secondaryState == STATE2_PENALTYSHOOT &&
           gameCtrlData.kickOffTeam == team.teamColour)
          setLED(rightFootRed, 0.f, 1.f, 0.f);
        else if(gameCtrlData.state == STATE_INITIAL &&
                gameCtrlData.secondaryState == STATE2_PENALTYSHOOT &&
                gameCtrlData.kickOffTeam != team.teamColour)
          setLED(rightFootRed, 1.f, 1.0f, 0.f);
        else if(now - whenPacketWasReceived < GAMECONTROLLER_TIMEOUT &&
                gameCtrlData.state <= STATE_SET &&
                gameCtrlData.kickOffTeam == team.teamColour)
          setLED(rightFootRed, 1.f, 1.f, 1.f);
        else
          setLED(rightFootRed, 0.f, 0.f, 0.f);

        if(team.players[*playerNumber - 1].penalty != PENALTY_NONE)
          setLED(chestRed, 1.f, 0.f, 0.f);
        else
          switch(gameCtrlData.state)
          {
            case STATE_READY:
              setLED(chestRed, 0.f, 0.f, 1.f);
              break;
            case STATE_SET:
              setLED(chestRed, 1.f, 1.0f, 0.f);
              break;
            case STATE_PLAYING:
              setLED(chestRed, 0.f, 1.f, 0.f);
              break;
            default:
              setLED(chestRed, 0.f, 0.f, 0.f);
          }

        ledRequest[4][0] = (int) now;
        proxy->setAlias(ledRequest);

        previousState = gameCtrlData.state;
        previousSecondaryState = gameCtrlData.secondaryState;
        previousKickOffTeam = gameCtrlData.kickOffTeam;
        previousTeamColour = team.teamColour;
        previousPenalty = team.players[*playerNumber - 1].penalty;
      }

      if(now - whenPacketWasReceived < GAMECONTROLLER_TIMEOUT &&
         now - whenPacketWasSent >= ALIVE_DELAY &&
         send(GAMECONTROLLER_RETURN_MSG_ALIVE))
        whenPacketWasSent = now;
    }
  }

  /**
   * Sets states in the LED request.
   * @param led The index of the red channel of an RGB LED.
   * @param red The red intensity [0..1].
   * @param green The green intensity [0..1].
   * @param blue The blue intensity [0..1].
   */
  void setLED(LED led, float red, float green, float blue)
  {
    ledRequest[5][led][0] = red;
    ledRequest[5][led + 1][0] = green;
    ledRequest[5][led + 2][0] = blue;
  }

  /**
   * Handles the button interface.
   * Resets the internal state when a new team number was set.
   * Receives packets from the GameController.
   * Initializes gameCtrlData when teamNumber and playerNumber are available.
   */
  void handleInput()
  {
    unsigned now = (unsigned) proxy->getTime(0);

    if(*teamNumberPtr != 0)
    { // new team number was set -> reset internal structure
      teamNumber = *teamNumberPtr;
      memory->insertData("GameCtrl/teamNumber", 0);
      init();
    }

    if(receive())
    {
      if(!whenPacketWasReceived)
        previousState = (uint8) -1; // force LED update on first packet received
      whenPacketWasReceived = now;
      publish();
    }

    if(teamNumber && *playerNumber)
    {
      // init gameCtrlData if invalid
      if(gameCtrlData.teams[0].teamNumber != teamNumber &&
         gameCtrlData.teams[1].teamNumber != teamNumber)
      {
        uint8 teamColour = *defaultTeamColour == TEAM_RED ? 1 : 0;
        gameCtrlData.teams[teamColour].teamNumber = (uint8) teamNumber;
        gameCtrlData.teams[teamColour].teamColour = teamColour;
        gameCtrlData.teams[1 - teamColour].teamColour = 1 - teamColour;
        if(!gameCtrlData.playersPerTeam)
          gameCtrlData.playersPerTeam = (uint8) *playerNumber; // we don't know better
        publish();
      }
      TeamInfo& team = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == teamNumber ? 0 : 1];

      if(*playerNumber <= gameCtrlData.playersPerTeam)
      {
        bool chestButtonPressed = *buttons[chest] != 0.f;
        if(chestButtonPressed != previousChestButtonPressed && now - whenChestButtonStateChanged >= BUTTON_DELAY)
        {
          if(chestButtonPressed && whenChestButtonStateChanged) // ignore first press, e.g. for getting up
          {
            RobotInfo& player = team.players[*playerNumber - 1];
            if(player.penalty == PENALTY_NONE)
            {
              player.penalty = PENALTY_MANUAL;
              if(now - whenPacketWasReceived < GAMECONTROLLER_TIMEOUT &&
                 send(GAMECONTROLLER_RETURN_MSG_MAN_PENALISE))
                whenPacketWasSent = now;
            }
            else
            {
              player.penalty = PENALTY_NONE;
              gameCtrlData.state = STATE_PLAYING;
              if(now - whenPacketWasReceived < GAMECONTROLLER_TIMEOUT &&
                 send(GAMECONTROLLER_RETURN_MSG_MAN_UNPENALISE))
                whenPacketWasSent = now;
            }
            publish();
          }

          previousChestButtonPressed = chestButtonPressed;
          whenChestButtonStateChanged = now;
        }

        if(gameCtrlData.state == STATE_INITIAL)
        {
          bool leftFootButtonPressed = *buttons[leftFootLeft] != 0.f || *buttons[leftFootRight] != 0.f;
          if(leftFootButtonPressed != previousLeftFootButtonPressed && now - whenLeftFootButtonStateChanged >= BUTTON_DELAY)
          {
            if(leftFootButtonPressed)
            {
              team.teamColour ^= 1;
              gameCtrlData.kickOffTeam ^= 1;
              publish();
            }
            previousLeftFootButtonPressed = leftFootButtonPressed;
            whenLeftFootButtonStateChanged = now;
          }

          bool rightFootButtonPressed = *buttons[rightFootLeft] != 0.f || *buttons[rightFootRight] != 0.f;
          if(rightFootButtonPressed != previousRightFootButtonPressed && now - whenRightFootButtonStateChanged >= BUTTON_DELAY)
          {
            if(rightFootButtonPressed)
            {
              if(gameCtrlData.secondaryState == STATE2_NORMAL)
              {
                gameCtrlData.secondaryState = STATE2_PENALTYSHOOT;
                gameCtrlData.kickOffTeam = team.teamColour;
              }
              else if(gameCtrlData.kickOffTeam == team.teamColour)
                gameCtrlData.kickOffTeam ^= 1;
              else
                gameCtrlData.secondaryState = STATE2_NORMAL;
              publish();
            }
            previousRightFootButtonPressed = rightFootButtonPressed;
            whenRightFootButtonStateChanged = now;
          }
        }
      }
      else
        fprintf(stderr, "Player number %d too big. Maximum number is %d.\n", *playerNumber, gameCtrlData.playersPerTeam);
    }
  }

  /**
   * Sends the return packet to the GameController.
   * @param message The message contained in the packet (GAMECONTROLLER_RETURN_MSG_MAN_PENALISE,
   *                GAMECONTROLLER_RETURN_MSG_MAN_UNPENALISE or GAMECONTROLLER_RETURN_MSG_ALIVE).
   */
  bool send(uint32 message)
  {
    RoboCupGameControlReturnData returnPacket;
    memcpy(returnPacket.header, GAMECONTROLLER_RETURN_STRUCT_HEADER, sizeof(returnPacket.header));
    returnPacket.version = GAMECONTROLLER_RETURN_STRUCT_VERSION;
    returnPacket.team = (uint8) teamNumber;
    returnPacket.player = (uint8) *playerNumber;
    returnPacket.message = message;
    return !udp || udp->write((const char*) &returnPacket, sizeof(returnPacket));
  }

  /**
   * Receives a packet from the GameController.
   * Packets are only accepted when the team number is know (nonzero) and
   * they are addressed to this team.
   */
  bool receive()
  {
    bool received = false;
    int size;
    RoboCupGameControlData buffer;
    while(udp && (size = udp->read((char*) &buffer, sizeof(buffer))) > 0)
    {
      if(size == sizeof(buffer) &&
         !std::memcmp(&buffer, GAMECONTROLLER_STRUCT_HEADER, 4) &&
         buffer.version == GAMECONTROLLER_STRUCT_VERSION &&
         teamNumber &&
         (buffer.teams[0].teamNumber == teamNumber ||
          buffer.teams[1].teamNumber == teamNumber))
      {
        gameCtrlData = buffer;
        received = true;
      }
    }
    return received;
  }

  /**
   * Publishes the current state of the GameController packet in ALMemory.
   */
  void publish()
  {
    AL::ALValue value((const char*) &gameCtrlData, sizeof(gameCtrlData));
    memory->insertData("GameCtrl/RoboCupGameControlData", value);
  }

  /**
   * Close all resources acquired.
   * Called when initialization failed or during destruction.
   */
  void close()
  {
    if(udp)
      delete udp;
    if(proxy)
    {
      proxy->getGenericProxy()->getModule()->removeAllPreProcess();
      proxy->getGenericProxy()->getModule()->removeAllPostProcess();
      delete proxy;
    }
    if(memory)
      delete memory;
  }

  /**
   * The method is called by NAOqi immediately before it communicates with the chest board.
   * It sets all the actuators.
   */
  static void onPreProcess()
  {
    theInstance->handleOutput();
  }

  /**
   * The method is called by NAOqi immediately after it communicated with the chest board.
   * It reads all sensors.
   */
  static void onPostProcess()
  {
    theInstance->handleInput();
  }

public:
  /**
   * The constructor sets up the structures required to communicate with NAOqi.
   * @param pBroker A NAOqi broker that allows accessing other NAOqi modules.
   */
  GameCtrl(boost::shared_ptr<AL::ALBroker> pBroker)
  : ALModule(pBroker, "GameCtrl"),
    proxy(0),
    memory(0),
    udp(0),
    teamNumber(0)
  {
    setModuleDescription("A module that provides packets from the GameController.");

    assert(numOfButtons == sizeof(buttonNames) / sizeof(*buttonNames));
    assert(numOfLEDs == sizeof(ledNames) / sizeof(*ledNames));

    init();

    try
    {
      memory = new AL::ALMemoryProxy(pBroker);
      proxy = new AL::DCMProxy(pBroker);

      AL::ALValue params;
      AL::ALValue result;
      params.arraySetSize(1);
      params.arraySetSize(2);

      params[0] = std::string("leds");
      params[1].arraySetSize(numOfLEDs);
      for(int i = 0; i < numOfLEDs; ++i)
        params[1][i] = std::string(ledNames[i]);
      result = proxy->createAlias(params);
      assert(result == params);

      ledRequest.arraySetSize(6);
      ledRequest[0] = std::string("leds");
      ledRequest[1] = std::string("ClearAll");
      ledRequest[2] = std::string("time-separate");
      ledRequest[3] = 0;
      ledRequest[4].arraySetSize(1);
      ledRequest[5].arraySetSize(numOfLEDs);
      for(int i = 0; i < numOfLEDs; ++i)
        ledRequest[5][i].arraySetSize(1);

      for(int i = 0; i < numOfButtons; ++i)
        buttons[i] = (float*) memory->getDataPtr(buttonNames[i]);

      playerNumber = (int*) memory->getDataPtr("GameCtrl/playerNumber");
      teamNumberPtr = (int*) memory->getDataPtr("GameCtrl/teamNumber");
      defaultTeamColour = (int*) memory->getDataPtr("GameCtrl/teamColour");

      // register "onPreProcess" and "onPostProcess" callbacks
      theInstance = this;
      proxy->getGenericProxy()->getModule()->atPreProcess(&onPreProcess);
      proxy->getGenericProxy()->getModule()->atPostProcess(&onPostProcess);

      udp = new UdpComm();
      if(!udp->setBlocking(false) ||
         !udp->setBroadcast(true) ||
         !udp->bind("0.0.0.0", GAMECONTROLLER_PORT) ||
         !udp->setTarget(UdpComm::getWifiBroadcastAddress(), GAMECONTROLLER_PORT) ||
         !udp->setLoopback(false))
      {
        fprintf(stderr, "libgamectrl: Could not open UDP port\n");
        delete udp;
        udp = 0;
        // continue, because button interface will still work
      }

      publish();
    }
    catch(AL::ALError& e)
    {
      fprintf(stderr, "libgamectrl: %s\n", e.toString().c_str());
      close();
    }
  }

  /**
   * Close all resources acquired.
   */
  ~GameCtrl()
  {
    close();
  }
};

GameCtrl* GameCtrl::theInstance = 0;

/**
 * This method is called by NAOqi when loading this library.
 * Creates an instance of class GameCtrl.
 * @param pBroker A NAOqi broker that allows accessing other NAOqi modules.
 */
extern "C" int _createModule(boost::shared_ptr<AL::ALBroker> pBroker)
{
  AL::ALModule::createModule<GameCtrl>(pBroker);
  return 0;
}
