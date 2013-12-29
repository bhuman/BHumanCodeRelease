/**
* @file Controller/TeamComm3DCtrl.cpp
* Declaration of a SimRobot controller that visualizes data from team comm in a SimRobot scene
* @author Colin Graf
*/

#include <QList>
#include <map>

#include "SimRobotCore2.h"
#include "Controller/Oracle.h"
#include "Tools/Settings.h"
#include "Tools/NTP.h"
#include "Tools/MessageQueue/MessageQueue.h"
#include "Tools/ProcessFramework/TeamHandler.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Perception/LinePercept.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Modeling/CombinedWorldModel.h"
#include "Representations/Modeling/FreePartOfOpponentGoalModel.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Tools/RingBuffer.h"
#include "Tools/RingBufferWithSum.h"
#include "Tools/Streams/StreamHandler.h"
#include "Controller/Visualization/DebugDrawing3D.h"

class TeamComm3DCtrl : public SimRobot::Module, public MessageHandler
{
public:
  static TeamComm3DCtrl* controller;
  static SimRobot::Application* application; /**< The interface to the SimRobot GUI */

  TeamComm3DCtrl(SimRobot::Application& simRobot);

private:
  ENUM(TeamColor,
    firstTeamColor,
    red = firstTeamColor,
    blue
  );

  class RobotData;

  class PuppetData
  {
  public:
    std::string name;
    int playerNumber;
    int teamColor;
    bool selected;
    bool online;
    SimRobotCore2::Object* robot;
    Vector3<> initialPosition;
    Oracle oracle;
    JointData jointData;
    unsigned int updateTimeStamp;
    unsigned int lastJointDataTimeStamp;
    RobotData* robotData;

    std::unordered_map<const char*, DebugDrawing3D> drawings3D; /**< Buffers for 3d drawings from the debug queue. */

    PuppetData() : selected(false), online(false), robot(0), updateTimeStamp(0), lastJointDataTimeStamp(0), robotData(0) {}
  };

  class RobotData
  {
  public:
    PuppetData* puppetData;
    unsigned int timeStamp;
    unsigned int ping;
    unsigned int lastPacketLatency;
    RingBufferWithSum<unsigned int, 100> packetSizes;
    RingBuffer<unsigned int, 1000> packetTimeStamps;
    RingBuffer<unsigned int, 600> goalPercepts;
    RingBuffer<unsigned int, 600> ballPercepts;
    RingBuffer<unsigned int, 600> linePercepts;
    RingBuffer<unsigned int, 600> robotHealthTimeStamps;

    int robotNumber;
    RobotPose robotPose;
    SideConfidence sideConfidence;
    BallModel ballModel;
    bool hasGroundContact;
    bool isPenalized;
    bool isUpright;
    RobotsModel robotsModel;
    BehaviorStatus behaviorStatus;

    // additional (optional) stuff not used for team play
    RobotHealth robotHealth;
    GoalPercept goalPercept;
    LinePercept linePercept;
    MotionRequest motionRequest;
    CombinedWorldModel combinedWorldModel;
    FreePartOfOpponentGoalModel freePartOfOpponentGoalModel;
    ObstacleModel obstacleModel;
    SensorData sensorData;
    JointData jointData;
    unsigned int jointDataTimeStamp;

    RobotData() : puppetData(0), robotNumber(0), jointDataTimeStamp(0) {}
  };

  class TeamListener
  {
  public:
    unsigned short port;
    MessageQueue in, out;
    TeamHandler teamHandler;
    NTP ntp;
    std::map<unsigned int, RobotData> robotData;

    TeamListener() : teamHandler(in, out) {}
  };

  Settings settings;
  StreamHandler streamHandler;
  DrawingManager drawingManager;
  DrawingManager3D drawingManager3D;
  DebugRequestTable debugRequestTable;
  MessageQueue debugMessageQueue;

  int port[2];
  std::string subnet[2];

  QList<SimRobot::Object*> views; /**< List of registered views */
  TeamListener teamListener[2];
  TeamListener* currentListener;
  RobotData* currentRobotData;
  PuppetData puppetData[numOfTeamColors][TeamMateData::numOfPlayers];
  unsigned int now;
  unsigned int lastMousePressed;

  virtual ~TeamComm3DCtrl();

  virtual bool compile();
  virtual void update();
  virtual void selectedObject(const SimRobot::Object& object);

  /**
  * Reads the teamPort.con file
  */
  void readTeamPort();

  /**
  * Executes a command from the team port
  */
  void executeConsoleCommand(const std::string& line);

  /**
  * Called from a MessageQueue to distribute messages.
  * Use message.getMessageID to decide if the message is relavant for
  * the MesssageHandler derivate.
  * Use message.bin, message.text or message.config as In streams to get the data from.
  * @param message The message that can be read.
  * @return true if the message was read (handled).
  */
  bool handleMessage(InMessage& message);

  friend class TeamComm3DWidget;
};
