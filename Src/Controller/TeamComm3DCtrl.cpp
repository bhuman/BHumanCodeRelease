/**
* @file Controller/TeamComm3DCtrl.cpp
* Declaration of a SimRobot controller that visualizes data from team comm in a SimRobot scene
* @author Colin Graf
*/

#include <QString>
#include <QApplication>
#include <Platform/OpenGL.h>
#ifdef WIN32
#include <windows.h>
#endif

#include "TeamComm3DCtrl.h"
#include "Views/TeamComm3DView.h"
#include "Tools/ProcessFramework/TeamHandler.h"

TeamComm3DCtrl* TeamComm3DCtrl::controller = 0;
SimRobot::Application* TeamComm3DCtrl::application = 0;

TeamComm3DCtrl::TeamComm3DCtrl(SimRobot::Application& simRobot) : currentListener(0), currentRobotData(0), lastMousePressed(0)
{
  controller = this;
  application = &simRobot;

  Global::theStreamHandler = &streamHandler;
  Global::theSettings = &settings;
  Global::theDrawingManager = &drawingManager;
  Global::theDrawingManager3D = &drawingManager3D;
  Global::theDebugRequestTable = &debugRequestTable;
  Global::theDebugOut = &debugMessageQueue.out;

  port[0] = settings.teamPort;
  port[1] = settings.teamPort + 100;
  subnet[0] = subnet[1] = "255.255.255.255";
}

TeamComm3DCtrl::~TeamComm3DCtrl()
{
  qDeleteAll(views);
}

void TeamComm3DCtrl::readTeamPort()
{
  std::string name = application->getFilePath().toUtf8().constData();
  int p = name.find_last_of("\\/");
  if(p >= 0)
    name = name.substr(0, p + 1);

  name += "teamPort.con";
  if(name[0] != '/' && name[0] != '\\' && (name.size() < 2 || name[1] != ':'))
    name = std::string("Scenes\\") + name;
  InBinaryFile stream(name.c_str());
  if(stream.exists())
  {
    std::string line;
    while(!stream.eof())
    {
      line.resize(0);
      while(!stream.eof())
      {
        char c[2] = " ";
        stream >> c[0];
        if(c[0] == '\n')
          break;
        else if(c[0] != '\r')
          line = line + c;
      }
      if(line.find_first_not_of(" ") != std::string::npos)
        executeConsoleCommand(line);
    }
  }
}

void TeamComm3DCtrl::executeConsoleCommand(const std::string& line)
{
  InConfigMemory stream(line.c_str(), line.size());
  std::string buffer;
  stream >> buffer;
  if(buffer == "") // comment
    return;
  else if(buffer == "tc" || buffer == "tc2")
  {
    int i = buffer == "tc" ? 0 : 1;
    stream >> port[i] >> subnet[i];
    if(subnet[i] == "")
      subnet[i] = "255.255.255.255";
  }
  else
  {
    ASSERT(false);
  }
}

bool TeamComm3DCtrl::compile()
{
  readTeamPort();

  // team comm monitor widget
  for(int i = 0; i < 2; ++i)
  {
    TeamComm3DView* view = new TeamComm3DView(QString("port %1").arg(port[i]), i);
    views.append(view);
    application->registerObject(*this, *view, 0/*category*/, 0);
  }

  // get simulated robots
  SimRobot::Object* group = application->resolveObject("RoboCup.robots", SimRobotCore2::compound);
  for(unsigned i = 0, count = application->getObjectChildCount(*group); i < count; ++i)
  {
    SimRobotCore2::Body* robot = (SimRobotCore2::Body*)application->getObjectChild(*group, i);
    QString fullName = robot->getFullName();
    QString name = fullName.mid(fullName.lastIndexOf('.') + 1);
    if(!name.startsWith("robot"))
      continue;
    TeamColor teamColor = name.endsWith("Red") ? red : name.endsWith("Blue") ? blue : numOfTeamColors;
    if(teamColor == numOfTeamColors)
      continue;
    int robotNumber = name.mid(5, 1).toInt();
    if(robotNumber < TeamMateData::firstPlayer || robotNumber >= TeamMateData::numOfPlayers)
      continue;
    PuppetData& puppetData = this->puppetData[teamColor][robotNumber];
    puppetData.name = name.toUtf8().constData();
    puppetData.teamColor = teamColor;
    puppetData.playerNumber = robotNumber;
    puppetData.robot = robot;
    float initialPosition[3];
    float initialRotation[3][3];
    robot->getPose(initialPosition, initialRotation);
    puppetData.initialPosition = Vector3<>(initialPosition[0] * 1000.f, initialPosition[1] * 1000.f, initialPosition[2] * 1000.f);
    puppetData.oracle.init(robot);
  }

  // start udp listener
  for(int i = 0; i < 2; ++i)
  {
    TeamListener& teamListener = this->teamListener[i];
    teamListener.port = (unsigned short) port[i];
    teamListener.teamHandler.start(teamListener.port, subnet[i].c_str());
    //teamListener.teamHandler.startLocal(10000 + i + 1, 100);
  }

#ifdef WIN32
  VERIFY(timeBeginPeriod(1) == TIMERR_NOERROR); // improves precision of getCurrentTime()
#endif

  // activate some drawings
  debugRequestTable.addRequest(DebugRequest("debug drawing 3d:" "representation:SideConfidence"));
  debugRequestTable.addRequest(DebugRequest("debug drawing 3d:" "representation:BallModel"));
  debugRequestTable.addRequest(DebugRequest("debug drawing 3d:" "representation:ObstacleModel"));
  debugRequestTable.addRequest(DebugRequest("debug drawing 3d:" "representation:CombinedWorldModel"));
  return true;
}

void TeamComm3DCtrl::update()
{
  // poll on udp port(s)
  now = SystemCall::getCurrentSystemTime();
  for(int i = 0; i < 2; ++i)
  {
    TeamListener& teamListener = this->teamListener[i];
    TeamHandler& teamHandler = teamListener.teamHandler;
    teamHandler.receive();
    if(!teamListener.in.isEmpty())
    {
      currentListener = &teamListener;
      teamListener.in.handleAllMessages(*this);
    }
    teamListener.in.clear();
    bool sendData = teamListener.ntp.doSynchronization(now, teamListener.out.out);
    if(sendData)
    {
      teamListener.teamHandler.send();
      teamListener.out.clear();
    }
  }

  // update robot position and drawings in the simulated scene
  now = SystemCall::getCurrentSystemTime();
  static const TeamMateData teamMateData;
  for(int teamColor = firstTeamColor; teamColor < numOfTeamColors; ++teamColor)
  {
    Global::getSettings().teamColor = (Settings::TeamColor) (1 - teamColor);
    for(int robotNumber = TeamMateData::firstPlayer; robotNumber < TeamMateData::numOfPlayers; ++robotNumber)
    {
      PuppetData& puppetData = this->puppetData[teamColor][robotNumber];
      Global::getSettings().playerNumber = robotNumber;

      if(puppetData.robot)
      {
        RobotData* robotData = puppetData.robotData;
        if(!robotData || (int) (now - robotData->timeStamp) > (int) teamMateData.networkTimeout || robotData->isPenalized)
        {
          // move robot back to field border
          if(puppetData.online)
          {
            puppetData.oracle.moveRobot(puppetData.initialPosition, Vector3<>(0, 0, pi * (robotData ? -0.5f : 0.5f)), true);
            ((SimRobotCore2::Body*)puppetData.robot)->resetDynamics();
            puppetData.online = false;
            puppetData.jointData.angles[JointData::LShoulderPitch] = 0.f;
            puppetData.jointData.angles[JointData::LShoulderRoll] = 0.f;
            puppetData.jointData.angles[JointData::RShoulderPitch] = 0.f;
            puppetData.jointData.angles[JointData::RShoulderRoll] = 0.f;
            for(std::unordered_map<const char*, DebugDrawing3D>::iterator i = puppetData.drawings3D.begin(), end = puppetData.drawings3D.end(); i != end; ++i)
              i->second.reset();
            puppetData.robotData = 0;
          }
        }
        else
        {
          // update joint angles
          if(!puppetData.online || robotData->jointDataTimeStamp != puppetData.lastJointDataTimeStamp)
          {
            puppetData.online = true;
            puppetData.lastJointDataTimeStamp = robotData->jointDataTimeStamp;
            if(!robotData->jointDataTimeStamp || now - robotData->jointDataTimeStamp >= 2000)
            {
              puppetData.jointData.angles[JointData::LShoulderPitch] = -pi_2;
              puppetData.jointData.angles[JointData::LShoulderRoll] = 0.15f;
              puppetData.jointData.angles[JointData::RShoulderPitch] = -pi_2;
              puppetData.jointData.angles[JointData::RShoulderRoll] = 0.15f;
            }
          }

          // update robot position
          if(puppetData.updateTimeStamp != robotData->timeStamp || !robotData->hasGroundContact)
          {
            Pose2D robotPose = robotData->robotPose;
            if(teamColor == blue)
            {
              static const Pose2D piPose(pi);
              robotPose = piPose + robotPose;
            }

            puppetData.oracle.moveRobot(
                Vector3<>(robotPose.translation.x, robotPose.translation.y, robotData->hasGroundContact ? puppetData.initialPosition.z : puppetData.initialPosition.z + 600.f),
                Vector3<>(0, 0, robotPose.rotation), true);
          }

          // update 3d drawings
          if(puppetData.updateTimeStamp != robotData->timeStamp)
          {
            for(std::unordered_map<const char*, DebugDrawing3D>::iterator i = puppetData.drawings3D.begin(), end = puppetData.drawings3D.end(); i != end; ++i)
              i->second.reset();

            //robotData->robotPose.draw(teamColor == red);
            robotData->ballModel.draw3D(robotData->robotPose);
            robotData->combinedWorldModel.draw();
            robotData->sideConfidence.draw();
            if(puppetData.selected)
              robotData->obstacleModel.draw3D(Pose3D());

            currentRobotData = robotData;
            debugMessageQueue.handleAllMessages(*this);
            debugMessageQueue.clear();

            for(std::unordered_map<const char*, DebugDrawing3D>::iterator i = puppetData.drawings3D.begin(), end = puppetData.drawings3D.end(); i != end; ++i)
            {
              DebugDrawing3D& debugDrawing3D = i->second;
              if(!debugDrawing3D.drawn)
              {
                debugDrawing3D.drawn = true;
                std::string type = drawingManager3D.getDrawingType(i->first);
                if(type != "unknown")
                {
                  QVector<QString> parts;
                  parts.append(puppetData.name.c_str());
                  if(type == "field")
                  {
                    debugDrawing3D.flip = teamColor == blue;
                    parts[0] = "RoboCup";
                  }
                  else if(type == "robot")
                    parts.append("origin");
                  else
                    parts.append(type.c_str());
                  SimRobotCore2::PhysicalObject* object = (SimRobotCore2::PhysicalObject*)application->resolveObject(parts);
                  object->registerDrawing(debugDrawing3D);
                }
              }
            }
          }

          //
          puppetData.updateTimeStamp = robotData->timeStamp;
        }

        JointData jointData;
        puppetData.oracle.getAndSetJointData(puppetData.jointData, jointData);
        if(QApplication::mouseButtons() & Qt::LeftButton)
          lastMousePressed = now;
        bool needPhysics = (int) (now - lastMousePressed) < 500;
        for(int i = 0; i < JointData::numOfJoints; ++i)
          needPhysics |= puppetData.jointData.angles[i] != JointData::off && std::abs(puppetData.jointData.angles[i] - jointData.angles[i]) > 0.001f;
        puppetData.oracle.enablePhysics(needPhysics);
      }
    }
  }
}

void TeamComm3DCtrl::selectedObject(const SimRobot::Object& object)
{
  for(int teamColor = firstTeamColor; teamColor < numOfTeamColors; ++teamColor)
    for(int robotNumber = TeamMateData::firstPlayer; robotNumber < TeamMateData::numOfPlayers; ++robotNumber)
    {
      PuppetData& puppetData = this->puppetData[teamColor][robotNumber];
      puppetData.selected = puppetData.robot == &object;
    }
}

bool TeamComm3DCtrl::handleMessage(InMessage& message)
{
  switch(message.getMessageID())
  {
  case idNTPHeader:
    VERIFY(currentListener->ntp.handleMessage(message));
    message.resetReadPosition();
    {
      unsigned int ipAddress;
      message.bin >> ipAddress;
      currentRobotData = &currentListener->robotData[ipAddress];
      unsigned int sendTimeStamp;
      message.bin >> sendTimeStamp;
      message.bin >> currentRobotData->timeStamp;
      unsigned short messageSize;
      message.bin >> messageSize;
      currentRobotData->packetSizes.add(messageSize + 20 + 8); // 20 = ip header size, 8 = udp header size
      currentRobotData->packetTimeStamps.add(currentRobotData->timeStamp);
      currentRobotData->lastPacketLatency = currentListener->ntp.receiveTimeStamp - currentListener->ntp.getRemoteTimeInLocalTime(currentListener->ntp.sendTimeStamp);
      currentRobotData->ping = currentListener->ntp.getRoundTripLength();
    }
    return true;
  case idNTPIdentifier:
  case idNTPRequest:
  case idNTPResponse:
    return currentListener->ntp.handleMessage(message);

  case idRobot:
    message.bin >> currentRobotData->robotNumber;
    return true;
  case idTeamMateIsPenalized:
    message.bin >> currentRobotData->isPenalized;
    return true;
  case idTeamMateHasGroundContact:
    message.bin >> currentRobotData->hasGroundContact;
    return true;
  case idTeamMateIsUpright:
    message.bin >> currentRobotData->isUpright;
    return true;
  case idTeamMateRobotsModel:
  {
    RobotsModelCompressed robotsModelCompressed;
    message.bin >> robotsModelCompressed;
    currentRobotData->robotsModel = robotsModelCompressed;
    for(size_t i = 0; i < currentRobotData->robotsModel.robots.size(); i++)
    {
      currentRobotData->robotsModel.robots[i].timeStamp = currentListener->ntp.getRemoteTimeInLocalTime(
            currentRobotData->robotsModel.robots[i].timeStamp);
    }
    return true;
  }
  case idTeamMateRobotPose:
  {
    RobotPoseCompressed robotPoseCompressed;
    message.bin >> robotPoseCompressed;
    currentRobotData->robotPose = robotPoseCompressed;
    return true;
  }
  case idTeamMateSideConfidence:
    message.bin >> currentRobotData->sideConfidence;
    return true;
  case idTeamMateBallModel:
  {
    BallModelCompressed ballModelCompressed;
    message.bin >> ballModelCompressed;
    currentRobotData->ballModel = ballModelCompressed;
    if(currentRobotData->ballModel.timeWhenLastSeen)
      currentRobotData->ballModel.timeWhenLastSeen = currentListener->ntp.getRemoteTimeInLocalTime(currentRobotData->ballModel.timeWhenLastSeen);
    if(currentRobotData->ballModel.timeWhenDisappeared)
      currentRobotData->ballModel.timeWhenDisappeared = currentListener->ntp.getRemoteTimeInLocalTime(currentRobotData->ballModel.timeWhenDisappeared);
    return true;
  }
  case idTeamMateBehaviorStatus:
  {
    BehaviorStatus& behaviorStatus = currentRobotData->behaviorStatus;
    message.bin >> behaviorStatus;
    int teamColor = behaviorStatus.teamColor == BehaviorStatus::red ? red : behaviorStatus.teamColor == BehaviorStatus::blue ? blue : numOfTeamColors;
    int& robotNumber = currentRobotData->robotNumber;
    if(robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers &&
       teamColor >= firstTeamColor && teamColor < numOfTeamColors)
    {
      if(currentRobotData->puppetData)
        currentRobotData->puppetData->robotData = 0;
      puppetData[teamColor][robotNumber].robotData = currentRobotData;
      currentRobotData->puppetData = &puppetData[teamColor][robotNumber];
    }
    return true;
  }
  case idRobotHealth:
    message.bin >> currentRobotData->robotHealth;
    currentRobotData->goalPercepts.add(currentRobotData->robotHealth.goalPercepts);
    currentRobotData->ballPercepts.add(currentRobotData->robotHealth.ballPercepts);
    currentRobotData->linePercepts.add(currentRobotData->robotHealth.linePercepts);
    currentRobotData->robotHealthTimeStamps.add(currentRobotData->timeStamp);
    return true;
  case idTeamMateGoalPercept:
  {
    GoalPercept& goalPercept = currentRobotData->goalPercept;
    message.bin >> goalPercept;
    if(goalPercept.timeWhenGoalPostLastSeen)
      goalPercept.timeWhenGoalPostLastSeen = currentListener->ntp.getRemoteTimeInLocalTime(goalPercept.timeWhenGoalPostLastSeen);
    if(goalPercept.timeWhenCompleteGoalLastSeen)
      goalPercept.timeWhenCompleteGoalLastSeen = currentListener->ntp.getRemoteTimeInLocalTime(goalPercept.timeWhenCompleteGoalLastSeen);
    return true;
  }
  case idLinePercept:
    message.bin >> currentRobotData->linePercept;
    return true;
  case idMotionRequest:
    message.bin >> currentRobotData->motionRequest;;
    return true;
  case idTeamMateCombinedWorldModel:
    message.bin >> currentRobotData->combinedWorldModel;
    return true;
  case idTeamMateFreePartOfOpponentGoalModel:
    message.bin >> currentRobotData->freePartOfOpponentGoalModel;
    return true;
  case idTeamMateObstacleModel:
  {
    ObstacleModelCompressed theObstacleModelCompressed;
    message.bin >> theObstacleModelCompressed;
    currentRobotData->obstacleModel = theObstacleModelCompressed;
    return true;
  }
  case idSensorData:
    message.bin >> currentRobotData->sensorData;
    return true;
  case idJointData:
    message.bin >> currentRobotData->jointData;
    currentRobotData->jointDataTimeStamp = currentRobotData->timeStamp;
    return true;

  case idDebugDrawing3D:
    {
      char shapeType, id;
      message.bin >> shapeType >> id;
      currentRobotData->puppetData->drawings3D[drawingManager3D.getDrawingName(id)].addShapeFromQueue(message, (::Drawings3D::ShapeType)shapeType, 0);
    }
    return true;
  }
  return false;
}
