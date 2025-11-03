/**
 * @file SimulatedNao/ConsoleRoboCupCtrl.cpp
 *
 * This file implements the class ConsoleRoboCupCtrl.
 *
 * @author Thomas Röfer
 */

#include "ConsoleRoboCupCtrl.h"
#include "Modules/Configuration/StaticInitialPoseProvider/StaticInitialPoseProvider.h"
#include "SimulatedNao/BHToolBar.h"
#include "SimulatedNao/ControllerRobot.h"
#include "SimulatedNao/LocalConsole.h"
#include "SimulatedNao/RemoteConsole.h"
#include "SimulatedNao/Views/ConsoleView.h"
#include "SimulatedNao/TestCommandParser.h"
#include "Platform/File.h"
#include "Platform/Time.h"
#include "Streaming/FunctionList.h"
#include "Streaming/InStreams.h"
#include "Streaming/OutStreams.h"
#include "Representations/BehaviorControl/SkillRequest.h"

#include <SimRobotEditor.h>

#include <QDir>
#include <QDirIterator>
#include <QFileDialog>
#include <QInputDialog>
#include <QRegularExpression>
#include <QSettings>

#include <algorithm>
#include <cctype>
#include <functional>
#include <iostream>
#include <regex>
#include <string>

#define FRAMES_PER_SECOND 60

ConsoleRoboCupCtrl::ConsoleRoboCupCtrl(SimRobot::Application& application) :
  RoboCupCtrl(application), calculateImageFps(FRAMES_PER_SECOND),
  mode(SystemCall::simulatedRobot), currentCompletionIndex(completion.end()),
  toolBar(*this)
{
  if(!application.isSimResetting())
    application.getLayoutSettings().setValue("Run", true);

  consoleView = new ConsoleView("Console", *this);
  addView(consoleView, nullptr, SimRobot::Flag::verticalTitleBar);
  addView(new ConsoleView("Console.Pad", *this, true), consoleView, SimRobot::Flag::verticalTitleBar);
}

bool ConsoleRoboCupCtrl::compile()
{
  if(!RoboCupCtrl::compile())
    return false;

  if(is2D)
    calculateImage = false;

  if(!robots.empty())
    selected.push_back((*robots.begin())->getRobotConsole());

  start();

  std::string scenePath = application->getFilePath().toStdString();
  std::string baseConPath = std::filesystem::path(scenePath).replace_extension("con").string();
  std::string userConPath;
  callPath = std::string(File::getBHDir()) + "/Config/Scenes";

  // If the test mode is enabled, load the test configuration file
  if(gameController.isTestActive())
  {
    baseConPath = gameController.getPrimaryConfigPath();
    userConPath = gameController.getSecondaryConfigPath();
    printLn("Test mode enabled. Using test configuration.");
    printLn("Files stored in " + gameController.getTestDirPath().string());
  }

  // Execute the scene configuration file
  executeFile("", baseConPath, false, nullptr);

  // Execute the user's test configuration file
  if(!userConPath.empty())
  {
    executeFile("", userConPath, false, nullptr);
  }

  for(ControllerRobot* robot : robots)
    robot->getRobotConsole()->handleConsole("endOfStartScript");
  for(RemoteConsole* remoteRobot : remoteRobots)
    remoteRobot->handleConsole("endOfStartScript");
  return true;
}

void ConsoleRoboCupCtrl::link()
{
  SimRobotEditor::Editor* editor = static_cast<SimRobotEditor::Editor*>(application->resolveObject("Editor"));
  if(editor)
  {
    QFileInfo fileInfo(application->getFilePath());
    editor->addFile(fileInfo.path() + "/" + fileInfo.baseName() + ".con", "call[ ]+([\\\\/a-z0-9\\.\\-_]+)");
  }
}

ConsoleRoboCupCtrl::~ConsoleRoboCupCtrl()
{
  for(RemoteConsole* remoteRobot : remoteRobots)
    remoteRobot->announceStop();

  for(RemoteConsole* remoteRobot : remoteRobots)
  {
    remoteRobot->stop();
    delete remoteRobot;
  }

  stop();

  mode = SystemCall::simulatedRobot;
  Global::theSettings = nullptr;
  File::clearSearchPath();
}

void ConsoleRoboCupCtrl::update()
{
  {
    SYNC;
    for(const std::string& textMessage : textMessages)
    {
      if(textMessage == "_cls")
        consoleView->clear();
      else if(newLine || &textMessage != &*textMessages.rend())
        consoleView->printLn(textMessage.c_str());
      else
        consoleView->print(textMessage.c_str());
    }
    textMessages.clear();
  }

  statusText.clear();

  RoboCupCtrl::update();

  for(RemoteConsole* remoteRobot : remoteRobots)
    remoteRobot->update();

  application->setStatusMessage(statusText);

  if(completion.empty())
    createCompletion();

  std::string msg = gameController.getConsoleMessage();
  if(!msg.empty())
  {
    printLn(msg);
  }
}

bool ConsoleRoboCupCtrl::executeFile(const std::string& name1, const std::string& name2,
                                     bool printError, RobotConsole* console)
{
  if(nesting == 10)
    printLn("Nesting Error");
  else
  {
    ++nesting;
    bool print = false;
    for(std::string name : std::vector<std::string>({name1, name2}))
    {
      if(name.empty())
      {
        print = printError;
        continue;
      }

      if(!File::hasExtension(name))
        name = name + ".con";
      if(!File::isAbsolute(name))
        name = callPath + "/" + name;
      InBinaryFile stream(name);
      if(!stream.exists())
      {
        if(print)
          printLn(name + " not found");
        print = printError;
      }
      else
      {
        const std::string callPathBackup = callPath;
        callPath = std::filesystem::path(name).parent_path().string();
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
          {
            if(!gameController.isConfigCommandAllowed(line))
            {
              continue;
            }
            if(executeConsoleCommand(line, console))
              console = nullptr;
            std::this_thread::yield();
          }
        }
        callPath = callPathBackup;
        break;
      }
    }
    --nesting;
  }
  return !console;
}

void ConsoleRoboCupCtrl::selectedObject(const SimRobot::Object& obj)
{
  const QString& fullName = obj.getFullName();
  const std::string robotName = fullName.mid(fullName.lastIndexOf('.') + 1).toUtf8().constData();

  for(ControllerRobot* robot : robots)
    if(robotName == robot->getName())
    {
      selected.clear();
      selected.push_back(robot->getRobotConsole());
      printLn("robot " + robot->getName());
      return;
    }
}

void ConsoleRoboCupCtrl::pressedKey(int key, bool pressed)
{
  if(key > 10)
    for(RobotConsole* selectedConsole : selected)
      selectedConsole->handleKeyEvent(key - 11, pressed);
}

SystemCall::Mode ConsoleRoboCupCtrl::getMode() const
{
  std::thread::id threadId = Thread::getCurrentId();
  for(const ControllerRobot* robot : robots)
    if(robot->getRobotConsole())
      for(const ThreadFrame* robotThread : *robot)
        if(robotThread->getId() == threadId)
          return robot->getRobotConsole()->mode;

  return mode;
}

void ConsoleRoboCupCtrl::setRepresentation(const std::string& representationName, const Streamable& representation)
{
  OutMapMemory memory(true, 16384);
  memory << representation;

  std::string command = "set representation:" + representationName + " " + memory.data();
  executeConsoleCommand(command);
}

bool ConsoleRoboCupCtrl::executeConsoleCommand(std::string command, RobotConsole* console)
{
  showInputDialog(command);
  std::string buffer;
  InConfigMemory stream(command.c_str(), command.size());
  stream >> buffer;
  if(buffer.empty()) // comment
    return false;
  else if(buffer == "call")
  {
    std::string optionalFile;
    stream >> buffer >> optionalFile;
    executeFile(optionalFile, buffer, true, console);
  }
  else if(buffer == "help" || buffer == "?")
    help(stream);
  else if(buffer == "robot")
  {
    auto getNumber = [](const std::string& str) -> int
    {
      int result = 0;
      bool foundDigit = false;

      for(char ch : str)
      {
        if(std::isdigit(ch))
        {
          result = result * 10 + (ch - '0');
          foundDigit = true;
        }
        else if(foundDigit)
        {
          break;
        }
      }
      return result;
    };
    stream >> buffer;
    if(buffer == "?")
    {
      for(const ControllerRobot* robot : robots)
        print(robot->getName() + " ");
      for(const RemoteConsole* remoteRobot : remoteRobots)
        print(remoteRobot->getName() + " ");
      printLn("");
      return true;
    }
    else if(buffer == "all")
    {
      selected.clear();
      for(const ControllerRobot* robot : robots)
      {
        selected.push_back(robot->getRobotConsole());
      }
      for(RemoteConsole* remoteRobot : remoteRobots)
        selected.push_back(remoteRobot);
      return true;
    }
    else if(buffer == "teamA")
    {
      selected.clear();
      for(const ControllerRobot* robot : robots)
      {
        if(getNumber(robot->getName()) <= Settings::highestValidPlayerNumber)
        {
          selected.push_back(robot->getRobotConsole());
        }
      }
      for(RemoteConsole* remoteRobot : remoteRobots)
      {
        if(getNumber(remoteRobot->getName()) <= Settings::highestValidPlayerNumber)
        {
          selected.push_back(remoteRobot);
        }
      }
      return true;
    }
    else if(buffer == "teamB")
    {
      selected.clear();
      for(const ControllerRobot* robot : robots)
      {
        if(getNumber(robot->getName()) > Settings::highestValidPlayerNumber)
        {
          selected.push_back(robot->getRobotConsole());
        }
      }
      for(RemoteConsole* remoteRobot : remoteRobots)
      {
        if(getNumber(remoteRobot->getName()) > Settings::highestValidPlayerNumber)
        {
          selected.push_back(remoteRobot);
        }
      }
      return true;
    }
    else
    {
      selected.clear();
      while(true)
      {
        if(!selectRobot(buffer))
          break;
        else if(stream.getEof())
          return true;
        stream >> buffer;
      }
    }
    printLn("Syntax Error");
  }
  else if(buffer == "ar")
  {
    unsigned mask = 0u;
    while(true)
    {
      stream >> buffer;
      if(buffer.empty() || buffer == "on" || buffer == "off")
        break;
      int automaticReferee = TypeRegistry::getEnumValue(typeid(GameController::AutomaticReferee).name(), buffer);
      if(automaticReferee < 0)
      {
        printLn("Syntax Error");
        break;
      }
      mask |= bit(automaticReferee);
    }
    if(!mask)
      mask = ~0u;
    if(buffer == "on" || buffer.empty())
      gameController.automatic |= mask;
    else if(buffer == "off")
      gameController.automatic &= ~mask;
  }
  else if(buffer == "ci")
  {
    if(is2D || !calcImage(stream))
      printLn("Syntax Error");
  }
  else if(buffer == "dt")
  {
    stream >> buffer;
    if(buffer == "on" || buffer.empty())
      delayTime = simStepLength;
    else if(buffer == "off")
      delayTime = 0.f;
    else
    {
      for(char& c : buffer)
        if(!isdigit(c))
        {
          printLn("Syntax Error");
          return false;
        }
      delayTime = 1000.f / std::max(1, atoi(buffer.c_str()));
    }
  }
  else if(buffer == "gc")
  {
    auto check = [this](bool result)
    {
      if(!result)
        printLn("Not allowed in this game state");
    };

    stream >> buffer;
    if(buffer == "initial")
      check(gameController.initial());
    else if(buffer == "standby")
      check(gameController.standby());
    else if(buffer == "ready")
      check(gameController.ready());
    else if(buffer == "set")
      check(gameController.set());
    else if(buffer == "playing")
      check(gameController.playing());
    else if(buffer == "finished")
      check(gameController.finished());
    else if(buffer == "competitionPhasePlayoff")
      check(gameController.competitionPhasePlayoff());
    else if(buffer == "competitionPhaseRoundRobin")
      check(gameController.competitionPhaseRoundrobin());
    else if(buffer == "competitionTypeChampionsCup")
      check(gameController.competitionTypeChampionsCup());
    else if(buffer == "competitionTypeChallengeShield")
      check(gameController.competitionTypeChallengeShield());
    else if(buffer == "globalGameStuck")
      check(gameController.globalGameStuck());
    else if(buffer == "goalByFirstTeam")
      check(gameController.goal(0));
    else if(buffer == "goalBySecondTeam")
      check(gameController.goal(1));
    else if(buffer == "goalKickForFirstTeam")
      check(gameController.goalKick(0));
    else if(buffer == "goalKickForSecondTeam")
      check(gameController.goalKick(1));
    else if(buffer == "pushingFreeKickForFirstTeam")
      check(gameController.pushingFreeKick(0));
    else if(buffer == "pushingFreeKickForSecondTeam")
      check(gameController.pushingFreeKick(1));
    else if(buffer == "cornerKickForFirstTeam")
      check(gameController.cornerKick(0));
    else if(buffer == "cornerKickForSecondTeam")
      check(gameController.cornerKick(1));
    else if(buffer == "kickInForFirstTeam")
      check(gameController.kickIn(0));
    else if(buffer == "kickInForSecondTeam")
      check(gameController.kickIn(1));
    else if(buffer == "penaltyKickForFirstTeam")
      check(gameController.teamPenaltyKick(0));
    else if(buffer == "penaltyKickForSecondTeam")
      check(gameController.teamPenaltyKick(1));
    else if(buffer == "kickOffFirstTeam")
      check(gameController.kickOff(0));
    else if(buffer == "kickOffSecondTeam")
      check(gameController.kickOff(1));
    else if(buffer == "dropBall")
      check(gameController.dropBall());
    else if(buffer == "halfFirst")
      check(gameController.setHalf(1));
    else if(buffer == "halfSecond")
      check(gameController.setHalf(2));
    else if(buffer == "gamePenaltyShootout")
      check(gameController.gamePhasePenaltyshoot());
    else if(buffer == "gameNormal")
      check(gameController.gamePhaseNormal());
    else
      printLn("Syntax Error");
  }
  else if(buffer == "mvo")
  {
    std::string objectID;
    Vector3f robotTranslationCmd;
    stream >> objectID >> robotTranslationCmd.x() >> robotTranslationCmd.y() >> robotTranslationCmd.z();
    SimRobot::Object* robot = application->resolveObject(QString::fromStdString(objectID), is2D ? static_cast<int>(SimRobotCore2D::body) : static_cast<int>(SimRobotCore3::body));
    if(robot)
    {
      if(is2D)
      {
        robotTranslationCmd.head<2>() *= 0.001f;
        static_cast<SimRobotCore2D::Body*>(robot)->move(robotTranslationCmd.data(), Angle::fromDegrees(robotTranslationCmd.z()));
      }
      else
      {
        Vector3f robotRotationCmd;
        Matrix3f robotRotation;
        if(!stream.eof())
        {
          stream >> robotRotationCmd.x() >> robotRotationCmd.y() >> robotRotationCmd.z();
          robotRotation = RotationMatrix::fromEulerAngles(robotRotationCmd * 1_deg);
        }
        else
          robotRotation = Matrix3f::Identity();
        float robotRotationConverted[3][3];
        for(int i = 0; i < 3; ++i)
          for(int j = 0; j < 3; ++j)
            robotRotationConverted[i][j] = robotRotation(i, j);
        const Vector3f translationInMeters = robotTranslationCmd * 0.001f;
        static_cast<SimRobotCore3::Body*>(robot)->move(translationInMeters.data(), robotRotationConverted);
      }
    }
    else
      printLn("Syntax Error");
  }
  else if(buffer == "sc")
  {
    if(!startRemote(stream))
    {
      selected.clear();
      if(!robots.empty())
        selected.push_back(robots.front()->getRobotConsole());
    }
  }
  else if(buffer == "sl")
  {
    if(!startLogFile(stream))
      printLn("Log file not found!");
  }
  else if(buffer == "st")
  {
    stream >> buffer;
    if(buffer == "on" || buffer.empty())
      Time::setSimulatedTime(true);
    else if(buffer == "off")
      Time::setSimulatedTime(false);
    else
      printLn("Syntax Error");
  }
  else if(buffer == "sv")
  {
    stream >> buffer;
    std::string fileName;
    bool fastScene = false;
    if(buffer == "oracle")
    {
      fastScene = false;
    }
    else if(buffer == "fast")
    {
      fastScene = true;
    }
    else
    {
      printLn("Syntax Error!");
      return false;
    }
    stream >> buffer;
    if(buffer.empty())
    {
      fileName = "Saved.con";
    }
    else
    {
      fileName = buffer;
    }
    if(fastScene)
    {
      writeFastConfiguration(fileName);
    }
    else
    {
      writePerceptOracleConfig(fileName);
    }

    return false;
  }
  else if(buffer == "test")
  {
    std::string normalizedCmd = TestCommandParser::normalize(command);
    InConfigMemory cmdStream(normalizedCmd.c_str(), normalizedCmd.size());
    TestCommandParser testCmdParser = TestCommandParser(cmdStream);
    try
    {
      testCmdParser.parse();
    }
    catch(const std::runtime_error& e)
    {
      printLn("Syntax error: " + std::string(e.what()));
      return false;
    }
    switch(testCmdParser.getCommandType())
    {
      case TestCommandType::help:
        printLn("See more information here: https://gitlab.informatik.uni-bremen.de/groups/B-Human/-/wikis/doku/Tests");
        break;
      case TestCommandType::stop:
        gameController.stopTest();
        break;
      case TestCommandType::test:
        gameController.startTest(testCmdParser.getTestParameters());
        break;
      case TestCommandType::none:
        break;
    }
    return false;
  }
  else if(console)
  {
    console->handleConsole(command);
  }
  else if(selected.empty())
    if(buffer == "cls")
      printLn("_cls");
    else if(buffer == "echo")
      echo(stream);
    else
      printLn("No robot selected!");
  else
  {
    for(RobotConsole* selectedConsole : selected)
      selectedConsole->handleConsole(command);
  }
  if(completion.empty())
    createCompletion();
  return false;
}

void ConsoleRoboCupCtrl::writeFastConfiguration(std::string fileName)
{
  File file("Scenes/" + fileName, "w", true);
  SimRobot::Object* ball = application->resolveObject(QString::fromStdString("RoboCup.balls.ball"), is2D ? static_cast<int>(SimRobotCore2D::body) : static_cast<int>(SimRobotCore3::body));
  const float positionFactor = 1000.f;
  std::string output = "";
  if(ball != nullptr)
  {
    if(is2D)
    {
      const float* ballPosition = static_cast<SimRobotCore2D::Body*>(ball)->getPosition();
      output += "mvo RoboCup.balls.ball " + std::to_string(ballPosition[0]) + " " + std::to_string(ballPosition[1]) + "\n";
    }
    else
    {
      const float* ballPosition = static_cast<SimRobotCore3::Body*>(ball)->getPosition();
      output += "mvo RoboCup.balls.ball " + std::to_string(ballPosition[0] * positionFactor) + " " + std::to_string(ballPosition[1] * positionFactor) + " " + std::to_string(ballPosition[2] * positionFactor) + "\n";
    }
  }
  for(const ControllerRobot* controllerRobot : robots)
  {
    SimRobot::Object* robot = application->resolveObject(QString::fromStdString("RoboCup.robots." + controllerRobot->getName()), is2D ? static_cast<int>(SimRobotCore2D::body) : static_cast<int>(SimRobotCore3::body));
    if(robot)
    {
      if(is2D)
      {
        Pose2f pos;
        Pose2f& pose = pos;
        static_cast<const SimRobotCore2D::Body*>(robot)->getPose(pose.translation.data(), reinterpret_cast<float*>(&pose.rotation));
        pose.translation *= 1000.f;
        output += "mvo " + robot->getFullName().toStdString() + " " + std::to_string(pose.translation[0]) + " " + std::to_string(pose.translation[1]) + " " + std::to_string(pose.rotation.toDegrees()) + "\n";
      }
      else
      {
        float position[3];
        float(rotation[3])[3];
        static_cast<SimRobotCore3::Body*>(robot)->getPose(position, rotation);
        const double rotationFactor = 57.2958;
        double yaw = std::atan2(rotation[1][0], rotation[0][0]) * rotationFactor;
        double pitch = std::atan2(-rotation[2][0], sqrt(pow(rotation[2][1], 2) + pow(rotation[2][2], 2))) * rotationFactor;
        double roll = std::atan2(rotation[2][1], rotation[2][2]) * rotationFactor;
        output += "mvo " + robot->getFullName().toStdString() + " " + std::to_string(position[0] * positionFactor) + " " + std::to_string(position[1] * positionFactor) + " " + std::to_string(position[2] * positionFactor) +
                  " " + std::to_string(roll) + " " + std::to_string(pitch) + " " + std::to_string(yaw) + "\n";
      }
    }
    else
    {
      printLn("robot is NULL");
    }
  }
  file.write(output.c_str(), output.size());
}

void ConsoleRoboCupCtrl::writePerceptOracleConfig(std::string fileName)
{
  SimRobot::Object* ball = application->resolveObject(QString::fromStdString("RoboCup.balls.ball"), is2D ? static_cast<int>(SimRobotCore2D::body) : static_cast<int>(SimRobotCore3::body));
  float positionFactor = 1000.f;
  std::string file = "Locations/Default/staticInitialPoseProvider.cfg";
  InMapFile stream(file);
  StaticInitialPoseProvider::Parameters params;
  if(stream.exists())
  {
    stream >> params;
  }
  else
  {
    printLn("Input File not found!");
  }
  PoseVariation variationTeamA;
  PoseVariation variationTeamB;

  params.isActive = true;
  params.loadVariation = 0;

  for(const ControllerRobot* controllerRobot : robots)
  {
    if(controllerRobot)
    {
      Pose2f pose;
      SimRobot::Object* robot = application->resolveObject(QString::fromStdString("RoboCup.robots." + controllerRobot->getName()), is2D ? static_cast<int>(SimRobotCore2D::body) : static_cast<int>(SimRobotCore3::body));
      controllerRobot->getRobotConsole()->simulatedRobot->getRobotPose(pose);
      if(SimulatedRobot::isFirstTeam(robot))
      {
        variationTeamA.poseVaria.push_back(pose);
      }
      else
      {
        variationTeamB.poseVaria.push_back(pose);
      }
    }
    else
    {
      printLn("controllerRobot is NULL");
    }
  }
  params.poseVariations.push_back(variationTeamA);
  params.poseVariations.push_back(variationTeamB);
  params.loadVariation = static_cast<int>(params.poseVariations.size() - 2);
  OutMapMemory outStreamTeamA(true, true);
  outStreamTeamA << params;
  params.loadVariation = static_cast<int>(params.poseVariations.size() - 1);
  OutMapMemory outStreamTeamB(true, true);
  outStreamTeamB << params;

  std::string text = "";
  if(ball != nullptr)
  {
    const float* ballPosition = static_cast<SimRobotCore3::Body*>(ball)->getPosition();
    text += "mvo RoboCup.balls.ball " + std::to_string(ballPosition[0] * positionFactor) + " " + std::to_string(ballPosition[1] * positionFactor) + " " + std::to_string(ballPosition[2] * positionFactor) + "\n";
  }
  std::string valuesTeamA = outStreamTeamA.data();
  std::string valuesTeamB = outStreamTeamB.data();
  text += "robot teamA\n";
  text += "set module:StaticInitialPoseProvider " + valuesTeamA + "\n";
  text += "dr module:StaticInitialPoseProvider\n";
  text += "robot teamB\n";
  text += "set module:StaticInitialPoseProvider " + valuesTeamB + "\n";
  text += "dr module:StaticInitialPoseProvider\n";
  text += "robot all\n";
  File moveFile("Scenes/" + fileName, "w", true);
  moveFile.write(text.c_str(), text.size());
}

bool ConsoleRoboCupCtrl::selectRobot(const std::string& name)
{
  for(ControllerRobot* robot : robots)
    if(name == robot->getName())
    {
      selected.push_back(robot->getRobotConsole());
      return true;
    }

  for(RemoteConsole* remoteRobot : remoteRobots)
    if(name == remoteRobot->getName())
    {
      selected.push_back(remoteRobot);
      return true;
    }

  return false;
}

void ConsoleRoboCupCtrl::help(In& stream)
{
  std::string pattern;
  stream >> pattern;
  list("Initialization commands:", pattern, true);
  list("  sc <name> [<a.b.c.d>] : Starts a TCP connection to a remote robot.", pattern, true);
  list("  sl <name> [ (<file> | remote) [<scenario> [<location>]]] : Starts a robot reading its input from a log file or some other source.", pattern, true);
  list("Global commands:", pattern, true);
  list("  ar {<feature>} off | on : Switches automatic referee on or off.", pattern, true);
  list("  call <file> [<file>] : Execute a script file. If the optional script file is present, execute it instead.", pattern, true);
  if(!is2D)
    list("  ci off | on | <fps> : Switch the calculation of images on or off or activate it and set the frame rate.", pattern, true);
  list("  cls : Clear console window.", pattern, true);
  list("  dt off | on | <fps> : Delay time of a simulation step to real time or a certain number of frames per second.", pattern, true);
  list("  echo <text> : Print text into console window. Useful in console.con.", pattern, true);
  list("  gc initial | standby | ready | set | playing | finished | goalByFirstTeam | goalBySecondTeam | kickOffFirstTeam | kickOffSecondTeam | dropBall | globalGameStuck | goalKickForFirstTeam | goalKickForSecondTeam | pushingFreeKickForFirstTeam | pushingFreeKickForSecondTeam | cornerKickForFirstTeam | cornerKickForSecondTeam | kickInForFirstTeam | kickInForSecondTeam | penaltyKickForFirstTeam | penaltyKickForSecondTeam | halfFirst | halfSecond | gameNormal | gamePenaltyShootout | competitionPhasePlayoff | competitionPhaseRoundRobin | competitionTypeChampionsCup | competitionTypeChallengeShield : Set GameController state.", pattern, true);
  list("  ( help | ? ) [<pattern>] : Display this text.", pattern, true);
  if(is2D)
    list("  mvo <name> <x> <y> [<rot>] : Move the object with the given name to the given position.", pattern, true);
  else
    list("  mvo <name> <x> <y> <z> [<rotX> <rotY> <rotZ>] : Move the object with the given name to the given position.", pattern, true);
  list("  robot ? | all | <name> {<name>} : Connect console to a set of active robots. Alternatively, double click on robot.", pattern, true);
  list("  st off | on : Switch simulation of time on or off.", pattern, true);
  list("  # <text> : Comment.", pattern, true);
  list("Robot commands:", pattern, true);
  list("  bc [<red%> [<green%> [<blue%>]]] : Set the background color of all 3-D views.", pattern, true);
  list("  dr ? [<pattern>] | off | <key> ( off | on ) : Send debug request.", pattern, true);
  list("  for <thread> {<thread>} ( dr | get | mr | set | si | vf | vi | vp ) {<param>} : Runs command for a set of threads. See help for individual commands for their parameters.", pattern, true);
  list("  get ? [<pattern>] | <key> [ ? | save [<file>]] : Show/save debug data or show its specification.", pattern, true);
  list("  jc hide | show | motion ( 1 | 2 ) <command> | ( press | release ) <button> <command> : Set joystick motion (use $1 .. $8) or button command.", pattern, true);
  list("  jm <axis> <button> <button> : Map two buttons on an axis.", pattern, true);
  list("  js <axis> <speed> <threshold> [<center>] : Set axis maximum speed and ignore threshold for \"jc motion\" commands.", pattern, true);
  list("  log start | stop | clear : Record log file.", pattern, true);
  list("  log save [<file>] : Save log file with given name or modified current log file name.", pattern, true);
  list("  log saveAudio [<file>] : Save audio data from log.", pattern, true);
  list("  log saveImages [raw] [onlyPlaying] [<takeEachNth>] [<dir>] : Save images from log.", pattern, true);
  list("  log trim ( until <end frame> | from <start frame> | between <start frame> <end frame> ) : Keep only the given section of the log. 'current' can be used as frame as well.", pattern, true);
  list("  log ? [<pattern>] : Display information about log file.", pattern, true);
  list("  log load <file> | clear : Load log-file or clear all frames.", pattern, true);
  list("  log ( keep | remove ) <message> {<message>} : Filter specified messages of all frames.", pattern, true);
  list("  log keep circlePercept : Keep frames that satisfy hard-coded criteria.", pattern, true);
  list("  log start | pause | stop | ( forward | backward ) [ fast | image ] | repeat | goto <number> | cycle | once : Replay log file.", pattern, true);
  list("  log mr [list] : Generate module requests to replay log file.", pattern, true);
  list("  mr ? [<pattern>] | modules [<pattern>] | save | <representation> ( ? [<pattern>] | <module> | off | default ) : Send module request.", pattern, true);
  list("  msg off | on | log <file> | enable | disable : Switch output of text messages on or off. Log text messages to a file. Switch message handling on or off.", pattern, true);
  list("  mv <x> <y> <z> [<rotX> <rotY> <rotZ>] : Move the selected simulated robot to the given position.", pattern, true);
  list("  mvb <x> <y> <z> : Move the ball to the given position.", pattern, true);
  list("  poll : Poll for all available debug requests and drawings. ", pattern, true);
  list("  pr none | illegalBallContact | playerPushing | foul | penaltyKick | illegalMotionInSet | inactivePlayer | illegalPosition | leavingTheField | requestForPickup | localGameStuck | illegalPositionInSet | playerStance | substitute | manual : Set robot penalty.", pattern, true);
  list("  rc ( local <team number> | remote <team number> <broadcast> ) : Start connection to remote-control a robot.", pattern, true);
  list("  set ? [<pattern>] | <key> ( ? | unchanged | <data> ) : Change debug data or show its specification.", pattern, true);
  if(!is2D)
    list("  si reset [<number>] | [number] [grayscale] [<file>] : Save camera image. Only \"reset\" works without \"for\".", pattern, true);
  list("  sn [ whiteNoise | timeDelay | discretization ] [ on | off ] : (De)activates the simulation of (specified) sensor noise.", pattern, true);
  list("  sv fast | oracle [ <fileName> ]: Save the current robot positions into a specified fileName. If no fileName given, it will be saved in Saved.con. Use fast to save it for Fast scenes. Use oracle to save it for PerceptOracle scenes.", pattern, true);
  list("  test game <numRuns> [<conFile>] [rt] [q]: Automatically test your code by playing multiple games.", pattern, true);
  list("  test help: get some more information about the tests.", pattern, true);
  list("  test situation <numRuns> <runTimeout> [<conFile>] [fuz ((all | pos | rot | x | y | z | rotx | roty | rotz) <deviation>)+] [rt] [q] ballInZone (<x1> <y1> <x2> <y2> | (fieldHalf | centerCircle | penaltyArea | goalArea)[A | B]) : Play till the ball is in the coordinates or till the time is gone.", pattern, true);
  list("  test situation <numRuns> <runTimeout> [<conFile>] [fuz ((all | pos | rot | x | y | z | rotx | roty | rotz) <deviation>)+] [rt] [q] goalTeamA : Play till Team A scores or till the time is gone.", pattern, true);
  list("  test situation <numRuns> <runTimeout> [<conFile>] [fuz ((all | pos | rot | x | y | z | rotx | roty | rotz) <deviation>)+] [rt] [q] goalTeamB : Play till Team B scores or till the time is gone.", pattern, true);
  list("  test situation <numRuns> <runTimeout> [<conFile>] [fuz ((all | pos | rot | x | y | z | rotx | roty | rotz) <deviation>)+] [rt] [q] penalized <robot name> : Play till the robot is penalized or till the time is gone.", pattern, true);
  list("  test situation <numRuns> <runTimeout> [<conFile>] [fuz ((all | pos | rot | x | y | z | rotx | roty | rotz) <deviation>)+] [rt] [q] robotTouch <robot name> : Play till the robot touches the ball or till the time is gone. The robot name is the name when you click on this robot.", pattern, true);
  list("  test situation <numRuns> <runTimeout> [<conFile>] [fuz ((all | pos | rot | x | y | z | rotx | roty | rotz) <deviation>)+] [rt] [q] robotInZone (<x1> <y1> <x2> <y2> | (fieldHalf | centerCircle | penaltyArea | goalArea)[A | B]) <robot name> : Play till the robot is in the coordinates or till the time is gone. The robot name is the name when you click on this robot.", pattern, true);
  list("  test situation <numRuns> <runTimeout> [<conFile>] [fuz ((all | pos | rot | x | y | z | rotx | roty | rotz) <deviation>)+] [rt] [q] teamPossession (teamA | teamB) : Play till Team A or Team B is in possession of the ball or till the time is gone.", pattern, true);
  list("  test situation <numRuns> <runTimeout> [<conFile>] [fuz ((all | pos | rot | x | y | z | rotx | roty | rotz) <deviation>)+] [rt] [q] budget (teamA | teamB) (< | > | >= | <=) <value> : Play till Team A or Team B has a budget that matches the condition or till the time is gone.", pattern, true);
  list("  test stop  : Stops the current test and deletes all working files", pattern, true);
  list("  vf <name> : Add field view.", pattern, true);
  list("  vfd ? [<pattern>] | off | ( all | <name> ) ( ? [<pattern>] | <drawing> [ on | off ] ) : (De)activate debug drawing in field view.", pattern, true);
  list("  vi ? [<pattern>] | ( <image> | none ) [<name>] [gain <value>] [ddScale <value>] : Add image view.", pattern, true);
  list("  vic ? [<pattern>] | ( all | <name> ) [ alt | noalt ] [ ctrl | noctrl ] [ shift | noshift ] <command> : Set image view button release command.", pattern, true);
  list("  vid ? [<pattern>] | off | ( all | <name> ) ( ? [<pattern>] | <drawing> [ on | off ] ) : (De)activate debug drawing in image view.", pattern, true);
  list("  vp <name> <numOfValues> <minValue> <maxValue> [<yUnit> [<xUnit> [<xScale>]]] : Add plot view.", pattern, true);
  list("  vpd ? [<pattern>] | <name> ( ? [<pattern>] | <drawing> ( ? [<pattern>] | <color> [<description>] | off ) ) : Plot data in a certain color in plot view.", pattern, true);
}

void ConsoleRoboCupCtrl::echo(In& stream)
{
  bool first = true;
  while(!stream.eof())
  {
    std::string text;
    stream >> text;
    if(first)
      first = false;
    else
      print(" ");
    print(text);
  }
  printLn("");
}

bool ConsoleRoboCupCtrl::startRemote(In& stream)
{
  std::string name, ip;
  stream >> name >> ip;

  mode = SystemCall::remoteRobot;

  const Settings::RobotType robotType = getRobotType();
  std::string robotName = TypeRegistry::getEnumName(robotType);
  robotName[0] &= ~0x20;
  robotName = "Simulated" + robotName;
  RemoteConsole* rc = new RemoteConsole(name, ip, this, Settings(robotName, robotName, simStepLength * 0.001f, robotType));
  if(!rc->isClient())
  {
    if(ip != "")
    {
      delete rc;
      Global::theSettings = nullptr;
      File::clearSearchPath();
      printLn(std::string("No connection to ") + ip + " established!");
      return false;
    }
    else
      printLn("Waiting for a connection...");
  }
  selected.clear();
  remoteRobots.push_back(rc);
  selected.push_back(remoteRobots.back());
  rc->addPerRobotViews();
  rc->start();
  return true;
}

bool ConsoleRoboCupCtrl::startLogFile(In& stream)
{
  std::string name, fileName, scenario, location;
  stream >> name >> fileName >> scenario >> location;
  if(fileName.empty())
  {
    if(logFile.substr(0, 5) == "Logs/")
      fileName = logFile.substr(5);
    else
      fileName = logFile;
  }
  if(fileName != "remote")
  {
    if(!File::hasExtension(fileName))
      fileName = fileName + ".log";
    if(!File::isAbsolute(fileName))
      fileName = std::string("Logs/") + fileName;
    {
      InBinaryFile test(fileName);
      if(!test.exists())
        return false;
    }
  }

  mode = fileName == "remote" ? SystemCall::remoteRobot : SystemCall::logFileReplay;
  logFile = fileName;
  robots.push_back(new ControllerRobot(Settings(logFile, simStepLength * 0.001f, getRobotType(), location == "" ? nullptr : &location,
                                                scenario == "" ? nullptr : &scenario), name, this, logFile));
  selected.clear();
  RobotConsole* rc = robots.back()->getRobotConsole();
  selected.push_back(rc);
  robots.back()->start();
  return true;
}

bool ConsoleRoboCupCtrl::calcImage(In& stream)
{
  std::string state;
  stream >> state;
  if(state == "off")
  {
    calculateImage = false;
    return true;
  }
  else if(state == "on" || state == "")
  {
    calculateImageFps = FRAMES_PER_SECOND;
    calculateImage = true;
    return true;
  }
  else
  {
    for(char c : state)
      if(!isdigit(c))
        return false;
    calculateImageFps = std::max(1, atoi(state.c_str()));
    calculateImage = true;
    return true;
  }
}

void ConsoleRoboCupCtrl::print(const std::string& text)
{
  SYNC;
  if(newLine)
    textMessages.push_back(text);
  else
    textMessages.back() += text;
  newLine = false;
}

void ConsoleRoboCupCtrl::printLn(const std::string& text)
{
  SYNC;
  if(newLine)
    textMessages.push_back(text);
  else
    textMessages.back() += text;
  newLine = true;
}

void ConsoleRoboCupCtrl::printStatusText(const QString& text)
{
  if(statusText != "")
    statusText += " | ";
  statusText += text;
}

void ConsoleRoboCupCtrl::createCompletion()
{
  const char* commands[] =
  {
    "ar off",
    "ar on",
    "bc",
    "call",
    "cls",
    "dr off",
    "dt off",
    "dt on",
    "echo",
    "gc initial",
    "gc standby",
    "gc ready",
    "gc set",
    "gc playing",
    "gc finished",
    "gc competitionTypeChampionsCup",
    "gc competitionTypeChallengeShield",
    "gc competitionPhasePlayoff",
    "gc competitionPhaseRoundRobin",
    "gc dropBall",
    "gc globalGameStuck",
    "gc goalByFirstTeam",
    "gc goalBySecondTeam",
    "gc goalKickForFirstTeam",
    "gc goalKickForSecondTeam",
    "gc pushingFreeKickForFirstTeam",
    "gc pushingFreeKickForSecondTeam",
    "gc cornerKickForFirstTeam",
    "gc cornerKickForSecondTeam",
    "gc kickInForFirstTeam",
    "gc kickInForSecondTeam",
    "gc penaltyKickForFirstTeam",
    "gc penaltyKickForSecondTeam",
    "gc kickOffFirstTeam",
    "gc kickOffSecondTeam",
    "gc halfFirst",
    "gc halfSecond",
    "gc gamePenaltyShootout",
    "gc gameNormal",
    "help",
    "jc motion",
    "jc hide",
    "jc show",
    "jc press",
    "jc release",
    "js",
    "log start",
    "log stop",
    "log clear",
    "log save",
    "log saveAudio",
    "log saveImages raw onlyPlaying",
    "log trim from current",
    "log trim until current",
    "log trim between current",
    "log ?",
    "log keep circlePercept",
    "log mr list",
    "log load",
    "log cycle",
    "log once",
    "log pause",
    "log forward fast",
    "log forward image",
    "log backward fast",
    "log backward image",
    "log repeat",
    "log goto",
    "mr modules",
    "mr save",
    "msg off",
    "msg on",
    "msg log",
    "msg enable",
    "msg disable",
    "mv",
    "mvb",
    "mvo",
    "poll",
    "rc local",
    "rc remote",
    "robot all",
    "sc",
    "sl",
    "sn off",
    "sn on",
    "st off",
    "st on",
    "sv fast",
    "sv oracle",
    "test game <numRuns> [rt] [q]",
    "test stop",
    "vf",
    "vp"
  };

  const char* commands3D[] =
  {
    "ci off",
    "ci on",
    "si reset"
  };

  completion.clear();
  const int num = sizeof(commands) / sizeof(commands[0]);
  for(int i = 0; i < num; ++i)
    completion.insert(commands[i]);

  std::list<std::string> robotList =
  {
    "robot1", "robot2", "robot3", "robot4", "robot5", "robot6", "robot7",
    "robot21", "robot22", "robot23", "robot24", "robot25", "robot26", "robot27"
  };

  FOREACH_ENUM(TestController::TargetType, i)
  {
    switch(i)
    {
      case TestController::goalTeamB:
        completion.insert("test situation <numRuns> <runTimeout> [<conFile>] [<fuzCmd>] [rt] [q] goalTeamB");
        break;
      case TestController::goalTeamA:
        completion.insert("test situation <numRuns> <runTimeout> [<conFile>] [<fuzCmd>] [rt] [q] goalTeamA");
        break;
      case TestController::ballInZone:
        completion.insert("test situation <numRuns> <runTimeout> [<conFile>] [<fuzCmd>] [rt] [q] ballInZone <zoneProps>");
        break;
      case TestController::robotTouch:
        for(std::string robot : robotList)
        {
          completion.insert("test situation <numRuns> <runTimeout> [<conFile>] [<fuzCmd>] [rt] [q] robotTouch " + robot);
        }
        break;
      case TestController::robotInZone:
        for(std::string robot : robotList)
        {
          completion.insert("test situation <numRuns> <runTimeout> [<conFile>] [<fuzCmd>] [rt] [q] robotInZone <zoneProps> " + robot);
        }
        break;
      case TestController::teamPossession:
        completion.insert("test situation <numRuns> <runTimeout> [<conFile>] [<fuzCmd>] [rt] [q] teamPossession teamA");
        completion.insert("test situation <numRuns> <runTimeout> [<conFile>] [<fuzCmd>] [rt] [q] teamPossession teamB");
        break;
      case TestController::penalized:
        for(std::string robot : robotList)
        {
          completion.insert("test situation <numRuns> <runTimeout> [<conFile>] [<fuzCmd>] [rt] [q] penalized " + robot);
        }
        break;
      case TestController::robotInSkill:
        FOREACH_ENUM(SkillRequest::Type, skill)
        {
          std::string skillName = TypeRegistry::getEnumName(skill);
          for(std::string robot : robotList)
          {
            completion.insert("test situation <numRuns> <runTimeout> [<conFile>] [<fuzCmd>] [rt] [q] robotInSkill " + skillName + " " + robot);
          }
        }
        break;
      case TestController::budget:
        for (std::string op : {"<", ">", "<=", ">="})
        {
          completion.insert("test situation <numRuns> <runTimeout> [<conFile>] [<fuzCmd>] [rt] [q] budget teamA " + op + " <value>");
          completion.insert("test situation <numRuns> <runTimeout> [<conFile>] [<fuzCmd>] [rt] [q] budget teamB " + op + " <value>");
        }
        break;
    }
  }

  if(!is2D)
  {
    const int num3D = sizeof(commands3D) / sizeof(commands3D[0]);
    for(int i = 0; i < num3D; ++i)
      completion.insert(commands3D[i]);
  }

  for(ControllerRobot* robot : robots)
    completion.insert("robot " + robot->getName());
  for(RemoteConsole* remoteRobot : remoteRobots)
    completion.insert("robot " + remoteRobot->getName());

  FOREACH_ENUM(MessageID, i)
  {
    completion.insert(std::string("log keep ") + TypeRegistry::getEnumName(i));
    completion.insert(std::string("log remove ") + TypeRegistry::getEnumName(i));
  }

  addCompletionFiles("log load ", std::string(File::getBHDir()) + "/Config/Logs/*.log");
  addCompletionFiles("log save ", std::string(File::getBHDir()) + "/Config/Logs/*.log", false);
  addCompletionFiles("call ", std::string(File::getBHDir()) + "/Config/Scenes/*.con");

  FOREACH_ENUM(GameController::Penalty, i)
    completion.insert(std::string("pr ") + TypeRegistry::getEnumName(i));

  for(int i = 0; i < GameController::numOfAutomaticReferees; ++i)
  {
    const std::string name = TypeRegistry::getEnumName(static_cast<GameController::AutomaticReferee>(i));
    completion.insert(std::string("ar ") + name + " on");
    completion.insert(std::string("ar ") + name + " off");
  }

  std::array<std::string, 3> noiseSources = {"whiteNoise", "timeDelay", "discretization"};
  for(std::string noise : noiseSources)
  {
    std::string command = "sn ";
    command += noise;
    completion.insert(command + " on");
    completion.insert(command + " off");
  }

  SYNC;
  if(moduleInfo)
  {
    for(const auto& r : moduleInfo->representations)
    {
      completion.insert(std::string("mr ") + r + " default");
      completion.insert(std::string("mr ") + r + " off ");
      for(const Configuration::Thread& thread : moduleInfo->config())
        completion.insert(std::string("for ") + thread.name + " mr " + r + " off ");
      for(const auto& m : moduleInfo->modules)
        if(std::find(m.representations.begin(), m.representations.end(), r) != m.representations.end())
        {
          for(const Configuration::Thread& thread : moduleInfo->config())
            completion.insert(std::string("for ") + thread.name + " mr " + r + " " + m.name);
        }
      bool once = false;
      for(const Configuration::Thread& thread : moduleInfo->config())
      {
        if(std::find_if(thread.representationProviders.begin(), thread.representationProviders.end(),
                        [&r](const Configuration::RepresentationProvider& rp)
      { return rp.representation == r; }) != thread.representationProviders.end())
        {
          once ^= true;
          if(!once)
            break;
        }
      }
      if(once)
        for(const auto& m : moduleInfo->modules)
          if(std::find(m.representations.begin(), m.representations.end(), r) != m.representations.end())
            completion.insert(std::string("mr ") + r + " " + m.name);
    }
  }

  if(debugRequestTable && threadData)
  {
    for(const auto& [name, _] : debugRequestTable->slowIndex)
    {
      completion.insert(std::string("dr ") + translate(name) + " on");
      completion.insert(std::string("dr ") + translate(name) + " off");
      if(name.substr(0, 11) == "debug data:")
      {
        const std::string cleanedName = translate(name.substr(11));

        completion.insert(std::string("get ") + cleanedName + " ?");
        if(getThreadsFor(*threadData, translate(name)).size() == 1)
          completion.insert(std::string("get ") + cleanedName + " save");
        completion.insert(std::string("set ") + cleanedName + " ?");
        completion.insert(std::string("set ") + cleanedName + " unchanged");
      }
      else if(name.substr(0, 13) == "debug images:" && getThreadsFor(*threadData, translate(name)).size() == 1)
        completion.insert("vi " + translate(name.substr(13)));
      else if(name == "representation:CameraImage" && getThreadsFor(*threadData, name).size() == 1)
        completion.insert("vi CameraImage");
      else if(name == "representation:JPEGImage" && getThreadsFor(*threadData, name).size() == 1)
        completion.insert("vi JPEGImage");
    }
  }

  if(!is2D)
  {
    completion.insert("si grayscale");
    completion.insert("si number grayscale");
  }

  if(imageViews)
  {
    for(unsigned int i = 0; i < 18; ++i)
      completion.insert(std::string("vic all ")
                        + ((i < 8 || i >= 14) ? (((i & 4) ? "no" : "") + std::string("alt ")) : "")
                        + (i < 12 ? (((i & 2) ? "no" : "") + std::string("ctrl ")) : "")
                        + ((i & 1) ? "no" : "") + "shift");
    for(const auto& [name, _] : *imageViews)
      for(unsigned int i = 0; i < 18; ++i)
        completion.insert(std::string("vic ") + name + " "
                          + ((i < 8 || i >= 14) ? (((i & 4) ? "no" : "") + std::string("alt ")) : "")
                          + (i < 12 ? (((i & 2) ? "no" : "") + std::string("ctrl ")) : "")
                          + ((i & 1) ? "no" : "") + "shift");
  }

  if(threadData)
  {
    for(const auto& [threadName, data] : *threadData)
    {
      completion.insert("for " + threadName + " dr ?");
      completion.insert("for " + threadName + " dr off");
      completion.insert("for " + threadName + " get ?");
      completion.insert("for " + threadName + " set ?");
      completion.insert("for " + threadName + " vf");
      completion.insert("for " + threadName + " vp");

      for(const auto& [requestName, _]  : data.debugRequestTable.slowIndex)
      {
        completion.insert("for " + threadName + " dr " + translate(requestName) + " on");
        completion.insert("for " + threadName + " dr " + translate(requestName) + " off");
        if(requestName.substr(0, 11) == "debug data:")
        {
          const std::string cleanedName = translate(requestName.substr(11));

          completion.insert("for " + threadName + " get " + cleanedName + " ?");
          completion.insert("for " + threadName + " get " + cleanedName + " save");
          completion.insert("for " + threadName + " set " + cleanedName + " ?");
          completion.insert("for " + threadName + " set " + cleanedName + " unchanged");
        }
        else if(requestName.substr(0, 13) == "debug images:")
          completion.insert("for " + threadName + " vi " + translate(requestName.substr(13)));
        else if(requestName == "representation:CameraImage")
          completion.insert("for " + threadName + " vi CameraImage");
        else if(requestName == "representation:JPEGImage")
          completion.insert("for " + threadName + " vi JPEGImage");
      }

      if(!is2D && data.images.contains("CameraImage"))
      {
        completion.insert("for " + threadName + " si grayscale");
        completion.insert("for " + threadName + " si number grayscale");
      }
    }

    completion.insert(std::string("vid off"));
    completion.insert(std::string("vfd off"));
    for(const auto& data : *threadData)
      for(const auto& [drawingName, _] : data.second.drawingManager.drawings)
      {
        if(!strcmp(data.second.drawingManager.getDrawingType(drawingName), "drawingOnImage") && imageViews)
        {
          completion.insert(std::string("vid all ") + translate(drawingName) + " on");
          completion.insert(std::string("vid all ") + translate(drawingName) + " off");
          for(const auto& [viewName, _] : *imageViews)
          {
            completion.insert(std::string("vid ") + viewName + " " + translate(drawingName) + " on");
            completion.insert(std::string("vid ") + viewName + " " + translate(drawingName) + " off");
          }
        }
        else if(!strcmp(data.second.drawingManager.getDrawingType(drawingName), "drawingOnField") && fieldViews)
        {
          completion.insert(std::string("vfd all ") + translate(drawingName) + " on");
          completion.insert(std::string("vfd all ") + translate(drawingName) + " off");
          for(const auto& [viewName, _] : *fieldViews)
          {
            completion.insert(std::string("vfd ") + viewName + " " + translate(drawingName) + " on");
            completion.insert(std::string("vfd ") + viewName + " " + translate(drawingName) + " off");
          }
        }
      }
  }

  if(plotViews && debugRequestTable)
    for(const auto& [viewName, _] : *plotViews)
      for(const auto& [requestName, _] : debugRequestTable->slowIndex)
        if(translate(requestName).substr(0, 5) == "plot:")
        {
          for(int color = 0; color < RobotConsole::numOfColors; ++color)
            completion.insert(std::string("vpd ") + viewName + " " +
                              translate(requestName).substr(5) + " " +
                              TypeRegistry::getEnumName(static_cast<RobotConsole::Color>(color)));
          completion.insert(std::string("vpd ") + viewName + " " +
                            translate(requestName).substr(5) + " off");
        }

  currentCompletionIndex = completion.end();
}

void ConsoleRoboCupCtrl::addCompletionFiles(const std::string& command, const std::string& pattern, bool removeExtension)
{
  QString qPattern(pattern.c_str());
  qPattern.replace("\\", "/");
  const auto lastSlashIdx = qPattern.lastIndexOf('/');

  QDir qDir(qPattern.left(lastSlashIdx));
  qDir.setFilter(QDir::Files | QDir::Dirs | QDir::NoDot | QDir::NoDotDot);
  QStringList qsl;
  qsl.append(qPattern.right(qPattern.size() - lastSlashIdx - 1));
  qDir.setNameFilters(qsl);
  QDirIterator it(qDir, QDirIterator::Subdirectories);
  while(it.hasNext())
  {
    QString filename = it.next().remove(0, lastSlashIdx + 1);
    if(removeExtension)
      filename.chop(filename.size() - filename.lastIndexOf("."));
    completion.insert(command + filename.toUtf8().constData());
  }
}

std::string ConsoleRoboCupCtrl::handleCompletionString(size_t pos, const std::string& s)
{
  const std::string separators = " :/";
  if(pos < s.length())
    ++pos;
  while(pos < s.length() && separators.find(s[pos]) == std::string::npos)
    ++pos;
  if(pos < s.length())
    ++pos;
  return s.substr(0, pos);
}

void ConsoleRoboCupCtrl::completeConsoleCommand(std::string& command, bool forward, bool nextSection)
{
  if(nextSection || currentCompletionIndex == completion.end())
    currentCompletionIndex = completion.lower_bound(command);

  if(currentCompletionIndex == completion.end() || std::strncmp((*currentCompletionIndex).c_str(), command.c_str(), command.length()))
    return;

  if(forward)
  {
    if(!nextSection)
    {
      std::string lastCompletion = handleCompletionString(command.length(), *currentCompletionIndex);
      ++currentCompletionIndex;

      while(currentCompletionIndex != completion.end() && lastCompletion == handleCompletionString(command.length(), *currentCompletionIndex))
        ++currentCompletionIndex;

      if(currentCompletionIndex == completion.end() || (*currentCompletionIndex).find(command) != 0)
        currentCompletionIndex = completion.lower_bound(command);
    }
  }
  else
  {
    if(!nextSection)
    {
      std::string lastCompletion = handleCompletionString(command.length(), *currentCompletionIndex);
      --currentCompletionIndex;

      while(currentCompletionIndex != completion.begin() && lastCompletion == handleCompletionString(command.length(), *currentCompletionIndex))
        --currentCompletionIndex;

      if(currentCompletionIndex == completion.begin() || (*currentCompletionIndex).find(command) != 0)
      {
        currentCompletionIndex = completion.lower_bound(command + "zzzzzz");
        --currentCompletionIndex;
      }
    }
  }

  command = handleCompletionString(command.length(), *currentCompletionIndex);
}

void ConsoleRoboCupCtrl::completeConsoleCommandOnLetterEntry(std::string& command)
{
  std::set<std::string>::const_iterator i = completion.lower_bound(command);

  if(i == completion.end() || std::strncmp((*i).c_str(), command.c_str(), command.length())
     || ((*i).length() > command.length() && (*i)[command.length()] == ' '))
    return;

  const std::string base = handleCompletionString(command.length(), *i);

  while(i != completion.end() && !std::strncmp((*i).c_str(), command.c_str(), command.length()))
  {
    if(base != handleCompletionString(command.length(), *i))
      return;
    ++i;
  }

  currentCompletionIndex = completion.end();
  command = base;
}

void ConsoleRoboCupCtrl::list(const std::string& text, const std::string& required, bool newLine)
{
  std::string s1 = text;
  std::string s2 = required;
  for(char& c : s1)
    c = static_cast<char>(toupper(c));
  for(char& c : s2)
    c = static_cast<char>(toupper(c));
  if(s1.find(s2) != std::string::npos)
  {
    if(newLine)
      printLn(text);
    else
      print(text + " ");
  }
}

std::string ConsoleRoboCupCtrl::translate(const std::string& text) const
{
  std::string s = text;
  for(unsigned i = 0; i < s.size(); ++i)
    if(s[i] == ' ' || s[i] == '-')
    {
      s = s.substr(0, i) + s.substr(i + 1);
      if(i < s.size())
      {
        if(s[i] >= 'a' && s[i] <= 'z')
          s[i] = s[i] - 32;
        --i;
      }
    }
    else if(i < s.size() - 1 && s[i] == ':' && s[i + 1] == ':')
      s = s.substr(0, i) + s.substr(i + 1);
  return s;
}

std::vector<std::string> ConsoleRoboCupCtrl::getThreadsFor(const std::unordered_map<std::string, RobotConsole::ThreadData>& threadData,
                                                           const std::string& debugRequest) const
{
  std::vector<std::string> threadsFound;
  for(const auto& [threadName, data] : threadData)
    for(const auto& [requestName, _] : data.debugRequestTable.slowIndex)
      if(translate(requestName) == debugRequest)
        threadsFound.push_back(threadName);
  return threadsFound;
}

void ConsoleRoboCupCtrl::showInputDialog(std::string& command)
{
  QString qStrCommand(command.c_str());
  QRegularExpression re("\\$\\{([^\\}]*)\\}");
  QRegularExpressionMatch match;
  bool ok = true;
  while(ok && (match = re.match(qStrCommand)).hasMatch())
  {
    QStringList list = match.captured(1).split(',');
    QString label = list.takeFirst();
    QString input;
    if(list.isEmpty())
    {
      // ${Text input:}
      input = QInputDialog::getText(0, "Input", label, QLineEdit::Normal, "", &ok);
    }
    else if(list.length() == 1)
    {
      QString qPattern(list.takeFirst());
      qPattern.replace("\\", "/");
      const auto lastSlashIdx = qPattern.lastIndexOf('/');

      QString path(qPattern.left(lastSlashIdx));

      QStringList qsl;
      qsl.append(qPattern.right(qPattern.size() - lastSlashIdx - 1));
      qsl.append("all (*.*)");

      QSettings settings("B-Human", "SimRobot");

      // ${Select Log File:,../Logs/*.log}
      input = QFileDialog::getOpenFileName(nullptr, label, settings.value(qsl.front(), path).toString(), qsl.join(";;")
#if defined LINUX && defined QT_VERSION && QT_VERSION < QT_VERSION_CHECK(6, 6, 0)
                                           , nullptr, QFileDialog::DontUseNativeDialog
#endif
                                          );
      if((ok = !input.isNull()))
        settings.setValue(qsl.front(), QDir().absoluteFilePath(input));
    }
    else if(list.length() == 2 && list.first().isEmpty())
    {
      // ${Select LogFolder:,,../Logs/}
      QString path(list.last());
      path.replace("\\", "/");

      QSettings settings("B-Human", "SimRobot");

      input = QFileDialog::getExistingDirectory(nullptr, label, settings.value("LogFolder", path).toString(), QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks
#if defined LINUX && defined QT_VERSION && QT_VERSION < QT_VERSION_CHECK(6, 6, 0)
                                                | QFileDialog::DontUseNativeDialog
#endif
                                               );
      if((ok = !input.isNull()))
        settings.setValue("LogFolder", input.left(input.lastIndexOf('/')));
    }
    else
    {
      // ${IP address:,192.168.5.11,192.168.5.12,192.168.5.13}
      input = QInputDialog::getItem(nullptr, "Input", label, list, 0, true, &ok);
    }
    qStrCommand.replace(match.captured(0), input);
  }
  if(ok)
  {
    command = qStrCommand.toStdString();
  }
  else
  {
    command = "";
  }
}

SystemCall::Mode SystemCall::getMode()
{
  if(RoboCupCtrl::controller)
  {
    static thread_local SystemCall::Mode mode = static_cast<ConsoleRoboCupCtrl*>(RoboCupCtrl::controller)->getMode();
    return mode;
  }
  else
    return simulatedRobot;
}

extern "C" DLL_EXPORT SimRobot::Module* createModule(SimRobot::Application& simRobot)
{
  FunctionList::execute();
  return new ConsoleRoboCupCtrl(simRobot);
}
