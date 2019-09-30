/**
 * @file Controller/ConsoleRoboCupCtrl.cpp
 *
 * This file implements the class ConsoleRoboCupCtrl.
 *
 * @author Thomas Röfer
 */

#include "ConsoleRoboCupCtrl.h"

#include <QDir>
#include <QDirIterator>
#include <QFileDialog>
#include <QInputDialog>
#include <QSettings>

#include <algorithm>
#include <iostream>
#include <cctype>
#include <functional>

#include "LocalRobot.h"
#include "Controller/Views/ConsoleView.h"
#include "Platform/Time.h"
#include "Tools/Framework/Robot.h"
#include "RemoteRobot.h"
#include <SimRobotEditor.h>
#include "Platform/File.h"
#include "BHToolBar.h"

#define FRAMES_PER_SECOND 60

ConsoleRoboCupCtrl::ConsoleRoboCupCtrl(SimRobot::Application& application) :
  RoboCupCtrl(application), calculateImageFps(FRAMES_PER_SECOND),
  mode(SystemCall::simulatedRobot), currentCompletionIndex(completion.end()),
  toolBar(*this)
{
  application.getLayoutSettings().setValue("Run", true);

  consoleView = new ConsoleView("Console", *this);
  addView(consoleView, nullptr, SimRobot::Flag::verticalTitleBar);
  addView(new ConsoleView("Console.Pad", *this, true), consoleView, SimRobot::Flag::verticalTitleBar);

  // file names for representations
  representationToFile["representation:BallSpecification"] = "ballSpecification.cfg";
  representationToFile["representation:CameraCalibration"] = "cameraCalibration.cfg";
  representationToFile["representation:CameraIntrinsics"] = "cameraIntrinsics.cfg";
  representationToFile["representation:CameraSettings"] = "cameraSettings.cfg";
  representationToFile["representation:FieldColors"] = "fieldColors.cfg";
  representationToFile["representation:HeadLimits"] = "headLimits.cfg";
  representationToFile["representation:IMUCalibration"] = "imuCalibration.cfg";
  representationToFile["representation:JointCalibration"] = "jointCalibration.cfg";
  representationToFile["representation:JointLimits"] = "jointLimits.cfg";
  representationToFile["representation:MassCalibration"] = "massCalibration.cfg";
  representationToFile["representation:RobotDimensions"] = "robotDimensions.cfg";
  representationToFile["representation:GetUpPhase"] = "getUpPhase.cfg";
}

bool ConsoleRoboCupCtrl::compile()
{
  std::string fileName = application->getFilePath().
#ifdef WINDOWS
                         toLatin1
#else
                         toUtf8
#endif
                         ().constData();
  std::string::size_type p = fileName.find_last_of("\\/");
  std::string::size_type p2 = fileName.find_last_of(".");
  if(p2 > p)
    fileName = fileName.substr(0, p2);
  executeFile("", fileName, false, nullptr, true);

  if(!RoboCupCtrl::compile())
    return false;

  if(!robots.empty())
    selected.push_back((*robots.begin())->getRobotThread());

  start();

  executeFile("", fileName, false, nullptr, false);

  for(Robot* robot : robots)
    robot->getRobotThread()->handleConsole("endOfStartScript");
  for(RemoteRobot* remoteRobot : remoteRobots)
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

    SimRobotEditor::Editor* kicksFolder = editor->addFolder("WalkKicks");
    QString configDir = QFileInfo(fileInfo.dir().path()).dir().path();
    for(int i = 1; i < WalkKicks::numOfTypes; ++i)
    {
      char filePath[256];
      sprintf(filePath, "/WalkKicks/%s.cfg", TypeRegistry::getEnumName(WalkKicks::Type(i)));
      kicksFolder->addFile(configDir + filePath, "");
    }
  }
}

ConsoleRoboCupCtrl::~ConsoleRoboCupCtrl()
{
  for(RemoteRobot* remoteRobot : remoteRobots)
    remoteRobot->announceStop();

  for(RemoteRobot* remoteRobot : remoteRobots)
  {
    remoteRobot->stop();
    delete remoteRobot;
  }

  stop();
  mode = SystemCall::simulatedRobot;
  Global::theSettings = nullptr;
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
  RoboCupCtrl::update();

  for(RemoteRobot* remoteRobot : remoteRobots)
    remoteRobot->update();

  {
    SYNC;
    application->setStatusMessage(statusText);
  }

  if(completion.empty())
    createCompletion();
}

void ConsoleRoboCupCtrl::executeFile(const std::string& name1, const std::string& name2,
                                     bool printError, RobotConsole* console, bool scenarioAndLocationOnly)
{
  if(nesting == 10)
    printLn("Nesting Error");
  else
  {
    ++nesting;
    bool print = false;
    for(std::string name : std::vector<std::string>({name1, name2}))
    {
      if(name == "")
      {
        print = printError;
        continue;
      }

      if(static_cast<int>(name.rfind('.')) <= static_cast<int>(name.find_last_of("\\/")))
        name = name + ".con";
      if(name[0] != '/' && name[0] != '\\' && (name.size() < 2 || name[1] != ':'))
        name = std::string("Scenes\\") + name;
      InBinaryFile stream(name);
      if(!stream.exists())
      {
        if(print)
          printLn(name + " not found");
        print = printError;
      }
      else
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
            executeConsoleCommand(line, console, scenarioAndLocationOnly);
        }
        break;
      }
    }
    --nesting;
  }
}

void ConsoleRoboCupCtrl::selectedObject(const SimRobot::Object& obj)
{
  const QString& fullName = obj.getFullName();
  std::string robotName = fullName.mid(fullName.lastIndexOf('.') + 1).toUtf8().constData();
  for(Robot* robot : robots)
    if(robotName == robot->getName())
    {
      selected.clear();
      selected.push_back(robot->getRobotThread());
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
  for(const Robot* robot : robots)
    if(robot->getRobotThread())
      for(const ThreadFrame* robotThread : *robot)
        if(robotThread->getId() == threadId)
          return robot->getRobotThread()->mode;

  return mode;
}

void ConsoleRoboCupCtrl::setRepresentation(const std::string& representationName, const Streamable& representation)
{
  OutMapMemory memory(true, 16384);
  memory << representation;

  std::string command = "set representation:" + representationName + " " + memory.data();
  executeConsoleCommand(command);
}

void ConsoleRoboCupCtrl::executeConsoleCommand(std::string command, RobotConsole* console, bool scenarioAndLocationOnly)
{
  if(!scenarioAndLocationOnly)
    showInputDialog(command);
  std::string buffer;
  InConfigMemory stream(command.c_str(), command.size());
  stream >> buffer;
  if(buffer == "call")
  {
    std::string optionalFile;
    stream >> buffer >> optionalFile;
    executeFile(optionalFile, buffer, true, console, scenarioAndLocationOnly);
  }
  else if(buffer == "cs" && nesting > 0)
  {
    stream >> buffer;
    bool exists = QDir(QString(File::getBHDir()) + "/Config/Scenarios/" + buffer.c_str()).exists();
    if(scenarioAndLocationOnly && exists)
    {
      std::string scenario = buffer;

      stream >> buffer;
      if(buffer.substr(0, 4) == "team")
      {
        int index = atoi(buffer.c_str() + 4) - 1;
        Settings::scenarios[index] = scenario;
      }
      else
      {
        Settings::settings.scenario = scenario;
      }
    }
    else if(!scenarioAndLocationOnly && !exists)
      printLn("Syntax Error: cs " + buffer);
  }
  else if(buffer == "cl" && nesting > 0)
  {
    stream >> buffer;
    bool exists = QDir(QString(File::getBHDir()) + "/Config/Locations/" + buffer.c_str()).exists();
    if(scenarioAndLocationOnly && exists)
      Settings::settings.location = buffer;
    else if(!scenarioAndLocationOnly && !exists)
      printLn("Syntax Error: cl " + buffer);
  }
  else if(buffer == "" || scenarioAndLocationOnly) // drop comments or everything if searching for location
    return;
  else if(buffer == "help" || buffer == "?")
    help(stream);
  else if(buffer == "robot")
  {
    stream >> buffer;
    if(buffer == "?")
    {
      for(const Robot* robot : robots)
        print(robot->getName() + " ");
      for(const RemoteRobot* remoteRobot : remoteRobots)
        print(remoteRobot->getName() + " ");
      printLn("");
      return;
    }
    else if(buffer == "all")
    {
      selected.clear();
      for(const Robot* robot : robots)
        selected.push_back(robot->getRobotThread());
      for(RemoteRobot* remoteRobot : remoteRobots)
        selected.push_back(remoteRobot);
      return;
    }
    else
    {
      selected.clear();
      while(true)
      {
        if(!selectRobot(buffer))
          break;
        else if(stream.getEof())
          return;
        stream >> buffer;
      }
    }
    printLn("Syntax Error");
  }
  else if(buffer == "ar")
  {
    stream >> buffer;
    if(buffer == "on" || buffer == "")
      gameController.automatic = true;
    else if(buffer == "off")
      gameController.automatic = false;
    else
      printLn("Syntax Error");
  }
  else if(buffer == "ci")
  {
    if(!calcImage(stream))
      printLn("Syntax Error");
  }
  else if(buffer == "dt")
  {
    stream >> buffer;
    if(buffer == "on" || buffer == "")
      delayTime = simStepLength;
    else if(buffer == "off")
      delayTime = 0.f;
    else
    {
      for(char& c : buffer)
        if(!isdigit(c))
        {
          printLn("Syntax Error");
          return;
        }
      delayTime = 1000.f / std::max(1, atoi(buffer.c_str()));
    }
  }
  else if(buffer == "gc")
  {
    if(!gameController.handleGlobalConsole(stream))
      printLn("Syntax Error");
  }
  else if(buffer == "mvo")
  {
    std::string objectID;
    Vector3f robotTranslationCmd;
    stream >> objectID >> robotTranslationCmd.x() >> robotTranslationCmd.y() >> robotTranslationCmd.z();
    Vector3f robotRotationCmd;
    Matrix3f robotRotation;
    if(!stream.eof())
    {
      stream >> robotRotationCmd.x() >> robotRotationCmd.y() >> robotRotationCmd.z();
      robotRotation = RotationMatrix::fromEulerAngles(robotRotationCmd * 1_deg);
    }
    SimRobot::Object* robot = application->resolveObject(QString::fromStdString(objectID));
    if(robot)
    {
      float robotRotationConverted[3][3];
      for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j)
          robotRotationConverted[i][j] = robotRotation(i, j);
      Vector3f translationmm = robotTranslationCmd * 0.001f;
      reinterpret_cast<SimRobotCore2::Body*>(robot)->move(&translationmm.x(), robotRotationConverted);
    }
  }
  else if(buffer == "sc")
  {
    if(!startRemote(stream))
    {
      selected.clear();
      if(!robots.empty())
        selected.push_back(robots.front()->getRobotThread());
    }
  }
  else if(buffer == "sl")
  {
    if(!startLogFile(stream))
      printLn("Log file not found!");
  }
  else if(buffer == "sml")
  {
    if(!startMultiLogFile(stream))
      printLn("Directory not found!");
  }
  else if(buffer == "st")
  {
    stream >> buffer;
    if(buffer == "on" || buffer == "")
    {
      if(!simTime)
      {
        // simulation time continues at real time
        time = getTime();
        simTime = true;
      }
    }
    else if(buffer == "off")
    {
      if(simTime)
      {
        // real time continues at simulation time
        time = getTime() - Time::getRealSystemTime();
        simTime = false;
      }
    }
    else
      printLn("Syntax Error");
  }
  else if(selected.empty())
    if(buffer == "cls")
      printLn("_cls");
    else if(buffer == "echo")
      echo(stream);
    else
      printLn("No robot selected!");
  else if(console)
  {
    console->handleConsole(command);
  }
  else
  {
    for(RobotConsole* selectedConsole : selected)
      selectedConsole->handleConsole(command);
  }
  if(completion.empty() && !scenarioAndLocationOnly)
    createCompletion();
}

void ConsoleRoboCupCtrl::executeConsoleCommandOnSelectedRobots(const std::string& command)
{
  for(auto console : selected)
  {
    console->handleConsole(command);
  }
}

bool ConsoleRoboCupCtrl::selectRobot(const std::string& name)
{
  for(Robot* robot : robots)
    if(name == robot->getName())
    {
      selected.push_back(robot->getRobotThread());
      return true;
    }

  for(RemoteRobot* remoteRobot : remoteRobots)
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
  list("  cl <location> : Change location (only during initial script execution).", pattern, true);
  list("  cs <scenario> [team1 | team2] : Change scenario (only during initial script execution).", pattern, true);
  list("  sc <name> [<a.b.c.d>] : Starts a TCP connection to a remote robot.", pattern, true);
  list("  sl <name> [<file>] : Starts a robot reading its input from a log file.", pattern, true);
  list("  sml <directory> : Starts robots reading their input from all log files in subfolders.", pattern, true);
  list("Global commands:", pattern, true);
  list("  ar off | on : Switches automatic referee on or off.", pattern, true);
  list("  call <file> [<file>] : Execute a script file. If the optional script file is present, execute it instead.", pattern, true);
  list("  ci off | on | <fps> : Switch the calculation of images on or off or activate it and set the frame rate.", pattern, true);
  list("  cls : Clear console window.", pattern, true);
  list("  dt off | on | <fps> : Delay time of a simulation step to real time or a certain number of frames per second.", pattern, true);
  list("  echo <text> : Print text into console window. Useful in console.con.", pattern, true);
  list("  gc initial | ready | set | playing | finished | goalByFirstTeam | goalBySecondTeam | kickOffFirstTeam | kickOffSecondTeam | manualPlacementFirstTeam | manualPlacementSecondTeam | goalFreeKickForFirstTeam | goalFreeKickForSecondTeam | pushingFreeKickForFirstTeam | pushingFreeKickForSecondTeam | cornerKickForFirstTeam | cornerKickForSecondTeam | kickInForFirstTeam | kickInForSecondTeam | gameNormal | gamePenaltyShootout | competitionPhasePlayoff | competitionPhaseRoundRobin | competitionTypeNormal | competitionTypeMixedTeam : Set GameController state.", pattern, true);
  list("  ( help | ? ) [<pattern>] : Display this text.", pattern, true);
  list("  mvo <name> <x> <y> <z> [<rotx> <roty> <rotz>] : Move the object with the given name to the given position.", pattern, true);
  list("  robot ? | all | <name> {<name>} : Connect console to a set of active robots. Alternatively, double click on robot.", pattern, true);
  list("  st off | on : Switch simulation of time on or off.", pattern, true);
  list("  # <text> : Comment.", pattern, true);
  list("Robot commands:", pattern, true);
  list("  bc [<red%> [<green%> [<blue%>]]] : Set the background color of all 3-D views.", pattern, true);
  list("  dr ? [<pattern>] | off | <key> ( off | on ) : Send debug request.", pattern, true);
  list("  get ? [<pattern>] | <key> [?]: Show debug data or show its specification.", pattern, true);
  list("  jc hide | show | motion ( 1 | 2 ) <command> | ( press | release ) <button> <command> : Set joystick motion (use $1 .. $8) or button command.", pattern, true);
  list("  jm <axis> <button> <button> : Map two buttons on an axis.", pattern, true);
  list("  js <axis> <speed> <threshold> [<center>] : Set axis maximum speed and ignore threshold for \"jc motion\" commands.", pattern, true);
  list("  kick : Adds the KickEngine view.", pattern, true);
  list("  log start | stop | clear | full | jpeg : Record log file and (de)activate image compression.", pattern, true);
  list("  log save [split <parts>] [<file>] : Save log file with given name or modified current log file name. Split command saves in given number of parts", pattern, true);
  list("  log saveAudio [<file>] : Save audio data from log.", pattern, true);
  list("  log saveImages [raw] [onlyPlaying] [<takeEachNth>] [<dir>] : Save images from log.", pattern, true);
  list("  log saveInertialSensorData [<file>] : Save the inertial sensor data from the log into a dataset. Require motion log.", pattern, true);
  list("  log saveJointAngleData [<file>] : Save the joint angle data from the lot into a dataset. Require motion log.", pattern, true);
  list("  log saveLabeledBallSpots [<file>] : Extracts labeled BallSpots.", pattern, true);
  list("  log saveTiming [<file>] : Save timing data from log to csv.", pattern, true);
  list("  log trim ( until <end frame> | from <start frame> | between <start frame> <end frame> ) : Keep only the given section of the log. WARNING: Overrides file!", pattern, true);
  list("  log ? [<pattern>] : Display information about log file.", pattern, true);
  list("  log load <file> | clear : Load log-file or clear all frames.", pattern, true);
  list("  log keep ( ballPercept [ seen | guessed ] | ballSpots | circlePercept | lower | option <option> [<state>] | penaltyMarkPercept | upper ): Remove the log's frames not matching specified criteria.", pattern, true);
  list("  log ( keep | remove ) <message> {<message>} : Filter specified messages of all frames.", pattern, true);
  list("  log start | pause | stop | forward [image] | backward [image] | repeat | goto <number> | time <minutes> <seconds> | cycle | once | fastForward | fastBackward : Replay log file.", pattern, true);
  list("  log mr [list] : Generate module requests to replay log file.", pattern, true);
  list("  mof : Recompile motion net and send it to the robot. ", pattern, true);
  list("  msg off | on | log <file> | enable | disable : Switch output of text messages on or off. Log text messages to a file. Switch message handling on or off.", pattern, true);
  list("  mr ? [<pattern>] | modules [<pattern>] | save | <representation> ( ? [<pattern>] | <module> [<thread>] | default | off ) : Send module request.", pattern, true);
  list("  mv <x> <y> <z> [<rotx> <roty> <rotz>] : Move the selected simulated robot to the given position.", pattern, true);
  list("  mvb <x> <y> <z> : Move the ball to the given position.", pattern, true);
  list("  poll : Poll for all available debug requests and drawings. ", pattern, true);
  list("  pr none | illegalBallContact | playerPushing | illegalMotionInSet | inactivePlayer | illegalDefender | leavingTheField | kickOffGoal | requestForPickup | localGameStuck | illegalPositioning | substitute | manual : Penalize robot.", pattern, true);
  list("  save ? [<pattern>] | <key> [<path>] : Save debug data to a configuration file.", pattern, true);
  list("  set ? [<pattern>] | <key> ( ? | unchanged | <data> ) : Change debug data or show its specification.", pattern, true);
  list("  si reset [<number>] | ( lower | upper ) [number] [grayscale] [region <left> <top> <right> <bottom>] [<file>] : Save the lower/upper camera's image.", pattern, true);
  list("  v3 ? [<pattern>] | <image> [jpeg] [<thread>] [<name>] : Add a set of 3-D views for a certain image.", pattern, true);
  list("  vd <debug data> ( on | off ) : Show debug data in a window or switch sending it off.", pattern, true);
  list("  vf <name> : Add field view.", pattern, true);
  list("  vfd ? [<pattern>] | off | ( all | <name> ) ( ? [<pattern>] | <drawing> ( on | off ) ) : (De)activate debug drawing in field view.", pattern, true);
  list("  vi ? [<pattern>] | <image> [jpeg] [segmented] [<thread>] [<name>] [gain <value>] [ddScale <value>] : Add image view.", pattern, true);
  list("  vic ? [<pattern>] | ( all | <name> ) [alt | noalt] [ctrl | noctrl] [shift | noshift] <command> : Set image view button release command.", pattern, true);
  list("  vid ? [<pattern>] | off | ( all | <name> ) ( ? [<pattern>] | <drawing> ( on | off ) ) : (De)activate debug drawing in image view.", pattern, true);
  list("  vp <name> <numOfValues> <minValue> <maxValue> [<yUnit> [<xUnit> [<xScale>]]]: Add plot view.", pattern, true);
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
  std::string robotName = std::string(".") + name;
  this->robotName = robotName.c_str();

  mode = SystemCall::remoteRobot;
  RemoteRobot* rr = new RemoteRobot(name, ip);
  this->robotName = nullptr;
  if(!rr->isClient())
  {
    if(ip != "")
    {
      delete rr;
      Global::theSettings = nullptr;
      printLn(std::string("No connection to ") + ip + " established!");
      return false;
    }
    else
      printLn("Waiting for a connection...");
  }
  selected.clear();
  remoteRobots.push_back(rr);
  selected.push_back(remoteRobots.back());
  rr->addPerRobotViews();
  rr->start();
  return true;
}

bool ConsoleRoboCupCtrl::startLogFile(In& stream)
{
  std::string name, fileName;
  stream >> name >> fileName;
  if(fileName.empty())
  {
    if(logFile.substr(0, 5) == "Logs/")
      fileName = logFile.substr(5);
    else
      fileName = logFile;
  }
  if(int(fileName.rfind('.')) <= int(fileName.find_last_of("\\/")))
    fileName = fileName + ".log";
  if(fileName[0] != '\\' && fileName[0] != '/' && (fileName.size() < 2 || fileName[1] != ':'))
    fileName = std::string("Logs/") + fileName;
  {
    InBinaryFile test(fileName);
    if(!test.exists())
      return false;
  }

  std::string robotName = std::string(".") + name;
  mode = SystemCall::logFileReplay;
  logFile = fileName;
  this->robotName = robotName.c_str();
  robots.push_back(new Robot(name));
  this->robotName = nullptr;
  selected.clear();
  RobotConsole* rc = robots.back()->getRobotThread();
  selected.push_back(rc);
  robots.back()->start();
  return true;
}

bool ConsoleRoboCupCtrl::startMultiLogFile(In& stream)
{
  std::string directory;
  stream >> directory;
  const std::function<bool(std::string)> func = [this, &func](const std::string& dirName)
  {
    QDir dir(dirName.c_str());
    if(!dir.exists())
      return false;
    for(const QString& folder : dir.entryList(QDir::Dirs | QDir::NoDotAndDotDot | QDir::NoSymLinks))
    {
      func(dirName + '/' + folder.toStdString());
    }
    for(const QString& fileName : dir.entryList(QStringList("*_*_*.log"), QDir::Files | QDir::NoDotAndDotDot | QDir::NoSymLinks))
    {
      const QStringList& list = fileName.left(fileName.indexOf('.')).split('_');
      // sl LOG_<NAME>[_<HALF>][_<EXTRA>]
      executeConsoleCommand("sl LOG_" + list.at(1).toStdString() + (list.size() > 8 ? '_' + list.at(7).toStdString() : "") + (list.size() > 9 ? '_' + list.at(9).toStdString() : "") + (" \"" + dirName + '/' + fileName.toStdString() + "\""));
      executeConsoleCommand("dr debug:keepAllMessages");
    }
    return true;
  };
  const bool correct = func(directory);
  executeConsoleCommand("robot all");
  return correct;
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
  SYNC;
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
    "kick",
    "call",
    "ci off",
    "ci on",
    "cls",
    "dr off",
    "dt off",
    "dt on",
    "echo",
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
    "log save split",
    "log full",
    "log jpeg",
    "log saveAudio",
    "log saveImages raw onlyPlaying",
    "log saveInertialSensorData",
    "log saveLabeledBallSpots gray",
    "log saveJointAngleData",
    "log saveTiming",
    "log trim from",
    "log trim until",
    "log trim between",
    "log ?",
    "log mr list",
    "log load",
    "log cycle",
    "log once",
    "log pause",
    "log forward image",
    "log backward image",
    "log repeat",
    "log goto",
    "log time",
    "log fastForward",
    "log fastBackward",
    "log keep ballPercept seen",
    "log keep ballPercept guessed",
    "log keep ballSpots",
    "log keep circlePercept",
    "log keep lower",
    "log keep upper",
    "log keep penaltyMarkPercept",
    "log keep option",
    "log analyzeJoints",
    "log checkForGyroChestBoardProblem",
    "mof",
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
    "robot all",
    "sc",
    "si lower grayscale region",
    "si upper grayscale region",
    "si lower number grayscale region",
    "si upper number grayscale region",
    "si lower number region",
    "si upper number region",
    "si lower region",
    "si upper region",
    "si reset",
    "sl",
    "sml",
    "st off",
    "st on",
    "v3 image Upper",
    "v3 image jpeg Upper",
    "v3 image Lower",
    "v3 image jpeg Lower",
    "vf",
    "vi none",
    "vi image jpeg segmented",
    "vi image segmented",
    "vi image Upper jpeg segmented",
    "vi image Upper segmented"
    "vi image Lower jpeg segmented",
    "vi image Lower segmented"
  };

  SYNC;
  completion.clear();
  const int num = sizeof(commands) / sizeof(commands[0]);
  for(int i = 0; i < num; ++i)
    completion.insert(commands[i]);

  for(Robot* robot : robots)
    completion.insert("robot " + robot->getName());
  for(RemoteRobot* remoteRobot : remoteRobots)
    completion.insert("robot " + remoteRobot->getName());

  FOREACH_ENUM(MessageID, i)
  {
    completion.insert(std::string("log keep ") + TypeRegistry::getEnumName(i));
    completion.insert(std::string("log remove ") + TypeRegistry::getEnumName(i));
  }

  addCompletionFiles("log load ", std::string(File::getBHDir()) + "/Config/Logs/*.log");
  addCompletionFiles("log save ", std::string(File::getBHDir()) + "/Config/Logs/*.log", false);
  addCompletionFiles("call ", std::string(File::getBHDir()) + "/Config/Scenes/*.con");

  if(moduleInfo)
  {
    for(const auto& r : moduleInfo->representations)
    {
      completion.insert(std::string("mr ") + r + " default");
      completion.insert(std::string("mr ") + r + " off");
      for(const auto& m : moduleInfo->modules)
        if(std::find(m.representations.begin(), m.representations.end(), r) != m.representations.end())
        {
          for(const Configuration::Thread& thread : moduleInfo->config())
            completion.insert(std::string("mr ") + r + " " + m.name + " " + thread.name);
        }
    }
  }

  if(debugRequestTable && moduleInfo)
  {
    for(const auto& i : debugRequestTable->slowIndex)
    {
      completion.insert(std::string("dr ") + translate(i.first) + " on");
      completion.insert(std::string("dr ") + translate(i.first) + " off");
      if(i.first.substr(0, 13) == "debug images:")
      {
        const std::string cleanedSubst = translate(i.first.substr(13));

        for(const auto& config : moduleInfo->config())
        {
          completion.insert(std::string("v3 ") + cleanedSubst + " " + config.name);
          completion.insert(std::string("v3 ") + cleanedSubst + " jpeg " + config.name);
          completion.insert(std::string("vi ") + cleanedSubst + " " + config.name);
          completion.insert(std::string("vi ") + cleanedSubst + " segmented " + config.name);
          completion.insert(std::string("vi ") + cleanedSubst + " jpeg " + config.name);
          completion.insert(std::string("vi ") + cleanedSubst + " jpeg segmented " + config.name);
        }
      }
      else if(i.first.substr(0, 11) == "debug data:")
      {
        const std::string cleanedSubst = translate(i.first.substr(11));

        completion.insert(std::string("vd ") + cleanedSubst + " off");
        completion.insert(std::string("vd ") + cleanedSubst + " on");

        completion.insert(std::string("get ") + cleanedSubst + " ?");
        completion.insert(std::string("set ") + cleanedSubst + " ?");
        completion.insert(std::string("set ") + cleanedSubst + " unchanged");

        if(std::count(cleanedSubst.begin(), cleanedSubst.end(), ':') == 1 && cleanedSubst.substr(0, 11) == "parameters:")
        {
          std::string parametersName = cleanedSubst.substr(11);

          completion.insert(std::string("save ") + cleanedSubst);
          if(representationToFile.find(cleanedSubst) == representationToFile.end())
          {
            parametersName[0] = static_cast<char>(tolower(parametersName[0]));
            if(parametersName.size() > 1 && isupper(parametersName[1]))
              for(int i = 1; i + 1 < static_cast<int>(parametersName.size()) && isupper(parametersName[i + 1]); ++i)
                parametersName[i] = static_cast<char>(tolower(parametersName[i]));
            representationToFile[cleanedSubst] = parametersName + ".cfg";
          }
        }
      }
    }
  }

  if(imageViews)
  {
    for(unsigned int i = 0; i < 18; ++i)
      completion.insert(std::string("vic all ")
                        + ((i < 8 || i >= 14) ? (((i & 4) ? "no" : "") + std::string("alt ")) : "")
                        + (i < 12 ? (((i & 2) ? "no" : "") + std::string("ctrl ")) : "")
                        + ((i & 1) ? "no" : "") + "shift");
    for(const auto& imageViewPair : *imageViews)
      for(unsigned int i = 0; i < 18; ++i)
        completion.insert(std::string("vic ") + imageViewPair.first + " "
                          + ((i < 8 || i >= 14) ? (((i & 4) ? "no" : "") + std::string("alt ")) : "")
                          + (i < 12 ? (((i & 2) ? "no" : "") + std::string("ctrl ")) : "")
                          + ((i & 1) ? "no" : "") + "shift");
  }

  if(threadData)
  {
    completion.insert(std::string("vid off"));
    completion.insert(std::string("vfd off"));
    for(const auto& data : *threadData)
      for(const auto& drawingsPair : data.second.drawingManager.drawings)
      {
        if(!strcmp(data.second.drawingManager.getDrawingType(drawingsPair.first), "drawingOnImage") && imageViews)
        {
          completion.insert(std::string("vid all ") + translate(drawingsPair.first) + " on");
          completion.insert(std::string("vid all ") + translate(drawingsPair.first) + " off");
          for(const auto& imageViewPair : *imageViews)
          {
            completion.insert(std::string("vid ") + imageViewPair.first + " " + translate(drawingsPair.first) + " on");
            completion.insert(std::string("vid ") + imageViewPair.first + " " + translate(drawingsPair.first) + " off");
          }
        }
        else if(!strcmp(data.second.drawingManager.getDrawingType(drawingsPair.first), "drawingOnField") && fieldViews)
        {
          completion.insert(std::string("vfd all ") + translate(drawingsPair.first) + " on");
          completion.insert(std::string("vfd all ") + translate(drawingsPair.first) + " off");
          for(const auto& fieldViewPair : *fieldViews)
          {
            completion.insert(std::string("vfd ") + fieldViewPair.first + " " + translate(drawingsPair.first) + " on");
            completion.insert(std::string("vfd ") + fieldViewPair.first + " " + translate(drawingsPair.first) + " off");
          }
        }
      }
  }

  if(plotViews && debugRequestTable)
    for(const auto& plotPair : *plotViews)
      for(const auto& i : debugRequestTable->slowIndex)
        if(translate(i.first).substr(0, 5) == "plot:")
        {
          for(int color = 0; color < RobotConsole::numOfColors; ++color)
            completion.insert(std::string("vpd ") + plotPair.first + " " +
                              translate(i.first).substr(5) + " " +
                              TypeRegistry::getEnumName(static_cast<RobotConsole::Color>(color)));
          completion.insert(std::string("vpd ") + plotPair.first + " " +
                            translate(i.first).substr(5) + " off");
        }

  for(const auto& repr : representationToFile)
    completion.insert(std::string("save ") + repr.first);

  completion.insert(std::string("save representation:CameraSettings"));
  completion.insert(std::string("save representation:FieldColors"));

  gameController.addCompletion(completion);

  currentCompletionIndex = completion.end();
}

void ConsoleRoboCupCtrl::addCompletionFiles(const std::string& command, const std::string& pattern, bool removeExtension)
{
  QString qpattern(pattern.c_str());
  qpattern.replace("\\", "/");
  const int lastSlashIdx = qpattern.lastIndexOf('/');

  QDir qdir(qpattern.left(lastSlashIdx));
  qdir.setFilter(QDir::Files | QDir::Dirs | QDir::NoDot | QDir::NoDotDot);
  QStringList qsl;
  qsl.append(qpattern.right(qpattern.size() - lastSlashIdx - 1));
  qdir.setNameFilters(qsl);
  QDirIterator it(qdir, QDirIterator::Subdirectories);
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
  SYNC;
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
  SYNC;
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

void ConsoleRoboCupCtrl::showInputDialog(std::string& command)
{
  QString qstrCommand(command.c_str());
  QRegExp re("\\$\\{([^\\}]*)\\}", Qt::CaseSensitive, QRegExp::RegExp2);
  bool ok = true;
  while(ok && re.indexIn(qstrCommand) != -1)
  {
    QStringList list = re.cap(1).split(',');
    QString label = list.takeFirst();
    QString input;
    if(list.isEmpty())
    {
      // ${Text input:}
      input = QInputDialog::getText(0, "Input", label, QLineEdit::Normal, "", &ok);
    }
    else if(list.length() == 1)
    {
      QString qpattern(list.takeFirst());
      qpattern.replace("\\", "/");
      const int lastSlashIdx = qpattern.lastIndexOf('/');

      QString path(qpattern.left(lastSlashIdx));

      QStringList qsl;
      qsl.append(qpattern.right(qpattern.size() - lastSlashIdx - 1));
      qsl.append("all (*.*)");

      QSettings settings("B-Human", "SimRobot");

      // ${Select Log File:,../Logs/*.log}
      // QFileDialog::DontUseNativeDialog is not fine, but it fixes visualisation qt-bug (see https://bugreports.qt.io/browse/QTBUG-29248)
      input = QFileDialog::getOpenFileName(nullptr, label, settings.value(qsl.front(), path).toString(), qsl.join(";;"), nullptr, QFileDialog::DontUseNativeDialog);
      if((ok = input != QString::null))
        settings.setValue(qsl.front(), QDir().absoluteFilePath(input));
    }
    else if(list.length() == 2 && list.first().isEmpty())
    {
      // ${Select LogFolder:,,../Logs/}
      QString path(list.last());
      path.replace("\\", "/");

      QSettings settings("B-Human", "SimRobot");

      input = QFileDialog::getExistingDirectory(nullptr, label, settings.value("LogFolder", path).toString(), QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
      if((ok = input != QString::null))
        settings.setValue("LogFolder", input.left(input.lastIndexOf('/')));
    }
    else
    {
      // ${Select Robot:,Leonard,Rajesh,Lenny}
      input = QInputDialog::getItem(nullptr, "Input", label, list, 0, true, &ok);
    }
    qstrCommand.replace(re.cap(0), input);
  }
  if(ok)
  {
    command = std::string(qstrCommand.
#ifdef WINDOWS
                          toLatin1()
#else
                          toUtf8()
#endif
                          .constData());
  }
  else
  {
    command = "";
  }
}
