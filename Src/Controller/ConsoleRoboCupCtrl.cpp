/**
 * @file Controller/ConsoleRoboCupCtrl.cpp
 *
 * This file implements the class ConsoleRoboCupCtrl.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "ConsoleRoboCupCtrl.h"

#include <QDir>
#include <QDirIterator>
#include <QInputDialog>
#include <QSettings>

#include <algorithm>

#include "LocalRobot.h"
#include "Controller/Views/ConsoleView.h"
#include "Controller/Views/CABSLGraphView.h"
#include "Platform/SimRobotQt/Robot.h"
#include "RemoteRobot.h"
#include "TeamRobot.h"
#include <SimRobotEditor.h>
#include "Platform/File.h"
#include "BHToolBar.h"

#define FRAMES_PER_SECOND 60

ConsoleRoboCupCtrl::ConsoleRoboCupCtrl(SimRobot::Application& application)
  : RoboCupCtrl(application),
    calculateImage(true),
    calculateImageFps(FRAMES_PER_SECOND),
    globalNextImageTimeStamp(0),
    mode(SystemCall::simulatedRobot),
    currentCompletionIndex(completion.end()),
    INIT_TEAM_COMM,
    timeStamp(0),
    robotNumber(-1),
    toolBar(*this)
{
  application.getLayoutSettings().setValue("Run", true);

  debugRequestTable = 0;
  moduleInfo = 0;
  drawingManager = 0;
  imageViews = 0;
  fieldViews = 0;
  plotViews = 0;
  newLine = true;
  nesting = 0;

  consoleView = new ConsoleView("Console", *this);
  addView(consoleView, 0, SimRobot::Flag::verticalTitleBar);
  addView(new ConsoleView("Console.Pad", *this, true), consoleView, SimRobot::Flag::verticalTitleBar);

  SimRobot::Object* documentationCategory = addCategory("Docs", 0, ":/Icons/page_white_h.png");
  addView(new CABSLGraphViewObject("Docs.BehaviorControl2015", "BehaviorControl2015", "Options.h"), documentationCategory);

  representationToFile["parameters:BodyContourProvider"] = "bodyContourProvider.cfg";
  representationToFile["representation:CameraCalibration"] = "cameraCalibration.cfg";
  representationToFile["representation:UpperCameraSettings"] = "upperCameraSettings.cfg";
  representationToFile["representation:LowerCameraSettings"] = "lowerCameraSettings.cfg";
  representationToFile["representation:JointCalibration"] = "jointCalibration.cfg";
  representationToFile["representation:MassCalibration"] = "massCalibration.cfg";
  representationToFile["representation:RobotDimensions"] = "robotDimensions.cfg";
  representationToFile["representation:CameraIntrinsics"] = "cameraIntrinsics.cfg";
  representationToFile["parameters:WalkingEngine"] = "walkingEngine.cfg";
  representationToFile["parameters:DmpKickEngine"] = "dmpKickEngine.cfg";
}

bool ConsoleRoboCupCtrl::compile()
{
  if(!RoboCupCtrl::compile())
    return false;

  if(!robots.empty())
    selected.push_back((*robots.begin())->getRobotProcess());

  start();
  Global::theStreamHandler = &streamHandler;

  std::string fileName = application->getFilePath().
#ifdef WINDOWS
                         toLatin1
#else
                         toUtf8
#endif
                         ().constData();
  std::string::size_type p = fileName.find_last_of("\\/"),
                         p2 = fileName.find_last_of(".");
  if(p2 > p)
    fileName = fileName.substr(0, p2);
  executeFile(fileName.c_str(), false, 0);

  for(std::list<Robot*>::iterator i = robots.begin(); i != robots.end(); ++i)
    (*i)->getRobotProcess()->handleConsole("endOfStartScript");
  for(std::list<RemoteRobot*>::iterator i = remoteRobots.begin(); i != remoteRobots.end(); ++i)
    (*i)->handleConsole("endOfStartScript");
  for(std::list<TeamRobot*>::iterator i = teamRobots.begin(); i != teamRobots.end(); ++i)
    (*i)->handleConsole("endOfStartScript");
  Global::theStreamHandler = &streamHandler;
  return true;
}

void ConsoleRoboCupCtrl::link()
{
  SimRobotEditor::Editor* editor = (SimRobotEditor::Editor*)application->resolveObject("Editor");
  if(editor)
  {
    QFileInfo fileInfo(application->getFilePath());
    editor->addFile(fileInfo.path() + "/" + fileInfo.baseName() + ".con", "call[ ]+([\\\\/a-z0-9\\.\\-_]+)");

    SimRobotEditor::Editor* kicksFolder = editor->addFolder("Kicks");
    QString configDir = QFileInfo(fileInfo.dir().path()).dir().path();
    for(int i = 1; i < WalkRequest::numOfKickTypes; i += 2)
    {
      char filePath[256];
      sprintf(filePath, "/Kicks/%s.cfg", WalkRequest::getName(WalkRequest::KickType(i)));
      kicksFolder->addFile(configDir + filePath, "");
    }
  }
}

ConsoleRoboCupCtrl::~ConsoleRoboCupCtrl()
{
  for(std::list<RemoteRobot*>::iterator i = remoteRobots.begin(); i != remoteRobots.end(); ++i)
    (*i)->announceStop();
  for(std::list<TeamRobot*>::iterator i = teamRobots.begin(); i != teamRobots.end(); ++i)
    (*i)->announceStop();

  for(std::list<RemoteRobot*>::iterator i = remoteRobots.begin(); i != remoteRobots.end(); ++i)
    delete *i;
  for(std::list<TeamRobot*>::iterator i = teamRobots.begin(); i != teamRobots.end(); ++i)
    delete *i;

  stop();
  mode = SystemCall::simulatedRobot;
  Global::theSettings = 0;
}

void ConsoleRoboCupCtrl::update()
{
  {
    SYNC;
    for(std::list<std::string>::const_iterator i = textMessages.begin(); i != textMessages.end(); ++i)
      if(*i == "_cls")
        consoleView->clear();
      else if(newLine || &*i != &*textMessages.rend())
        consoleView->printLn(i->c_str());
      else
        consoleView->print(i->c_str());
    textMessages.clear();
  }
  RoboCupCtrl::update();

  for(std::list<RemoteRobot*>::iterator i = remoteRobots.begin(); i != remoteRobots.end(); ++i)
    (*i)->update();

  Global::theStreamHandler = &streamHandler;
  ntp.doSynchronization(SystemCall::getRealSystemTime(), theTeamSender.out);
  theTeamHandler.send();
  (void) theTeamHandler.receive();
  theTeamReceiver.handleAllMessages(*this);
  theTeamReceiver.clear();
  for(std::list<TeamRobot*>::iterator i = teamRobots.begin(); i != teamRobots.end(); ++i)
    (*i)->update();
  Global::theStreamHandler = &streamHandler;

  application->setStatusMessage(statusText.c_str());

  if(completion.empty())
    createCompletion();
}

void ConsoleRoboCupCtrl::executeFile(std::string name, bool printError, RobotConsole* console)
{
  if(nesting == 10)
    printLn("Nesting Error");
  else
  {
    ++nesting;
    if((int) name.rfind('.') <= (int) name.find_last_of("\\/"))
      name = name + ".con";
    if(name[0] != '/' && name[0] != '\\' && (name.size() < 2 || name[1] != ':'))
      name = std::string("Scenes\\") + name;
    InBinaryFile stream(name.c_str());
    if(!stream.exists())
    {
      if(printError)
        printLn(name + " not found");
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
          executeConsoleCommand(line, console);
      }
    }
    --nesting;
  }
}

void ConsoleRoboCupCtrl::selectedObject(const SimRobot::Object& obj)
{
  const QString& fullName = obj.getFullName();
  std::string robotName = fullName.mid(fullName.lastIndexOf('.') + 1).toUtf8().constData();
  for(std::list<Robot*>::iterator i = robots.begin(); i != robots.end(); ++i)
    if(robotName == (*i)->getName())
    {
      selected.clear();
      selected.push_back((*i)->getRobotProcess());
      printLn(std::string("robot ") + (*i)->getName());
      return;
    }
}

void ConsoleRoboCupCtrl::pressedKey(int key, bool pressed)
{
  if(key > 10)
    for(std::list<RobotConsole*>::iterator i = selected.begin(); i != selected.end(); ++i)
      (*i)->handleKeyEvent(key - 11, pressed);
}

SystemCall::Mode ConsoleRoboCupCtrl::getMode() const
{
  size_t threadId = Thread<ProcessBase>::getCurrentId();
  for(std::list<Robot*>::const_iterator i = robots.begin(); i != robots.end(); ++i)
    if((*i)->getRobotProcess())
      for(ProcessList::const_iterator j = (*i)->begin(); j != (*i)->end(); ++j)
        if((*j)->getId() == threadId)
          return (*i)->getRobotProcess()->mode;
  return mode;
}

void ConsoleRoboCupCtrl::setRepresentation(const std::string representationName, const Streamable& representation)
{
  OutMapSize size(true);
  size << representation;
  char* buf = new char[size.getSize()];
  OutMapMemory memory(buf, true);
  memory << representation;
  buf[size.getSize() - 1] = 0; // overwrite final space

  std::string command = "set representation:" + representationName + " " + buf;
  executeConsoleCommand(command);

  delete[] buf;
}

void ConsoleRoboCupCtrl::executeConsoleCommand(std::string command, RobotConsole* console)
{
  showInputDialog(command);
  std::string buffer;
  InConfigMemory stream(command.c_str(), command.size());
  stream >> buffer;
  if(buffer == "") // comment
    return;
  else if(buffer == "help" || buffer == "?")
    help(stream);
  else if(buffer == "robot")
  {
    stream >> buffer;
    if(buffer == "?")
    {
      for(std::list<Robot*>::iterator i = robots.begin(); i != robots.end(); ++i)
        print(std::string((*i)->getName()) + " ");
      for(std::list<RemoteRobot*>::iterator i = remoteRobots.begin(); i != remoteRobots.end(); ++i)
        print(std::string((*i)->getName()) + " ");
      for(std::list<TeamRobot*>::iterator i = teamRobots.begin(); i != teamRobots.end(); ++i)
        print(std::string((*i)->getName()) + " ");
      printLn("");
      return;
    }
    else if(buffer == "all")
    {
      selected.clear();
      for(std::list<Robot*>::iterator i = robots.begin(); i != robots.end(); ++i)
        selected.push_back((*i)->getRobotProcess());
      for(std::list<RemoteRobot*>::iterator i = remoteRobots.begin(); i != remoteRobots.end(); ++i)
        selected.push_back(*i);
      for(std::list<TeamRobot*>::iterator i = teamRobots.begin(); i != teamRobots.end(); ++i)
        selected.push_back(*i);
      return;
    }
    else
    {
      selected.clear();
      for(;;)
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
    if(buffer == "on")
      gameController.automatic = true;
    else if(buffer == "off")
      gameController.automatic = false;
    else
      printLn("Syntax Error");
  }
  else if(buffer == "call")
  {
    stream >> buffer;
    executeFile(buffer, true, console);
  }
  else if(buffer == "ci")
  {
    if(!calcImage(stream))
      printLn("Syntax Error");
  }
  else if(buffer == "st")
  {
    stream >> buffer;
    if(buffer == "on")
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
        // real time contiues at simulation time
        time = getTime() - SystemCall::getRealSystemTime();
        simTime = false;
      }
    }
    else
      printLn("Syntax Error");
  }
  else if(buffer == "dt")
  {
    stream >> buffer;
    if(buffer == "on" || buffer == "")
      delayTime = simStepLength;
    else if(buffer == "off")
      delayTime = 0;
    else
    {
      for(size_t i = 0; i < buffer.size(); ++i)
        if(!isdigit(buffer[i]))
        {
          printLn("Syntax Error");
          return;
        }
      delayTime = 1000.f / (float) std::max(1, atoi(buffer.c_str()));
    }
  }
  else if(buffer == "gc")
  {
    if(!gameController.handleGlobalConsole(stream))
      printLn("Syntax Error");
  }
  else if(buffer == "sc")
  {
    if(!startRemote(stream))
    {
      selected.clear();
      if(!robots.empty())
        selected.push_back((*robots.begin())->getRobotProcess());
    }
  }
  else if(buffer == "sl")
  {
    if(!startLogFile(stream))
      printLn("Logfile not found!");
  }
  else if(buffer == "su")
    startTeamRobot(stream);
  else if(buffer == "tc")
  {
    int port;
    std::string subnet;
    stream >> port >> subnet;
    if(subnet == "")
      subnet = "255.255.255.255";
    if(port)
      theTeamHandler.start(port, subnet.c_str());
  }
  else if(buffer == "tc2")
  {
    // ignore this command (it is used in the TeamComm3D scene)
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
    Global::theStreamHandler = &streamHandler;
  }
  else
  {
    for(std::list<RobotConsole*>::iterator i = selected.begin(); i != selected.end(); ++i)
      (*i)->handleConsole(command);
    Global::theStreamHandler = &streamHandler;
  }
  if(completion.empty())
    createCompletion();
}

bool ConsoleRoboCupCtrl::selectRobot(const std::string& name)
{
  for(std::list<Robot*>::iterator i = robots.begin(); i != robots.end(); ++i)
    if(name == (*i)->getName())
    {
      selected.push_back((*i)->getRobotProcess());
      return true;
    }
  for(std::list<RemoteRobot*>::iterator i = remoteRobots.begin(); i != remoteRobots.end(); ++i)
    if(name == (*i)->getName())
    {
      selected.push_back(*i);
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
  list("  sl <name> <file> : Starts a robot reading its inputs from a log file.", pattern, true);
  list("  su <name> <number> : Starts a UDP team connection to a remote robot with a certain player number. Requires command tc.", pattern, true);
  list("  tc <port> <subnet>: Listen to team communication on given port and broadcast to a subnet.", pattern, true);
  list("Global commands:", pattern, true);
  list("  ar off | on : Switches automatic referee on or off.", pattern, true);
  list("  call <file> : Execute a script file.", pattern, true);
  list("  ci off | on | <fps> : Switch the calculation of images on or off or activate it and set the frame rate.", pattern, true);
  list("  cls : Clear console window.", pattern, true);
  list("  dt off | on | <fps> : Delay time of a simulation step to real time or a certain number of frames per second.", pattern, true);
  list("  echo <text> : Print text into console window. Useful in console.con.", pattern, true);
  list("  gc initial | ready | set | playing | finished | kickOffBlue | kickOffRed | outByBlue | outByRed | gamePlayoff | gameRoundRobin : Set GameController state.", pattern, true);
  list("  help | ? [<pattern>] : Display this text.", pattern, true);
  list("  robot ? | all | <name> {<name>} : Connect console to a set of active robots. Alternatively, double click on robot.", pattern, true);
  list("  st off | on : Switch simulation of time on or off.", pattern, true);
  list("  # <text> : Comment.", pattern, true);
  list("Robot commands:", pattern, true);
  list("  ac ? | both | lower | upper : Change camera source shown in field views.", pattern, true);
  list("  bc <red%> <green%> <blue%> : Set the background color of all 3-D views.", pattern, true);
  list("  kick : Adds the KickEngine view.", pattern, true);
  list("  dr ? [<pattern>] | off | <key> ( off | on ) : Send debug request.", pattern, true);
  list("  get ? [<pattern>] | <key> [?]: Show debug data or show its specification.", pattern, true);
  list("  jc hide | show | motion <num> <command> | ( press | release ) <button> <command> : Set joystick motion (use $1 .. $6) or button command.", pattern, true);
  list("  jm <axis> ( off | <button> <button> ) : Map two buttons on an axis.", pattern, true);
  list("  js <axis> <speed> <threshold> [<center>] : Set axis maximum speed and ignore threshold for \"jc motion <num>\" commands.", pattern, true);
  list("  log start | stop | clear | save <file> | full | jpeg : Record log file and (de)activate image compression.", pattern, true);
  list("  log saveAudio <file> : Save audio data from log.", pattern, true);
  list("  log saveImages [raw] <file> : Save images from log.", pattern, true);
  list("  log saveTiming <file> : Save timing data from log to csv.", pattern, true);
  list("  log ? [<pattern>] | load <file> | ( keep | remove ) <message> {<message>} : Load, filter, and display information about log file.", pattern, true);
  list("  log start | pause | stop | forward [image] | backward [image] | repeat | goto <number> | cycle | once | fast_forward | fast_rewind : Replay log file.", pattern, true);
  list("  mof : Recompile motion net and send it to the robot. ", pattern, true);
  list("  msg off | on | log <file> | enable | disable : Switch output of text messages on or off. Log text messages to a file. Switch message handling on or off.", pattern, true);
  list("  mr ? [<pattern>] | modules [<pattern>] | save | <representation> ( ? [<pattern>] | <module> | off ) : Send module request.", pattern, true);
  list("  mv <x> <y> <z> [<rotx> <roty> <rotz>] : Move the selected simulated robot to the given position.", pattern, true);
  list("  mvb <x> <y> <z> : Move the ball to the given position.", pattern, true);
  list("  poll : Poll for all available debug requests and drawings. ", pattern, true);
  list("  pr none | ballHolding | playerPushing | inactivePlayer | illegalDefender | leavingTheField | playingWithHands | requestForPickup : Penalize robot.", pattern, true);
  list("  qfr queue | replace | reject | collect <seconds> | save [<seconds>] : Send queue fill request.", pattern, true);
  list("  set ? [<pattern>] | <key> ( ? | unchanged | <data> ) : Change debug data or show its specification.", pattern, true);
  list("  save ? [<pattern>] | <key> [<path>] : Save debug data to a configuration file.", pattern, true);
  list("  si reset | (upper | lower) [number] <file> : Save the upper/lower cams image.", pattern, true);
  list("  v3 ? [<pattern>] | <image> [jpeg] [upper] [<name>] : Add a set of 3-D views for a certain image.", pattern, true);
  list("  vd <debug data> on | off : Show debug data in a window or switch sending it off.", pattern, true);
  list("  vf <name> : Add field view.", pattern, true);
  list("  vfd ? [<pattern>] | <name> ( ? [<pattern>] | <drawing> ( on | off ) ) : (De)activate debug drawing in field view.", pattern, true);
  list("  vi ? [<pattern>] | <image> [jpeg] [segmented] [upper] [<name>] [gain <value>] : Add image view.", pattern, true);
  list("  vid ? [<pattern>] | <name> ( ? [<pattern>] | <drawing> ( on | off ) ) : (De)activate debug drawing in image view.", pattern, true);
  list("  vp <name> <numOfValues> <minValue> <maxValue> [<yUnit> <xUnit> <xScale>]: Add plot view.", pattern, true);
  list("  vpd ? [<pattern>] | <name> ( ? [<pattern>] | <drawing> ( ? [<pattern>] | <color> [<description>] | off ) ) : Plot data in a certain color in plot view.", pattern, true);
  list("  wek : Send walking engine kicks to the robot. ", pattern, true);
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
  std::string name,
      ip;
  stream >> name >> ip;
  std::string robotName = std::string(".") + name;
  this->robotName = robotName.c_str();

  mode = SystemCall::remoteRobot;
  RemoteRobot* rr = new RemoteRobot(name.c_str(), ip.c_str());
  this->robotName = 0;
  if(!rr->isClient())
  {
    if(ip != "")
    {
      delete rr;
      Global::theStreamHandler = &streamHandler;
      Global::theSettings = 0;
      printLn(std::string("No connection to ") + ip + " established!");
      return false;
    }
    else
      printLn("Waiting for a connection...");
  }
  selected.clear();
  remoteRobots.push_back(rr);
  selected.push_back(remoteRobots.back());
  rr->addViews();
  rr->start();
  return true;
}

bool ConsoleRoboCupCtrl::startLogFile(In& stream)
{
  std::string name,
      fileName;
  int offset;
  stream >> name >> fileName >> offset;
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
  mode = SystemCall::logfileReplay;
  logFile = fileName;
  this->robotName = robotName.c_str();
  robots.push_back(new Robot(name.c_str()));
  this->robotName = 0;
  logFile = "";
  selected.clear();
  RobotConsole* rc = robots.back()->getRobotProcess();
  selected.push_back(rc);
  robots.back()->start();
  return true;
}

bool ConsoleRoboCupCtrl::startTeamRobot(In& stream)
{
  std::string name;
  int number;
  stream >> name >> number;
  if(!number)
    return false;

  std::string robotName = std::string(".") + name;
  this->robotName = robotName.c_str();
  TeamRobot* tr = new TeamRobot(name.c_str(), number);
  this->robotName = 0;
  teamRobots.push_back(tr);
  selected.clear();
  selected.push_back(teamRobots.back());
  tr->addViews();
  tr->start();
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
    for(unsigned i = 0; i < state.size(); ++i)
      if(!isdigit(state[i]))
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

void ConsoleRoboCupCtrl::printStatusText(const std::string& text)
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
    "ac both",
    "ac lower",
    "ac upper",
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
    "log save",
    "log saveAudio",
    "log saveImages",
    "log saveImages raw",
    "log saveTiming",
    "log clear",
    "log full",
    "log jpeg",
    "log ?",
    "log mr",
    "log mr list",
    "log load",
    "log cycle",
    "log once",
    "log pause",
    "log forward",
    "log backward",
    "log repeat",
    "log goto",
    "log fast_forward",
    "log fast_rewind",
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
    "poll",
    "qfr queue",
    "qfr replace",
    "qfr reject",
    "qfr collect",
    "qfr save",
    "robot all",
    "sc",
    "si lower number",
    "si upper number",
    "si reset",
    "sl",
    "su",
    "st off",
    "st on",
    "tc",
    "v3 image upper",
    "v3 image jpeg upper",
    "vf",
    "vi none",
    "vi image jpeg segmented",
    "vi image segmented",
    "vi image upper jpeg segmented",
    "vi image upper segmented",
    "wek"
  };

  SYNC;
  completion.clear();
  const int num = sizeof(commands) / sizeof(commands[0]);
  for(int i = 0; i < num; ++i)
    completion.insert(commands[i]);

  for(std::list<Robot*>::iterator i = robots.begin(); i != robots.end(); ++i)
    completion.insert(std::string("robot ") + (*i)->getName());
  for(std::list<RemoteRobot*>::iterator i = remoteRobots.begin(); i != remoteRobots.end(); ++i)
    completion.insert(std::string("robot ") + (*i)->getName());
  for(std::list<TeamRobot*>::iterator i = teamRobots.begin(); i != teamRobots.end(); ++i)
    completion.insert(std::string("robot ") + (*i)->getName());

  for(int i = 1; i < numOfMessageIDs; ++i)
  {
    completion.insert(std::string("log keep ") + getName(MessageID(i)));
    completion.insert(std::string("log remove ") + getName(MessageID(i)));
  }

  addCompletionFiles("log load ", std::string(File::getBHDir()) + "/Config/Logs/*.log");
  addCompletionFiles("log save ", std::string(File::getBHDir()) + "/Config/Logs/*.log");
  addCompletionFiles("call ", std::string(File::getBHDir()) + "/Config/Scenes/*.con");

  if(moduleInfo)
  {
    for(const auto& r : moduleInfo->representations)
    {
      completion.insert(std::string("mr ") + r + " default");
      completion.insert(std::string("mr ") + r + " off");
      for(const auto& m : moduleInfo->modules)
        if(std::find(m.representations.begin(), m.representations.end(), r) != m.representations.end())
          completion.insert(std::string("mr ") + r + " " + m.name);
    }
  }

  if(debugRequestTable)
  {
    for(int i = 0; i < debugRequestTable->currentNumberOfDebugRequests; ++i)
    {
      completion.insert(std::string("dr ") + translate(debugRequestTable->debugRequests[i].description) + " on");
      completion.insert(std::string("dr ") + translate(debugRequestTable->debugRequests[i].description) + " off");
      if(debugRequestTable->debugRequests[i].description.substr(0, 13) == "debug images:")
      {
        completion.insert(std::string("v3 ") + translate(debugRequestTable->debugRequests[i].description.substr(13)) + " upper");
        completion.insert(std::string("v3 ") + translate(debugRequestTable->debugRequests[i].description.substr(13)) + " jpeg upper");
        completion.insert(std::string("vi ") + translate(debugRequestTable->debugRequests[i].description.substr(13)) + " upper");
        completion.insert(std::string("vi ") + translate(debugRequestTable->debugRequests[i].description.substr(13)) + " segmented upper");
        completion.insert(std::string("vi ") + translate(debugRequestTable->debugRequests[i].description.substr(13)) + " jpeg upper");
        completion.insert(std::string("vi ") + translate(debugRequestTable->debugRequests[i].description.substr(13)) + " jpeg segmented upper");
      }
      else if(debugRequestTable->debugRequests[i].description.substr(0, 11) == "debug data:")
      {
        completion.insert(std::string("vd ") + translate(debugRequestTable->debugRequests[i].description.substr(11)) + " off");
        completion.insert(std::string("vd ") + translate(debugRequestTable->debugRequests[i].description.substr(11)) + " on");

        completion.insert(std::string("get ") + translate(debugRequestTable->debugRequests[i].description.substr(11)) + " ?");
        completion.insert(std::string("set ") + translate(debugRequestTable->debugRequests[i].description.substr(11)) + " ?");
        completion.insert(std::string("set ") + translate(debugRequestTable->debugRequests[i].description.substr(11)) + " unchanged");
      }
    }
  }

  if(drawingManager)
  {
    if(imageViews)
      for(RobotConsole::Views::const_iterator v = imageViews->begin(); v != imageViews->end(); ++v)
      {
        completion.insert(std::string("cameraCalibrator ") + translate(v->first) + " on");
        completion.insert(std::string("cameraCalibrator ") + translate(v->first) + " off");
      }
    for(std::unordered_map< const char*, DrawingManager::Drawing>::const_iterator i = drawingManager->drawings.begin(); i != drawingManager->drawings.end(); ++i)
    {
      if(!strcmp(drawingManager->getDrawingType(i->first), "drawingOnImage") && imageViews)
        for(RobotConsole::Views::const_iterator j = imageViews->begin(); j != imageViews->end(); ++j)
        {
          completion.insert(std::string("vid ") + j->first + " " + translate(i->first) + " on");
          completion.insert(std::string("vid ") + j->first + " " + translate(i->first) + " off");
        }
      else if(!strcmp(drawingManager->getDrawingType(i->first), "drawingOnField") && fieldViews)
        for(RobotConsole::Views::const_iterator j = fieldViews->begin(); j != fieldViews->end(); ++j)
        {
          completion.insert(std::string("vfd ") + j->first + " " + translate(i->first) + " on");
          completion.insert(std::string("vfd ") + j->first + " " + translate(i->first) + " off");
        }
    }
  }

  if(plotViews)
    for(RobotConsole::PlotViews::const_iterator i = plotViews->begin(); i != plotViews->end(); ++i)
      for(int j = 0; j < debugRequestTable->currentNumberOfDebugRequests; ++j)
        if(translate(debugRequestTable->debugRequests[j].description).substr(0, 5) == "plot:")
        {
          for(int color = 0; color < RobotConsole::numOfColors; ++color)
            completion.insert(std::string("vpd ") + i->first + " " +
                              translate(debugRequestTable->debugRequests[j].description).substr(5) + " " +
                              RobotConsole::getName((RobotConsole::Color) color));
          completion.insert(std::string("vpd ") + i->first + " " +
                            translate(debugRequestTable->debugRequests[j].description).substr(5) + " off");
        }

  for(std::unordered_map<std::string, std::string>::const_iterator i = representationToFile.begin();
      i != representationToFile.end(); ++i)
    completion.insert(std::string("save ") + i->first);

  if(imageViews)
    for(RobotConsole::Views::const_iterator v = imageViews->begin(); v != imageViews->end(); ++v)
    {
      completion.insert(std::string("ac lower ") + translate(v->first));
      completion.insert(std::string("ac upper ") + translate(v->first));
    }

  gameController.addCompletion(completion);
}

void ConsoleRoboCupCtrl::addCompletionFiles(const std::string& command, const std::string& pattern)
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
    filename.chop(filename.size() - filename.lastIndexOf("."));
    completion.insert(command + filename.toUtf8().constData());
  }
}

std::string ConsoleRoboCupCtrl::handleCompletionString(size_t pos, const std::string& s)
{
  const std::string separators = " :./";
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
  std::string s1(text), s2(required);
  for(std::string::iterator i = s1.begin(); i != s1.end(); ++i)
    *i = (char) toupper(*i);
  for(std::string::iterator i = s2.begin(); i != s2.end(); ++i)
    *i = (char) toupper(*i);
  if(s1.find(s2) != std::string::npos)
  {
    if(newLine)
    {
      printLn(text);
    }
    else
    {
      print(text + " ");
    }
  }
}

std::string ConsoleRoboCupCtrl::translate(const std::string& text) const
{
  std::string s(text);
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

bool ConsoleRoboCupCtrl::handleMessage(InMessage& message)
{
  switch(message.getMessageID())
  {
    case idRobot:
      message.bin >> robotNumber;
      return true;

    default:
      if(!ntp.handleMessage(message))
        for(std::list<TeamRobot*>::iterator i = teamRobots.begin(); i != teamRobots.end(); ++i)
          if((*i)->getNumber() == robotNumber)
          {
            (*i)->setGlobals();
            (*i)->handleMessage(message);
            message.resetReadPosition();
          }

      return true;
  }
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

      QDir qdir(qpattern.left(lastSlashIdx));
      qdir.setFilter(QDir::Files | QDir::Dirs | QDir::NoDot | QDir::NoDotDot);
      QStringList qsl;
      qsl.append(qpattern.right(qpattern.size() - lastSlashIdx - 1));
      qdir.setNameFilters(qsl);
      QDirIterator it(qdir, QDirIterator::Subdirectories);
      while(it.hasNext())
      {
        QString filename = it.next().remove(0, lastSlashIdx + 1);
        filename.chop(filename.size() - filename.lastIndexOf("."));
        list.append(filename);
      }

      // ${Select Logfile:,../Logs/*.log}
      input = QInputDialog::getItem(0, "Input", label, list, 0, true, &ok);
    }
    else
    {
      // ${Select Robot:,Leonard,Rajesh,Lenny}
      input = QInputDialog::getItem(0, "Input", label, list, 0, true, &ok);
    }
    qstrCommand.replace(re.cap(0), input);
  }
  if(ok)
    command = std::string(qstrCommand.toUtf8().constData());
  else
    command = "";
}
