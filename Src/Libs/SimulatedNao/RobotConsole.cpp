/**
 * @file SimulatedNao/RobotConsole.cpp
 *
 * Implementation of RobotConsole.
 *
 * @author Thomas Röfer
 */

#include "RobotConsole.h"

#include "ConsoleRoboCupCtrl.h"
#include "Debugging/DebugDataStreamer.h"
#include "Framework/LoggingTools.h"
#include "Framework/Settings.h"
#include "LogPlayback/ImageExport.h"
#include "Platform/File.h"
#include "Platform/Time.h"
#include "Representations/Infrastructure/JPEGImage.h"
#include "Representations/Perception/FieldPercepts/CirclePercept.h"
#include "Views/AnnotationView.h"
#include "Views/CABSLBehaviorView.h"
#include "Views/ColorSpaceView.h"
#include "Views/FieldView.h"
#include "Views/ImageView.h"
#include "Views/JointView.h"
#include "Views/LogPlayerControlView.h"
#include "Views/ModuleGraphView.h"
#include "Views/PlotView.h"
#include "Views/TimeView.h"

#include <algorithm>
#include <cctype>
#include <iostream>
#include <set>

#define PREREQUISITE(p) pollingFor = #p; if(!poll(p)) return false;

RobotConsole::RobotConsole(const Settings& settings, const std::string& robotName, ConsoleRoboCupCtrl* ctrl, SystemCall::Mode mode, DebugReceiver<MessageQueue>* receiver, DebugSender<MessageQueue>* sender) :
  ThreadFrame(settings, robotName, receiver, sender),
  mode(mode),
  ctrl(ctrl),
  logPlayer(*sender),
  typeInfo(false),
  logExtractor(logPlayer)
{
  FOREACH_ENUM(MessageID, i)
  {
    waitingFor[i] = 0;
    polled[i] = false;
  }
  for(int i = 0; i < Joystick::numOfAxes; ++i)
  {
    joystickAxisMaxSpeeds[i] = joystickAxisThresholds[i] = joystickAxisCenters[i] = 0.f;
    joystickAxisMappings[i] = 0;
  }
  logPlayer.reserve(0xfffffffff); // max. 64 GB
}

RobotConsole::~RobotConsole()
{
  SYNC;
  setGlobals();
  delete logMessages;
  for(auto& [_, data] : threadData)
    for(auto& [_, info] : data.debugDataInfos)
      delete info.second;
  destructed = true;
}

void RobotConsole::init()
{
  if(mode == SystemCall::remoteRobot)
    poll(idRobotName);
  poll(idTypeInfo);
  poll(idModuleTable);
}

void RobotConsole::update()
{
  setGlobals(); // this is called in GUI thread -> set globals for this thread
  handleJoystick();
  handleRemoteControlMessages();

  if(!perThreadViewsAdded && !moduleInfo.config().empty())
  {
    addPerThreadViews();
    perThreadViewsAdded = true;
  }

  while(!lines.empty())
  {
    std::list<std::string> temp = lines;
    lines.clear();
    if(handleConsoleLine(temp.front()))
    {
      temp.pop_front();
      lines.splice(lines.end(), temp);
    }
    else
    {
      lines = temp;
      break;
    }
  }

  if(!commands.empty())
  {
    std::list<std::string> commands;
    {
      SYNC;
      commands.swap(this->commands);
    }
    for(const std::string& command : commands)
      ctrl->executeConsoleCommand(command, this);
  }

  pollForDirectMode();

  if(newDebugDrawing3D)
  {
    SYNC;
    for(auto& [_, data] : threadData)
      for(auto& [_, drawing] : data.drawings3D)
      {
        if(!drawing.drawn)
        {
          SimRobotCore3::PhysicalObject* object = static_cast<SimRobotCore3::PhysicalObject*>(ConsoleRoboCupCtrl::application->resolveObject(drawing.parts));
          object->registerDrawing(drawing);
          drawing.drawn = true;
        }
      }
    newDebugDrawing3D = false;
  }

  if(updateCompletion)
  {
    SYNC;
    ctrl->updateCommandCompletion();
    if(updateDataViews)
      updateDataViewTree();
    updateCompletion = false;
  }
}

void RobotConsole::addPerRobotViews()
{
  SimRobot::Object* category = ctrl->addCategory(QString::fromStdString(robotName), nullptr, ":/Icons/b-human.png");

  ctrl->addCategory("annotations", category);

  ctrl->addView(new CABSLBehaviorView(QString::fromStdString(robotName) + ".behavior", *this, activationGraph, activationGraphReceived), category);

  ctrl->addCategory("colorSpace", category);
  ctrl->addCategory("data", category);
  ctrl->addCategory("field", category);
  ctrl->addCategory("image", category);

  ctrl->addView(new JointView(QString::fromStdString(robotName) + ".jointData", *this, jointSensorData, jointRequest), category);
  if(mode == SystemCall::logFileReplay)
    ctrl->addView(new LogPlayerControlView(QString::fromStdString(robotName) + ".logPlayer", logPlayer, *this), category);

  ctrl->addCategory("modules", category);
  ctrl->addCategory("plot", ConsoleRoboCupCtrl::application->resolveObject(QString::fromStdString(robotName)));
  ctrl->addCategory("timing", category);
}

void RobotConsole::addPerThreadViews()
{
  SimRobot::Object* annotationCategory = ConsoleRoboCupCtrl::application->resolveObject(QString::fromStdString(robotName) + ".annotations");
  SimRobot::Object* dataCategory = ConsoleRoboCupCtrl::application->resolveObject(QString::fromStdString(robotName) + ".data");
  SimRobot::Object* modulesCategory = ConsoleRoboCupCtrl::application->resolveObject(QString::fromStdString(robotName) + ".modules");
  SimRobot::Object* timingCategory = ConsoleRoboCupCtrl::application->resolveObject(QString::fromStdString(robotName) + ".timing");

  for(const auto& config : moduleInfo.config())
  {
    const std::string threadName = static_cast<char>(config.name[0] | 0x20) + config.name.substr(1);
    ctrl->addView(new AnnotationView(QString::fromStdString(robotName) + ".annotations." + threadName.c_str(), threadData[config.name].annotationInfo, *this, logPlayer, mode, ctrl->application), annotationCategory);
    ctrl->addCategory(threadName.c_str(), dataCategory);
    ctrl->addView(new ModuleGraphViewObject(QString::fromStdString(robotName) + ".modules." + threadName.c_str(),
                                            *this, config.name), modulesCategory);
    ctrl->addView(new TimeView(QString::fromStdString(robotName) + ".timing." + threadName.c_str(), *this, threadData[config.name].timeInfo), timingCategory);
  }
}

void RobotConsole::addColorSpaceViews(const QString& path, const std::string& name, const std::string& threadName)
{
  for(int cm = 0; cm < ColorSpaceView::numOfColorModels; ++cm)
  {
    QString modelCategoryName = path + "." + TypeRegistry::getEnumName(ColorSpaceView::ColorModel(cm));
    for(int channel = 0; channel < 4; ++channel)
      ctrl->addView(new ColorSpaceView(
                      modelCategoryName + "." + ColorSpaceView::getChannelNameForColorModel(ColorSpaceView::ColorModel(cm), channel),
                      *this, name, ColorSpaceView::ColorModel(cm), channel, background, threadName), modelCategoryName);
  }
}

void RobotConsole::updateDataViewTree()
{
  auto addView = [&](const std::string& threadName, ThreadData& threadData, const std::string& name)
  {
    QString path = QString(name.c_str()).replace(':', '.');

    threadData.dataViews[name] = new DataView(QString::fromStdString(robotName) + ".data."
                                              + QString(threadName.c_str()).toLower() + "." + path,
                                              name, threadName, *this, typeInfo);
    QString additionalPath = "";
    qsizetype p = path.lastIndexOf('.');
    if(p != -1)
      additionalPath = "." + path.left(p);
    ctrl->addView(threadData.dataViews[name], QString::fromStdString(robotName) + ".data."
                  + QString(threadName.c_str()).toLower() + additionalPath,
                  SimRobot::Flag::copy | SimRobot::Flag::exportAsImage | SimRobot::Flag::sorted);
  };

  auto removeView = [&](ThreadData& threadData, const std::string& name)
  {
    DataView* dataView = threadData.dataViews[name];
    ctrl->removeView(dataView);
    threadData.dataViews.erase(name);
    // TODO: remove empty categories
  };

  for(auto& [threadName, threadData] : threadData)
  {
    // Create which views currently exist and which should actually exist.
    std::set<std::string> oldViews;
    std::set<std::string> newViews;
    for(const auto& [name, _] : threadData.dataViews)
      oldViews.insert(name);
    for(const auto& [name, _] : threadData.debugRequestTable.slowIndex)
      if(name.starts_with("debug data:"))
        newViews.insert(ctrl->translate(name.substr(11)));

    auto i = oldViews.begin();
    auto j = newViews.begin();

    // Add and remove views until the end of one of the two sets is reached.
    while(i != oldViews.end() && j != newViews.end())
    {
      if(*i == *j)
      {
        ++i;
        ++j;
      }
      else if(*i > *j)
        addView(threadName, threadData, *j++);
      else
        removeView(threadData, *i++);
    }

    // Clean up the remaining views. Only one of these two loops can actually run.
    while(i != oldViews.end())
      removeView(threadData, *i++);
    while(j != newViews.end())
      addView(threadName, threadData, *j++);

    // Let all open views repoll their data.
    for(auto& [_, dataView] : threadData.dataViews)
      dataView->repoll();
  }
  updateDataViews = false;
}

void RobotConsole::handleConsole(std::string line)
{
  setGlobals(); // this is called in GUI thread -> set globals for this thread
  for(;;)
  {
    std::string::size_type pos = line.find('\n');
    lines.push_back(line.substr(0, pos));
    if(pos == std::string::npos)
      break;
    else
      line = line.substr(pos + 1);
  }
  while(!lines.empty())
  {
    std::list<std::string> temp = lines;
    lines.clear();
    if(handleConsoleLine(temp.front()))
    {
      temp.pop_front();
      lines.splice(lines.end(), temp);
    }
    else
    {
      lines = temp;
      break;
    }
  }

  pollForDirectMode();
}

void RobotConsole::handleKeyEvent(int key, bool pressed)
{
  if(joystickTrace && pressed)
  {
    char buf[33];
    sprintf(buf, "%u", key + 1);
    ctrl->printLn(std::string("shortcut: ") + buf);
  }
  std::string* joystickButtonCommand(pressed ? joystickButtonPressCommand : joystickButtonReleaseCommand);
  if(key >= 0 && key < Joystick::numOfButtons && !joystickButtonCommand[key].empty())
    ctrl->executeConsoleCommand(joystickButtonCommand[key], this);
}

bool RobotConsole::handleConsoleLine(const std::string& line)
{
  InConfigMemory stream(line.c_str(), line.size());
  std::string command;
  stream >> command;
  bool result = false;
  if(command.empty()) // comment
    result = true;
  else if(command == "endOfStartScript")
  {
    directMode = true;
    result = true;
  }
  else if(command == "bc")
    result = backgroundColor(stream);
  else if(command == "cls")
  {
    ctrl->printLn("_cls");
    result = true;
  }
  else if(command == "echo")
  {
    ctrl->echo(stream);
    result = true;
  }
  else if(command == "for")
  {
    PREREQUISITE(idModuleTable);
    if(!for_(stream, result))
      return false;
  }
  else if(command == "jc")
    result = joystickCommand(stream);
  else if(command == "js")
    result = joystickSpeeds(stream);
  else if(command == "jm")
    result = joystickMaps(stream);
  else if(command == "log")
  {
    PREREQUISITE(idModuleTable);
    PREREQUISITE(idTypeInfo);
    result = log(stream);
  }
  else if(command == "msg")
  {
    result = msg(stream);
  }
  else if(command == "mv")
  {
    result = moveRobot(stream);
  }
  else if(command == "mvb")
  {
    result = moveBall(stream);
  }
  else if(command == "poll")
  {
    PREREQUISITE(idModuleTable);
    result = repoll(stream);
  }
  else if(command == "pr")
  {
    result = penalizeRobot(stream);
  }
  else if(command == "rc")
  {
    result = remoteControl(stream);
  }
  else if(command == "sn")
  {
    result = sensorNoise(stream);
  }
  else if(command == "vfd")
  {
    PREREQUISITE(idModuleTable);
    PREREQUISITE(idDebugResponse);
    PREREQUISITE(idDrawingManager);
    result = viewDrawing(stream, fieldViews, "drawingOnField");
  }
  else if(command == "vpd")
  {
    PREREQUISITE(idModuleTable);
    PREREQUISITE(idDebugResponse);
    result = viewPlotDrawing(stream);
  }
  else if(command == "vid")
  {
    PREREQUISITE(idModuleTable);
    PREREQUISITE(idDebugResponse);
    PREREQUISITE(idDrawingManager);
    result = viewDrawing(stream, imageViews, "drawingOnImage");
  }
  else if(command == "vic")
    result = viewImageCommand(stream);
  else if(!handleForCommands(command, stream, "", result))
    return false;

  pollingFor = nullptr;
  if(!result)
  {
    if(directMode)
    {
      ctrl->printLn("Syntax Error");
    }
    else
    {
      ctrl->printLn("Syntax Error: " + line);
    }
  }
  return true;
}

bool RobotConsole::handleForCommands(const std::string& command, In& stream,
                                     const std::string& threadName, bool& result)
{
  if(command == "dr")
  {
    PREREQUISITE(idDebugResponse);
    PREREQUISITE(idDrawingManager3D);
    result = debugRequest(stream, threadName);
  }
  else if(command == "get")
  {
    PREREQUISITE(idModuleTable);
    PREREQUISITE(idDebugResponse);
    PREREQUISITE(idTypeInfo);
    result = get(stream, threadName, true);
  }
  else if(command == "_get") // get, part 2
  {
    PREREQUISITE(idDebugDataResponse);
    PREREQUISITE(idTypeInfo);
    result = get(stream, threadName, false);
  }
  else if(command == "mr")
  {
    PREREQUISITE(idModuleTable);
    result = moduleRequest(stream, threadName);
  }
  else if(command == "set")
  {
    PREREQUISITE(idModuleTable);
    PREREQUISITE(idDebugResponse);
    PREREQUISITE(idTypeInfo);
    result = set(stream, threadName);
  }
  else if(command == "_set") // set, part 2
  {
    PREREQUISITE(idDebugDataResponse);
    PREREQUISITE(idTypeInfo);
    result = set(stream, threadName);
  }
  else if(command == "si")
  {
    result = !ctrl->is2D && saveImage(stream, threadName);
  }
  else if(command == "vf")
  {
    PREREQUISITE(idModuleTable);
    PREREQUISITE(idDebugResponse);
    PREREQUISITE(idDrawingManager);
    result = viewField(stream, threadName);
  }
  else if(command == "vi")
  {
    PREREQUISITE(idModuleTable);
    PREREQUISITE(idDebugResponse);
    PREREQUISITE(idDrawingManager);
    result = viewImage(stream, threadName);
  }
  else if(command == "vp")
  {
    PREREQUISITE(idModuleTable);
    PREREQUISITE(idDebugResponse);
    result = viewPlot(stream, threadName);
  }

  return true;
}

void RobotConsole::initJoystick()
{
  if(!joystick)
  {
    joystick = std::make_unique<Joystick>();
    joystick->init();
  }
}

void RobotConsole::handleJoystick()
{
  if(!joystick || !joystick->update())
  {
    joystickState.valid = false;
    return; //return if no joystick was found
  }
  else
    joystickState.valid = true;

  // Update joystick state
  for(int axis = 0; axis < Joystick::numOfAxes; ++axis)
    joystickState.axes[axis] = joystick->getAxisState(axis);
  for(int button = Joystick::numOfButtons - 1; button >= 0; --button) // buttons 32..39 will be lost
    joystickState.buttons = joystickState.buttons << 1 | static_cast<unsigned>(joystick->isButtonPressed(button));

  // handle joystick events
  unsigned int buttonId;
  bool pressed;
  bool buttonCommandExecuted(false);
  while(joystick->getNextEvent(buttonId, pressed))
  {
    ASSERT(buttonId < Joystick::numOfButtons);
    buttonCommandExecuted |= handleJoystickCommand(pressed ? joystickButtonPressCommand[buttonId] : joystickButtonReleaseCommand[buttonId]);
    if(!pressed)
      for(auto& joystickMotionCommand : joystickMotionCommands)
        joystickMotionCommand.lastCommand = "";
  }

  // walk and move head only when there is no button command
  if(buttonCommandExecuted)
    return;

  unsigned int timeNow = Time::getCurrentSystemTime();
  if(lines.empty()
     && timeNow - joystickLastTime >= 100)  // don't generate too many commands
  {
    joystickLastTime = timeNow;
    float speeds[Joystick::numOfAxes];
    bool preparedSpeeds = false;
    for(auto& cmd : joystickMotionCommands)
    {
      if(!cmd.command.empty())
      {
        if(!preparedSpeeds)
        {
          for(int i = 0; i < Joystick::numOfAxes; ++i)
          {
            float d = joystick->getAxisState(i);
            float threshold = joystickAxisThresholds[i];
            if(d < -threshold)
              speeds[i] = (d + threshold) / (1 - threshold);
            else if(d > threshold)
              speeds[i] = (d - threshold) / (1 - threshold);
            else
              speeds[i] = 0;
            if(joystickAxisMappings[i])
            {
              bool pressed1 = joystick->isButtonPressed(joystickAxisMappings[i] & 0xffff);
              bool pressed2 = joystick->isButtonPressed(joystickAxisMappings[i] >> 16);
              if(pressed1 != pressed2)
                speeds[i] = pressed1 ? 1.f : -1.f;
            }
            speeds[i] *= joystickAxisMaxSpeeds[i];
            speeds[i] += joystickAxisCenters[i];
          }
          preparedSpeeds = true;
        }
        if(joystickCommandBuffer.size() < cmd.command.length() + 258)
          joystickCommandBuffer.resize(cmd.command.length() + 258);
        ASSERT(Joystick::numOfAxes == 8);
        sprintf(&joystickCommandBuffer[0], cmd.command.c_str(),
                speeds[cmd.indices[0]],
                speeds[cmd.indices[1]],
                speeds[cmd.indices[2]],
                speeds[cmd.indices[3]],
                speeds[cmd.indices[4]],
                speeds[cmd.indices[5]],
                speeds[cmd.indices[6]],
                speeds[cmd.indices[7]]);
        if(strcmp(cmd.lastCommand.c_str(), &joystickCommandBuffer[0]))
        {
          handleJoystickCommand(&joystickCommandBuffer[0]);
          cmd.lastCommand = &joystickCommandBuffer[0];
        }
      }
    }
  }
}

bool RobotConsole::handleJoystickCommand(const std::string& cmd)
{
  if(cmd.empty())
    return false;

  ctrl->executeConsoleCommand(cmd, this);
  if(joystickTrace)
    ctrl->printLn(cmd);

  return true;
}

void RobotConsole::handleRemoteControlMessages()
{
  if(!remoteControlChannel)
    return;

  char buffer[25001];

  // sending commands to the robot
  if(Time::getRealTimeSince(timeRemoteControlMessageSent) >= 100)
  {
    timeRemoteControlMessageSent = Time::getRealSystemTime();
    MessageQueue queue;
    queue.setBuffer(buffer, 0, sizeof(buffer) - 1);
    queue.bin(idJoystickState) << joystickState;
    remoteControlChannel->send(buffer, static_cast<int>(queue.end() - queue.begin()));
  }

  // receiving data from the robot
  for(;;)
  {
    const int size = remoteControlChannel->receive(buffer, sizeof(buffer));
    if(size <= 0) // no packet available -> stop
      break;
    else if(static_cast<unsigned>(size) < sizeof(buffer))
    {
      MessageQueue queue;
      queue.setBuffer(buffer, size);
      if(!queue.empty() && (*queue.begin()).id() == idFrameBegin)
      {
        SYNC;
        *debugSender << queue;
      }
    }
  }
}

void RobotConsole::requestDebugData(const std::string& threadName, const std::string& name, bool active)
{
  SYNC;
  debugSender->bin(idThread) << threadName;
  debugSender->bin(idDebugRequest) << DebugRequest("debug data:" + name, active);
}

void RobotConsole::sendDebugData(const std::string& threadName, const std::string& name, OutBinaryMemory* data)
{
  SYNC;
  for(const auto& [requestName, _] : debugRequestTable.slowIndex)
    if(std::string("debugData:") + name == ctrl->translate(requestName))
    {
      debugSender->bin(idThread) << threadName;
      auto stream = debugSender->bin(idDebugDataChangeRequest);
      stream << requestName.substr(11);
      if(data)
      {
        stream << char(1);
        stream.write(data->data(), data->size());
      }
      else
        stream << char(0);
      break;
    }
}

bool RobotConsole::handleMessage(MessageQueue::Message message)
{
  if(!handleMessages)
    return true;

  if(destructed) // if object is already destroyed, abort here
    return true; // avoid further processing of this message

  if(logPlayer.state == LogPlayer::recording && message.id() > idNumOfDataMessageIDs && message.id() < numOfDataMessageIDs)
    logPlayer << message;

  auto stream = message.bin();
  switch(message.id())
  {
    case idFrameBegin:
    {
      stream >> threadName;
      if(mode == SystemCall::logFileReplay)
        currentFrame = static_cast<unsigned>(threadData[threadName].currentFrame);
      return true;
    }
    case idFrameFinished:
    {
#ifndef NDEBUG
      std::string threadName;
      message.bin() >> threadName;
      ASSERT(threadName == this->threadName);
#endif

      ++currentFrame;
      ThreadData& data = threadData[threadName];

      data.images.clear();

      for(auto& [name, incomplete] : incompleteImages)
      {
        ImagePtr& imagePtr = data.images[name];
        imagePtr.image = incomplete.image;
        imagePtr.timestamp = incomplete.timestamp;
        imagePtr.threadName = threadName;
        incomplete.image = nullptr;
      }

      // Remove drawings generated by this thread from this thread
      for(auto i = data.imageDrawings.begin(); i != data.imageDrawings.end();)
        if(i->second.threadName.empty())
          i = data.imageDrawings.erase(i);
        else
          ++i;

      for(auto i = data.fieldDrawings.begin(); i != data.fieldDrawings.end();)
        if(i->second.threadName.empty())
          i = data.fieldDrawings.erase(i);
        else
          ++i;

      // Add new Field and Image drawings
      for(const auto& [name, drawing] : incompleteImageDrawings)
        threadData[drawing.threadName.empty() ? threadName : drawing.threadName].imageDrawings[name] = drawing;

      for(const auto& [name, drawing] : incompleteFieldDrawings)
        threadData[drawing.threadName.empty() ? threadName : drawing.threadName].fieldDrawings[name] = drawing;

      // 3D Drawings
      if(!ctrl->is2D && polled[idDrawingManager3D] && !waitingFor[idDrawingManager3D])
      {
        // reset all 3d drawings originated from current thread
        for(auto& [_, drawing] : data.drawings3D)
          drawing.reset();

        // copy and register newly received 3d debug drawings
        for(auto& [name, drawing] : incompleteDrawings3D)
        {
          std::string type = data.drawingManager3D.getDrawingType(data.drawingManager3D.getString(name));
          DebugDrawing3DAdapter& debugDrawing3D = data.drawings3D[type == "camera" ? name + threadName : name];
          debugDrawing3D.copyFrom(drawing);
          if(debugDrawing3D.parts.empty())
          {
            if(type != "unknown")
            {
              // Set robotConsole, parts and flip
              debugDrawing3D.robotConsole = this;
              debugDrawing3D.parts.append(QString::fromStdString(robotName));
              if(type == "field")
              {
                QString robotNumberString(QString::fromStdString(robotName));
                robotNumberString.remove(0, 5);
                debugDrawing3D.flip = robotNumberString.toInt() <= SimulatedRobot::robotsPerTeam;
                debugDrawing3D.parts[0] = "RoboCup";
              }
              else if(type == "robot")
                debugDrawing3D.parts.append("origin");
              else if(type == "camera")
                debugDrawing3D.parts.append(threadName == "Upper" ? "CameraTop" : "CameraBottom");
              else
                debugDrawing3D.parts.append(type.c_str());
              newDebugDrawing3D = true;
            }
          }
        }
      }

      incompleteImages.clear();
      incompleteImageDrawings.clear();
      incompleteFieldDrawings.clear();
      incompleteDrawings3D.clear();

      return true;
    }
    case idActivationGraph:
      stream >> activationGraph;
      activationGraphReceived = Time::getCurrentSystemTime();
      return true;
    case idAnnotation:
      threadData[threadName].annotationInfo.addMessage(message, currentFrame);
      return true;
    case idCameraImage:
    {
      CameraImage ci;
      stream >> ci;
      if(incompleteImages["CameraImage"].image)
        incompleteImages["CameraImage"].image->from(ci);
      else
        incompleteImages["CameraImage"].image = new DebugImage(ci, true);
      incompleteImages["CameraImage"].timestamp = ci.timestamp;
      return true;
    }
    case idJointCalibration:
    {
      stream >> jointCalibration;
      jointCalibrationChanged = true;
      return true;
    }
    case idJointRequest:
    {
      stream >> jointRequest;
      return true;
    }
    case idJointSensorData:
    {
      stream >> jointSensorData;
      return true;
    }
    case idJPEGImage:
    {
      CameraImage ci;
      JPEGImage jpi;
      stream >> jpi;
      jpi.toCameraImage(ci);
      if(incompleteImages["CameraImage"].image)
        incompleteImages["CameraImage"].image->from(ci);
      else
        incompleteImages["CameraImage"].image = new DebugImage(ci, true);
      incompleteImages["CameraImage"].timestamp = ci.timestamp;
      return true;
    }
    case idMotionRequest:
      stream >> motionRequest;
      return true;
    case idRawInertialSensorData:
    {
      stream >> rawInertialSensorData;
      return true;
    }
    case idStopwatch:
      threadData[threadName].timeInfo.handleMessage(message);
      return true;
    case idConsole:
      commands.push_back(message.text().readAll());
      return true;
    case idDebugDataResponse:
    {
      std::string name, type;
      stream >> name >> type;
      DebugDataInfos& debugDataInfos = threadData[threadName].debugDataInfos;
      if(debugDataInfos.find(name) == debugDataInfos.end())
        debugDataInfos[name] = DebugDataInfoPair(type, new MessageQueue);
      debugDataInfos[name].second->clear();
      *debugDataInfos[name].second << message;

      auto view = threadData[threadName].dataViews.find(name);
      if(view != threadData[threadName].dataViews.end())
        view->second->setMessageReceived(debugDataInfos[name].second);

      // console command requested this one?
      if(threadData[threadName].getOrSetWaitsFor == name)
      {
        --waitingFor[idDebugDataResponse];
        threadData[threadName].getOrSetWaitsFor = "";
        if(view == threadData[threadName].dataViews.end() || !view->second->isPolling())
        {
          debugSender->bin(idThread) << threadName;
          debugSender->bin(idDebugRequest) << DebugRequest("debug data:" + name, false);
        }
      }
      return true;
    }
    case idDebugDrawing:
    {
      if(polled[idDrawingManager] && !waitingFor[idDrawingManager]) // drawing manager not up-to-date
      {
        ThreadData& data = threadData[threadName];
        char shapeType, id;
        stream >> shapeType >> id;
        const char* name = data.drawingManager.getDrawingName(id); // const char* is required here
        std::string type = data.drawingManager.getDrawingType(name);

        if(type == "drawingOnImage")
          incompleteImageDrawings[name].addShapeFromQueue(stream, static_cast<::Drawings::ShapeType>(shapeType));
        else if(type == "drawingOnField")
          incompleteFieldDrawings[name].addShapeFromQueue(stream, static_cast<::Drawings::ShapeType>(shapeType));
      }
      return true;
    }
    case idDebugDrawing3D:
    {
      if(polled[idDrawingManager3D] && !waitingFor[idDrawingManager3D])
      {
        auto stream = message.bin();
        char shapeType, id;
        stream >> shapeType >> id;
        incompleteDrawings3D[threadData[threadName].drawingManager3D.getDrawingName(id)]
          .addShapeFromQueue(stream, static_cast<::Drawings3D::ShapeType>(shapeType));
      }
      return true;
    }
    case idDebugImage:
    {
      std::string id;
      stream >> id;
      if(!incompleteImages[id].image)
        incompleteImages[id].image = new DebugImage();
      stream >> *incompleteImages[id].image;
      incompleteImages[id].timestamp = Time::getCurrentSystemTime();
      break;
    }
    case idDebugResponse:
    {
      std::string description;
      bool enable;
      message.text() >> description >> enable;
      SYNC_WITH(*ctrl);
      if(description != "pollingFinished")
      {
        debugRequestTable.addRequest(DebugRequest(description, enable));
        threadData[threadName].debugRequestTable.addRequest(DebugRequest(description, enable));
      }
      else if(--waitingFor[idDebugResponse] <= 0)
      {
        ctrl->setDebugRequestTable(debugRequestTable);
        updateCompletion = true;
        updateDataViews = true;
      }
      return true;
    }
    case idDrawingManager:
    {
      SYNC_WITH(*ctrl);
      stream >> threadData[threadName].drawingManager;
      if(--waitingFor[idDrawingManager] <= 0)
      {
        ctrl->setThreadData(threadData);
        updateCompletion = true;
      }
      return true;
    }
    case idDrawingManager3D:
    {
      SYNC_WITH(*ctrl);
      stream >> threadData[threadName].drawingManager3D;
      --waitingFor[idDrawingManager3D];
      return true;
    }
    case idLogResponse:
      threadData[threadName].logAcknowledged = true;
      return true;
    case idModuleTable:
    {
      SYNC_WITH(*ctrl);
      moduleInfo.handleMessage(message);
      if(--waitingFor[idModuleTable] <= 0)
      {
        ctrl->setModuleInfo(moduleInfo);
        updateCompletion = true;
      }
      return true;
    }
    case idPlot:
    {
      std::string id;
      float value;
      stream >> id >> value;
      Plot& plot = threadData[threadName].plots[ctrl->translate(id)];
      plot.points.push_back(value);
      while(plot.points.size() > maxPlotSize)
        plot.points.pop_front();
      plot.timestamp = Time::getCurrentSystemTime();
      return true;
    }
    case idRobotName:
    {
      stream >> Global::getSettings().headName
             >> Global::getSettings().bodyName
             >> Global::getSettings().location
             >> Global::getSettings().scenario
             >> Global::getSettings().playerNumber;
      Global::getSettings().updateSearchPath();
      File::setSearchPath(Global::getSettings().searchPath);
      --waitingFor[idRobotName];
      return true;
    }
    case idSkillRequest:
    {
      stream >> skillRequest;
      ctrl->gameController.setTestSkill(skillRequest.skill);
      return true;
    }
    case idText:
    {
      std::string buffer = message.text().readAll();
      if(printMessages)
        ctrl->printLn(buffer);
      if(logMessages)
        *logMessages << buffer << endl;
      return true;
    }
    case idTypeInfo:
    {
      stream >> typeInfo;
      --waitingFor[idTypeInfo];
      return true;
    }
    case idTypeInfoRequest:
    {
      logPlayer.requestTypeInfo();
      return true;
    }
  }

  return false;
}

void RobotConsole::handleAllMessages(MessageQueue& messageQueue)
{
  SYNC;  // Only one thread can access *this now.
  ThreadFrame::handleAllMessages(messageQueue);
}

void RobotConsole::updateAnnotationsFromLog()
{
  for(const auto& [threadName, annotations] : logPlayer.annotations())
  {
    threadData[threadName].annotationInfo.newAnnotations = annotations;
    threadData[threadName].annotationInfo.timeOfLastMessage = Time::getCurrentSystemTime();
  }
}

bool RobotConsole::poll(MessageID id)
{
  if(moduleRequestChanged && (id == idDebugResponse || id == idDrawingManager || id == idDrawingManager3D))
  {
    SYNC;
    moduleInfo.timestamp = Time::getCurrentSystemTime() + ++mrCounter;
    auto stream = debugSender->bin(idModuleRequest);
    stream << moduleInfo.timestamp;
    moduleInfo.sendRequest(stream);
    polled[idDebugResponse] = polled[idDrawingManager] = polled[idDrawingManager3D] = false;
    moduleRequestChanged = false;
  }

  if(waitingFor[id] > 0)
  {
    // When in replay log file mode, force replay while polling to keep all threads running
    triggerThreads();
    return false;
  }
  else if(polled[id])
    return true;
  else
  {
    polled[id] = true;
    switch(id)
    {
      case idDebugResponse:
      {
        SYNC;
        debugRequestTable.clear();
        for(auto& [_, data] : threadData)
          data.debugRequestTable.clear();
        debugSender->bin(idDebugRequest) << DebugRequest("poll");
        waitingFor[id] = static_cast<int>(moduleInfo.config().size()) + 1; // Threads + Debug will answer
        updateDataViews = false;
        break;
      }
      case idModuleTable:
      {
        SYNC;
        moduleInfo.clear();
        debugSender->bin(idDebugRequest) << DebugRequest("automated requests:ModuleTable", true);
        waitingFor[id] = 1;  // Debug will answer
        break;
      }
      case idTypeInfo:
      {
        SYNC;
        debugSender->bin(idDebugRequest) << DebugRequest("automated requests:TypeInfo", true);
        waitingFor[id] = 1;  // Debug will answer
        break;
      }
      case idDrawingManager:
      {
        SYNC;
        for(auto& [_, data] : threadData)
          data.drawingManager.clear();
        debugSender->bin(idDebugRequest) << DebugRequest("automated requests:DrawingManager", true);
        waitingFor[id] = static_cast<int>(moduleInfo.config().size()); // Threads will answer
        break;
      }
      case idDrawingManager3D:
      {
        SYNC;
        for(auto& [_, data] : threadData)
          data.drawingManager3D.clear();
        debugSender->bin(idDebugRequest) << DebugRequest("automated requests:DrawingManager3D", true);
        waitingFor[id] = static_cast<int>(moduleInfo.config().size());  // Threads will answer
        break;
      }
      case idRobotName:
      {
        SYNC;
        debugSender->bin(idDebugRequest) << DebugRequest("module:NaoProvider:robotName", true);
        waitingFor[id] = 1;  // Motion will answer
        break;
      }
      default:
        ASSERT(false);
    }
    return false;
  }
}

void RobotConsole::pollForDirectMode()
{
  if(directMode)
  {
    poll(idDebugResponse);
    poll(idDrawingManager);
    poll(idDrawingManager3D);
    poll(idModuleTable);
  }
}

void RobotConsole::triggerThreads()
{
  if(mode == SystemCall::logFileReplay)
  {
    SYNC;
    for(const Configuration::Thread& thread : moduleInfo.config())
    {
      debugSender->bin(idFrameBegin) << thread.name;
      debugSender->bin(idFrameFinished) << thread.name;
    }
  }
}

void RobotConsole::printType(const std::string& type, const std::string& field)
{
  if(type[type.size() - 1] == ']')
  {
    size_t endOfType = type.rfind('[');
    printType(type.substr(0, endOfType), field + type.substr(endOfType));
  }
  else if(type[type.size() - 1] == '*')
    printType(type.substr(0, type.size() - 1), field + "[]");
  else
  {
    if(typeInfo.primitives.find(type) != typeInfo.primitives.end() || typeInfo.enums.find(type) != typeInfo.enums.end())
      ctrl->print(type);
    else if(typeInfo.classes.find(type) != typeInfo.classes.end())
    {
      ctrl->print("{");
      const char* space = "";
      for(const TypeInfo::Attribute& attribute : typeInfo.classes[type])
      {
        ctrl->print(space);
        space = " ";
        printType(attribute.type, attribute.name);
        ctrl->print(";");
      }
      ctrl->print("}");
    }
    else
      ctrl->print("UNKNOWN");

    if(!field.empty())
      ctrl->print(" " + field);
  }
}

void RobotConsole::notifyDataViewsAboutSetStatus(const std::string& data, bool set, const std::string& threadName)
{
  for(const auto& [name, threadData] : this->threadData)
    if(threadName.empty() || threadName == name)
    {
      auto dataView = threadData.dataViews.find(data);
      if(dataView != threadData.dataViews.end())
        dataView->second->notifyAboutSetStatus(set);
    }
}

std::string RobotConsole::getPathForRepresentation(const std::string& representation) const
{
  size_t p = representation.rfind(':');
  std::string filename = p == std::string::npos ? representation : representation.substr(p + 1);

  if(!filename.empty())
  {
    filename[0] = static_cast<char>(tolower(filename[0]));
    if(filename.size() > 1 && isupper(filename[1]))
      for(size_t i = 1; i + 1 < filename.size() && isupper(filename[i + 1]); ++i)
        filename[i] = static_cast<char>(tolower(filename[i]));
    filename += ".cfg";

    // Use file in search path if it exists
    for(const std::string& name : File::getFullNames(filename))
    {
      File path(name, "r", false);
      if(path.exists())
        return name;
    }
  }

  return filename;
}

bool RobotConsole::backgroundColor(In& stream)
{
  stream >> background.x() >> background.y() >> background.z();
  background *= 0.01f;
  return true;
}

bool RobotConsole::debugRequest(In& stream, const std::string& threadName)
{
  DebugRequestTable& debugRequestTable = threadName.empty()
    ? this->debugRequestTable
    : threadData[threadName].debugRequestTable;

  std::string debugRequestString, state;
  stream >> debugRequestString >> state;

  if(debugRequestString == "?")
  {
    for(const auto& [name, _] : debugRequestTable.slowIndex)
      ctrl->list(ctrl->translate(name), state);
    ctrl->printLn("");
    return true;
  }
  else
  {
    if(debugRequestString == "off")
    {
      SYNC;
      if(!threadName.empty())
        debugSender->bin(idThread) << threadName;
      debugSender->bin(idDebugRequest) << DebugRequest("disableAll");
      return true;
    }
    else
      for(const auto& [name, _] : debugRequestTable.slowIndex)
        if(ctrl->translate(name) == debugRequestString)
        {
          bool enabled = false;
          if(state == "on" || state.empty())
            enabled = true;
          else if(state != "off")
            return false;

          SYNC;
          if(!threadName.empty())
            debugSender->bin(idThread) << threadName;
          debugSender->bin(idDebugRequest) << DebugRequest(name, enabled);
          return true;
        }
  }
  return false;
}

bool RobotConsole::for_(In& stream, bool& result)
{
  std::vector<std::string> threadNames;
  std::string param;
  stream >> param;

  while(std::find_if(moduleInfo.config().begin(), moduleInfo.config().end(),
                     [&param](const Configuration::Thread& thread)
                     { return thread.name == param; }) != moduleInfo.config().end())
  {
    if(std::find(threadNames.begin(), threadNames.end(), param) == threadNames.end())
      threadNames.push_back(param);
    stream >> param;
  }

  if(!threadNames.empty())
  {
    std::string command = param;

    // Collect all parameters for multiple use
    OutTextMemory params(10000);
    while(!stream.eof())
    {
      stream >> param;
      params << param;
    }

    // Run command for multiple threads
    for(const std::string& threadName : threadNames)
    {
      bool result2 = false;
      InConfigMemory stream2(params.data(), params.size());
      if(!handleForCommands(command, stream2, threadName, result2))
        return false; // prerequisite missing
      else if(!result2)
        return true; // failed: result is still false
    }

    result = true; // ok
  }

  return true;
}

bool RobotConsole::get(In& stream, const std::string& threadName, bool first)
{
  std::string request, option, filename;
  stream >> request >> option >> filename;
  if(request == "?")
  {
    for(const auto& [name, _] : (threadName.empty() ? this->debugRequestTable : threadData[threadName].debugRequestTable).slowIndex)
      if(name.substr(0, 11) == "debug data:")
        ctrl->list(ctrl->translate(name.substr(11)), option);
    ctrl->printLn("");
    return true;
  }
  else
  {
    std::string threadForRequest = threadName;
    if(threadForRequest.empty())
    {
      const auto threadsFound = ctrl->getThreadsFor(threadData, std::string("debugData:") + request);
      if(threadsFound.empty())
        return false;
      else if(threadsFound.size() > 1 && option != "?")
      {
        std::string threadNames = "";
        for(const std::string& threadName : threadsFound)
          threadNames += " " + threadName;
        if(option == "save")
          ctrl->printLn("Multiple threads provide this data. Use \"for\" to pick one of:" + threadNames);
        else
          handleConsole("for" + threadNames + " get " + request + " " + option + " " + filename);
        return true;
      }
      threadForRequest = threadsFound.front();
    }
    for(const auto& [name, _] : threadData[threadForRequest].debugRequestTable.slowIndex)
      if(std::string("debugData:") + request == ctrl->translate(name))
      {
        if(first)
        {
          // request up-to-date data
          SYNC;
          debugSender->bin(idThread) << threadForRequest;
          debugSender->bin(idDebugRequest) << DebugRequest(name, true);
          threadData[threadForRequest].getOrSetWaitsFor = name.substr(11);
          ++waitingFor[idDebugDataResponse];
          polled[idDebugDataResponse] = true; // no automatic repolling
          handleConsole((threadName.empty() ? "" : "for " + threadName + " ")
                        + "_get " + request + " " + option + " " + filename);
          return true;
        }
        else
        {
          const auto& [_, info] = *threadData[threadForRequest].debugDataInfos.find(name.substr(11));
          if(option == "?")
          {
            printType(info.first);
            ctrl->printLn("");
            return true;
          }
          else
          {
            Out* outMap;
            if(option == "save")
            {
              if(filename.empty()) // no path specified, use default location
              {
                filename = getPathForRepresentation(name.substr(11));
                if(filename.empty())
                {
                  ctrl->printLn("Error getting filename for " + name.substr(11) + ". Representation can not be saved.");
                  return true;
                }
              }
              else if(!File::hasExtension(filename))
                filename += ".cfg";

              outMap = new OutMapFile(filename, true);
            }
            else if(!option.empty())
              return false;
            else
              outMap = new OutMapMemory(true, 16384);

            {
              SYNC;
              for(MessageQueue::Message message : *info.second)
              {
                ASSERT(message.id() == idDebugDataResponse);
                std::string name, type;
                auto stream = message.bin();
                stream >> name >> type;
                DebugDataStreamer streamer(typeInfo, stream, type);
                *outMap << streamer;
              }
            }

            if(option.empty())
              ctrl->printLn((threadName.empty() ? "" : "for " + threadName + " ")
                            + "set " + request + " " + static_cast<OutMapMemory*>(outMap)->data());

            delete outMap;
            return true;
          }
        }
        break;
      }
  }
  return false;
}

bool RobotConsole::joystickCommand(In& stream)
{
  initJoystick();
  std::string command;
  stream >> command;
  if(command == "show")
  {
    joystickTrace = true;
    return true;
  }
  else if(command == "hide")
  {
    joystickTrace = false;
    return true;
  }

  int number;
  stream >> number;
  //rest of line into one string (std::stringstream-like .str() would be nice :-/):
  std::string line;
  stream >> line;
  while(!stream.eof())
  {
    std::string text;
    stream >> text;
    line += ' ';
    line += text;
  }

  if(command == "press" || command == "release")
  {
    if(number > 0 && number <= Joystick::numOfButtons)
    {
      if(command == "release")
        joystickButtonReleaseCommand[number - 1].swap(line);
      else
        joystickButtonPressCommand[number - 1].swap(line);
      return true;
    }
    return false;
  }
  else if(command == "motion")
  {
    if(number > 0 && number <= joystickNumOfMotionCommands)
    {
      JoystickMotionCommand& cmd(joystickMotionCommands[number - 1]);
      for(int& index : cmd.indices)
        index = 0;
      std::string::size_type pos = line.find('$');
      int i = 0;
      while(i < Joystick::numOfAxes && pos != std::string::npos)
      {
        int id = line[pos + 1] - '1';
        if(id >= 0 && id < Joystick::numOfAxes)
        {
          cmd.indices[i++] = id;
          line.replace(pos, 2, "%lf");
          pos = line.find('$');
        }
        else
          return false;
      }
      cmd.command.swap(line);
      cmd.lastCommand.clear();
      return true;
    }
    return false;
  }
  return false;
}

bool RobotConsole::joystickMaps(In& stream)
{
  initJoystick();
  int axis, button1, button2;
  stream >> axis >> button1 >> button2;
  if(axis > 0 && axis <= Joystick::numOfAxes && button1 >= 0 && button1 <= Joystick::numOfButtons && button2 > 0 && button2 <= Joystick::numOfButtons)
  {
    joystickAxisMappings[axis - 1] = button1 == 0 ? 0 : ((button1 - 1) | ((button2 - 1) << 16));
    return true;
  }
  return false;
}

bool RobotConsole::joystickSpeeds(In& stream)
{
  initJoystick();
  int id;
  stream >> id;
  if(id > 0 && id <= Joystick::numOfAxes)
  {
    stream >> joystickAxisMaxSpeeds[id - 1] >> joystickAxisThresholds[id - 1] >> joystickAxisCenters[id - 1];
    return true;
  }
  return false;
}

bool RobotConsole::log(In& stream)
{
  try
  {
    std::string command, option;
    stream >> command >> option;
    if(command == "start")
    {
      SYNC;
      logPlayer.state = mode == SystemCall::logFileReplay ? LogPlayer::playing : LogPlayer::recording;
    }
    else if(command == "stop")
    {
      SYNC;
      logPlayer.state = LogPlayer::stopped;
      if(mode == SystemCall::logFileReplay)
        logPlayer.playBack(-1);
    }
    else if(command == "clear" && mode != SystemCall::logFileReplay)
    {
      SYNC;
      logPlayer.clear();
    }
    else if(command == "save")
    {
      if(!option.empty())
      {
        if(!File::hasExtension(option))
          option += ".log";
        if(!File::isAbsolute(option))
          option = std::string("Logs/") + option;
      }
      SYNC;
      return logPlayer.save(option, typeInfo);
    }
    else if(command == "saveImages")
    {
      SYNC;
      bool raw = false;
      bool onlyPlaying = false;
      int takeEachNthFrame = 1;
      if(option == "raw")
      {
        raw = true;
        stream >> option;
      }
      if(option == "onlyPlaying")
      {
        onlyPlaying = true;
        stream >> option;
      }
      if(!option.empty())
      {
        takeEachNthFrame = std::stoi(option);
        stream >> option;
      }
      if(option.empty())
      {
        std::string::size_type pos = logFile.rfind('.');
        if(pos == std::string::npos)
          return false;
        else
          option = logFile.substr(0, pos) + "_Images";
      }
      if(option.back() != '/')
        option += '/';
      if(!File::isAbsolute(option.c_str()))
        option = "Images/" + option;
      return logExtractor.saveImages(option, raw, onlyPlaying, takeEachNthFrame);
    }
    else if(command == "saveAudio")
    {
      SYNC;
      if(option.empty())
      {
        std::string::size_type pos = logFile.rfind('.');
        if(pos == std::string::npos)
          return false;
        else
          option = logFile.substr(0, pos) + "_" + command.substr(4);
      }

      if(!File::hasExtension(option))
        option += ".wav";

      return logExtractor.saveAudioFile(File::isAbsolute(option) ? option : "Sounds/" + option);
    }
    else if(command == "keep" || command == "remove")
    {
      if(command == "keep" && option == "circlePercept")
      {
        logPlayer.filterFrames([&](MessageQueue::const_iterator i) -> bool
        {
          if(logPlayer.id(*i) == idCirclePercept)
          {
            CirclePercept circlePercept;
            (*i).bin() >> circlePercept;
            return circlePercept.wasSeen;
          }
          else
            return false;
        });
        updateAnnotationsFromLog();
        return true;
      }

      std::vector<MessageID> messageIDs;
      while(!option.empty())
      {
        FOREACH_ENUM(MessageID, i)
          if(option == TypeRegistry::getEnumName(i))
          {
            messageIDs.push_back(i);
            goto found;
          }

        return false;
      found:
        stream >> option;
      }
      if(!messageIDs.empty())
      {
        SYNC;
        if(command == "keep")
        {
          messageIDs.push_back(idFrameBegin);
          messageIDs.push_back(idFrameFinished);
          messageIDs.push_back(undefined);
          logPlayer.filter([&](MessageQueue::const_iterator i) -> bool
          {
            const MessageID id = logPlayer.id(*i);
            const MessageID* m = &messageIDs[0];
            while(*m)
              if(id == *m++)
                return true;
            return false;
          });
        }
        else
        {
          messageIDs.push_back(undefined);
          logPlayer.filter([&](MessageQueue::const_iterator i) -> bool
          {
            const MessageID id = logPlayer.id(*i);
            const MessageID* m = &messageIDs[0];
            while(*m)
              if(id == *m++)
                return false;
            return true;
          });
        }

        updateAnnotationsFromLog();
        return true;
      }

      return false;
    }
    else if(command == "trim")
    {
      if(logPlayer.empty())
        return false;
      std::string frame;
      stream >> frame;
      if(frame == "current")
        frame = std::to_string(logPlayer.frame());
      size_t startFrame = 0;
      size_t endFrame = logPlayer.frames() - 1;

      if(option == "from")
        startFrame = std::stoul(frame);
      else if(option == "until")
        endFrame = std::stoul(frame);
      else if(option == "between")
      {
        startFrame = std::stoul(frame);
        stream >> frame;
        endFrame = frame == "current" ? logPlayer.frame() : std::stoi(frame);
      }
      else
        return false;

      if(endFrame >= logPlayer.frames() || startFrame > endFrame)
        return false;

      size_t frameCounter = -1;

      SYNC;
      logPlayer.filter([&](MessageQueue::const_iterator i) -> bool
      {
        if((*i).id() == idFrameBegin)
          ++frameCounter;
        return frameCounter >= startFrame && frameCounter <= endFrame;
      });
      updateAnnotationsFromLog();
    }
    else if(command == "?")
    {
      SYNC;
      std::string option;
      stream >> option;

      size_t messages = 0;
      float size = 0;
      FOREACH_ENUM(MessageID, id, numOfDataMessageIDs)
      {
        messages += logPlayer.frequencyOf(id);
        size += static_cast<float>(logPlayer.sizeOf(id));
      }

      char buf[100];
      FOREACH_ENUM(MessageID, id, numOfDataMessageIDs)
        if(logPlayer.frequencyOf(id))
        {
          sprintf(buf, "%llu\t%.2f%%", static_cast<unsigned long long>(logPlayer.frequencyOf(id)), static_cast<float>(logPlayer.sizeOf(id)) * 100.f / size);
          ctrl->list(std::string(buf) + "\t" + TypeRegistry::getEnumName(id), option, true);
        }
      sprintf(buf, "%llu", static_cast<unsigned long long>(messages));
      ctrl->printLn(std::string(buf) + "\ttotal");
    }
    else if(command == "mr") //log mr
    {
      SYNC;
      std::string param;
      stream >> param;

      // We need special handling for perception providers if more than one perception thread exists
      int numOfPerceptionThreads = 0;
      for(const auto& config : moduleInfo.config())
        if(config.executionUnit == "Perception")
          ++numOfPerceptionThreads;

      std::list<std::string> commands;
      const auto log = std::find(moduleInfo.modules.begin(), moduleInfo.modules.end(), "LogDataProvider");
      if(log != moduleInfo.modules.end())
        for(int i = idFrameFinished + 1; i < numOfDataMessageIDs; ++i)
          if(logPlayer.frequencyOf(static_cast<MessageID>(i)) > 0)
          { // This representation is provided in at least one thread
            std::string representation = std::string(TypeRegistry::getEnumName(MessageID(i))).substr(2);
            if(representation == "JPEGImage")
              representation = "CameraImage";
            if(std::find(log->representations.begin(), log->representations.end(), representation) != log->representations.end())
            {
              // First activate log data providers in the desired threads
              for(const auto& config : moduleInfo.config())
                if(logPlayer.frequencyOf(static_cast<MessageID>(i), config.name) > 0)
                  commands.push_back("for " + config.name + " mr " + representation + " LogDataProvider");

              // Then, switch providers for the representation off in all other threads
              for(const auto& config : moduleInfo.config())
                if(logPlayer.frequencyOf(static_cast<MessageID>(i), config.name) == 0)
                {
                  if(config.name == "Cognition")
                  {
                    auto j = std::find(moduleInfo.modules.begin(), moduleInfo.modules.end(), "Perception" + representation + "Provider");
                    if(j != moduleInfo.modules.end())
                    {
                      if(numOfPerceptionThreads > 1)
                        commands.push_back("for Cognition mr " + representation + " Perception" + representation + "Provider");
                      continue;
                    }
                  }
                  // Skipping this for OdometryData is a workaround for log files in which OdometryData was not logged in all threads.
                  // (Turning OdometryData off in Upper/Lower/Cognition would result in errors, e.g. because the UpperProvider/LowerProvider needs OdometryData from Upper/Lower).
                  // Sorry.
                  if(representation != "OdometryData")
                    commands.push_back("for " + config.name + " mr " + representation + " off");
                }
            }
          }

      bool success = true;
      for(const std::string& command : commands)
        if(param == "list")
          ctrl->printLn(command);
        else
        {
          InTextMemory strMem(command.c_str(), command.size());
          std::string for_, threadName, mr;
          strMem >> for_ >> threadName >> mr;
          ASSERT(for_ == "for" && mr == "mr");
          success &= moduleRequest(strMem, threadName);
        }

      return success;
    }
    else if(mode == SystemCall::logFileReplay)
    {
      SYNC;
      if(command == "load")
      {
        if(option.empty())
          return false;
        else
        {
          if(!File::hasExtension(option))
            option += ".log";
          if(!File::isAbsolute(option))
            option = std::string("Logs/") + option;
          if(logPlayer.open(option))
          {
            logFile = option;
            updateAnnotationsFromLog();
          }
          else
            return false;
        }
      }
      else if(command == "cycle")
        logPlayer.cycle = true;
      else if(command == "once")
        logPlayer.cycle = false;
      else
      {
        auto state = logPlayer.state;
        logPlayer.state = LogPlayer::stopped;
        if(command == "forward" && option == "fast")
          logPlayer.playBack(logPlayer.frame() + 100);
        else if(command == "forward" && option == "image")
          logPlayer.playBack(logPlayer.nextImageFrame(logPlayer.frame()));
        else if(command == "forward" && option.empty())
          logPlayer.playBack(logPlayer.frame() + 1);
        else if(command == "backward" && option == "fast")
          logPlayer.playBack(logPlayer.frame() - 100);
        else if(command == "backward" && option == "image")
          logPlayer.playBack(logPlayer.prevImageFrame(logPlayer.frame()));
        else if(command == "backward" && option.empty())
          logPlayer.playBack(logPlayer.frame() - 1);
        else if(command == "repeat")
          logPlayer.playBack(logPlayer.frame());
        else if(command == "goto")
        {
          const size_t targetFrame = std::stoul(option);
          size_t frame = logPlayer.prevImageFrame(logPlayer.prevImageFrame(targetFrame));
          while(frame <= targetFrame)
            logPlayer.playBack(frame++);
        }
        else if(command != "pause")
        {
          logPlayer.state = state;
          return false;
        }
      }
    }
    return true;
  }
  catch(const std::exception&)
  {
    return false;
  }
}

bool RobotConsole::moduleRequest(In& stream, std::string threadName)
{
  SYNC;
  std::string representation, module, pattern;
  stream >> representation >> module >> pattern;
  if(representation == "modules")
  {
    for(const ModuleInfo::Module& m : moduleInfo.modules)
    {
      std::string text = m.name + ": ";
      for(const std::string& r : m.requirements)
        text += r + " ";
      text += "-> ";
      for(const std::string& r : m.representations)
      {
        bool selected = false;
        for(const Configuration::Thread& thread : moduleInfo.config())
          for(const auto& rp : thread.representationProviders)
            selected |= rp.provider == m.name;
        text += r + (selected ? "* " : " ");
      }
      ctrl->list(text, module, true);
    }

    return true;
  }
  else if(representation == "?")
  {
    for(const std::string& repr : moduleInfo.representations)
    {
      bool provided = false;
      for(const Configuration::Thread& thread : moduleInfo.config())
      {
        for(const auto& rp : thread.representationProviders)
          if(rp.representation == repr)
          {
            provided = true;
            std::string text = repr + " (" + thread.name + "): ";
            for(const ModuleInfo::Module& m : moduleInfo.modules)
              if(std::find(m.representations.begin(), m.representations.end(), repr) != m.representations.end())
                text += m.name + (m.name == rp.provider ? "* " : " ");

            text += "default off";
            ctrl->list(text, module, true);
            break;
          }
      }
      if(!provided)
      {
        std::string text = repr + " (All): ";
        for(const ModuleInfo::Module& m : moduleInfo.modules)
          if(std::find(m.representations.begin(), m.representations.end(), repr) != m.representations.end())
            text += m.name + " ";

        text += "default";
        for(const std::string& r : moduleInfo.config.defaultRepresentations)
          if((provided = r == repr))
          {
            text += "*";
            break;
          }
        text += provided ? " off" : " off*";
        ctrl->list(text, module, true);
      }
    }
    return true;
  }
  else if(representation == "save")
  {
    std::string fileName = "threads.cfg";
    for(const std::string& name : File::getFullNames(fileName))
    {
      File path(name, "r", false);
      if(path.exists())
      {
        fileName = name;
        break;
      }
    }

    OutMapFile stream(fileName, true, 120);
    moduleInfo.sendRequest(stream, true);
    return true;
  }
  else
  {
    if(moduleInfo.representations.find(representation) == moduleInfo.representations.end())
      return false;
    // <representation> ?
    else if(module == "?")
    {
      bool provided = false;
      for(const Configuration::Thread& thread : moduleInfo.config())
      {
        for(const auto& rp : thread.representationProviders)
          if(rp.representation == representation)
          {
            provided = true;
            std::string text = thread.name + ": ";
            for(const ModuleInfo::Module& m : moduleInfo.modules)
              if(std::find(m.representations.begin(), m.representations.end(), representation) != m.representations.end())
                text += m.name + (m.name == rp.provider ? "* " : " ");
            text += "default off";
            ctrl->list(text, pattern, true);
            break;
          }
      }
      if(!provided)
      {
        std::string text = "All: ";
        for(const ModuleInfo::Module& m : moduleInfo.modules)
          if(std::find(m.representations.begin(), m.representations.end(), representation) != m.representations.end())
            text += m.name + " ";
        text += "default";
        for(const std::string& r : moduleInfo.config.defaultRepresentations)
          if((provided = r == representation))
          {
            text += "*";
            break;
          }
        text += provided ? " off" : " off*";
        ctrl->list(text, pattern, true);
      }
      return true;
    }
    else
    {
      // <representation> ( <module> | off | default )

      // "default" cannot be set per thread.
      if(module == "default" && !threadName.empty())
        return false;

      // Check if the module that should provide the representation exists at all.
      if(module != "off" && module != "default" &&
         std::find_if(moduleInfo.modules.begin(), moduleInfo.modules.end(),
                      [&module](const ModuleInfo::Module& m)
                      { return m.name == module; }) == moduleInfo.modules.end())
        return false;

      if(module != "default")
      {
        // If no thread is given, use the one that currently provides the representation (off turns the representation off in all threads in that case).
        if(module != "off" && threadName.empty())
        {
          Configuration::Thread* onlyThread = nullptr;
          for(Configuration::Thread& thread : moduleInfo.config())
          {
            if(std::find_if(thread.representationProviders.begin(), thread.representationProviders.end(),
                            [&representation](const Configuration::RepresentationProvider& rp)
                            { return rp.representation == representation; }) != thread.representationProviders.end())
            {
              if(!onlyThread)
                onlyThread = &thread;
              else
                return false;
            }
          }
          if(onlyThread)
            threadName = onlyThread->name;
          else
            return false;
        }

        // If the module is currently provided by default, remove it from that list.
        for(auto i = moduleInfo.config.defaultRepresentations.begin(); i != moduleInfo.config.defaultRepresentations.end(); ++i)
          if(*i == representation)
          {
            moduleInfo.config.defaultRepresentations.erase(i);
            moduleRequestChanged |= true;
            break;
          }
      }

      // Remove or change existing providers.
      bool addNewProvider = true;
      for(Configuration::Thread& thread : moduleInfo.config())
      {
        // Skip if not "default", "off" for all threads or the expected thread.
        if(!(module == "default" || (module == "off" && threadName.empty()) || threadName == thread.name))
          continue;
        for(auto i = thread.representationProviders.begin(); i != thread.representationProviders.end(); ++i)
        {
          if(i->representation == representation)
          {
            addNewProvider = false;
            if(module != i->provider)
            {
              if(module == "off" || module == "default")
                thread.representationProviders.erase(i);
              else
                i->provider = module;
              moduleRequestChanged |= true;
            }
            break;
          }
        }
      }

      // Add new to config if necessary.
      if(module == "default")
      {
        if(std::find(moduleInfo.config.defaultRepresentations.begin(), moduleInfo.config.defaultRepresentations.end(), representation) == moduleInfo.config.defaultRepresentations.end())
        {
          moduleInfo.config.defaultRepresentations.emplace_back(representation);
          moduleRequestChanged |= true;
        }
      }
      else if(module != "off" && addNewProvider)
      {
        for(Configuration::Thread& thread : moduleInfo.config())
        {
          if(thread.name == threadName)
          {
            thread.representationProviders.emplace_back(representation, module);
            moduleRequestChanged |= true;
            break;
          }
        }
      }

      return true;
    }
  }
  return false;
}

bool RobotConsole::moveBall(In& stream)
{
  if(!simulatedRobot)
    return true;
  Vector3f movePos;
  stream >> movePos.x() >> movePos.y() >> movePos.z();
  simulatedRobot->moveBallPerTeam(movePos);
  return true;
}

bool RobotConsole::moveRobot(In& stream)
{
  if(!simulatedRobot)
    return true;
  Vector3f movePos, moveRot;
  stream >> movePos.x() >> movePos.y() >> movePos.z();
  if(stream.eof())
    simulatedRobot->moveRobotPerTeam(movePos, Vector3f::Zero(), false);
  else
  {
    stream >> moveRot.x() >> moveRot.y() >> moveRot.z();
    simulatedRobot->moveRobotPerTeam(movePos, moveRot * 1_deg, true);
  }
  return true;
}

bool RobotConsole::msg(In& stream)
{
  std::string state;
  stream >> state;
  if(state == "off")
  {
    printMessages = false;
    return true;
  }
  else if(state == "on")
  {
    printMessages = true;
    return true;
  }
  else if(state == "log")
  {
    stream >> state;
    std::string name(state);
    if(name.empty())
      return false;
    else
    {
      if(!File::hasExtension(name))
        name = name + ".txt";

      SYNC;
      delete logMessages;
      logMessages = new OutTextRawFile(name);
      return true;
    }
  }
  else if(state == "disable")
  {
    handleMessages = false;
    return true;
  }
  else if(state == "enable")
  {
    handleMessages = true;
    return true;
  }
  return false;
}

bool RobotConsole::penalizeRobot(In& stream)
{
  if(mode != SystemCall::simulatedRobot)
    return false;
  const int robot = std::atoi(robotName.substr(5).c_str()) - 1;
  std::string command;
  stream >> command;
  FOREACH_ENUM(GameController::Penalty, i)
    if(command == TypeRegistry::getEnumName(i))
      return ctrl->gameController.penalty(robot, i);
  return false;
}

bool RobotConsole::repoll(In&)
{
  polled[idDebugResponse] = polled[idDrawingManager] = polled[idDrawingManager3D] = false;
  return true;
}

bool RobotConsole::saveImage(In& stream, std::string threadName)
{
  ImageExport::ExportMode exportMode = ImageExport::rgb;
  std::string filename;
  stream >> filename;
  if(filename == "reset")
  {
    stream >> imageSaveNumber;
    return true;
  }
  else
  {
    if(threadName.empty())
    {
      if(std::count_if(threadData.begin(), threadData.end(),
                    [&](const auto& pair) {return pair.second.images.contains("CameraImage");}) != 1)
        return false;
      threadName = std::find_if(threadData.begin(), threadData.end(),
                                [&](const auto& pair) {return pair.second.images.contains("CameraImage");})->first;
    }
    const Images& images = threadData[threadName].images;
    int number = -1;
    if(filename == "number")
    {
      number = imageSaveNumber++;
      stream >> filename;
    }
    if(filename == "grayscale")
    {
      exportMode = ImageExport::grayscale;
      stream >> filename;
    }

    SYNC;
    const DebugImage* srcImage;
    auto image = images.find("CameraImage");
    if(image != images.end())
      srcImage = image->second.image;
    else
      return false;

    if(filename.empty())
      filename = "raw_image.bmp";

    if(srcImage)
      return ImageExport::exportImage(ImageWrapper<PixelTypes::YUYVPixel>(srcImage->width, srcImage->height,
                                                                          static_cast<PixelTypes::YUYVPixel*>(srcImage->data)),
                                      filename, number, exportMode);
    return false;
  }
}

bool RobotConsole::sensorNoise(In& stream)
{
  std::string option;
  std::string command;
  stream >> command;
  if(command == "whiteNoise" || command == "timeDelay" || command == "discretization")
  {
    option = command;
    stream >> command;
  }
  if(!command.empty() && command != "on" && command != "off")
    return false;
  if(option.empty() || option == "whiteNoise")
    simulatedRobot->enableSensorWhiteNoise(command != "off");
  if(option.empty() || option == "timeDelay")
    simulatedRobot->enableSensorDelay(command != "off");
  if(option.empty() || option == "discretization")
    simulatedRobot->enableSensorDiscretization(command != "off");
  return true;
}

bool RobotConsole::set(In& stream, const std::string& threadName)
{
  const DebugRequestTable& debugRequestTable = threadName.empty()
    ? this->debugRequestTable
    : threadData[threadName].debugRequestTable;

  std::string request, option;
  stream >> request >> option;
  if(request == "?")
  {
    for(const auto& [name, _] : debugRequestTable.slowIndex)
      if(name.substr(0, 11) == "debug data:")
        ctrl->list(ctrl->translate(name.substr(11)), option);
    ctrl->printLn("");
    return true;
  }
  else
    for(const auto& [name, _] : debugRequestTable.slowIndex)
      if(std::string("debugData:") + request == ctrl->translate(name))
      {
        const std::string dataName = name.substr(11);
        if(option == "unchanged")
        {
          notifyDataViewsAboutSetStatus(request, false, threadName);
          SYNC;
          if(!threadName.empty())
            debugSender->bin(idThread) << threadName;
          debugSender->bin(idDebugDataChangeRequest) << dataName << char(0);
          return true;
        }
        else
        {
          OutTextMemory temp(10000);
          temp << option;
          bool singleValue = true;
          while(!stream.eof())
          {
            std::string text;
            stream >> text;
            if(!text.empty() && text[0] != '"' &&
               text.find('=') != std::string::npos)
              singleValue = false;
            temp << text;
          }
          std::string line(temp.data());
          std::string threadForRequest = threadName;
          if(threadForRequest.empty())
          {
            const auto threadsFound = ctrl->getThreadsFor(threadData, std::string("debugData:") + request);
            if(threadsFound.empty())
              return false;
            threadForRequest = threadsFound.front();
          }
          DebugDataInfos::const_iterator j = threadData[threadForRequest].debugDataInfos.find(dataName);
          if(j == threadData[threadForRequest].debugDataInfos.end())
          {
            // request type specification
            {
              SYNC;
              debugSender->bin(idThread) << threadForRequest;
              debugSender->bin(idDebugRequest) << DebugRequest(name, true);
              threadData[threadForRequest].getOrSetWaitsFor = dataName;
              ++waitingFor[idDebugDataResponse];
              polled[idDebugDataResponse] = true; // no automatic repolling
            }
            handleConsole((threadName.empty() ? "" : "for " + threadName + " ")
                          + std::string("_set ").append(request).append(" ").append(line));
            return true;
          }
          else
          {
            const auto& [type, _] = j->second;
            if(option == "?")
            {
              printType(type);
              ctrl->printLn("");
              return true;
            }
            else
            {
              SYNC;
              if(singleValue)
                line = std::string("value = ").append(line).append(";");
              MessageQueue errors;
              Global::theDebugOut = &errors;
              InMapMemory stream(line.c_str(), line.size());
              if(!stream.eof())
              {
                size_t originalSize = debugSender->size();
                if(!threadName.empty())
                  debugSender->bin(idThread) << threadName;
                auto outStream = debugSender->bin(idDebugDataChangeRequest);
                outStream << dataName << char(1);
                DebugDataStreamer streamer(typeInfo, outStream, type, singleValue ? "value" : nullptr);
                stream >> streamer;
                if(errors.empty())
                {
                  setGlobals();
                  notifyDataViewsAboutSetStatus(request, true, threadName); // Maybe call this outside SYNC?
                  return true;
                }
                else
                  debugSender->resize(originalSize);
              }
              setGlobals();
              for(MessageQueue::Message message : errors)
              {
                ASSERT(message.id() == idText);
                ctrl->printLn(message.text().readAll());
              }
              return !errors.empty(); // return true if error was already printed
            }
          }
        }
        break;
      }
  return false;
}

bool RobotConsole::remoteControl(In& stream)
{
  if(remoteControlChannel)
    return false; // already started
  initJoystick();
  std::string mode;
  stream >> mode;
  int team;
  std::string bcastAddr;
  stream >> team >> bcastAddr;
  if(team <= 0 || team > 254)
    return false;
  else if(mode == "local" && bcastAddr.empty())
  {
    remoteControlChannel = std::make_unique<RemoteControlChannel>();
    remoteControlChannel->startLocal(10100 + team, 13);
  }
  else if(mode == "remote" && !bcastAddr.empty())
  {
    remoteControlChannel = std::make_unique<RemoteControlChannel>();
    remoteControlChannel->start(10100 + team, bcastAddr);
  }
  else
    return false;
  return true;
}

bool RobotConsole::viewDrawing(In& stream, RobotConsole::Views& views, const char* type)
{
  bool found = false;
  std::string view;
  std::string drawing;
  std::string command;
  stream >> view >> drawing >> command;
  if(view == "?")
  {
    for(const auto& [name, _] : views)
      ctrl->list(name, view);
    ctrl->printLn("");
    return true;
  }
  else if(view == "off")
  {
    // remove every drawing that is not listed below.
    for(const auto& [name, _] : views)
    {
      views[name].remove_if([](const auto& drawing)
      {
        return drawing != "field lines"
               && drawing != "goal frame"
               && drawing != "field polygons";
      });
    }
    return true;
  }
  else
  {
    bool all = view == "all";
    for(const auto& [name, _] : views)
      if(name == view || all)
      {
        if(drawing == "?")
        {
          std::set<std::string> drawings;
          for(const auto& [_, data] : threadData)
            for(const auto& [drawingName, _] : data.drawingManager.drawings)
              if(!strcmp(data.drawingManager.getDrawingType(drawingName), type))
                drawings.insert(ctrl->translate(drawingName));
          for(const std::string& drawing : drawings)
            ctrl->list(drawing, command);
          ctrl->printLn("");
          return true;
        }
        else
        {
          for(auto& [_, data] : threadData)
          {
            DrawingManager& drawingManager = data.drawingManager;
            for(const auto& [drawingName, _] : drawingManager.drawings)
              if(ctrl->translate(drawingName) == drawing && !strcmp(drawingManager.getDrawingType(drawingName), type))
              {
                if(command == "on" || command.empty())
                {
                  if(drawing.substr(0, 11) != "perception:" && drawing.substr(0, 10) != "cognition:")
                    views[name].remove(drawingName);
                  views[name].push_back(drawingName);
                  if(!found)
                    handleConsole(std::string("dr debugDrawing:") + drawing + " on");
                  found = true;
                  if(!all)
                    return true;
                }
                else if(command == "off")
                {
                  views[name].remove(drawingName);
                  bool found2 = found;
                  if(!all)
                    for(const auto& [drawingName, _] : views)
                      for(const auto& d : views[drawingName])
                        if(ctrl->translate(d) == drawing && !strcmp(drawingManager.getDrawingType(drawingManager.getString(d)), type))
                          found2 = true;
                  if(!found2)
                    handleConsole(std::string("dr debugDrawing:") + drawing + " off");
                  found = true;
                  if(!all)
                    return true;
                }
                else
                  return false;
              }
          }
        }
      }
  }
  return found;
}

bool RobotConsole::viewField(In& stream, const std::string& threadName)
{
  std::string name;
  stream >> name;
  if(fieldViews.find(name) != fieldViews.end())
    ctrl->printLn("View already exists. Specify a different name.");
  else
  {
    fieldViews[name];
    ctrl->setFieldViews(fieldViews);
    ctrl->updateCommandCompletion();
    ctrl->addView(new FieldView(QString::fromStdString(robotName) + ".field." + name.c_str(), *this, name, threadName),
                  QString::fromStdString(robotName) + ".field", SimRobot::Flag::copy | SimRobot::Flag::exportAsImage);
  }
  return true;
}

bool RobotConsole::viewImage(In& stream, std::string threadName)
{
  std::string buffer, buffer2;
  stream >> buffer >> buffer2;
  if(buffer == "?")
  {
    const auto threadsWithImage = ctrl->getThreadsFor(threadData, "representation:CameraImage");
    const auto threadsWithJPEG = ctrl->getThreadsFor(threadData, "representation:JPEGImage");
    ctrl->list("none", buffer2);
    if(threadName.empty())
    {
      if(threadsWithImage.size() == 1)
        ctrl->list("CameraImage", buffer2);
      if(threadsWithJPEG.size() == 1)
        ctrl->list("JPEGImage", buffer2);
      for(const auto& [name, _] : debugRequestTable.slowIndex)
        if(name.substr(0, 13) == "debug images:"
           && ctrl->getThreadsFor(threadData, ctrl->translate(name)).size() == 1)
          ctrl->list(ctrl->translate(name.substr(13)), buffer2);
    }
    else
    {
      if(std::find(threadsWithImage.begin(), threadsWithImage.end(), threadName) != threadsWithImage.end())
        ctrl->list("CameraImage", buffer2);
      if(std::find(threadsWithJPEG.begin(), threadsWithJPEG.end(), threadName) != threadsWithJPEG.end())
        ctrl->list("JPEGImage", buffer2);
      for(const auto& [name, _] : (threadName.empty() ? debugRequestTable : threadData[threadName].debugRequestTable).slowIndex)
        if(name.substr(0, 13) == "debug images:")
          ctrl->list(ctrl->translate(name.substr(13)), buffer2);
    }
    ctrl->printLn("");
    return true;
  }
  else if(buffer == "none")
  {
    std::string name = !buffer2.empty() ? buffer2 : "none" + threadName;
    if(imageViews.find(name) != imageViews.end())
    {
      ctrl->printLn("View already exists. Specify a (different) name.");
      return true;
    }
    imageViews[name];
    ctrl->setImageViews(imageViews);
    ctrl->updateCommandCompletion();
    ctrl->addView(new ImageView(QString::fromStdString(robotName) + ".image." + name.c_str(), *this, "none", name, threadName), QString::fromStdString(robotName) + ".image", SimRobot::Flag::copy | SimRobot::Flag::exportAsImage);
    return true;
  }
  else if(buffer == "CameraImage" || buffer == "JPEGImage")
  {
    // Check which threads provide image.
    const auto threadsFound = ctrl->getThreadsFor(threadData, "representation:" + buffer);

    // If no thread was specified and only one provides an image, pick that one.
    if(threadName.empty() && threadsFound.size() == 1)
      threadName = threadsFound.front();

    // Check whether selected thread actually provides image (or whether a thread was specified).
    else if(std::find(threadsFound.begin(), threadsFound.end(), threadName) == threadsFound.end())
      return false;

    std::string name = !buffer2.empty() ? buffer2 : static_cast<char>(threadName[0] | 0x20) + threadName.substr(1);
    if(imageViews.find(name) != imageViews.end())
    {
      ctrl->printLn("View already exists. Specify a (different) name.");
      return true;
    }

    imageViews[name];
    ctrl->setImageViews(imageViews);
    ctrl->updateCommandCompletion();
    std::string enableGain;
    stream >> enableGain;
    float gain = 1.0f;
    if(enableGain == "gain")
      stream >> gain;
    QString path = QString::fromStdString(robotName) + ".image." + name.c_str();
    ctrl->addView(new ImageView(path, *this, "CameraImage", name, threadName, gain),
                  QString::fromStdString(robotName) + ".image", SimRobot::Flag::copy | SimRobot::Flag::exportAsImage);
    addColorSpaceViews(path, "CameraImage", threadName);
    handleConsole("for " + threadName + " dr representation:" + buffer + " on");
    return true;
  }
  else
  {
    // Check which threads provide image.
    const auto threadsFound = ctrl->getThreadsFor(threadData, "debugImages:" + buffer);

    // If no thread was specified and only one provides an image, pick that one.
    if(threadName.empty() && threadsFound.size() == 1)
      threadName = threadsFound.front();

    // Check whether selected thread actually provides image (or whether a thread was specified).
    else if(std::find(threadsFound.begin(), threadsFound.end(), threadName) == threadsFound.end())
      return false;

    for(const auto& [requestName, _] : threadData[threadName].debugRequestTable.slowIndex)
      if(requestName.substr(0, 13) == "debug images:" &&
         ctrl->translate(requestName.substr(13)) == buffer)
      {
        std::string name = !buffer2.empty() ? buffer2 : std::string(buffer) + threadName;
        if(imageViews.find(name) != imageViews.end())
        {
          ctrl->printLn("View already exists. Specify a (different) name.");
          return true;
        }
        float gain = 1.0f;
        float ddScale = 1.0f;
        for(;;)
        {
          std::string enableStuff;
          stream >> enableStuff;
          if(enableStuff == "gain")
            stream >> gain;
          else if(enableStuff == "ddScale")
            stream >> ddScale;
          else
            break;
        }
        imageViews[name];
        ctrl->setImageViews(imageViews);
        ctrl->updateCommandCompletion();
        QString path = QString::fromStdString(robotName) + ".image." + name.c_str();
        ctrl->addView(new ImageView(path, *this, requestName.substr(13), name, threadName, gain, ddScale),
                      QString::fromStdString(robotName) + ".image", SimRobot::Flag::copy | SimRobot::Flag::exportAsImage);
        addColorSpaceViews(path, requestName.substr(13), threadName);
        handleConsole("for " + threadName + " dr " + ctrl->translate(requestName) + " on");
        return true;
      }
  }

  return false;
}

bool RobotConsole::viewImageCommand(In& stream)
{
  std::string view;
  stream >> view;
  if(view == "?")
  {
    for(const auto& [name, _] : imageViews)
      ctrl->list(name, view);
    ctrl->printLn("");
    return true;
  }
  ImageViewCommand command;
  std::vector<std::pair<std::string, Qt::KeyboardModifier>> modifiers = { { "alt", Qt::AltModifier }, { "ctrl", Qt::ControlModifier }, { "shift", Qt::ShiftModifier } };
  std::string text;
  stream >> text;
  for(auto& [name, modifier] : modifiers)
  {
    if(text == name)
    {
      command.modifiers |= modifier;
      command.modifierMask |= modifier;
      stream >> text;
    }
    else if(text == "no" + name)
    {
      command.modifierMask |= modifier;
      stream >> text;
    }
  }
  std::string line = text;
  while(!stream.eof())
  {
    std::string text;
    stream >> text;
    if(text == "\\n")
      text = "\n";
    line += ' ';
    line += text;
  }
  std::string::size_type prevPos = 0;
  std::string::size_type pos = line.find('$');
  while(pos != std::string::npos)
  {
    if(pos == line.length() - 1 || !std::isdigit(line[pos + 1]))
      return false;
    if(pos > prevPos)
      command.tokens.emplace_back(line.substr(prevPos, pos - prevPos));
    prevPos = ++pos;
    while(++pos < line.length() && std::isdigit(line[pos]));
    int id = std::stoi(line.substr(prevPos, pos - prevPos));
    command.tokens.emplace_back(id);
    prevPos = pos;
    pos = line.find('$', prevPos);
  }
  if(prevPos != line.length())
    command.tokens.emplace_back(line.substr(prevPos));

  bool all = view == "all";
  for(const auto& [name, _] : imageViews)
    if(name == view || all)
    {
      auto& commands = imageViewCommands[name];
      commands.erase(std::remove_if(commands.begin(), commands.end(), [&command](const ImageViewCommand& cmd)
      {
        return command.modifiers == cmd.modifiers && command.modifierMask == cmd.modifierMask;
      }), commands.end());
      if(!command.tokens.empty())
        commands.push_back(command);
      if(!all)
        return true;
    }
  return all;
}

bool RobotConsole::viewPlot(In& stream, const std::string& threadName)
{
  std::string name;
  int plotSize;
  float minValue, maxValue;
  std::string yUnit, xUnit;
  float xScale;
  stream >> name >> plotSize >> minValue >> maxValue >> yUnit >> xUnit >> xScale;
  if(plotSize < 2 || minValue >= maxValue)
    return false;
  if(xScale == 0.f)
    xScale = 1.f;
  QString fullName = QString::fromStdString(robotName) + ".plot." + name.c_str();
  if(static_cast<unsigned int>(plotSize) > maxPlotSize)
    maxPlotSize = plotSize;
  PlotView* view;
  if(plotViews.find(name) != plotViews.end())
    VERIFY(view = static_cast<PlotView*>(ConsoleRoboCupCtrl::application->resolveObject(fullName)));
  else
  {
    plotViews[name];
    ctrl->setPlotViews(plotViews);
    ctrl->updateCommandCompletion();
    view = new PlotView(fullName, *this, name, threadName);
    ctrl->addView(view, QString::fromStdString(robotName) + ".plot", SimRobot::Flag::copy | SimRobot::Flag::exportAsImage);
  }
  view->setParameters(static_cast<unsigned int>(plotSize), minValue, maxValue, yUnit, xUnit, xScale);
  return true;
}

bool RobotConsole::viewPlotDrawing(In& stream)
{
  std::string buffer;
  stream >> buffer;
  if(buffer == "?")
  {
    stream >> buffer;
    for(const auto& [name, _] : plotViews)
      ctrl->list(name, buffer);
    ctrl->printLn("");
    return true;
  }
  else
    for(const auto& [name, _] : plotViews)
      if(name == buffer)
      {
        stream >> buffer;
        if(buffer == "?")
        {
          stream >> buffer;
          for(const auto& i : debugRequestTable.slowIndex)
            if(i.first.substr(0, 5) == "plot:")
              ctrl->list(ctrl->translate(i.first.substr(5)), buffer);
          ctrl->printLn("");
          return true;
        }
        else
        {
          for(const auto& i : debugRequestTable.slowIndex)
            if(ctrl->translate(i.first) == std::string("plot:") + buffer)
            {
              Layer layer;
              layer.layer = buffer;
              stream >> buffer;
              if(buffer == "?")
              {
                stream >> buffer;
                FOREACH_ENUM(Color, color)
                  ctrl->list(ctrl->translate(TypeRegistry::getEnumName(color)), buffer);
                ctrl->printLn("");
                return true;
              }
              if(buffer == "off")
              {
                {
                  SYNC;
                  plotViews[name].remove(layer);
                }
                handleConsole(std::string("dr plot:") + layer.layer + " off");
                return true;
              }
              else
              {
                int c = 0;
                sscanf(buffer.c_str(), "%x", &c);
                layer.color = ColorRGBA((c >> 16) & 0xff, (c >> 8) & 0xff, c & 0xff);
                FOREACH_ENUM(Color, color)
                  if(buffer == TypeRegistry::getEnumName(color))
                    layer.color = (&ColorRGBA::black)[color];
                stream >> layer.description;
                if(layer.description.empty())
                {
                  const size_t parts = std::count(layer.layer.begin(), layer.layer.end(), ':');
                  if(parts == 0)
                    layer.description = layer.layer;
                  else if(parts <= 2)
                  {
                    const size_t pos = layer.layer.rfind(':');
                    layer.description = layer.layer.substr(pos + 1);
                  }
                  else
                  {
                    const size_t pos1 = layer.layer.find(':');
                    const size_t pos2 = layer.layer.find(':', pos1 + 1);
                    layer.description = layer.layer.substr(pos2 + 1);
                  }
                }
                {
                  SYNC;
                  plotViews[name].remove(layer);
                  plotViews[name].push_back(layer);
                }
                handleConsole(std::string("dr plot:") + layer.layer + " on");
                return true;
              }
              return false;
            }
        }
      }
  return false;
}
