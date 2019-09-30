/**
 * @file Controller/RobotConsole.cpp
 *
 * Implementation of RobotConsole.
 *
 * @author Thomas Röfer
 */

#include "RobotConsole.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <cctype>
#include "Platform/Time.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/JPEGImage.h"
#include "Representations/Infrastructure/Thumbnail.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Perception/BallPercepts/BallSpots.h"
#include "Representations/Perception/FieldPercepts/CirclePercept.h"
#include "Representations/Perception/FieldPercepts/PenaltyMarkPercept.h"
#include "Tools/Settings.h"
#include "Tools/Debugging/DebugDataStreamer.h"
#include "Tools/Logging/LoggingTools.h"
#include "Tools/Motion/MofCompiler.h"

#include "ConsoleRoboCupCtrl.h"
#include "Platform/File.h"
#include "Views/AnnotationView.h"
#include "Views/ColorCalibrationView/ColorCalibrationView.h"
#include "Views/ColorSpaceView.h"
#include "Views/FieldView.h"
#include "Views/ImageView.h"
#include "Views/JointView.h"
#include "Views/KickView/KickView.h"
#include "Views/ModuleGraphView.h"
#include "Views/LogPlayerControlView.h"
#include "Views/PlotView.h"
#include "Views/SensorView.h"
#include "Views/CABSLBehaviorView.h"
#include "Views/TimeView.h"

#define PREREQUISITE(p) pollingFor = #p; if(!poll(p)) return false;

bool RobotConsole::MapWriter::handleMessage(InMessage& message)
{
  ASSERT(message.getMessageID() == idDebugDataResponse);
  std::string name, type;
  message.bin >> name >> type;
  DebugDataStreamer streamer(typeInfo, message.bin, type);
  stream << streamer;
  return true;
}

bool RobotConsole::Printer::handleMessage(InMessage& message)
{
  ASSERT(message.getMessageID() == idText);
  ctrl->printLn(message.text.readAll());
  return true;
}

RobotConsole::RobotConsole(DebugReceiver<MessageQueue>* receiver, DebugSender<MessageQueue>* sender) :
  ThreadFrame(receiver, sender),
  logPlayer(*sender),
  logExtractor(logPlayer),
  dataViewWriter(&dataViews)
{
  // this is a hack: call global functions to get parameters
  ctrl = static_cast<ConsoleRoboCupCtrl*>(RoboCupCtrl::controller);
  robotFullName = RoboCupCtrl::getRobotFullName();
  robotName = robotFullName.mid(robotFullName.lastIndexOf('.') + 1);
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
  logPlayer.setSize(0xfffffffff); // max. 64 GB
}

RobotConsole::~RobotConsole()
{
  SYNC;
  setGlobals();
  if(logMessages)
    delete logMessages;
  for(auto& pair : debugDataInfos)
    delete pair.second.second;
  destructed = true;
}

void RobotConsole::init()
{
  if(mode == SystemCall::remoteRobot)
  {
    poll(idRobotname);
    poll(idRobotInfo);
  }
  poll(idTypeInfo);
  poll(idModuleTable);
  joystick.init();
}

void RobotConsole::addPerRobotViews()
{
  SimRobot::Object* category = ctrl->addCategory(robotName, 0, ":/Icons/SimRobot.png");

  SimRobot::Object* annotationCategory = ctrl->addCategory("annotations", category);
  for(auto& data : threadData)
  {
    const std::string threadName = static_cast<char>(data.first[0] | 0x20) + data.first.substr(1);
    ctrl->addView(new AnnotationView(robotName + ".annotations." + threadName.c_str(), data.second.annotationInfo, logPlayer, mode, ctrl->application), annotationCategory);
  }

  ctrl->addView(new CABSLBehaviorView(robotName + ".behavior", *this, activationGraph, activationGraphReceived), category);
  ctrl->addView(new CABSLBehaviorView(robotName + ".teamBehavior", *this, teamActivationGraph, teamActivationGraphReceived), category);

  colorCalibrationView = new ColorCalibrationView(robotName + ".colorCalibration", *this);
  ctrl->addView(colorCalibrationView, category);
  colorCalibrationChanged = false;

  ctrl->addCategory("colorSpace", category);
  ctrl->addCategory("data", category);
  ctrl->addCategory("field", category);
  ctrl->addCategory("image", category);
  ctrl->addView(new JointView(robotName + ".jointData", *this, jointSensorData, jointRequest), category);
  if(mode == SystemCall::logFileReplay)
    ctrl->addView(new LogPlayerControlView(robotName + ".logPlayer", logPlayer, *this), category);

  SimRobot::Object* modulesCategory = ctrl->addCategory("modules", category);
  FOREACH_ENUM(ModuleBase::Category, category)
    ctrl->addView(new ModuleGraphViewObject(robotName + ".modules." + TypeRegistry::getEnumName(category), *this, {category}), modulesCategory);

  ctrl->addCategory("plot", ctrl->application->resolveObject(robotName));

  ctrl->addView(new SensorView(robotName + ".sensorData", *this, fsrSensorData, inertialSensorData, keyStates, systemSensorData, sensorDataTimestamp), category);

  ctrl->addCategory("timing", category);
}

void RobotConsole::addPerThreadViews()
{
  SimRobot::Object* annotationCategory = ctrl->application->resolveObject(robotName + ".annotations");
  SimRobot::Object* timingCategory = ctrl->application->resolveObject(robotName + ".timing");

  for(const auto& config : moduleInfo.config())
  {
    const std::string threadName = static_cast<char>(config.name[0] | 0x20) + config.name.substr(1);
    ctrl->addView(new AnnotationView(robotName + ".annotations." + threadName.c_str(), threadData[config.name].annotationInfo, logPlayer, mode, ctrl->application), annotationCategory);
    ctrl->addView(new TimeView(robotName + ".timing." + threadName.c_str(), *this, threadData[config.name].timeInfo), timingCategory);
  }
}

void RobotConsole::addColorSpaceViews(const std::string& id, const std::string& name, bool user, const std::string& threadIdentifier)
{
  SimRobot::Object* colorSpaceCategory = ctrl->application->resolveObject(robotName + ".colorSpace");
  SimRobot::Object* nameCategory = ctrl->addCategory(name.c_str(), colorSpaceCategory);

  for(int cm = 0; cm < ColorSpaceView::numOfColorModels - (user ? 0 : 1); ++cm)
  {
    SimRobot::Object* modelCategory = ctrl->addCategory(TypeRegistry::getEnumName(ColorSpaceView::ColorModel(cm)), nameCategory);
    const QString& modelCategoryName = modelCategory->getFullName();

    if(ColorSpaceView::ColorModel(cm) == ColorSpaceView::user)
    {
      for(int channel = 1; channel < 4; ++channel)
      {
        ctrl->addView(new ColorSpaceView(
                        modelCategoryName + "." + ColorSpaceView::getChannelNameForColorModel(ColorSpaceView::ColorModel(cm), channel),
                        *this, id, ColorSpaceView::YCbCr, channel + 3, background, threadIdentifier), modelCategory);
      }
    }
    else
    {
      for(int channel = 0; channel < 4; ++channel)
      {
        ctrl->addView(new ColorSpaceView(
                        modelCategoryName + "." + ColorSpaceView::getChannelNameForColorModel(ColorSpaceView::ColorModel(cm), channel),
                        *this, id, ColorSpaceView::ColorModel(cm), channel + (cm == ColorSpaceView::YCbCr&& channel ? 1 : 0), background, threadIdentifier), modelCategory);
      }
    }
  }
}

void RobotConsole::handleAllMessages(MessageQueue& messageQueue)
{
  SYNC;  // Only one thread can access *this now.
  messageQueue.handleAllMessages(*this);
}

bool RobotConsole::handleMessage(InMessage& message)
{
  if(!handleMessages)
    return true;

  if(destructed) // if object is already destroyed, abort here
    return true; // avoid further processing of this message

  if(message.getMessageID() < numOfDataMessageIDs)
  {
    if(logImagesAsJPEGs && message.getMessageID() == idCameraImage)
    {
      CameraImage cameraImage;
      message.bin >> cameraImage;
      MessageQueue queue;
      queue.out.bin << JPEGImage(cameraImage);
      queue.out.finishMessage(idJPEGImage);
      logPlayer.handleMessage(queue.in);
    }
    else
      logPlayer.handleMessage(message);
    message.resetReadPosition();
  }

  switch(message.getMessageID())
  {
    case idText:
    {
      std::string buffer(message.text.readAll());
      if(printMessages)
        ctrl->printLn(buffer);
      if(logMessages)
        *logMessages << buffer << endl;
      return true;
    }
    case idConsole:
      commands.push_back(message.text.readAll());
      return true;
    case idCameraImage:
    {
      CameraImage ci;
      message.bin >> ci;
      if(incompleteImages["raw image"].image)
        incompleteImages["raw image"].image->from(ci);
      else
        incompleteImages["raw image"].image = new DebugImage(ci, true);
      return true;
    }
    case idJPEGImage:
    {
      CameraImage ci;
      JPEGImage jpi;
      message.bin >> jpi;
      jpi.toCameraImage(ci);
      if(incompleteImages["raw image"].image)
        incompleteImages["raw image"].image->from(ci);
      else
        incompleteImages["raw image"].image = new DebugImage(ci, true);
      return true;
    }
    case idThumbnail:
    {
      Thumbnail thumbnail;
      message.bin >> thumbnail;
      if(thumbnail.mode != Thumbnail::yuv)
      {
        if(incompleteImages["raw image"].image)
          incompleteImages["raw image"].image->from(thumbnail.imageY);
        else
          incompleteImages["raw image"].image = new DebugImage(thumbnail.imageY, true);
      }
      else
      {
        Image<PixelTypes::YUYVPixel> i;
        thumbnail.toYUYV(i);
        if(incompleteImages["raw image"].image)
          incompleteImages["raw image"].image->from(i);
        else
          incompleteImages["raw image"].image = new DebugImage(i, true);
      }
      incompleteImages["raw image"].image->timestamp = Time::getCurrentSystemTime();
      return true;
    }
    case idDebugImage:
    {
      std::string id;
      message.bin >> id;
      if(!incompleteImages[id].image)
        incompleteImages[id].image = new DebugImage();
      message.bin >> *incompleteImages[id].image;
      incompleteImages[id].image->timestamp = Time::getCurrentSystemTime();
      break;
    }
    case idFsrSensorData:
    {
      message.bin >> fsrSensorData;
      sensorDataTimestamp = Time::getCurrentSystemTime();
      return true;
    }
    case idInertialSensorData:
    {
      message.bin >> inertialSensorData;
      sensorDataTimestamp = Time::getCurrentSystemTime();
      return true;
    }
    case idJointCalibration:
    {
      message.bin >> jointCalibration;
      jointCalibrationChanged = true;
      return true;
    }
    case idJointRequest:
    {
      message.bin >> jointRequest;
      jointRequest.angles[Joints::rHipYawPitch] = jointRequest.angles[Joints::lHipYawPitch];
      return true;
    }
    case idJointSensorData:
    {
      message.bin >> jointSensorData;
      return true;
    }
    case idKeyStates:
    {
      message.bin >> keyStates;
      return true;
    }
    case idSystemSensorData:
    {
      message.bin >> systemSensorData;
      sensorDataTimestamp = Time::getCurrentSystemTime();
      return true;
    }
    case idDebugDrawing:
    {
      if(polled[idDrawingManager] && !waitingFor[idDrawingManager]) // drawing manager not up-to-date
      {
        ThreadData& data = threadData[threadIdentifier];
        char shapeType, id;
        message.bin >> shapeType >> id;
        const char* name = data.drawingManager.getDrawingName(id); // const char* is required here
        std::string type = data.drawingManager.getDrawingType(name);

        if(type == "drawingOnImage")
          incompleteImageDrawings[name].addShapeFromQueue(message, static_cast<::Drawings::ShapeType>(shapeType));
        else if(type == "drawingOnField")
          incompleteFieldDrawings[name].addShapeFromQueue(message, static_cast<::Drawings::ShapeType>(shapeType));
      }
      return true;
    }
    case idDebugDrawing3D:
    {
      if(polled[idDrawingManager3D] && !waitingFor[idDrawingManager3D])
      {
        char shapeType, id;
        message.bin >> shapeType >> id;
        incompleteDrawings3D[threadData[threadIdentifier].drawingManager3D.getDrawingName(id)]
        .addShapeFromQueue(message, static_cast<::Drawings3D::ShapeType>(shapeType));
      }
      return true;
    }
    case idPlot:
    {
      std::string id;
      float value;
      message.bin >> id >> value;
      Plot& plot = plots[ctrl->translate(id)];
      plot.points.push_back(value);
      while(plot.points.size() > maxPlotSize)
        plot.points.pop_front();
      plot.timestamp = Time::getCurrentSystemTime();
      return true;
    }
    case idFrameBegin:
    {
      threadIdentifier = message.readThreadIdentifier();
      ++currentFrame;
      return true;
    }
    case idFrameFinished:
    {
      ASSERT(threadIdentifier == message.readThreadIdentifier());

      ThreadData& data = threadData[threadIdentifier];

      data.images.clear();
      for(auto& pair : incompleteImages)
      {
        ImagePtr& imagePtr = data.images[pair.first];
        imagePtr.image = pair.second.image;
        imagePtr.threadIdentifier = threadIdentifier;
        pair.second.image = nullptr;
      }

      // Remove drawings generated by this thread from this thread
      for(auto i = data.imageDrawings.begin(); i != data.imageDrawings.end();)
        if(i->second.threadIdentifier.empty())
          i = data.imageDrawings.erase(i);
        else
          ++i;

      for(auto i = data.fieldDrawings.begin(); i != data.fieldDrawings.end();)
        if(i->second.threadIdentifier.empty())
          i = data.fieldDrawings.erase(i);
        else
          ++i;

      // Add new Field and Image drawings
      for(const auto& pair : incompleteImageDrawings)
        threadData[pair.second.threadIdentifier.empty() ? threadIdentifier : pair.second.threadIdentifier].imageDrawings[pair.first] = pair.second;

      for(const auto& pair : incompleteFieldDrawings)
        threadData[pair.second.threadIdentifier.empty() ? threadIdentifier : pair.second.threadIdentifier].fieldDrawings[pair.first] = pair.second;

      // 3D Drawings
      if(polled[idDrawingManager3D] && !waitingFor[idDrawingManager3D])
      {
        // reset all 3d drawings originated from current thread
        for(auto& pair : data.drawings3D)
          pair.second.reset();

        // copy and register newly received 3d debug drawings
        for(auto& pair : incompleteDrawings3D)
        {
          std::string type = data.drawingManager3D.getDrawingType(data.drawingManager3D.getString(pair.first));
          std::string name = type == "camera" ? pair.first + threadIdentifier : pair.first;
          DebugDrawing3D& debugDrawing3D = data.drawings3D[name];
          bool drawn = debugDrawing3D.drawn;
          bool flip = debugDrawing3D.flip;
          pair.second.robotConsole = this;
          debugDrawing3D = pair.second;
          debugDrawing3D.drawn = drawn;
          debugDrawing3D.flip = flip;
          if(!drawn)
          {
            if(type != "unknown")
            {
              QVector<QString> parts;
              parts.append(robotName);
              if(type == "field")
              {
                QString robotNumberString(robotName);
                robotNumberString.remove(0, 5);
                debugDrawing3D.flip = robotNumberString.toInt() < 6;
                parts[0] = "RoboCup";
              }
              else if(type == "robot")
                parts.append("origin");
              else if(type == "camera")
                parts.append(threadIdentifier == "Upper" ? "CameraTop" : "CameraBottom");
              else
                parts.append(type.c_str());
              SYNC_WITH(*ctrl);
              SimRobotCore2::PhysicalObject* object = static_cast<SimRobotCore2::PhysicalObject*>(ctrl->application->resolveObject(parts));
              object->registerDrawing(debugDrawing3D);
              debugDrawing3D.drawn = true;
            }
          }
        }
      }

      incompleteImages.clear();
      incompleteImageDrawings.clear();
      incompleteFieldDrawings.clear();
      incompleteDrawings3D.clear();
      message.resetReadPosition();

      return true;
    }
    case idActivationGraph:
      message.bin >> activationGraph;
      activationGraphReceived = Time::getCurrentSystemTime();
      return true;
    case idTeamActivationGraph:
      message.bin >> teamActivationGraph;
      teamActivationGraphReceived = Time::getCurrentSystemTime();
      return true;
    case idAnnotation:
      threadData[threadIdentifier].annotationInfo.addMessage(message, currentFrame);
      return true;
    case idStopwatch:
      threadData[threadIdentifier].timeInfo.handleMessage(message);
      return true;
    case idDebugResponse:
    {
      SYNC_WITH(*ctrl);
      std::string description;
      bool enable;
      message.text >> description >> enable;
      if(description != "pollingFinished")
        debugRequestTable.addRequest(DebugRequest(description, enable));
      else if(--waitingFor[idDebugResponse] <= 0)
      {
        ctrl->setDebugRequestTable(debugRequestTable);
        updateCompletion = true;
      }
      return true;
    }
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
    case idDrawingManager:
    {
      SYNC_WITH(*ctrl);
      message.bin >> threadData[threadIdentifier].drawingManager;
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
      message.bin >> threadData[threadIdentifier].drawingManager3D;
      --waitingFor[idDrawingManager3D];
      return true;
    }
    case idFieldColors:
      message.bin >> colorCalibration;
      colorCalibrationChanged = true;
      if(colorCalibrationView && colorCalibrationView->widget)
        colorCalibrationView->widget->currentCalibrationChanged();
      waitingFor[idFieldColors] = 0;
      return true;
    case idTypeInfo:
    {
      message.bin >> typeInfo;
      --waitingFor[idTypeInfo];
      return true;
    }
    case idTypeInfoRequest:
    {
      logPlayer.typeInfoReplayed = false;
      return true;
    }
    case idDebugDataResponse:
    {
      std::string name, type;
      message.bin >> name >> type;
      if(debugDataInfos.find(name) == debugDataInfos.end())
        debugDataInfos[name] = DebugDataInfoPair(type, new MessageQueue);
      debugDataInfos[name].second->clear();
      message >> *debugDataInfos[name].second;
      dataViewWriter.handleMessage(message, type, name);
      if(getOrSetWaitsFor == name) // console command requested this one?
      {
        waitingFor[idDebugDataResponse] = 0;
        getOrSetWaitsFor = "";
      }
      else if(threadsOfDebugData[name].empty()
              || threadsOfDebugData[name] == threadIdentifier) // Avoid flooding
      {
        threadsOfDebugData[name] = threadIdentifier;

        // no, representation view requested it
        for(const auto& i : debugRequestTable.slowIndex)
          if("debug data:" + name == i.first)
          {
            if(debugRequestTable.enabled[i.second]) // still enabled?
            {
              // then request it again for the next update
              debugSender->out.bin << DebugRequest("debug data:" + name, true);
              debugSender->out.finishMessage(idDebugRequest);
            }
            break;
          }
      }

      return true;
    }
    case idLogResponse:
      threadData[threadIdentifier].logAcknowledged = true;
      return true;
    case idMotionRequest:
      message.bin >> motionRequest;
      return true;
    case idRobotname:
    {
      message.bin >> Global::getSettings().headName
                  >> Global::getSettings().bodyName
                  >> Global::getSettings().location
                  >> Global::getSettings().scenario
                  >> Global::getSettings().playerNumber;
      --waitingFor[idRobotname];
      return true;
    }
    case idRobotInfo:
    {
      message.bin >> robotInfo;
      --waitingFor[idRobotInfo];
      return true;
    }
    case idRobotDimensions:
      message.bin >> robotDimensions;
      return true;
    case idJointLimits:
      message.bin >> jointLimits;
      return true;
    default:
      ;
  }

  return false;
}

void RobotConsole::update()
{
  setGlobals(); // this is called in GUI thread -> set globals for this thread
  handleJoystick();

  if(!perThreadViewsAdded && !moduleInfo.config().empty())
  {
    addPerThreadViews();
    perThreadViewsAdded = true;
  }

  if(colorCalibrationChanged)
  {
    SYNC;
    colorCalibrationChanged = false;
    colorCalibrationTimestamp = Time::getCurrentSystemTime();
    debugSender->out.bin << colorCalibration;
    debugSender->out.finishMessage(idColorCalibration);
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

  if(updateCompletion)
  {
    SYNC;
    ctrl->updateCommandCompletion();
    updateCompletion = false;
  }
}

void RobotConsole::handleConsole(std::string line)
{
  setGlobals(); // this is called in GUI thread -> set globals for this thread
  for(;;)
  {
    std::string::size_type pos = line.find("\n");
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

void RobotConsole::triggerThreads()
{
  if(mode == SystemCall::logFileReplay)
  {
    SYNC;
    for(const Configuration::Thread& thread : moduleInfo.config())
    {
      debugSender->out.bin << thread.name;
      debugSender->out.finishMessage(idFrameBegin);
      debugSender->out.bin << thread.name;
      debugSender->out.finishMessage(idFrameFinished);
    }
  }
}

bool RobotConsole::poll(MessageID id)
{
  if(id == idDebugResponse || id == idDrawingManager || id == idDrawingManager3D)
    sendModuleRequest();

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
        debugSender->out.bin << DebugRequest("poll");
        debugSender->out.finishMessage(idDebugRequest);
        waitingFor[id] = static_cast<int>(moduleInfo.config().size()) + 1; // Threads + Debug will answer
        break;
      }
      case idModuleTable:
      {
        SYNC;
        moduleInfo.clear();
        debugSender->out.bin << DebugRequest("automated requests:ModuleTable", true);
        debugSender->out.finishMessage(idDebugRequest);
        waitingFor[id] = 1;  // Debug will answer
        break;
      }
      case idTypeInfo:
      {
        SYNC;
        debugSender->out.bin << DebugRequest("automated requests:TypeInfo", true);
        debugSender->out.finishMessage(idDebugRequest);
        waitingFor[id] = 1;  // Debug will answer
        break;
      }
      case idDrawingManager:
      {
        SYNC;
        for(auto& data : threadData)
          data.second.drawingManager.clear();
        debugSender->out.bin << DebugRequest("automated requests:DrawingManager", true);
        debugSender->out.finishMessage(idDebugRequest);
        waitingFor[id] = static_cast<int>(moduleInfo.config().size()); // Threads will answer
        break;
      }
      case idDrawingManager3D:
      {
        SYNC;
        for(auto& data : threadData)
          data.second.drawingManager3D.clear();
        debugSender->out.bin << DebugRequest("automated requests:DrawingManager3D", true);
        debugSender->out.finishMessage(idDebugRequest);
        waitingFor[id] = static_cast<int>(moduleInfo.config().size());  // Threads will answer
        break;
      }
      case idFieldColors:
      {
        SYNC;
        debugSender->out.bin << DebugRequest("representation:FieldColors:once", true);
        debugSender->out.finishMessage(idDebugRequest);
        waitingFor[id] = 1;  // Cognition will answer
        break;
      }
      case idRobotname:
      {
        SYNC;
        debugSender->out.bin << DebugRequest("module:NaoProvider:robotName", true);
        debugSender->out.finishMessage(idDebugRequest);
        waitingFor[id] = 1;  // Motion will answer
        break;
      }
      case idRobotInfo:
      {
        SYNC;
        debugSender->out.bin << DebugRequest("module:GameDataProvider:robotInfo", true);
        debugSender->out.finishMessage(idDebugRequest);
        waitingFor[id] = 1;  // Cognition will answer
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
    poll(idFieldColors);
    if(logFile != "")
    {
      SYNC;
      logPlayer.replayTypeInfo();
    }
  }
}

bool RobotConsole::handleConsoleLine(const std::string& line)
{
  InConfigMemory stream(line.c_str(), line.size());
  std::string command;
  stream >> command;
  bool result = false;
  if(command == "") // comment
    result = true;
  else if(command == "endOfStartScript")
  {
    directMode = true;
    result = true;
  }
  else if(command == "bc")
    result = backgroundColor(stream);
  else if(command == "kick")
  {
    result = kickView();
  }
  else if(command == "cls")
  {
    ctrl->printLn("_cls");
    result = true;
  }
  else if(command == "dr")
  {
    PREREQUISITE(idDebugResponse);
    PREREQUISITE(idDrawingManager3D);
    result = debugRequest(stream);
  }
  else if(command == "echo")
  {
    ctrl->echo(stream);
    result = true;
  }
  else if(command == "get")
  {
    PREREQUISITE(idModuleTable);
    PREREQUISITE(idDebugResponse);
    PREREQUISITE(idTypeInfo);
    result = get(stream, true, true);
  }
  else if(command == "_get") // get, part 2
  {
    PREREQUISITE(idDebugDataResponse);
    PREREQUISITE(idTypeInfo);
    result = get(stream, false, true);
  }
  else if(command == "_get2") // get, part 1 without printing
  {
    PREREQUISITE(idModuleTable);
    PREREQUISITE(idDebugResponse);
    result = get(stream, true, false);
  }
  else if(command == "_get3") // get, part 2 without printing
  {
    PREREQUISITE(idDebugDataResponse);
    PREREQUISITE(idTypeInfo);
    result = get(stream, false, false);
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
  else if(command == "mof")
    result = sendMof(stream);
  else if(command == "mr")
  {
    PREREQUISITE(idModuleTable);
    result = moduleRequest(stream);
  }
  else if(command == "msg")
  {
    result = msg(stream);
  }
  else if(command == "mv")
  {
    if(moveOp != noMove)
      return false;
    result = moveRobot(stream);
  }
  else if(command == "mvb")
  {
    if(moveOp != noMove)
      return false;
    result = moveBall(stream);
  }
  else if(command == "poll")
  {
    PREREQUISITE(idModuleTable);
    result = repoll(stream);
  }
  else if(command == "pr" && mode == SystemCall::simulatedRobot)
    result = ctrl->gameController.handleRobotConsole(robotName.mid(5).toInt() - 1, stream);
  else if(command == "save")
  {
    PREREQUISITE(idModuleTable);
    PREREQUISITE(idDebugResponse);
    result = saveRequest(stream, true);
  }
  else if(command == "_save")
  {
    PREREQUISITE(idDebugDataResponse);
    PREREQUISITE(idTypeInfo);
    result = saveRequest(stream, false);
  }
  else if(command == "set")
  {
    PREREQUISITE(idModuleTable);
    PREREQUISITE(idDebugResponse);
    PREREQUISITE(idTypeInfo);
    result = set(stream);
  }
  else if(command == "_set") // set, part 2
  {
    PREREQUISITE(idDebugDataResponse);
    PREREQUISITE(idTypeInfo);
    result = set(stream);
  }
  else if(command == "si")
  {
    result = saveImage(stream);
  }
  else if(command == "vf")
  {
    PREREQUISITE(idModuleTable);
    PREREQUISITE(idDebugResponse);
    PREREQUISITE(idDrawingManager);
    result = viewField(stream);
  }
  else if(command == "vfd")
  {
    PREREQUISITE(idModuleTable);
    PREREQUISITE(idDebugResponse);
    PREREQUISITE(idDrawingManager);
    result = viewDrawing(stream, fieldViews, "drawingOnField");
  }
  else if(command == "vd") //view data part 1
  {
    PREREQUISITE(idModuleTable);
    PREREQUISITE(idDebugResponse);
    PREREQUISITE(idTypeInfo);
    result = viewData(stream);
  }
  else if(command == "vp")
  {
    PREREQUISITE(idModuleTable);
    PREREQUISITE(idDebugResponse);
    result = viewPlot(stream);
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
  else if(command == "vi")
  {
    PREREQUISITE(idModuleTable);
    PREREQUISITE(idDebugResponse);
    PREREQUISITE(idDrawingManager);
    result = viewImage(stream);
  }
  else if(command == "v3")
  {
    PREREQUISITE(idModuleTable);
    PREREQUISITE(idDebugResponse);
    PREREQUISITE(idDrawingManager);
    result = view3D(stream);
  }

  pollingFor = 0;
  if(!result)
  {
    if(directMode)
    {
      ctrl->printLn("Syntax Error");
    }
    else
    {
      ctrl->printLn((std::string("Syntax Error: ") + line).c_str());
    }
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
      if(static_cast<int>(name.rfind('.')) <= static_cast<int>(name.find_last_of("\\/")))
        name = name + ".txt";
      if(logMessages)
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

bool RobotConsole::backgroundColor(In& stream)
{
  stream >> background.x() >> background.y() >> background.z();
  background *= 0.01F;
  return true;
}

bool RobotConsole::debugRequest(In& stream)
{
  std::string debugRequestString, state;
  stream >> debugRequestString >> state;

  if(debugRequestString == "?")
  {
    for(const auto& i : debugRequestTable.slowIndex)
      ctrl->list(ctrl->translate(i.first).c_str(), state);
    ctrl->printLn("");
    return true;
  }
  else
  {
    if(debugRequestString == "off")
    {
      SYNC;
      debugSender->out.bin << DebugRequest("disableAll");
      debugSender->out.finishMessage(idDebugRequest);
      return true;
    }
    else
      for(const auto& i : debugRequestTable.slowIndex)
        if(ctrl->translate(i.first) == debugRequestString)
        {
          if(state == "off")
            debugRequestTable.enabled[i.second] = false;
          else if(state == "on" || state == "")
            debugRequestTable.enabled[i.second] = true;
          else
            return false;

          SYNC;
          debugSender->out.bin << DebugRequest(i.first, debugRequestTable.enabled[i.second]);
          debugSender->out.finishMessage(idDebugRequest);
          return true;
        }
  }
  return false;
}

#define KEEP(_command, representation, condition) \
  else if(command == "keep" && buf == #_command) \
  { \
    logPlayer.keepFrames([&](InMessage& message) -> bool \
    { \
      if(message.getMessageID() == id##representation) \
      { \
        representation temp; \
        message.bin >> temp; \
        return temp.condition; \
      } \
      return false; \
    }); \
    updateAnnotationsFromLog(); \
    return true; \
  }

bool RobotConsole::log(In& stream)
{
  std::string command;
  stream >> command;
  if(command == "start")
  {
    SYNC;
    if(logFile != "")
    {
      if(logPlayer.state == LogPlayer::playing)
        logPlayer.pause();
      else
        logPlayer.play();
    }
    else if(logPlayer.state != LogPlayer::recording)
      logPlayer.recordStart();
    return true;
  }
  else if(command == "stop")
  {
    SYNC;
    if(logPlayer.state == LogPlayer::recording)
      logPlayer.recordStop();
    else
      logPlayer.stop();
    return true;
  }
  else if(command == "clear")
  {
    SYNC;
    logPlayer.init();
    return true;
  }
  else if(command == "save")
  {
    std::string splitCommand;
    std::string filename;
    int numberLogs = 0;
    stream >> splitCommand;
    if(splitCommand == "split")
    {
      std::string s;
      stream >> s;
      try
      {
        numberLogs = std::stoi(s);
      }
      catch(const std::exception&)
      {
        return false;
      }

      stream >> filename;
    }
    else
      filename = splitCommand;

    const bool hasExtension = static_cast<int>(filename.rfind('.')) > static_cast<int>(filename.find_last_of("\\/"));
    std::string name;

    // If no filename was given, use the one of the log file
    if(!logFile.empty() && filename.empty())
      name = logFile;
    // If the given filename has an extension, use it literally
    else if(hasExtension)
      name = filename;
    // Otherwise, merge additional information into the filename
    else
    {
      std::string path;
      std::string suffix;

      // If the filename starts with a path, use the path
      size_t filenameIndex = filename.find_last_of("\\/");
      if(filenameIndex != std::string::npos)
      {
        path = filename.substr(0, filenameIndex + 1);
        filename = filename.substr(filenameIndex + 1);
      }
      // If the log file has a name, its path can be used instead
      else if(!logFile.empty())
      {
        size_t filenameIndex = logFile.find_last_of("\\/") + 1;
        name = logFile.substr(0, filenameIndex);
        // If the file name is empty, extract that part from the log file name
        LoggingTools::parseName(logFile.substr(filenameIndex), nullptr, nullptr, nullptr, nullptr, nullptr, filename.empty() ? &filename : nullptr, nullptr, &suffix);
      }

      // If the filename is still empty, we cannot construct a valid name
      if(filename.empty())
        return false;

      name += LoggingTools::createName(path, Global::getSettings().headName, Global::getSettings().bodyName, Global::getSettings().scenario, Global::getSettings().location, filename, Global::getSettings().playerNumber, suffix);
      name += ".log";
    }

    if(name[0] != '/' && name[0] != '\\' && (name.size() < 2 || name[1] != ':'))
      name = std::string("Logs/") + name;
    SYNC;
    if(numberLogs > 0)
      return logExtractor.split(name, logFile.empty() ? &typeInfo : nullptr, numberLogs);
    else
      return logExtractor.save(name, logFile.empty() ? &typeInfo : nullptr);
  }
  else if(command == "full")
  {
    logImagesAsJPEGs = false;
    return true;
  }
  else if(command == "jpeg")
  {
    logImagesAsJPEGs = true;
    return true;
  }
  else if(command == "saveAudio")
  {
    SYNC;
    std::string name;
    stream >> name;
    if(name.empty())
    {
      std::string::size_type pos = logPlayer.logfilePath.rfind(".");
      if(pos == std::string::npos)
        return false;
      else
        name = logPlayer.logfilePath.substr(0, pos);
    }

    if(static_cast<int>(name.rfind('.')) <= static_cast<int>(name.find_last_of("\\/")))
      name = name + ".wav";
    if(!File::isAbsolute(name.c_str()))
      name = "Sounds/" + name;

    return logExtractor.saveAudioFile(name);
  }
  else if(command == "saveImages")
  {
    SYNC;
    stream >> command;
    bool raw = false;
    bool onlyPlaying = false;
    int takeEachNthFrame = 1;
    if(command == "raw")
    {
      raw = true;
      stream >> command;
    }
    if(command == "onlyPlaying")
    {
      onlyPlaying = true;
      stream >> command;
    }
    if(!command.empty())
    {
      try {takeEachNthFrame = std::stoi(command);}
      catch(const std::invalid_argument&) {return false;}
      catch(const std::out_of_range) {return false;}
      stream >> command;
    }
    if(command.empty())
    {
      std::string::size_type pos = logPlayer.logfilePath.rfind(".");
      if(pos == std::string::npos)
        return false;
      else
        command = logPlayer.logfilePath.substr(0, pos) + "_Images";
    }

    if(command.back() != '/')
      command += '/';
    if(!File::isAbsolute(command.c_str()))
      command = "Images/" + command;

    return logExtractor.saveImages(command, raw, onlyPlaying, takeEachNthFrame);
  }
  else if(command == "saveInertialSensorData"
          || command == "saveJointAngleData"
          || command == "saveLabeledBallSpots"
          || command == "saveWalkingData"
          || command == "saveGetUpEngineFailData"
          || command == "saveTiming")
  {
    SYNC;
    std::string name;
    stream >> name;
    if(name.empty())
    {
      std::string::size_type pos = logPlayer.logfilePath.rfind(".");
      if(pos == std::string::npos)
        return false;
      else
        name = logPlayer.logfilePath.substr(0, pos) + "_" + command.substr(4);
    }

    if(static_cast<int>(name.rfind('.')) <= static_cast<int>(name.find_last_of("\\/")))
      name = name + ".csv";
    if(command == "saveInertialSensorData")
      return logExtractor.saveInertialSensorData(name);
    else if(command == "saveJointAngleData")
      return logExtractor.saveJointAngleData(name);
    else if(command == "saveLabeledBallSpots")
      return logExtractor.saveLabeledBallSpots(name);
    else if(command == "saveTiming")
      return logExtractor.writeTimingData(name);;
  }
  else if(command == "keep" || command == "remove")
  {
    SYNC;
    std::string buf;
    stream >> buf;
    if(command == "keep" && buf == "ballPercept")
    {
      stream >> buf;
      logPlayer.keepFrames([&](InMessage& message) -> bool
      {
        if(message.getMessageID() == idBallPercept)
        {
          BallPercept ballPercept;
          message.bin >> ballPercept;
          return (buf == "" && ballPercept.status != BallPercept::notSeen)
          || (buf == "seen" && ballPercept.status == BallPercept::seen)
          || (buf == "guessed" && ballPercept.status == BallPercept::guessed);
        }
        return false;
      });

      updateAnnotationsFromLog();
      return true;
    }
    else if(command == "keep" && buf == "option")
    {
      stream >> buf;
      std::string state;
      stream >> state;
      logPlayer.keepFrames([&](InMessage& message) -> bool
      {
        if(message.getMessageID() == idActivationGraph)
        {
          ActivationGraph activationGraph;
          message.bin >> activationGraph;
          for(const auto& node : activationGraph.graph)
            if(node.option == buf && (state.empty() || node.state == state))
              return true;
          return false;
        }
        return false;
      });

      updateAnnotationsFromLog();
      return true;
    }

    KEEP(ballSpots, BallSpots, ballSpots.size() > 0)
    KEEP(circlePercept, CirclePercept, wasSeen)
    KEEP(lower, CameraInfo, camera == CameraInfo::lower)
    KEEP(upper, CameraInfo, camera == CameraInfo::upper)
    KEEP(penaltyMarkPercept, PenaltyMarkPercept, wasSeen)

    else
    {
      std::vector<MessageID> messageIDs;
      while(buf != "")
      {
        FOREACH_ENUM(MessageID, i)
          if(buf == TypeRegistry::getEnumName(i))
          {
            messageIDs.push_back(i);
            goto found;
          }

        return false;
      found:
        stream >> buf;
      }
      if(messageIDs.size())
      {
        messageIDs.push_back(undefined);
        if(command == "keep")
          logPlayer.keep([&](InMessage& message) -> bool
        {
          MessageID* m = &messageIDs[0];
          while(*m)
          {
            if(message.getMessageID() == *m ||
               message.getMessageID() == idFrameBegin ||
               message.getMessageID() == idFrameFinished)
              return true;
            ++m;
          }
          return false;
        });
        else
          logPlayer.keep([&](InMessage& message) -> bool
        {
          MessageID* m = &messageIDs[0];
          while(*m)
          {
            if(message.getMessageID() == *m)
              return false;
            ++m;
          }
          return true;
        });

        updateAnnotationsFromLog();
        return true;
      }
    }
  }
  else if(command == "trim")
  {
    std::string type, firstArgument;
    stream >> type >> firstArgument;
    bool isValidCommand = false;
    if(type == "from")
    {
      int startFrame;
      try {startFrame = std::stoi(firstArgument);}
      catch(const std::invalid_argument&) {return false;}
      catch(const std::out_of_range) {return false;}
      if(startFrame >= 0 && startFrame < logPlayer.numberOfFrames - 1)
      {
        logPlayer.trim(startFrame, logPlayer.numberOfFrames - 1);
        isValidCommand = true;
      }
      else { return false; }
    }
    else if(type == "until")
    {
      int endFrame;
      try { endFrame = std::stoi(firstArgument); }
      catch(const std::invalid_argument&) { return false; }
      catch(const std::out_of_range) { return false; }
      if(endFrame >= 0 && endFrame < logPlayer.numberOfFrames)
      {
        logPlayer.trim(0, endFrame);
        isValidCommand = true;
      }
      else { return false; }
    }
    else if(type == "between")
    {
      std::string secondArgument;
      stream >> secondArgument;
      volatile int startFrame, endFrame;
      try
      {
        startFrame = std::stoi(firstArgument);
        endFrame = std::stoi(secondArgument);
      }
      catch(const std::invalid_argument&) { return false; }
      catch(const std::out_of_range) { return false; }
      if(startFrame >= 0 && startFrame < logPlayer.numberOfFrames &&
         endFrame >= 0 && endFrame < logPlayer.numberOfFrames && startFrame < endFrame)
      {
        logPlayer.trim(startFrame, endFrame);
        isValidCommand = true;
      }
      else {return false;}
    }

    if(isValidCommand)
    {
      logPlayer.gotoFrame(0);
      logExtractor.save(logFile, &typeInfo);
    }
  }
  else if(command == "?")
  {
    SYNC;
    std::string option;
    stream >> option;
    int frequencies[numOfMessageIDs];
    unsigned sizes[numOfMessageIDs];
    logPlayer.statistics(frequencies, sizes);

    float size = 0;
    FOREACH_ENUM(MessageID, id, numOfDataMessageIDs)
      size += static_cast<float>(sizes[id]);

    char buf[100];
    FOREACH_ENUM(MessageID, id, numOfDataMessageIDs)
      if(frequencies[id])
      {
        sprintf(buf, "%u\t%.2f%%", frequencies[id], static_cast<float>(sizes[id]) * 100.f / size);
        ctrl->list(std::string(buf) + "\t" + TypeRegistry::getEnumName(id), option, true);
      }
    sprintf(buf, "%u", logPlayer.getNumberOfMessages());
    ctrl->printLn(std::string(buf) + "\ttotal");
    return true;
  }
  else if(command == "mr") //log mr
  {
    SYNC;
    std::string param;
    stream >> param;

    // Determine, which representations are available for which thread.
    std::unordered_map<std::string, int[numOfDataMessageIDs]> frequencies;
    for(const auto& config : moduleInfo.config())
      logPlayer.statistics(frequencies[config.name], nullptr, config.name);

    std::list<std::string> commands;
    const auto log = std::find(moduleInfo.modules.begin(), moduleInfo.modules.end(), "LogDataProvider");
    if(log != moduleInfo.modules.end())
      for(int i = idFrameFinished + 1; i < numOfDataMessageIDs; ++i)
        for(const auto& frequency : frequencies)
          if(frequency.second[i] > 0)
          { // This representation is provided in at least one thread
            std::string representation = std::string(TypeRegistry::getEnumName(MessageID(i))).substr(2);
            if(representation == "JPEGImage" || representation == "Thumbnail")
              representation = "CameraImage";
            if(std::find(log->representations.begin(), log->representations.end(), representation) != log->representations.end())
            {
              // First activate log data providers in the desired threads
              for(const auto& frequency : frequencies)
                if(frequency.second[i] > 0)
                  commands.push_back(representation + " LogDataProvider " + frequency.first);

              // Then, switch providers for the representation off in all other threads
              for(const auto& frequency : frequencies)
                if(frequency.second[i] == 0)
                {
                  if(frequency.first == "Cognition")
                  {
                    auto j = std::find(moduleInfo.modules.begin(), moduleInfo.modules.end(), "Perception" + representation + "Provider");
                    if(j != moduleInfo.modules.end())
                    {
                      commands.push_back(representation + " Perception" + representation + "Provider Cognition");
                      continue;
                    }
                  }
                  commands.push_back(representation + " off " + frequency.first);
                }
            }
            break;
          }

    bool success = true;
    for(const std::string& command : commands)
      if(param == "list")
        ctrl->printLn("mr " + command);
      else
      {
        InTextMemory strMem(command.c_str(), command.size());
        success &= moduleRequest(strMem);
      }

    return success;
  }
  else if(logFile != "")
  {
    SYNC;
    if(command == "load")
    {
      std::string name;
      stream >> name;
      if(name.empty())
        return false;
      else
      {
        if(static_cast<int>(name.rfind('.')) <= static_cast<int>(name.find_last_of("\\/")))
          name = name + ".log";
        if(name[0] != '/' && name[0] != '\\' && (name.size() < 2 || name[1] != ':'))
          name = std::string("Logs/") + name;
        logFile = name;
        LogPlayer::LogPlayerState state = logPlayer.state;
        bool result = logPlayer.open(name);
        if(result)
          updateAnnotationsFromLog();
        if(result && state == LogPlayer::playing)
          logPlayer.play();
        return result;
      }
    }
    else if(command == "cycle")
    {
      logPlayer.setLoop(true);
      return true;
    }
    else if(command == "once")
    {
      logPlayer.setLoop(false);
      return true;
    }
    else if(command == "pause")
    {
      logPlayer.pause();
      return true;
    }
    else if(command == "forward")
    {
      std::string opt;
      stream >> opt;
      if(opt == "image")
        logPlayer.stepImageForward();
      else
        logPlayer.stepForward();
      return true;
    }
    else if(command == "backward")
    {
      std::string opt;
      stream >> opt;
      if(opt == "image")
        logPlayer.stepImageBackward();
      else
        logPlayer.stepBackward();
      return true;
    }
    else if(command == "repeat")
    {
      logPlayer.stepRepeat();
      return true;
    }
    else if(command == "goto")
    {
      LogPlayer::LogPlayerState state = logPlayer.state;

      int frame;
      stream >> frame;
      logPlayer.gotoFrame(std::max<>(std::min<>(frame, logPlayer.numberOfFrames - 1), 0));
      if(state == LogPlayer::playing)
        logPlayer.play();
      return true;
    }
    else if(command == "time")
    {
      int minutes, seconds;
      stream >> minutes;
      stream >> seconds;
      const int time = minutes * 60 + seconds;
      int frame = logPlayer.getFrameForRemainingGCTime(time);
      if(frame < 0)
      {
        minutes = time / 60;
        seconds = time % 60;
        std::ostringstream stream;
        stream << "No frame corresponds to GC time" << (minutes < 10 ? " 0" : " ") << minutes << (seconds < 10 ? ":0" : ":") << seconds;
        ctrl->printLn(stream.str());
      }
      else
      {
        logPlayer.gotoFrame(std::max<>(std::min<>(frame - 1, logPlayer.numberOfFrames - 1), 0));
        if(logPlayer.state == LogPlayer::playing)
          logPlayer.play();
      }
      return true;
    }
    else if(command == "fastForward")
    {
      //backup state, gotoFrame will change the state.
      LogPlayer::LogPlayerState state = logPlayer.state;
      int frame = logPlayer.currentFrameNumber + 100;

      logPlayer.gotoFrame(std::max<>(std::min<>(frame - 1, logPlayer.numberOfFrames - 1), 0));
      if(state == LogPlayer::playing)
      {
        //if the state was playing before, continue playing
        logPlayer.play();
      }
      return true;
    }
    else if(command == "fastBackward")
    {
      //backup state, gotoFrame will change the state.
      LogPlayer::LogPlayerState state = logPlayer.state;
      int frame = logPlayer.currentFrameNumber - 100;

      logPlayer.gotoFrame(std::max<>(std::min<>(frame - 1, logPlayer.numberOfFrames - 1), 0));
      if(state == LogPlayer::playing)
      {
        //if the state was playing before, continue playing
        logPlayer.play();
      }
      return true;
    }
  }
  return false;
}

bool RobotConsole::get(In& stream, bool first, bool print)
{
  std::string request, option;
  stream >> request >> option;
  if(request == "?")
  {
    for(const auto& i : debugRequestTable.slowIndex)
      if(i.first.substr(0, 11) == "debug data:")
        ctrl->list(ctrl->translate(i.first.substr(11)), option);
    ctrl->printLn("");
    return true;
  }
  else
    for(const auto& i : debugRequestTable.slowIndex)
      if(std::string("debugData:") + request == ctrl->translate(i.first))
      {
        if(first)
        {
          // request up-to-date data
          if(!debugRequestTable.enabled[i.second])
          {
            SYNC;
            debugSender->out.bin << DebugRequest(i.first, true);
            debugSender->out.finishMessage(idDebugRequest);
            waitingFor[idDebugDataResponse] = 1;
            getOrSetWaitsFor = i.first.substr(11);
          }
          polled[idDebugDataResponse] = true; // no automatic repolling
          handleConsole(std::string(print ? "_get " : "_get3 ") + request + " " + option);
          return true;
        }
        else
        {
          DebugDataInfos::const_iterator j = debugDataInfos.find(i.first.substr(11));
          ASSERT(j != debugDataInfos.end());
          if(option == "?")
          {
            printType(j->second.first.c_str());
            ctrl->printLn("");
            return true;
          }
          else if(option == "")
          {
            SYNC;
            OutMapMemory memory(true, 16384);
            MapWriter memoryWriter(typeInfo, memory);
            j->second.second->handleAllMessages(memoryWriter);
            std::string buffer = "set " + request + " " + memory.data();
            if(print)
              ctrl->printLn(buffer);
            else
              printBuffer = buffer;
            return true;
          }
        }
        break;
      }
  return false;
}

bool RobotConsole::DataViewWriter::handleMessage(InMessage& message)
{
  std::string name, type;
  message.bin >> name >> type;

  return handleMessage(message, type, name);
}

bool RobotConsole::DataViewWriter::handleMessage(InMessage& message, const std::string& type, const std::string& name)
{
  auto view = pDataViews->find(name);
  ASSERT(message.getMessageID() == idDebugDataResponse);
  return view != pDataViews->end() && view->second->handleMessage(message, type, name);
}

bool RobotConsole::set(In& stream)
{
  std::string request, option;
  stream >> request >> option;
  if(request == "?")
  {
    for(const auto& i : debugRequestTable.slowIndex)
      if(i.first.substr(0, 11) == "debug data:")
        ctrl->list(ctrl->translate(i.first.substr(11)), option);
    ctrl->printLn("");
    return true;
  }
  else
    for(const auto& i : debugRequestTable.slowIndex)
      if(std::string("debugData:") + request == ctrl->translate(i.first))
      {
        if(option == "unchanged")
        {
          SYNC;
          debugSender->out.bin << i.first.substr(11) << char(0);
          debugSender->out.finishMessage(idDebugDataChangeRequest);
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
            if(text.size() > 0 && text[0] != '"' &&
               text.find('=') != std::string::npos)
              singleValue = false;
            temp << text;
          }
          std::string line(temp.data());
          DebugDataInfos::const_iterator j = debugDataInfos.find(i.first.substr(11)); //the substr(11) removes "debug data:" from the description string
          if(j == debugDataInfos.end())
          {
            // request type specification
            {
              SYNC;
              debugSender->out.bin << DebugRequest(i.first, true);
              debugSender->out.finishMessage(idDebugRequest);
              waitingFor[idDebugDataResponse] = 1;
              polled[idDebugDataResponse] = true; // no automatic repolling
              getOrSetWaitsFor = i.first.substr(11);
            }
            handleConsole(std::string("_set ") + request + " " + line);
            return true;
          }
          else
          {
            if(option == "?")
            {
              printType(j->second.first.c_str());
              ctrl->printLn("");
              return true;
            }
            else
            {
              SYNC;
              if(singleValue)
                line = "value = " + line + ";";
              MessageQueue errors;
              Global::theDebugOut = &errors.out;
              InMapMemory stream(line.c_str(), line.size());
              if(!stream.eof())
              {
                debugSender->out.bin << i.first.substr(11) << char(1);
                DebugDataStreamer streamer(typeInfo, debugSender->out.bin, j->second.first, singleValue ? "value" : 0);
                stream >> streamer;
                if(errors.isEmpty())
                {
                  debugSender->out.finishMessage(idDebugDataChangeRequest);
                  setGlobals();
                  return true;
                }
                else
                  debugSender->out.cancelMessage();
              }
              setGlobals();
              Printer printer(ctrl);
              errors.handleAllMessages(printer);
              return !errors.isEmpty(); // return true if error was already printed
            }
          }
        }
        break;
      }
  return false;
}

void RobotConsole::printType(const std::string& type, const std::string& field)
{
  if(type[type.size() - 1] == ']')
  {
    size_t endOfType = type.find_last_of('[');
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

bool RobotConsole::sendMof(In& stream)
{
  std::string parameter;
  stream >> parameter;
  if(parameter != "")
    return false;

  std::vector<float> motionData;
  char buffer[10000];
  MofCompiler* mofCompiler = new MofCompiler;
  const bool success = mofCompiler->compileMofs(buffer, sizeof(buffer), motionData);
  delete mofCompiler;

  char* p = buffer;
  while(*p)
  {
    char* p2 = strchr(p, '\n');
    *p2 = 0;
    ctrl->printLn(p);
    p = p2 + 1;
  }

  if(success)
  {
    ctrl->printLn("SpecialActions compiled");
    SYNC;
    for(const float f : motionData)
      debugSender->out.bin << f;
    debugSender->out.finishMessage(idMotionNet);
  }
  return true;
}

bool RobotConsole::repoll(In& stream)
{
  polled[idDebugResponse] = polled[idDrawingManager] = polled[idDrawingManager3D] = false;
  return true;
}

bool RobotConsole::moduleRequest(In& stream)
{
  SYNC;
  std::string representation, module, pattern;
  stream >> representation >> module >> pattern;
  if(representation == "modules")
  {
    for(const ModuleInfo::Module& m : moduleInfo.modules)
    {
      std::string text = m.name + " (" + TypeRegistry::getEnumName(m.category) + "): ";
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
        for(const std::string r : moduleInfo.config.defaultRepresentations)
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
      // <representation> <module> <thread>
      if(module != "off" && module != "default")
      {
        if(std::find_if(moduleInfo.modules.begin(), moduleInfo.modules.end(), [&module](const ModuleInfo::Module& m) { return m.name == module; }) == moduleInfo.modules.end())
        {
          return false;
        }
      }

      bool moduleIsAlreadyProvidingThisRepresentation = false;
      bool erasedSomething = false;

      for(auto i = moduleInfo.config.defaultRepresentations.begin(); i != moduleInfo.config.defaultRepresentations.end();)
      {
        if(*i == representation)
        {
          if(module != "default")
            erasedSomething = true;
          moduleInfo.config.defaultRepresentations.erase(i);
          break;
        }
        ++i;
      }

      // If no thread is given, use the one that currently provides the representation.
      if(module != "default" && module != "off" && pattern == "")
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
          pattern = onlyThread->name;
        else
          return false;
      }

      // Check if the module has not changed or remove the previous module.
      for(size_t index = 0; index < moduleInfo.config().size(); index++)
      {
        // Skip if not "default" or the expected thread.
        if(!pattern.empty() && pattern != moduleInfo.config()[index].name)
          continue;
        for(auto i = moduleInfo.config()[index].representationProviders.begin(); i != moduleInfo.config()[index].representationProviders.end();)
        {
          if(i->representation == representation)
          {
            if(i->provider == module)
              moduleIsAlreadyProvidingThisRepresentation = true;
            else
            {
              i = moduleInfo.config()[index].representationProviders.erase(i);
              erasedSomething = true;
              continue;
            }
          }
          ++i;
        }
      }

      // Add new to config if necessary.
      if(module == "default")
        moduleInfo.config.defaultRepresentations.emplace_back(representation);
      else if(module != "off" && !moduleIsAlreadyProvidingThisRepresentation)
      {
        for(Configuration::Thread& thread : moduleInfo.config())
        {
          if(thread.name == pattern)
          {
            thread.representationProviders.emplace_back(representation, module);
            break;
          }
          // Threadname is unavailable
          if(thread.name == moduleInfo.config().back().name)
            return false;
        }
      }

      if(!moduleIsAlreadyProvidingThisRepresentation || erasedSomething)
        moduleRequestChanged = true;
      return true;
    }
  }
  return false;
}

bool RobotConsole::moveRobot(In& stream)
{
  SYNC;
  stream >> movePos.x() >> movePos.y() >> movePos.z();
  if(stream.eof())
    moveOp = movePosition;
  else
  {
    stream >> moveRot.x() >> moveRot.y() >> moveRot.z();
    moveOp = moveBoth;
  }
  return true;
}

bool RobotConsole::moveBall(In& stream)
{
  SYNC;
  stream >> movePos.x() >> movePos.y() >> movePos.z();
  moveOp = moveBallPosition;
  return true;
}

void RobotConsole::printLn(const std::string& line)
{
  if(nullptr != ctrl)
  {
    ctrl->printLn(line);
  }
}

bool RobotConsole::view3D(In& stream)
{
  std::string buffer;
  stream >> buffer;
  if(buffer == "?")
  {
    stream >> buffer;
    ctrl->list("image", buffer);
    for(const auto& i : debugRequestTable.slowIndex)
      if(i.first.substr(0, 13) == "debug images:")
        ctrl->list(ctrl->translate(i.first.substr(13)), buffer);
    ctrl->printLn("");
    return true;
  }
  else
  {
    std::string buffer2;
    bool jpeg = false;
    std::string thread;
    for(;;)
    {
      stream >> buffer2;
      if(!jpeg && buffer2 == "jpeg")
        jpeg = true;
      else
      {
        if(thread.empty())
          for(const auto& config : moduleInfo.config())
            if(config.name == buffer2)
            {
              thread = config.name;
              goto found;
            }
        break;
      found: ;
      }
    }

    if(thread.empty())
      thread = "Lower";
    if(buffer == "image")
    {
      std::string name = buffer2 != "" ? buffer2 : static_cast<char>(thread[0] | 0x20) + thread.substr(1);
      if(imageViews3D.find(name) != imageViews3D.end())
      {
        ctrl->printLn("View already exists. Specify a (different) name.");
        return true;
      }
      imageViews3D[name];
      addColorSpaceViews("raw image", name, false, thread);
      if(jpeg)
        handleConsole("dr representation:JPEGImage on");
      else
        handleConsole("dr representation:CameraImage on");
      return true;
    }
    else if(!jpeg)
      for(const auto& i : debugRequestTable.slowIndex)
        if(i.first.substr(0, 13) == "debug images:" &&
           ctrl->translate(i.first.substr(13)) == buffer)
        {
          std::string name = buffer2 != "" ? buffer2 : std::string(buffer) + thread;
          if(imageViews3D.find(name) != imageViews3D.end())
          {
            ctrl->printLn("View already exists. Specify a (different) name.");
            return true;
          }
          addColorSpaceViews(i.first.substr(13), name, true, thread);
          handleConsole(std::string("dr ") + ctrl->translate(i.first) + " on");
          return true;
        }
  }
  return false;
}

bool RobotConsole::viewField(In& stream)
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
    ctrl->addView(new FieldView(robotName + ".field." + name.c_str(), *this, name), robotName + ".field", SimRobot::Flag::copy | SimRobot::Flag::exportAsImage);
  }
  return true;
}

bool RobotConsole::viewData(In& stream)
{
  std::string name, option;
  stream >> name >> option;
  for(const auto& i : debugRequestTable.slowIndex)
    if(std::string("debugData:") + name == ctrl->translate(i.first))
    {
      // enable the debug request if it is not already enabled
      if(option == "off")
      {
        debugRequestTable.enabled[i.second] = false;
        return true;
      }
      else if(option == "on" || option == "")
      {
        if(dataViews.find(name) == dataViews.end())
        {
          dataViews[name] = new DataView(robotName + ".data." + name.c_str(), name, *this, typeInfo);
          ctrl->addView(dataViews[name], robotName + ".data", SimRobot::Flag::copy | SimRobot::Flag::exportAsImage);
        }
        requestDebugData(name, true);
        return true;
      }
      else
        return false;
    }
  return false;
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
    for(const auto& viewPair : views)
      ctrl->list(viewPair.first, view);
    ctrl->printLn("");
    return true;
  }
  else if(view == "off")
  {
    // remove every drawing that is not listed below.
    for(const auto& viewPair : views)
    {
      views[viewPair.first].remove_if([](const auto & drawing)
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
    for(const auto& viewPair : views)
      if(viewPair.first == view || all)
      {
        if(drawing == "?")
        {
          std::set<std::string> drawings;
          for(const auto& data : threadData)
            for(const auto& drawingPair : data.second.drawingManager.drawings)
              if(!strcmp(data.second.drawingManager.getDrawingType(drawingPair.first), type))
                drawings.insert(ctrl->translate(drawingPair.first));
          for(const std::string& drawing : drawings)
            ctrl->list(drawing, command);
          ctrl->printLn("");
          return true;
        }
        else
        {
          for(auto& data : threadData)
          {
            DrawingManager& drawingManager = data.second.drawingManager;
            for(const auto& drawingPair : drawingManager.drawings)
              if(ctrl->translate(drawingPair.first) == drawing && !strcmp(drawingManager.getDrawingType(drawingPair.first), type))
              {
                if(command == "on" || command == "")
                {
                  if(drawing.substr(0, 11) != "perception:" && drawing.substr(0, 10) != "cognition:")
                    views[viewPair.first].remove(drawingPair.first);
                  views[viewPair.first].push_back(drawingPair.first);
                  if(!found)
                    handleConsole(std::string("dr debugDrawing:") + drawing + " on");
                  found = true;
                  if(!all)
                    return true;
                }
                else if(command == "off")
                {
                  views[viewPair.first].remove(drawingPair.first);
                  bool found2 = found;
                  if(!all)
                    for(const auto& viewPair : views)
                      for(const auto& d : views[viewPair.first])
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

bool RobotConsole::viewImage(In& stream)
{
  std::string buffer;
  stream >> buffer;
  if(buffer == "?")
  {
    stream >> buffer;
    ctrl->list("none", buffer);
    ctrl->list("image", buffer);
    for(const auto& i : debugRequestTable.slowIndex)
      if(i.first.substr(0, 13) == "debug images:")
        ctrl->list(ctrl->translate(i.first.substr(13)), buffer);
    ctrl->printLn("");
    return true;
  }
  else if(buffer == "none")
  {
    std::string thread = "Lower";
    stream >> buffer;
    for(const auto& config : moduleInfo.config())
      if(config.name == buffer)
      {
        thread = buffer;
        stream >> buffer;
        break;
      }

    std::string name = buffer != "" ? buffer : "none" + thread;
    if(imageViews.find(name) != imageViews.end())
    {
      ctrl->printLn("View already exists. Specify a (different) name.");
      return true;
    }
    imageViews[name];
    ctrl->setImageViews(imageViews);
    ctrl->updateCommandCompletion();
    ctrl->addView(new ImageView(robotName + ".image." + name.c_str(), *this, "none", name, false, thread), robotName + ".image", SimRobot::Flag::copy | SimRobot::Flag::exportAsImage);
    return true;
  }
  else
  {
    std::string buffer2;
    bool jpeg = false;
    bool segmented = false;
    std::string thread;
    for(;;)
    {
      stream >> buffer2;
      if(!jpeg && buffer2 == "jpeg")
        jpeg = true;
      else if(!segmented && buffer2 == "segmented")
        segmented = true;
      else
      {
        if(thread.empty())
          for(const auto& config : moduleInfo.config())
            if(config.name == buffer2)
            {
              thread = buffer2;
              goto found;
            }
        break;
      found: ;
      }
    }

    if(thread.empty())
      thread = "Lower";

    if(buffer == "image")
    {
      std::string name = buffer2 != "" ? buffer2 : segmented ? "segmented" + thread : static_cast<char>(thread[0] | 0x20) + thread.substr(1);
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
      ctrl->addView(new ImageView(robotName + ".image." + name.c_str(), *this, "raw image", name, segmented, thread, gain),
                    robotName + ".image", SimRobot::Flag::copy | SimRobot::Flag::exportAsImage);
      if(jpeg)
        handleConsole("dr representation:JPEGImage on");
      else
        handleConsole("dr representation:CameraImage on");

      return true;
    }
    else if(!jpeg)
      for(const auto& i : debugRequestTable.slowIndex)
        if(i.first.substr(0, 13) == "debug images:" &&
           ctrl->translate(i.first.substr(13)) == buffer)
        {
          std::string name = buffer2 != "" ? buffer2 : std::string(buffer) + (segmented ? "Segmented" : "") + thread;
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
          ctrl->addView(new ImageView(robotName + ".image." + name.c_str(), *this, i.first.substr(13), name, false, thread, gain, ddScale),
                        robotName + ".image", SimRobot::Flag::copy | SimRobot::Flag::exportAsImage);
          handleConsole(std::string("dr ") + ctrl->translate(i.first) + " on");
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
    for(const auto& viewPair : imageViews)
      ctrl->list(viewPair.first, view);
    ctrl->printLn("");
    return true;
  }
  ImageViewCommand command;
  std::vector<std::pair<std::string, Qt::KeyboardModifier>> modifiers = { { "alt", Qt::AltModifier }, { "ctrl", Qt::ControlModifier }, { "shift", Qt::ShiftModifier } };
  std::string text;
  stream >> text;
  for(auto& modifier : modifiers)
  {
    if(text == modifier.first)
    {
      command.modifiers |= modifier.second;
      command.modifierMask |= modifier.second;
      stream >> text;
    }
    else if(text == "no" + modifier.first)
    {
      command.modifierMask |= modifier.second;
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
  std::string::size_type pos = line.find("$");
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
    pos = line.find("$", prevPos);
  }
  if(prevPos != line.length())
    command.tokens.emplace_back(line.substr(prevPos));

  bool all = view == "all";
  for(const auto& viewPair : imageViews)
    if(viewPair.first == view || all)
    {
      auto& commands = imageViewCommands[viewPair.first];
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

bool RobotConsole::viewPlot(In& stream)
{
  std::string name;
  int plotSize;
  float minValue, maxValue;
  std::string yUnit, xUnit;
  float xScale;
  stream >> name >> plotSize >> minValue >> maxValue >> yUnit >> xUnit >> xScale;
  if(plotSize < 2 || minValue >= maxValue)
    return false;
  if(xScale == 0.)
    xScale = 1.;
  QString fullName = robotName + ".plot." + name.c_str();
  if(static_cast<unsigned int>(plotSize) > maxPlotSize)
    maxPlotSize = plotSize;
  if(plotViews.find(name) != plotViews.end())
  {
    PlotView* plotView = static_cast<PlotView*>(ctrl->application->resolveObject(fullName));
    ASSERT(plotView);
    plotView->setParameters(static_cast<unsigned int>(plotSize), minValue, maxValue, yUnit, xUnit, xScale);
  }
  else
  {
    plotViews[name];
    ctrl->setPlotViews(plotViews);
    ctrl->updateCommandCompletion();
    ctrl->addView(new PlotView(fullName, *this, name, static_cast<unsigned int>(plotSize), minValue, maxValue, yUnit, xUnit, xScale), robotName + ".plot", SimRobot::Flag::copy | SimRobot::Flag::exportAsImage);
  }
  return true;
}

bool RobotConsole::kickView()
{
  if(kickViewSet)
    ctrl->printLn("View already exists.");
  else
  {
    if(mode == SystemCall::simulatedRobot)
    {
      kickViewSet = true;
      ctrl->addView(new KickView(robotName + ".KickView", *this, motionRequest, jointSensorData, jointLimits, robotDimensions, printBuffer,
                                 (SimRobotCore2::Body*)ctrl->application->resolveObject(robotFullName, SimRobotCore2::body)), robotName);
      return true;
    }
    if(mode == SystemCall::remoteRobot)
    {
      kickViewSet = true;
      QString puppetName("RoboCup.puppets." + robotName);

      ctrl->addView(new KickView(robotName + ".KickView", *this, motionRequest, jointSensorData, jointLimits, robotDimensions, printBuffer,
                                 (SimRobotCore2::Body*)ctrl->application->resolveObject(puppetName, SimRobotCore2::body)), robotName);
      return true;
    }
  }
  return false;
}

void RobotConsole::sendDebugMessage(InMessage& msg)
{
  SYNC;
  msg >> *debugSender;
}

std::string RobotConsole::getDebugRequest(const std::string& name)
{
  SYNC;
  for(const auto& i : debugRequestTable.slowIndex)
  {
    if(std::string("debugData:") + name == ctrl->translate(i.first))
    {
      return i.first.substr(11);
    }
  }
  printLn("Error: RobotConsole: DebugRequest not found.");
  return "";
}

bool RobotConsole::viewPlotDrawing(In& stream)
{
  std::string buffer;
  stream >> buffer;
  if(buffer == "?")
  {
    stream >> buffer;
    for(const auto& plotPair : plotViews)
      ctrl->list(plotPair.first.c_str(), buffer);
    ctrl->printLn("");
    return true;
  }
  else
    for(const auto& plotPair : plotViews)
      if(plotPair.first == buffer)
      {
        stream >> buffer;
        if(buffer == "?")
        {
          stream >> buffer;
          for(const auto& i : debugRequestTable.slowIndex)
            if(i.first.substr(0, 5) == "plot:")
              ctrl->list(ctrl->translate(i.first.substr(5)).c_str(), buffer);
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
                  ctrl->list(ctrl->translate(TypeRegistry::getEnumName(color)).c_str(), buffer);
                ctrl->printLn("");
                return true;
              }
              if(buffer == "off")
              {
                {
                  SYNC;
                  plotViews[plotPair.first].remove(layer);
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
                  plotViews[plotPair.first].remove(layer);
                  plotViews[plotPair.first].push_back(layer);
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

bool RobotConsole::joystickExecCommand(const std::string& cmd)
{
  if(cmd == "")
    return false;

  ctrl->executeConsoleCommand(cmd, this);
  if(joystickTrace)
    ctrl->printLn(cmd);

  return true;
}

void RobotConsole::handleJoystick()
{
  if(!joystick.update())
    return; //return if no joystick was found

  // handle joystick events
  unsigned int buttonId;
  bool pressed;
  bool buttonCommandExecuted(false);
  while(joystick.getNextEvent(buttonId, pressed))
  {
    ASSERT(buttonId < Joystick::numOfButtons);
    buttonCommandExecuted |= joystickExecCommand(pressed ? joystickButtonPressCommand[buttonId] : joystickButtonReleaseCommand[buttonId]);
    if(!pressed)
      for(int j = 0; j < joystickNumOfMotionCommands; ++j)
        joystickMotionCommands[j].lastCommand = "";
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
    for(int j = 0; j < joystickNumOfMotionCommands; ++j)
    {
      JoystickMotionCommand& cmd(joystickMotionCommands[j]);
      if(!cmd.command.empty())
      {
        if(!preparedSpeeds)
        {
          for(int i = 0; i < Joystick::numOfAxes; ++i)
          {
            float d = joystick.getAxisState(i);
            float threshold = joystickAxisThresholds[i];
            if(d < -threshold)
              speeds[i] = (d + threshold) / (1 - threshold);
            else if(d > threshold)
              speeds[i] = (d - threshold) / (1 - threshold);
            else
              speeds[i] = 0;
            if(joystickAxisMappings[i])
            {
              bool pressed1 = joystick.isButtonPressed(joystickAxisMappings[i] & 0xffff);
              bool pressed2 = joystick.isButtonPressed(joystickAxisMappings[i] >> 16);
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
          joystickExecCommand(&joystickCommandBuffer[0]);
          cmd.lastCommand = &joystickCommandBuffer[0];
        }
      }
    }
  }
}

bool RobotConsole::joystickCommand(In& stream)
{
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
      for(int i = 0; i < Joystick::numOfAxes; ++i)
        cmd.indices[i] = 0;
      std::string::size_type pos = line.find("$");
      int i = 0;
      while(i < Joystick::numOfAxes && pos != std::string::npos)
      {
        int id = line[pos + 1] - '1';
        if(id >= 0 && id < Joystick::numOfAxes)
        {
          cmd.indices[i++] = id;
          line.replace(pos, 2, "%lf");
          pos = line.find("$");
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

bool RobotConsole::joystickSpeeds(In& stream)
{
  int id;
  stream >> id;
  if(id > 0 && id <= Joystick::numOfAxes)
  {
    stream >> joystickAxisMaxSpeeds[id - 1] >> joystickAxisThresholds[id - 1] >> joystickAxisCenters[id - 1];
    return true;
  }
  return false;
}

bool RobotConsole::joystickMaps(In& stream)
{
  int axis, button1, button2;
  stream >> axis >> button1 >> button2;
  if(axis > 0 && axis <= Joystick::numOfAxes && button1 >= 0 && button1 <= Joystick::numOfButtons && button2 > 0 && button2 <= Joystick::numOfButtons)
  {
    joystickAxisMappings[axis - 1] = button1 == 0 ? 0 : ((button1 - 1) | ((button2 - 1) << 16));
    return true;
  }
  return false;
}

bool RobotConsole::saveRequest(In& stream, bool first)
{
  std::string buffer;
  std::string path;
  stream >> buffer >> path;

  if(buffer == "?")
  {
    for(auto& entry : ctrl->representationToFile)
      ctrl->list(entry.first.c_str(), path);
    ctrl->printLn("");
    return true;
  }
  else
  {
    for(const auto& i : debugRequestTable.slowIndex)
      if(std::string("debugData:") + buffer == ctrl->translate(i.first))
      {
        if(first) // request current Values
        {
          SYNC;
          debugSender->out.bin << DebugRequest(i.first, true);
          debugSender->out.finishMessage(idDebugRequest);

          waitingFor[idDebugDataResponse] = 1;
          polled[idDebugDataResponse] = true; // no automatic repolling
          getOrSetWaitsFor = i.first.substr(11);
          handleConsole(std::string("_save ") + buffer + " " + path);
          return true;
        }
        else
        {
          DebugDataInfos::const_iterator j = debugDataInfos.find(i.first.substr(11));
          ASSERT(j != debugDataInfos.end());

          std::string filename = path;
          if(filename == "") // no path specified, use default location
          {
            filename = getPathForRepresentation(i.first.substr(11));
            if(filename == "")
            {
              ctrl->printLn("Error getting filename for " + i.first.substr(11) + ". Representation can not be saved.");
              return true;
            }
          }
          OutMapFile file(filename, true);
          MapWriter writer(typeInfo, file);
          j->second.second->handleAllMessages(writer);
          return true;
        }
      }
    return false;
  }
}

bool RobotConsole::saveImage(In& stream)
{
  Image<PixelTypes::YUYVPixel>::ExportMode exportMode = Image<PixelTypes::YUYVPixel>::rgb;
  std::string buffer;
  stream >> buffer;
  if(buffer == "reset")
  {
    stream >> imageSaveNumber;
    return true;
  }
  else
  {
    if(buffer != "upper" && buffer != "lower")
      return false;
    std::string camera = static_cast<char>(buffer[0] & ~0x20) + buffer.substr(1);

    std::string filename;
    stream >> filename;
    int number = -1;
    if(filename == "number")
    {
      number = imageSaveNumber++;
      stream >> filename;
    }
    if(filename == "grayscale")
    {
      exportMode = Image<PixelTypes::YUYVPixel>::grayscale;
      stream >> filename;
    }

    SYNC;
    DebugImage* srcImage;
    if(threadData[camera].images.find("raw image") != threadData[camera].images.end())
      srcImage = threadData[camera].images["raw image"].image;
    else
      return false;

    int left = 0, top = 0, right = srcImage->width * 2, bottom = srcImage->height;
    if(filename == "region")
    {
      stream >> left >> top >> right >> bottom;
      stream >> filename;
    }
    if(filename == "")
      filename = "raw_image.bmp";

    if(srcImage)
      return ImageWrapper<PixelTypes::YUYVPixel>(srcImage->width, srcImage->height,
                                                 static_cast<PixelTypes::YUYVPixel*>(srcImage->data))
             .exportImage(filename.c_str(), left, top, right, bottom, number, exportMode);
    return false;
  }
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
  if(key >= 0 && key < Joystick::numOfButtons && joystickButtonCommand[key] != "")
    ctrl->executeConsoleCommand(joystickButtonCommand[key], this);
}

std::string RobotConsole::getPathForRepresentation(const std::string& representation)
{
  std::string fileName;
  std::unordered_map<std::string, std::string>::const_iterator i = ctrl->representationToFile.find(representation);
  if(i == ctrl->representationToFile.end())
    return "";
  fileName = i->second;

  for(const std::string& name : File::getFullNames(fileName))
  {
    File path(name, "r", false);
    if(path.exists())
      return name;
  }

  // if file is not anywhere else, return config directory as default directory
  return fileName;
}

void RobotConsole::saveColorCalibration()
{
  SYNC;
  std::string name = "fieldColors.cfg";
  for(std::string& fullName : File::getFullNames(name))
  {
    File path(fullName, "r", false);
    if(path.exists())
    {
      name = std::move(fullName);
      break;
    }
  }
  OutMapFile stream(name, true);
  stream << colorCalibration;
}

void RobotConsole::requestDebugData(const std::string& name, bool enable)
{
  SYNC;
  DebugRequest d("debug data:" + name, enable);
  debugRequestTable.addRequest(d);
  if(enable)
  {
    threadsOfDebugData[name] = "";
    debugSender->out.bin << d;
    debugSender->out.finishMessage(idDebugRequest);
  }
}

void RobotConsole::sendModuleRequest()
{
  if(moduleRequestChanged)
  {
    SYNC;
    moduleInfo.timestamp = Time::getCurrentSystemTime() + ++mrCounter;
    debugSender->out.bin << moduleInfo.timestamp;
    moduleInfo.sendRequest(debugSender->out.bin);
    debugSender->out.finishMessage(idModuleRequest);
    polled[idDebugResponse] = polled[idDrawingManager] = polled[idDrawingManager3D] = false;
    logPlayer.typeInfoReplayed = false;
    moduleRequestChanged = false;
  }
}

void RobotConsole::updateAnnotationsFromLog()
{
  struct Handler : public MessageHandler
  {
    std::unordered_map<std::string, ThreadData>& threadData;
    unsigned currentFrame = 0;
    std::string threadIdentifier;

  public:
    Handler(std::unordered_map<std::string, ThreadData>& threadData)
      : threadData(threadData) {}

    bool handleMessage(InMessage& message)
    {
      switch(message.getMessageID())
      {
        case idFrameBegin:
          threadIdentifier = message.readThreadIdentifier();
          ++currentFrame;
          return true;
        case idAnnotation:
          threadData[threadIdentifier].annotationInfo.addMessage(message, currentFrame);
          return true;
        default:
          return false;
      }
    }
  } handler(threadData);

  for(auto& data : threadData)
    data.second.annotationInfo.clear();
  logPlayer.handleAllMessages(handler);
}
