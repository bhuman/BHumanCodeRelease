/**
 * @file Controller/RobotConsole.cpp
 *
 * Implementation of RobotConsole.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
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
#include "Tools/Debugging/QueueFillRequest.h"
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

RobotConsole::RobotConsole(MessageQueue& in, MessageQueue& out) :
  Process(in, out),
  logPlayer(out),
  logExtractor(logPlayer),
  debugOut(out),
  dataViewWriter(&dataViews)
{
  // this is a hack: call global functions to get parameters
  ctrl = (ConsoleRoboCupCtrl*)RoboCupCtrl::controller;
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
  logPlayer.setSize(std::numeric_limits<unsigned>::max()); // max. 4 GB

  currentImages = &lowerCamImages;
  currentImageDrawings = &lowerCamImageDrawings;
  currentFieldDrawings = &lowerCamFieldDrawings;
  currentDrawings3D = &lowerCamDrawings3D;
  timeInfos['c'] = TimeInfo("Upper", 2);
  timeInfos['d'] = TimeInfo("Lower", 2);
  timeInfos['b'] = TimeInfo("Cognition", 1);
  timeInfos['m'] = TimeInfo("Motion", 1);
}

RobotConsole::~RobotConsole()
{
  SYNC;
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
  joystick.init();
}

void RobotConsole::addViews()
{
  SimRobot::Object* category = ctrl->addCategory(robotName, 0, ":/Icons/SimRobot.png");

  SimRobot::Object* annotationCategory = ctrl->addCategory("annotations", category);
  {
    auto* cognitionAnnotationView = new AnnotationView(robotName + ".annotations.cognition", annotationInfos.annotationProcesses['c'], logPlayer, mode, ctrl->application);
    auto* motionAnnotationView = new AnnotationView(robotName + ".annotations.motion", annotationInfos.annotationProcesses['m'], logPlayer, mode, ctrl->application);
    annotationInfos.annotationProcesses['c'].view = cognitionAnnotationView;
    annotationInfos.annotationProcesses['m'].view = motionAnnotationView;
    ctrl->addView(cognitionAnnotationView, annotationCategory);
    ctrl->addView(motionAnnotationView, annotationCategory);
  }

  ctrl->addView(new CABSLBehaviorView(robotName + ".behavior", *this, activationGraph, activationGraphReceived), category);

  colorCalibrationView = new ColorCalibrationView(robotName + ".colorCalibration", *this);
  ctrl->addView(colorCalibrationView, category);
  colorCalibrationChanged = false;

  ctrl->addCategory("colorSpace", ctrl->application->resolveObject(robotName));
  ctrl->addCategory("data", ctrl->application->resolveObject(robotName));
  ctrl->addCategory("field", ctrl->application->resolveObject(robotName));
  ctrl->addView(new JointView(robotName + ".jointData", *this, jointSensorData, jointRequest), category);
  if(mode == SystemCall::logfileReplay)
    ctrl->addView(new LogPlayerControlView(robotName + ".logPlayer", logPlayer, *this), category);
  SimRobot::Object* modulesCategory = ctrl->addCategory("modules", category);
  SimRobot::Object* cognitionCategory = ctrl->addCategory("cognition", modulesCategory);
  ctrl->addView(new ModuleGraphViewObject(robotName + ".modules.cognition.all", *this, 'c'), cognitionCategory);
  ctrl->addView(new ModuleGraphViewObject(robotName + ".modules.cognition.behaviorControl", *this, 'c', ModuleBase::behaviorControl), cognitionCategory);
  ctrl->addView(new ModuleGraphViewObject(robotName + ".modules.cognition.communication", *this, 'c', ModuleBase::communication), cognitionCategory);
  ctrl->addView(new ModuleGraphViewObject(robotName + ".modules.cognition.infrastructure", *this, 'c', ModuleBase::cognitionInfrastructure), cognitionCategory);
  ctrl->addView(new ModuleGraphViewObject(robotName + ".modules.cognition.modeling", *this, 'c', ModuleBase::modeling), cognitionCategory);
  ctrl->addView(new ModuleGraphViewObject(robotName + ".modules.cognition.perception", *this, 'c', ModuleBase::perception), cognitionCategory);
  SimRobot::Object* motionCategory = ctrl->addCategory("motion", modulesCategory);
  ctrl->addView(new ModuleGraphViewObject(robotName + ".modules.motion.all", *this, 'm'), motionCategory);
  ctrl->addView(new ModuleGraphViewObject(robotName + ".modules.motion.infrastructure", *this, 'm', ModuleBase::motionInfrastructure), motionCategory);
  ctrl->addView(new ModuleGraphViewObject(robotName + ".modules.motion.motionControl", *this, 'm', ModuleBase::motionControl), motionCategory);
  ctrl->addView(new ModuleGraphViewObject(robotName + ".modules.motion.sensing", *this, 'm', ModuleBase::sensing), motionCategory);
  ctrl->addCategory("plot", ctrl->application->resolveObject(robotName));

  ctrl->addView(new SensorView(robotName + ".sensorData", *this, fsrSensorData, inertialSensorData, keyStates, systemSensorData, sensorDataTimeStamp), category);

  SimRobot::Object* timingCategory = ctrl->addCategory("timing", category);
  ctrl->addView(new TimeView(robotName + ".timing.upper", *this, timeInfos.at('c')), timingCategory);
  ctrl->addView(new TimeView(robotName + ".timing.lower", *this, timeInfos.at('d')), timingCategory);
  ctrl->addView(new TimeView(robotName + ".timing.cognition", *this, timeInfos.at('b')), timingCategory);
  ctrl->addView(new TimeView(robotName + ".timing.motion", *this, timeInfos.at('m')), timingCategory);
}

void RobotConsole::addColorSpaceViews(const std::string& id, const std::string& name, bool user, bool upperCam)
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
                        *this, id, ColorSpaceView::YCbCr, channel + 3, background, upperCam), modelCategory);
      }
    }
    else
    {
      for(int channel = 0; channel < 4; ++channel)
      {
        ctrl->addView(new ColorSpaceView(
                        modelCategoryName + "." + ColorSpaceView::getChannelNameForColorModel(ColorSpaceView::ColorModel(cm), channel),
                        *this, id, ColorSpaceView::ColorModel(cm), channel + (cm == ColorSpaceView::YCbCr&& channel ? 1 : 0), background, upperCam), modelCategory);
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
    if(logImagesAsJPEGs && message.getMessageID() == idImage)
    {
      Image image;
      message.bin >> image;
      MessageQueue queue;
      queue.out.bin << JPEGImage(image);
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
    case idImage:
    {
      Image i(false);
      message.bin >> i;
      if(incompleteImages["raw image"].image)
        incompleteImages["raw image"].image->from(i);
      else
        incompleteImages["raw image"].image = new DebugImage(i, true);
      return true;
    }
    case idJPEGImage:
    {
      Image i(false);
      JPEGImage jpi;
      message.bin >> jpi;
      jpi.toImage(i);
      if(incompleteImages["raw image"].image)
        incompleteImages["raw image"].image->from(i);
      else
        incompleteImages["raw image"].image = new DebugImage(i, true);
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
        TImage<PixelTypes::YUYVPixel> i;
        thumbnail.toYUYV(i);
        if(incompleteImages["raw image"].image)
          incompleteImages["raw image"].image->from(i);
        else
          incompleteImages["raw image"].image = new DebugImage(i, true);
      }
      incompleteImages["raw image"].image->timeStamp = Time::getCurrentSystemTime();
      return true;
    }
    case idDebugImage:
    {
      std::string id;
      message.bin >> id;
      if(!incompleteImages[id].image)
        incompleteImages[id].image = new DebugImage();
      message.bin >> *incompleteImages[id].image;
      incompleteImages[id].image->timeStamp = Time::getCurrentSystemTime();
      break;
    }
    case idFsrSensorData:
    {
      message.bin >> fsrSensorData;
      sensorDataTimeStamp = Time::getCurrentSystemTime();
      return true;
    }
    case idInertialSensorData:
    {
      message.bin >> inertialSensorData;
      sensorDataTimeStamp = Time::getCurrentSystemTime();
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
      sensorDataTimeStamp = Time::getCurrentSystemTime();
      return true;
    }
    case idDebugDrawing:
    {
      if(polled[idDrawingManager] && !waitingFor[idDrawingManager]) // drawing manager not up-to-date
      {
        char shapeType, id;
        message.bin >> shapeType >> id;
        const char* name = drawingManager.getDrawingName(id); // const char* is required here
        std::string type = drawingManager.getDrawingType(name);

        if(type == "drawingOnImage")
          incompleteImageDrawings[name].addShapeFromQueue(message, (::Drawings::ShapeType)shapeType);
        else if(type == "drawingOnField")
          incompleteFieldDrawings[name].addShapeFromQueue(message, (::Drawings::ShapeType)shapeType);
      }
      return true;
    }
    case idDebugDrawing3D:
    {
      if(polled[idDrawingManager3D] && !waitingFor[idDrawingManager3D])
      {
        char shapeType, id;
        message.bin >> shapeType >> id;
        incompleteDrawings3D[drawingManager3D.getDrawingName(id)].addShapeFromQueue(message, (::Drawings3D::ShapeType)shapeType, processIdentifier);
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
      plot.timeStamp = Time::getCurrentSystemTime();
      return true;
    }
    case idProcessBegin:
    {
      message.bin >> processIdentifier;
      drawingManager.setProcess(processIdentifier == 'd' ? 'c' : processIdentifier);
      drawingManager3D.setProcess(processIdentifier == 'd' ? 'c' : processIdentifier);
      message.resetReadPosition();
      annotationInfos.handleMessage(message);
      return true;
    }
    case idProcessFinished:
    {
      char c;
      message.bin >> c;
      ASSERT(processIdentifier == c);

      if(processIdentifier != 'm')
      {
        if(processIdentifier == 'c')
        {
          currentImages = &upperCamImages;
          currentImageDrawings = &upperCamImageDrawings;
          currentFieldDrawings = &upperCamFieldDrawings;
          currentDrawings3D = &upperCamDrawings3D;
        }
        else //processIdentifier == 'd'
        {
          currentImages = &lowerCamImages;
          currentImageDrawings = &lowerCamImageDrawings;
          currentFieldDrawings = &lowerCamFieldDrawings;
          currentDrawings3D = &lowerCamDrawings3D;
        }

        currentImages->clear();
        for(auto& pair : incompleteImages)
        {
          ImagePtr& imagePtr = (*currentImages)[pair.first];
          imagePtr.image = pair.second.image;
          imagePtr.processIdentifier = processIdentifier;
          pair.second.image = nullptr;
        }
      }
      else //processIdentifier == 'm'
      {
        currentImageDrawings = &motionImageDrawings;
        currentFieldDrawings = &motionFieldDrawings;
        currentDrawings3D = &motionDrawings3D;
      }

      // Add new Field and Image drawings
      currentImageDrawings->clear();
      currentFieldDrawings->clear();
      if(processIdentifier == 'm' || drawingsViaProcess == 'b' || drawingsViaProcess == processIdentifier)
      {
        for(const auto& pair : incompleteImageDrawings)
        {
          DebugDrawing& debugDrawing = (*currentImageDrawings)[pair.first];
          debugDrawing = pair.second;
        }
        for(const auto& pair : incompleteFieldDrawings)
        {
          DebugDrawing& debugDrawing = (*currentFieldDrawings)[pair.first];
          debugDrawing = pair.second;
        }
      }

      // 3D Drawings
      if(polled[idDrawingManager3D] && !waitingFor[idDrawingManager3D])
      {
        // reset all 3d drawings originated from current process
        for(auto& pair : *currentDrawings3D)
        {
          DebugDrawing3D& debugDrawing3D = pair.second;
          debugDrawing3D.reset();
        }

        // copy and register newly received 3d debug drawings
        if(processIdentifier == 'm' || drawingsViaProcess == 'b' || drawingsViaProcess == processIdentifier)
        {
          for(auto& pair : incompleteDrawings3D)
          {
            std::string type = drawingManager3D.getDrawingType(drawingManager3D.getString(pair.first));
            std::string name = type == "camera" ? pair.first + processIdentifier : pair.first;
            DebugDrawing3D& debugDrawing3D = (*currentDrawings3D)[name];
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
                  parts.append(processIdentifier == 'c' ? "CameraTop" : "CameraBottom");
                else
                  parts.append(type.c_str());
                SYNC_WITH(*ctrl);
                SimRobotCore2::PhysicalObject* object = (SimRobotCore2::PhysicalObject*)ctrl->application->resolveObject(parts);
                object->registerDrawing(debugDrawing3D);
                debugDrawing3D.drawn = true;
              }
            }
          }
        }
      }

      incompleteImages.clear();
      incompleteImageDrawings.clear();
      incompleteFieldDrawings.clear();
      incompleteDrawings3D.clear();
      message.resetReadPosition();
      annotationInfos.handleMessage(message);

      return true;
    }
    case idActivationGraph:
      message.bin >> activationGraph;
      activationGraphReceived = Time::getCurrentSystemTime();
      return true;
    case idAnnotation:
      ASSERT(annotationInfos.annotationProcesses.find(processIdentifier == 'd' ? 'c' : processIdentifier) != annotationInfos.annotationProcesses.end());
      annotationInfos.handleMessage(message);
      return true;
    case idStopwatch:
      ASSERT(timeInfos.find(processIdentifier) != timeInfos.end());
      timeInfos.at(processIdentifier).handleMessage(message);
      if(processIdentifier != 'm')
      {
        message.resetReadPosition();
        timeInfos.at('c' + 'd' - processIdentifier).handleMessage(message, true);
        message.resetReadPosition();
        timeInfos.at('b').handleMessage(message);
      }
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
      moduleInfo.handleMessage(message, processIdentifier == 'd' ? 'c' : processIdentifier);
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
      message.bin >> drawingManager;
      if(--waitingFor[idDrawingManager] <= 0)
      {
        ctrl->setDrawingManager(drawingManager);
        updateCompletion = true;
      }
      return true;
    }
    case idDrawingManager3D:
    {
      SYNC_WITH(*ctrl);
      message.bin >> drawingManager3D;
      if(--waitingFor[idDrawingManager3D] <= 0)
      {
        ctrl->setDrawingManager3D(drawingManager3D);
        updateCompletion = true;
      }
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
      if(!processesOfDebugData[name] || (processesOfDebugData[name] != 'm') == (processIdentifier != 'm'))
      {
        processesOfDebugData[name] = processIdentifier;
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
        else
        {
          // no, representation view requested it
          for(const auto& i : debugRequestTable.slowIndex)
            if("debug data:" + name == i.first)
            {
              if(debugRequestTable.enabled[i.second]) // still enabled?
              {
                // then request it again for the next update
                debugOut.out.bin << DebugRequest("debug data:" + name, true);
                debugOut.out.finishMessage(idDebugRequest);
              }
              break;
            }
        }
      }

      return true;
    }
    case idLogResponse:
      logAcknowledged = true;
      return true;
    case idMotionRequest:
      message.bin >> motionRequest;
      return true;
    case idRobotname:
    {
      message.bin >> Global::getSettings().headName
                  >> Global::getSettings().bodyName
                  >> Global::getSettings().location;
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
  setGlobals(); // this is called in GUI thread -> set globals for this process
  handleJoystick();

  if(colorCalibrationChanged)
  {
    SYNC;
    colorCalibrationChanged = false;
    colorCalibrationTimeStamp = Time::getCurrentSystemTime();
    debugOut.out.bin << colorCalibration;
    debugOut.out.finishMessage(idColorCalibration);
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
  setGlobals(); // this is called in GUI thread -> set globals for this process
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

void RobotConsole::triggerProcesses()
{
  if(mode == SystemCall::logfileReplay)
  {
    SYNC;
    debugOut.out.bin << 'c';
    debugOut.out.finishMessage(idProcessBegin);
    debugOut.out.bin << 'c';
    debugOut.out.finishMessage(idProcessFinished);
    debugOut.out.bin << 'm';
    debugOut.out.finishMessage(idProcessBegin);
    debugOut.out.bin << 'm';
    debugOut.out.finishMessage(idProcessFinished);
  }
}

bool RobotConsole::poll(MessageID id)
{
  if(waitingFor[id] > 0)
  {
    // When in replay log file mode, force replay while polling to keep Cognition running
    triggerProcesses();
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
        debugOut.out.bin << DebugRequest("poll");
        debugOut.out.finishMessage(idDebugRequest);
        waitingFor[id] = 3; // Cognition + Motion + Debug will answer
        break;
      }
      case idModuleTable:
      {
        SYNC;
        moduleInfo.clear();
        debugOut.out.bin << DebugRequest("automated requests:ModuleTable", true);
        debugOut.out.finishMessage(idDebugRequest);
        waitingFor[id] = 2;  // Cognition + Motion will answer
        break;
      }
      case idTypeInfo:
      {
        SYNC;
        debugOut.out.bin << DebugRequest("automated requests:TypeInfo", true);
        debugOut.out.finishMessage(idDebugRequest);
        waitingFor[id] = 1;  // Debug will answer
        break;
      }
      case idDrawingManager:
      {
        SYNC;
        drawingManager.clear();
        debugOut.out.bin << DebugRequest("automated requests:DrawingManager", true);
        debugOut.out.finishMessage(idDebugRequest);
        waitingFor[id] = 2; // Cognition + Motion will answer
        break;
      }
      case idDrawingManager3D:
      {
        SYNC;
        drawingManager3D.clear();
        debugOut.out.bin << DebugRequest("automated requests:DrawingManager3D", true);
        debugOut.out.finishMessage(idDebugRequest);
        waitingFor[id] = 2;  // Cognition + Motion will answer
        break;
      }
      case idFieldColors:
      {
        SYNC;
        debugOut.out.bin << DebugRequest("representation:FieldColors:once", true);
        debugOut.out.finishMessage(idDebugRequest);
        waitingFor[id] = 1;  // Cognition will answer
        break;
      }
      case idRobotname:
      {
        SYNC;
        debugOut.out.bin << DebugRequest("module:NaoProvider:robotName", true);
        debugOut.out.finishMessage(idDebugRequest);
        waitingFor[id] = 1;  // Motion will answer
        break;
      }
      case idRobotInfo:
      {
        SYNC;
        debugOut.out.bin << DebugRequest("module:NaoProvider:robotInfo", true);
        debugOut.out.finishMessage(idDebugRequest);
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
  else if(command == "ac")
    result = acceptCamera(stream);
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
    result = repoll(stream);
  else if(command == "pr" && mode == SystemCall::simulatedRobot)
    result = ctrl->gameController.handleRobotConsole(robotName.mid(5).toInt() - 1, stream);
  else if(command == "qfr")
    result = queueFillRequest(stream);
  else if(command == "save")
  {
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
    PREREQUISITE(idDebugResponse);
    PREREQUISITE(idDrawingManager);
    result = viewField(stream);
  }
  else if(command == "vfd")
  {
    PREREQUISITE(idDebugResponse);
    PREREQUISITE(idDrawingManager);
    result = viewDrawing(stream, fieldViews, "drawingOnField");
  }
  else if(command == "vd") //view data part 1
  {
    PREREQUISITE(idDebugResponse);
    PREREQUISITE(idTypeInfo);
    result = viewData(stream);
  }
  else if(command == "vp")
  {
    PREREQUISITE(idDebugResponse);
    result = viewPlot(stream);
  }
  else if(command == "vpd")
  {
    PREREQUISITE(idDebugResponse);
    result = viewPlotDrawing(stream);
  }
  else if(command == "vid")
  {
    PREREQUISITE(idDebugResponse);
    PREREQUISITE(idDrawingManager);
    result = viewDrawing(stream, imageViews, "drawingOnImage");
  }
  else if(command == "vic")
    result = viewImageCommand(stream);
  else if(command == "vi")
  {
    PREREQUISITE(idDebugResponse);
    PREREQUISITE(idDrawingManager);
    result = viewImage(stream);
  }
  else if(command == "v3")
  {
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
      if((int)name.rfind('.') <= (int)name.find_last_of("\\/"))
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
      debugOut.out.bin << DebugRequest("disableAll");
      debugOut.out.finishMessage(idDebugRequest);
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
          debugOut.out.bin << DebugRequest(i.first, debugRequestTable.enabled[i.second]);
          debugOut.out.finishMessage(idDebugRequest);
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
    annotationInfos.clear(); \
    logPlayer.handleAllMessages(annotationInfos); \
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
    std::string name;
    std::string replacement;
    stream >> name >> replacement;
    if(name.empty() || (logFile.empty() && !replacement.empty()))
      return false;
    else
    {
      if(!replacement.empty())
        name = QString(logFile.c_str()).replace(name.c_str(), replacement.c_str()).toUtf8().constData();
      if((int)name.rfind('.') <= (int)name.find_last_of("\\/"))
        name = name + ".log";
      if(name[0] != '/' && name[0] != '\\' && (name.size() < 2 || name[1] != ':'))
        name = std::string("Logs\\") + name;
      SYNC;
      return logExtractor.save(name, logFile.empty() ? &typeInfo : nullptr);
    }
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

    return logExtractor.saveImages(command, raw, onlyPlaying);
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
    else if(command == "saveWalkingData")
      return logExtractor.saveWalkingData(name);
    else if(command == "saveGetUpEngineFailData")
      return logExtractor.saveGetUpEngineFailData(name);
    else if(command == "saveTiming")
      return logExtractor.writeTimingData(name);
  }
  else if(command == "statistics")
  {
    SYNC;
    if(!logPlayer.logfileMerged)
    {
      logPlayer.merge();
      handleConsole("log mr");
    }
    return logExtractor.statistics(ctrl->statistics);
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

      annotationInfos.clear();
      logPlayer.handleAllMessages(annotationInfos);

      return true;
    }
    else if(command == "keep" && buf == "image")
    {
      logPlayer.keepFrames([&](InMessage& message) -> bool
      {
        return message.getMessageID() == idLowFrameRateImage && message.getMessageSize() > 1000;
      });

      annotationInfos.clear();
      logPlayer.handleAllMessages(annotationInfos);

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

      annotationInfos.clear();
      logPlayer.handleAllMessages(annotationInfos);

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
               message.getMessageID() == idProcessBegin ||
               message.getMessageID() == idProcessFinished)
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

        annotationInfos.clear();
        logPlayer.handleAllMessages(annotationInfos);

        return true;
      }
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
  else if(command == "merge")
  {
    SYNC;
    logPlayer.merge();
    annotationInfos.clear();
    logPlayer.handleAllMessages(annotationInfos);
    handleConsole("log mr");
    return true;
  }
  else if(command == "mr") //log mr
  {
    SYNC;
    std::string param;
    stream >> param;

    int upperFrequencies[numOfDataMessageIDs];
    int lowerFrequencies[numOfDataMessageIDs];
    int motionFrequencies[numOfDataMessageIDs];
    logPlayer.statistics(upperFrequencies, nullptr, 'c');
    logPlayer.statistics(lowerFrequencies, nullptr, 'd');
    logPlayer.statistics(motionFrequencies, nullptr, 'm');

    std::list<std::string> commands;
    const auto cLog = std::find(moduleInfo.modules.begin(), moduleInfo.modules.end(), "CognitionLogDataProvider");
    const auto mLog = std::find(moduleInfo.modules.begin(), moduleInfo.modules.end(), "MotionLogDataProvider");
    for(int i = idProcessFinished + 1; i < numOfDataMessageIDs; ++i)
    {
      if(upperFrequencies[i] || lowerFrequencies[i] || motionFrequencies[i])
      {
        std::string representation = std::string(TypeRegistry::getEnumName(MessageID(i))).substr(2);
        if(representation == "JPEGImage" || representation == "LowFrameRateImage"
           || representation == "Thumbnail" || representation == "ImagePatches")
          representation = "Image";
        bool inCognition = cLog != moduleInfo.modules.end() &&
                           std::find(cLog->representations.begin(), cLog->representations.end(), representation) != cLog->representations.end() &&
                           (upperFrequencies[i] || lowerFrequencies[i]);
        bool inMotion = mLog != moduleInfo.modules.end() &&
                        std::find(mLog->representations.begin(), mLog->representations.end(), representation) != mLog->representations.end() &&
                        motionFrequencies[i];
        if(inCognition || inMotion)
          commands.push_back(representation + " default");
        if(inCognition)
          commands.push_back(representation + " " + cLog->name);
        if(inMotion)
          commands.push_back(representation + " " + mLog->name);
      }
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
        if((int)name.rfind('.') <= (int)name.find_last_of("\\/"))
          name = name + ".log";
        if(name[0] != '/' && name[0] != '\\' && (name.size() < 2 || name[1] != ':'))
          name = std::string("Logs\\") + name;
        logFile = name;
        LogPlayer::LogPlayerState state = logPlayer.state;
        bool result = logPlayer.open(name);
        if(result)
        {
          annotationInfos.clear();
          logPlayer.handleAllMessages(annotationInfos);
        }
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
            processesOfDebugData[request] = 0;
            debugOut.out.bin << DebugRequest(i.first, true);
            debugOut.out.finishMessage(idDebugRequest);
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
  std::map<std::string, DataView*>::const_iterator view = pDataViews->find(name);
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
          debugOut.out.bin << i.first.substr(11) << char(0);
          debugOut.out.finishMessage(idDebugDataChangeRequest);
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
              processesOfDebugData[request] = 0;
              debugOut.out.bin << DebugRequest(i.first, true);
              debugOut.out.finishMessage(idDebugRequest);
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
                debugOut.out.bin << i.first.substr(11) << char(1);
                DebugDataStreamer streamer(typeInfo, debugOut.out.bin, j->second.first, singleValue ? "value" : 0);
                stream >> streamer;
                if(errors.isEmpty())
                {
                  debugOut.out.finishMessage(idDebugDataChangeRequest);
                  setGlobals();
                  return true;
                }
                else
                  debugOut.out.cancelMessage();
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
      debugOut.out.bin << f;
    debugOut.out.finishMessage(idMotionNet);
  }
  return true;
}

bool RobotConsole::repoll(In& stream)
{
  polled[idDebugResponse] = polled[idDrawingManager] = polled[idDrawingManager3D] = false;
  return true;
}

bool RobotConsole::queueFillRequest(In& stream)
{
  std::string request;
  stream >> request;
  QueueFillRequest qfr;
  if(request == "queue")
  {
    qfr.behavior = QueueFillRequest::sendImmediately;
    qfr.filter = QueueFillRequest::sendEverything;
    qfr.target = QueueFillRequest::sendViaNetwork;
  }
  else if(request == "replace")
  {
    qfr.behavior = QueueFillRequest::sendImmediately;
    qfr.filter = QueueFillRequest::latestOnly;
    qfr.target = QueueFillRequest::sendViaNetwork;
  }
  else if(request == "reject")
  {
    qfr.behavior = QueueFillRequest::discardAll;
  }
  else if(request == "collect")
  {
    qfr.behavior = QueueFillRequest::sendAfter;
    qfr.filter = QueueFillRequest::sendEverything;
    qfr.target = QueueFillRequest::sendViaNetwork;

    stream >> qfr.timingMilliseconds;
    qfr.timingMilliseconds *= 1000;
    if(!qfr.timingMilliseconds)
      return false;
  }
  else if(request == "save")
  {
    qfr.filter = QueueFillRequest::sendEverything;
    qfr.target = QueueFillRequest::writeToStick;

    stream >> qfr.timingMilliseconds;
    qfr.timingMilliseconds *= 1000;
    if(!qfr.timingMilliseconds)
      qfr.behavior = QueueFillRequest::sendImmediately;
    else
      qfr.behavior = QueueFillRequest::sendAfter;
  }
  else
    return false;
  SYNC;
  debugOut.out.bin << qfr;
  debugOut.out.finishMessage(idQueueFillRequest);
  return true;
}

bool RobotConsole::moduleRequest(In& stream)
{
  SYNC;
  std::string representation, module, pattern;
  stream >> representation >> module >> pattern;
  if(representation == "modules")
  {
    for(const auto& m : moduleInfo.modules)
    {
      std::string text = m.name + " (" + TypeRegistry::getEnumName(m.category) + "): ";
      for(const auto& r : m.requirements)
        text += r + " ";
      text += "-> ";
      for(const auto& r : m.representations)
      {
        bool selected = false;
        for(const auto& rp : moduleInfo.config.representationProviders)
          selected |= rp.provider == m.name;
        text += r + (selected ? "* " : " ");
      }
      ctrl->list(text, module, true);
    }

    return true;
  }
  else if(representation == "?")
  {
    for(const auto& rp : moduleInfo.config.representationProviders)
    {
      std::string process = "both";
      for(const auto& m : moduleInfo.modules)
        if(m.name == rp.provider)
          process = m.processIdentifier != 'c' ? "Motion" : "Cognition";
      std::string text = rp.representation + " (" + process + "): ";
      for(const auto& m : moduleInfo.modules)
        if(std::find(m.representations.begin(), m.representations.end(), rp.representation) != m.representations.end())
          text += m.name + (m.name == rp.provider ? "* " : " ");
      text += rp.provider == "default" ? "default*" : "default";
      ctrl->list(text, module, true);
    }
    return true;
  }
  else if(representation == "save")
  {
    OutMapFile stream("modules.cfg");
    moduleInfo.sendRequest(stream, true);
    return true;
  }
  else
  {
    if(std::find(moduleInfo.representations.begin(), moduleInfo.representations.end(), representation) == moduleInfo.representations.end())
      return false;
    else if(module == "?")
    {
      std::string provider;
      for(const auto& rp : moduleInfo.config.representationProviders)
        if(rp.representation == representation)
          provider = rp.provider;
      for(const auto& m : moduleInfo.modules)
        if(std::find(m.representations.begin(), m.representations.end(), representation) != m.representations.end())
          ctrl->list(m.name + (m.name == provider ? "*" : ""), pattern);
      ctrl->list(std::string("default") + ("default" == provider ? "*" : ""), pattern);
      ctrl->printLn("");
      return true;
    }
    else
    {
      char process = 0;
      if(module != "off" && module != "default")
      {
        const auto newModule = std::find_if(moduleInfo.modules.begin(), moduleInfo.modules.end(), [&module](const ModuleInfo::Module& m) { return m.name == module; });
        if(newModule == moduleInfo.modules.end())
          return false;

        process = newModule->processIdentifier;
      }

      bool moduleIsAlreadyProvidingThisRepresentation = false;
      bool erasedSomething = false;
      for(auto i = moduleInfo.config.representationProviders.begin(); i != moduleInfo.config.representationProviders.end();)
      {
        if(i->representation == representation)
        {
          char processOld = 0;
          if(process)
            for(const auto& m : moduleInfo.modules)
              if(m.name == i->provider)
              {
                processOld = m.processIdentifier;
                break;
              }
          if(!process || !processOld || process == processOld)
          {
            if(i->provider == module)
              moduleIsAlreadyProvidingThisRepresentation = true;
            else
            {
              i = moduleInfo.config.representationProviders.erase(i);
              erasedSomething = true;
              continue;
            }
          }
        }
        ++i;
      }

      if(module != "off" && !moduleIsAlreadyProvidingThisRepresentation)
        moduleInfo.config.representationProviders.emplace_back(representation, module);

      if(!moduleIsAlreadyProvidingThisRepresentation || erasedSomething)
      {
        moduleInfo.timeStamp = Time::getCurrentSystemTime() + ++mrCounter;
        debugOut.out.bin << moduleInfo.timeStamp;
        moduleInfo.sendRequest(debugOut.out.bin);
        debugOut.out.finishMessage(idModuleRequest);
        polled[idDebugResponse] = polled[idDrawingManager] = polled[idDrawingManager3D] = false;
        logPlayer.typeInfoReplayed = false;
      }
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
    int camera = 0;
    for(;;)
    {
      stream >> buffer2;
      if(!jpeg && buffer2 == "jpeg")
        jpeg = true;
      else if(!camera && buffer2 == "upper")
        camera = 1;
      else if(!camera && buffer2 == "lower")
        camera = 2;
      else
        break;
    }

    if(buffer == "image")
    {
      std::string name = buffer2 != "" ? buffer2 : std::string(camera == 1 ? "upper" : "lower");
      if(imageViews3D.find(name) != imageViews3D.end())
      {
        ctrl->printLn("View already exists. Specify a (different) name.");
        return true;
      }
      imageViews3D[name];
      addColorSpaceViews("raw image", name, false, camera == 1);
      if(jpeg)
        handleConsole("dr representation:JPEGImage on");
      else
        handleConsole("dr representation:Image on");
      return true;
    }
    else if(!jpeg)
      for(const auto& i : debugRequestTable.slowIndex)
        if(i.first.substr(0, 13) == "debug images:" &&
           ctrl->translate(i.first.substr(13)) == buffer)
        {
          std::string name = buffer2 != "" ? buffer2 : std::string(buffer) + (camera == 1 ? "Upper" : "Lower");
          if(imageViews3D.find(name) != imageViews3D.end())
          {
            ctrl->printLn("View already exists. Specify a (different) name.");
            return true;
          }
          addColorSpaceViews(i.first.substr(13), name, true, camera == 1);
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
          for(const auto& drawingPair : drawingManager.drawings)
            if(!strcmp(drawingManager.getDrawingType(drawingPair.first), type))
              ctrl->list(ctrl->translate(drawingPair.first), command);
          ctrl->printLn("");
          return true;
        }
        else
        {
          for(const auto& drawingPair : drawingManager.drawings)
            if(ctrl->translate(drawingPair.first) == drawing && !strcmp(drawingManager.getDrawingType(drawingPair.first), type))
            {
              if(command == "on" || command == "")
              {
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
    bool upperCam = false;
    stream >> buffer;
    if(buffer == "upper")
    {
      upperCam = true;
      stream >> buffer;
    }
    else if(buffer == "lower")
      stream >> buffer;

    std::string name = buffer != "" ? buffer : upperCam ? "noneUpper" : "noneLower";
    if(imageViews.find(name) != imageViews.end())
    {
      ctrl->printLn("View already exists. Specify a (different) name.");
      return true;
    }
    imageViews[name];
    ctrl->setImageViews(imageViews);
    ctrl->updateCommandCompletion();
    ctrl->addView(new ImageView(robotName + ".image." + name.c_str(), *this, "none", name, false, upperCam), robotName + ".image", SimRobot::Flag::copy | SimRobot::Flag::exportAsImage);
    return true;
  }
  else
  {
    std::string buffer2;
    bool jpeg = false;
    bool segmented = false;
    int camera = 0;
    for(;;)
    {
      stream >> buffer2;
      if(!jpeg && buffer2 == "jpeg")
        jpeg = true;
      else if(!segmented && buffer2 == "segmented")
        segmented = true;
      else if(!camera && buffer2 == "upper")
        camera = 1;
      else if(!camera && buffer2 == "lower")
        camera = 2;
      else
        break;
    }

    if(buffer == "image")
    {
      std::string name = buffer2 != "" ? buffer2 : segmented ? (camera == 1 ? "segmentedUpper" : "segmentedLower") : camera == 1 ? "upper" : "lower";
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
      actualImageViews[name] = new ImageView(robotName + ".image." + name.c_str(), *this, "raw image", name, segmented, camera == 1, gain);
      ctrl->addView(actualImageViews[name], robotName + ".image", SimRobot::Flag::copy | SimRobot::Flag::exportAsImage);
      if(segmented)
        segmentedImageViews.emplace_back(actualImageViews[name]);
      if(jpeg)
        handleConsole("dr representation:JPEGImage on");
      else
        handleConsole("dr representation:Image on");

      return true;
    }
    else if(!jpeg)
      for(const auto& i : debugRequestTable.slowIndex)
        if(i.first.substr(0, 13) == "debug images:" &&
           ctrl->translate(i.first.substr(13)) == buffer)
        {
          std::string name = buffer2 != "" ? buffer2 : std::string(buffer) + (segmented ? "Segmented" : "") + (camera == 1 ? "Upper" : "Lower");
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
          actualImageViews[name] = new ImageView(robotName + ".image." + name.c_str(), *this, i.first.substr(13), name, false, camera == 1, gain, ddScale);
          ctrl->addView(actualImageViews[name], robotName + ".image", SimRobot::Flag::copy | SimRobot::Flag::exportAsImage);
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
  if((unsigned int)plotSize > maxPlotSize)
    maxPlotSize = plotSize;
  if(plotViews.find(name) != plotViews.end())
  {
    PlotView* plotView = (PlotView*)ctrl->application->resolveObject(fullName);
    ASSERT(plotView);
    plotView->setParameters((unsigned int)plotSize, minValue, maxValue, yUnit, xUnit, xScale);
  }
  else
  {
    plotViews[name];
    ctrl->setPlotViews(plotViews);
    ctrl->updateCommandCompletion();
    ctrl->addView(new PlotView(fullName, *this, name, (unsigned int)plotSize, minValue, maxValue, yUnit, xUnit, xScale), robotName + ".plot", SimRobot::Flag::copy | SimRobot::Flag::exportAsImage);
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
      ctrl->addView(new KickView(robotName + ".KikeView", *this, motionRequest, jointSensorData, jointLimits, robotDimensions, printBuffer,
                                 (SimRobotCore2::Body*)ctrl->application->resolveObject(robotFullName, SimRobotCore2::body)), robotName);
      return true;
    }
    if(mode == SystemCall::remoteRobot)
    {
      kickViewSet = true;
      QString puppetName("RoboCup.puppets." + robotName);

      ctrl->addView(new KickView(robotName + ".KikeView", *this, motionRequest, jointSensorData, jointLimits, robotDimensions, printBuffer,
                                 (SimRobotCore2::Body*)ctrl->application->resolveObject(puppetName, SimRobotCore2::body)), robotName);
      return true;
    }
  }
  return false;
}

void RobotConsole::sendDebugMessage(InMessage& msg)
{
  SYNC;
  msg >> debugOut;
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
                if(buffer == "black")
                  layer.color = ColorRGBA::black;
                else if(buffer == "red")
                  layer.color = ColorRGBA::red;
                else if(buffer == "green")
                  layer.color = ColorRGBA::green;
                else if(buffer == "blue")
                  layer.color = ColorRGBA::blue;
                else if(buffer == "yellow")
                  layer.color = ColorRGBA::yellow;
                else if(buffer == "cyan")
                  layer.color = ColorRGBA::cyan;
                else if(buffer == "magenta")
                  layer.color = ColorRGBA::magenta;
                else if(buffer == "orange")
                  layer.color = ColorRGBA::orange;
                else if(buffer == "violet")
                  layer.color = ColorRGBA::violet;
                else if(buffer == "gray")
                  layer.color = ColorRGBA::gray;
                else
                {
                  int c = 0;
                  sscanf(buffer.c_str(), "%x", &c);
                  layer.color = ColorRGBA((c >> 16) & 0xff, (c >> 8) & 0xff, c & 0xff);
                }
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
    ctrl->list("representation:CameraSettings", path);
    ctrl->list("representation:FieldColors", path);
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
          processesOfDebugData[buffer] = 0;
          debugOut.out.bin << DebugRequest(i.first, true);
          debugOut.out.finishMessage(idDebugRequest);

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
          OutMapFile file(filename);
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
  TImage<PixelTypes::YUYVPixel>::ExportMode exportMode = TImage<PixelTypes::YUYVPixel>::rgb;
  std::string cam;
  stream >> cam;
  if(cam == "reset")
  {
    stream >> imageSaveNumber;
    return true;
  }
  else
  {
    bool useUpperCam;
    if(cam == "upper")
      useUpperCam = true;
    else if(cam == "lower")
      useUpperCam = false;
    else
      return false;

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
      exportMode = TImage<PixelTypes::YUYVPixel>::grayscale;
      stream >> filename;
    }

    SYNC;
    DebugImage* srcImage;
    if(useUpperCam && upperCamImages.find("raw image") != upperCamImages.end())
      srcImage = upperCamImages["raw image"].image;
    else if(!useUpperCam && lowerCamImages.find("raw image") != lowerCamImages.end())
      srcImage = lowerCamImages["raw image"].image;
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
      return TImageWrapper<PixelTypes::YUYVPixel>(srcImage->width, srcImage->height, (PixelTypes::YUYVPixel*)srcImage->data).exportImage(filename.c_str(), left, top, right, bottom, number, exportMode);
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
  if(representation == "representation:CameraSettings")
    fileName = "cameraSettings" + std::string(TypeRegistry::getEnumName(robotInfo.headVersion)) + ".cfg";
  else if(representation == "representation:FieldColors")
    fileName = "fieldColorsCalibration" + std::string(TypeRegistry::getEnumName(robotInfo.headVersion)) + ".cfg";
  else
  {
    std::unordered_map<std::string, std::string>::const_iterator i = ctrl->representationToFile.find(representation);
    if(i == ctrl->representationToFile.end())
      return "";
    fileName = i->second;
  }

  for(const std::string& name : File::getFullNames(fileName))
  {
    File path(name, "r", false);
    if(path.exists())
      return name;
  }

  // if file is not anywhere else, return config directory as default directory
  return fileName;
}

bool RobotConsole::acceptCamera(In& stream)
{
  std::string command;
  stream >> command;

  if(command == "?")
  {
    stream >> command;
    ctrl->list(drawingsViaProcess == 'b' ? "both*" : "both", command);
    ctrl->list(drawingsViaProcess == 'd' ? "lower*" : "lower", command);
    ctrl->list(drawingsViaProcess == 'c' ? "upper*" : "upper", command);
    ctrl->printLn("");
  }
  else
  {
    SYNC;
    if(command == "upper")
      drawingsViaProcess = 'c';
    else if(command == "lower")
      drawingsViaProcess = 'd';
    else if(command == "both")
      drawingsViaProcess = 'b';
    else
      return false;
  }

  return true;
}

void RobotConsole::saveColorCalibration()
{
  SYNC;
  std::string name = "fieldColorsCalibration" + std::string(TypeRegistry::getEnumName(robotInfo.headVersion)) + ".cfg";
  for(std::string& fullName : File::getFullNames(name))
  {
    File path(fullName, "r", false);
    if(path.exists())
    {
      name = std::move(fullName);
      break;
    }
  }
  OutMapFile stream(name);
  stream << colorCalibration;
}

void RobotConsole::requestDebugData(const std::string& name, bool enable)
{
  SYNC;
  DebugRequest d("debug data:" + name, enable);
  debugRequestTable.addRequest(d);
  if(enable)
  {
    processesOfDebugData[name] = 0;
    debugOut.out.bin << d;
    debugOut.out.finishMessage(idDebugRequest);
  }
}
