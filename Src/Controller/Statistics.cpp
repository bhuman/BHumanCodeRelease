/**
 * @file Controller/Statistics.cpp
 *
 * Implementation of class Statistics
 *
 * @author <a href="mailto:jan_fie@uni-bremen.de">Jan Fiedler</a>
 * @author <a href="mailto:urbant@uni-bremen.de">Timo Urban</a>
 */

#include "Statistics.h"
#include "ConsoleRoboCupCtrl.h"
#include "Views/Statistics/StatisticRobotView.h"
#include "Views/Statistics/StatisticsFieldView.h"
#include "Views/Statistics/StatisticsPlotView.h"
#include "Platform/Time.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/GetUpEngineOutputLog.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Sensing/FallDownState.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/Debugging.h"

#include <ctime>
#include <map>
#include <QFileDialog>
#include <QSettings>

Statistics::Statistics(ConsoleRoboCupCtrl& ctrl) : ctrl(ctrl) {}

void Statistics::initialize()
{
  SYNC;
  if(active)
    return;
  active = true;
  fieldDimensions.load();
  fieldPartSize = (Vector2f() << ((fieldDimensions.boundary.x.max * 2.f / Statistic::xFieldParts) + 1.f), ((fieldDimensions.boundary.y.max * 2.f / Statistic::yFieldParts) + 1.f)).finished(); // + 1 to prevent coordinate outside of the field
  if(!category)
  {
    category = ctrl.addCategory(catName.left(catName.size() - 1), nullptr, ":/Icons/page_white_h.png");
    ctrl.addView(new StatisticsFieldView(catName + "teamHeatmap", *this, "teamHeatmap"), category, SimRobot::Flag::copy | SimRobot::Flag::exportAsImage);
  }
  // Add Fieldlines
  const ColorRGBA lineColor(192, 192, 192);
  int id = 0;
  for(std::vector<FieldDimensions::LinesTable::Line>::const_iterator i = fieldDimensions.fieldLines.lines.begin(); i != fieldDimensions.fieldLines.lines.end(); ++i)
  {
    DebugDrawing drawing;
    drawing.line(i->from.x(), i->from.y(), i->to.x(), i->to.y(), Drawings::solidPen, fieldDimensions.fieldLinesWidth, lineColor);
    fieldDrawings["fieldLines" + std::to_string(id)] = drawing;
    ++id;
  }
}

void Statistics::updateLogFrame(const char frameType, const size_t index, const Statistic::Representations& repr)
{
  Statistic::StatisticRobot& robot = statistic.robots[index];
  ++robot.frames;
  switch(frameType)
  {
    case 'c':
      updateCognitionFrame(robot, repr.ballModel, repr.ballPercept, repr.frameInfo, repr.gameInfo, repr.robotHealth, repr.robotInfo, repr.robotPose, repr.teamData);
      if(repr.frameInfo.time > 100)
      {
        robot.configuration = repr.robotHealth.configuration;
        robot.location = repr.robotHealth.location;
        robot.scenario = repr.robotHealth.scenario;
        robot.number = repr.robotInfo.number;
      }
      break;
    case 'm':
      updateMotionFrame(robot, repr.fallDownState, repr.frameInfo, repr.getUpEngineOutputLog, repr.jointSensorData, repr.keyStates, repr.motionRequest);
      break;
    default:
      OUTPUT_WARNING("Frame with wrong Type found: " << frameType);
      break;
  }
}

size_t Statistics::getRobotIndex(const std::string& robotName)
{
  SYNC;
  for(size_t i = 0; i < statistic.robots.size(); ++i)
  {
    if(statistic.robots[i].name == robotName)
    {
      ++statistic.robots[i].logCount;
      return static_cast<int>(i);
    }
  }
  // else
  statistic.robots.emplace_back();
  statistic.robots.back().name = robotName;
  return statistic.robots.size() - 1;
}

void Statistics::viewRobot(const size_t index)
{
  SYNC;
  const Statistic::StatisticRobot& robot = statistic.robots[index];

  SimRobot::Object* robotCategory = ctrl.addCategory(robot.name.c_str(), category);
  ctrl.addView(new StatisticRobotView(catName + robot.name.c_str() + ".statistics", statistic.robots, index), robotCategory);
  ctrl.addView(new StatisticsFieldView(catName + robot.name.c_str() + ".heatmap", *this, robot.name + "Heatmap"), robotCategory, SimRobot::Flag::copy | SimRobot::Flag::exportAsImage);

  fillHeatmap(static_cast<int>(index), robot.name);
  for(size_t i = 0; i < fieldDimensions.fieldLines.lines.size(); ++i)
    fieldViews[robot.name + "Heatmap"].emplace_back("fieldLines" + std::to_string(i));
  fillHeatmap(-1, "team");
  for(size_t i = 0; i < fieldDimensions.fieldLines.lines.size(); ++i)
    fieldViews["teamHeatmap"].emplace_back("fieldLines" + std::to_string(i));
  plotJointTemperatures(robot, robotCategory);
}

void Statistics::exportSR() const
{
  std::time_t now = std::time(0);
  char timestamp[17];
  std::strftime(timestamp, sizeof(timestamp), "%Y-%m-%d-%H-%M", std::localtime(&now));
  std::string fileName = "Statistics/StatisticRobots" + std::string(timestamp) + ".stf";

  OutMapFile file(fileName);
  if(!file.exists())
  {
    OUTPUT_ERROR("Could not creat file " << fileName);
    return;
  }
  file << statistic;
  OUTPUT_TEXT("File was created successfully: " << fileName);
}

void Statistics::importSR()
{
  initialize();
  // showInputDialog, works like ConsoleRoboCupCtrl::showInputDialog()
  QStringList qsl;
  qsl.append("StatisticRobots*.stf");
  qsl.append("all (*.*)");
  QSettings settings("B-Human", "SimRobot");

  // QFileDialog::DontUseNativeDialog is not fine, but it fixes visualisation qt-bug (see https://bugreports.qt.io/browse/QTBUG-29248)
  QString fileName = QFileDialog::getOpenFileName(nullptr, "StatisticRobotfile", settings.value(qsl.front(), "../../Statistics").toString(), qsl.join(";;"), nullptr, QFileDialog::DontUseNativeDialog);
  if(fileName != QString::null)
    settings.setValue(qsl.front(), QDir().absoluteFilePath(fileName));

  // import
  InMapFile file(fileName.toStdString());
  if(!file.exists())
  {
    OUTPUT_ERROR("Could not load StatisticRobots from " << fileName.toStdString());
    return;
  }
  file >> statistic;
  for(size_t i = 0; i < statistic.robots.size(); i++)
    viewRobot(i);
}

void Statistics::exportHeatMaps()
{
  if(!exportCSV(
    "TeamHeatMap",
    [&](Out& file, const std::string& sep)
    {
      for(unsigned i = 0; i < statistic.teamHeatmap.width; i++)
        file << i << sep;
    },
    [&](Out& file, const std::string& sep)
    {
      for(unsigned y = 0; y < statistic.teamHeatmap.height; y++)
      {
        for(unsigned x = 0; x < statistic.teamHeatmap.width; x++)
          file << statistic.teamHeatmap(x, y) << sep;
        file << endl;
      }
    }
  ))
    OUTPUT_ERROR("TeamHeatMap could not be exported");

  bool valid = true;
  for(Statistic::StatisticRobot& robot : statistic.robots)
  {
    const std::string fileName = "HeatMap_" + robot.name;
    if(!valid)
      break;
    valid = exportCSV(
      fileName,
      [&](Out& file, const std::string& sep)
      {
        for(unsigned i = 0; i < robot.heatmap.width; i++)
          file << i << sep;
      },
      [&](Out& file, const std::string& sep)
      {
        for(unsigned y = 0; y < robot.heatmap.height; y++)
        {
          for(unsigned x = 0; x < robot.heatmap.width; x++)
            file << robot.heatmap(x, y) << sep;
          file << endl;
        }
      }
    );
  }
  if(!valid)
    OUTPUT_ERROR("HeatMaps could not be exported");
}

void Statistics::plotJointTemperatures()
{
  SimRobot::Object* robotCategory = ctrl.addCategory("JointTemperatures", category);
  for(Statistic::StatisticRobot& robot : statistic.robots)
  {
    FOREACH_ENUM(Joints::Joint, joint)
    {
      if(robot.maxJointTemperatures[joint] == 0)
      {
        std::string id = TypeRegistry::getEnumName(joint);
        RobotConsole::Layer layer;
        layer.layer = id;
        layer.description = id;
        layer.color = ColorRGBA::red;
        RobotConsole::Plot& plot = plots[layer.layer];
        for(unsigned value : robot.jointTemperatures[joint])
        {
          plot.points.push_back(value);
          plot.timeStamp = Time::getCurrentSystemTime();
        }
        plotViews[robot.name + id].push_back(layer);
        const unsigned plotSize = static_cast<unsigned>(robot.jointTemperatures[joint].size());
        ctrl.addView(new StatisticsPlotView(catName + "JointTemperatures." + robot.name.c_str() + "_" + TypeRegistry::getEnumName(joint), *this, robot.name + TypeRegistry::getEnumName(joint), plotSize, 0, 100, "Temperature (in °C)", "Time (in s)", robot.jointTemperatureTime), robotCategory, SimRobot::Flag::copy | SimRobot::Flag::exportAsImage);
      }
    }
  }
}

void Statistics::updateCognitionFrame(Statistic::StatisticRobot& robot, const BallModel& ballModel, const BallPercept& ballPercept, const FrameInfo& frameInfo, const GameInfo& gameInfo, const RobotHealth& robotHealth, const RobotInfo& robotInfo, const RobotPose& robotPose, const TeamData& teamData)
{
  if(!robot.isPenalized && robotInfo.penalty != PENALTY_NONE)
    robot.isPenalized = true;
  else if(robot.isPenalized && robotInfo.penalty == PENALTY_NONE)
  {
    robot.isPenalized = false;
    ++robot.penaltys;
  }

  if(robotPose.validity == 1.f && gameInfo.state == STATE_PLAYING) // validity should probably be smaller
  {
    const int x = static_cast<int>((robotPose.translation.x() + fieldDimensions.boundary.x.max) / fieldPartSize.x());
    const int y = static_cast<int>(Statistic::yFieldParts - (robotPose.translation.y() + fieldDimensions.boundary.y.max) / fieldPartSize.y());  // transform heatmap y to worldcoordinate
    ++robot.heatmap(x, y);
    SYNC;
    ++statistic.teamHeatmap(x, y);
  }

  switch(ballPercept.status)
  {
    case BallPercept::Status::seen:
      ++robot.seenBalls;
      break;
    case BallPercept::Status::guessed:
      ++robot.guessedBalls;
      break;
    default:
      break;
  }
  if(robot.isInWalkKicking)
  {
    if(((robot.inWalkDuration + 120) >= frameInfo.time))
    {
      const Vector2f& distance = robot.ballPositionInWalk - ballModel.lastPerception;
      if(distance.norm() > 1000)
        ++robot.inWalkKicks;
    }
  }
  else
    robot.ballPositionInWalk = ballModel.lastPerception;

  if(robot.lastPosition != Vector2f::Zero() && robot.isUpright)
  {
    const Vector2f& distance = robot.lastPosition - robotPose.translation;
    if(distance.norm() < 50)
      robot.walkedDistance += distance.norm() / 2000;
    else
    {
      if(distance.norm() > 1000)
        ++robot.badLocalisations;
    }
  }
  robot.lastPosition = robotPose.translation;
  robot.receivedMessages = teamData.receivedMessages;
  for(auto& teammate : teamData.teammates)
  {
    //if(robot.receivedMessagesRobots.size() == 0)
    if(teammate.number > 0)
      ++robot.receivedMessagesRobots[teammate.number - 1];
  }
}

void Statistics::updateMotionFrame(Statistic::StatisticRobot& robot, const FallDownState& fallDownState, const FrameInfo& frameInfo, const GetUpEngineOutputLog& getUpEngineOutputLog, const JointSensorData& jointSensorData, const KeyStates& keyStates, const MotionRequest& motionRequest)
{
  if(!robot.isPickedUp && fallDownState.state == FallDownState::State::pickedUp)
    robot.isPickedUp = true;
  else if(robot.isPickedUp && fallDownState.state != FallDownState::State::pickedUp)
  {
    robot.isPickedUp = false;
    ++robot.pickedUps;
  }

  if(!robot.isFallen && fallDownState.state == FallDownState::State::fallen)
  {
    ++robot.fallen;
    robot.fallenFrames.emplace_back(robot.frames);
    robot.isFallen = true;
    robot.tryCounterBuffer = 0;
  }
  else if(fallDownState.state == FallDownState::State::fallen)
  {
    if(robot.fallenState < getUpEngineOutputLog.tryCounter && robot.tryCounterBuffer > 10) 
    {
      robot.fallenState = getUpEngineOutputLog.tryCounter;
      ++robot.failedGetUps;
      robot.failedGetUpFrames.emplace_back(robot.frames);
    } else
      ++robot.tryCounterBuffer;
  }
  else if(robot.isFallen && fallDownState.state != FallDownState::State::falling)
  {
    robot.isFallen = false;
    robot.fallenState = 0;
  }

  if(fallDownState.state == FallDownState::State::upright)
    robot.isUpright = true;
  else
    robot.isUpright = false;

  const bool areFootBumpersPressed = keyStates.pressed[KeyStates::Key::lFootLeft] || keyStates.pressed[KeyStates::Key::lFootRight] || keyStates.pressed[KeyStates::Key::rFootLeft] || keyStates.pressed[KeyStates::Key::rFootRight];

  if(motionRequest.motion == MotionRequest::Motion::kick)
  {
    ++robot.kickRequests;
    if(robot.kickDuration == 0) //can change to isKicking later, if duration of Kicking is not needed
      robot.isKicking = true;
    if(robot.isKicking)
    {
      if(areFootBumpersPressed && !robot.isBallKicked)
      {
        ++robot.ballKicked;
        robot.isBallKicked = true;
      }
      ++robot.kickDuration;
    }
  }
  else if(robot.isKicking)
  {
    robot.isKicking = false;
    robot.isBallKicked = false;
    robot.kickDuration = 0;
  }

  switch(motionRequest.motion)
  {
    case MotionRequest::Motion::getUp:
      ++robot.getUpRequests;
      break;
    case MotionRequest::Motion::specialAction:
      ++robot.specialActionRequests;
      break;
    case MotionRequest::Motion::stand:
      ++robot.standRequests;
      break;
    case MotionRequest::Motion::walk:
      ++robot.walkRequests;
      break;
    default:
      break;
  }

  if(motionRequest.motion == MotionRequest::Motion::walk)
  {
    if(areFootBumpersPressed)
    {
      robot.inWalkDuration = frameInfo.time;
      robot.isInWalkKicking = true;
    }
  }

  if(frameInfo.getTimeSince(robot.timeWhenLastAddTemperature) >= static_cast<int>(robot.jointTemperatureTime * 100))
  {
    robot.timeWhenLastAddTemperature = frameInfo.time;
    FOREACH_ENUM(Joints::Joint, joint)
    {
      robot.jointTemperatures[joint].emplace_back(jointSensorData.temperatures.at(joint));
      const unsigned char temperature = jointSensorData.temperatures[joint];
      if(temperature > Statistic::heatStatus && temperature > robot.maxJointTemperatures[joint])
        robot.maxJointTemperatures[joint] = jointSensorData.temperatures.at(joint);
    }
  }
  robot.lastMotion = motionRequest.motion;
}

void Statistics::fillHeatmap(const int index, const std::string& namePrefix)
{
  Statistic::Heatmap& heatmap = index == -1 ? statistic.teamHeatmap : statistic.robots[index].heatmap;
  const unsigned max = heatmap.maxValue();

  for(unsigned x = 0; x < Statistic::xFieldParts; ++x)
  {
    for(unsigned y = 0; y < Statistic::yFieldParts; ++y)
    {
      const std::string name(namePrefix + std::to_string(x) + ',' + std::to_string(y));
      fieldViews[namePrefix + "Heatmap"].emplace_back(name);
      Vector2i points[4];
      points[0].x() = points[1].x() = x * static_cast<int>(fieldPartSize.x()) + static_cast<int>(fieldDimensions.xPosOwnFieldBorder);
      points[2].x() = points[3].x() = (x + 1) * static_cast<int>(fieldPartSize.x()) + static_cast<int>(fieldDimensions.xPosOwnFieldBorder);
      points[0].y() = points[3].y() = y * static_cast<int>(fieldPartSize.y()) + static_cast<int>(fieldDimensions.yPosRightFieldBorder);
      points[1].y() = points[2].y() = (y + 1) * static_cast<int>(fieldPartSize.y()) + static_cast<int>(fieldDimensions.yPosRightFieldBorder);
      const unsigned char alpha = static_cast<unsigned char>((max - heatmap(x, Statistic::yFieldParts - 1 - y)) / static_cast<float>(max) * 255);
      const ColorRGBA color = ColorRGBA(0, 140, 0, alpha).blend(ColorRGBA(255, 0, 0, 255 - alpha));
      DebugDrawing drawing;
      drawing.polygon(points, 4, 1, Drawings::solidPen, ColorRGBA(0, 180, 0), Drawings::solidBrush, color);
      fieldDrawings[name] = drawing;
    }
  }
}

void Statistics::plotJointTemperatures(const Statistic::StatisticRobot& robot, SimRobot::Object* robotCategory)
{
  FOREACH_ENUM(Joints::Joint, joint)
  {
    if(robot.maxJointTemperatures[joint] > 0)
    {
      std::string id = TypeRegistry::getEnumName(joint);
      RobotConsole::Layer layer;
      layer.layer = id;
      layer.description = id;
      layer.color = ColorRGBA::red;
      RobotConsole::Plot& plot = plots[layer.layer];
      for(unsigned value : robot.jointTemperatures[joint])
      {
        plot.points.push_back(value);
        plot.timeStamp = Time::getCurrentSystemTime();
      }
      plotViews[robot.name + id].push_back(layer);
      const unsigned plotSize = static_cast<unsigned>(robot.jointTemperatures[joint].size());
      ctrl.addView(new StatisticsPlotView(catName + robot.name.c_str() + "." + TypeRegistry::getEnumName(joint), *this, robot.name + TypeRegistry::getEnumName(joint), plotSize, 0, 100, "Temperature (in °C)", "Time (in s)", robot.jointTemperatureTime), robotCategory, SimRobot::Flag::copy | SimRobot::Flag::exportAsImage);
    }
  }
}

bool Statistics::exportCSV(const std::string& fileName, const std::function<void(Out& file, const std::string& sep)>& writeHeader, const std::function<void(Out& file, const std::string& sep)>& writeInFile) const
{
  std::string name;
  if((int)fileName.rfind('.') <= (int)fileName.find_last_of("\\/"))
    name = fileName + ".csv";
  if(name[0] != '/' && name[0] != '\\' && (name.size() < 2 || name[1] != ':'))
    name = "Statistics/" + name;

  OutTextRawFile file(name);
  if(!file.exists())
    return false;
  const std::string sep = ";";
  writeHeader(file, sep);
  file << endl;
  writeInFile(file, sep);
  file << endl;
  OUTPUT_TEXT("File was created successfully: " << name);
  return true;
}
