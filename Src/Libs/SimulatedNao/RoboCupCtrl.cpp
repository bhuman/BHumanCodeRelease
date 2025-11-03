/**
 * @file SimulatedNao/RoboCupCtrl.cpp
 *
 * This file implements the class RoboCupCtrl.
 *
 * @author Thomas Röfer
 * @author <A href="mailto:kspiess@tzi.de">Kai Spiess</A>
 * @author Colin Graf
 */

#include "RoboCupCtrl.h"
#include "ConsoleRoboCupCtrl.h"
#include "ControllerRobot.h"
#include "RobotConsole.h"
#include "Visualization/PaintMethods3DOpenGL.h"
#include "Platform/File.h"
#include "Platform/Time.h"
#include "Framework/Settings.h"
#include "TestUtils.h"

#include <QApplication>

#ifdef MACOS
#include "AppleHelper/Helper.h"
#define TOLERANCE (3.f * simStepLength)
#else
#define TOLERANCE simStepLength
#endif

RoboCupCtrl* RoboCupCtrl::controller = nullptr;
SimRobot::Application* RoboCupCtrl::application = nullptr;

RoboCupCtrl::RoboCupCtrl(SimRobot::Application& application)
  : gameController()
{
  Thread::nameCurrentThread("Main");

  controller = this;
  RoboCupCtrl::application = &application;
  Q_INIT_RESOURCE(Controller);
}

bool RoboCupCtrl::compile()
{
  static_assert(static_cast<int>(SimRobotCore3::scene) != static_cast<int>(SimRobotCore2D::scene),
                "The kinds 'scene' must be different to distinguish between simulation cores.");

  // find simulation object
  SimRobotCore2D::Scene* scene2D = nullptr;
  SimRobotCore3::Scene* scene = static_cast<SimRobotCore3::Scene*>(application->resolveObject("RoboCup", SimRobotCore3::scene));
  if(!scene)
  {
    scene2D = static_cast<SimRobotCore2D::Scene*>(application->resolveObject("RoboCup", SimRobotCore2D::scene));
    if(!scene2D)
      return false;
    is2D = true;
  }
  else
  {
    paintMethods3D = new PaintMethods3DOpenGL;
    scene->registerDrawingManager(*paintMethods3D);
    is2D = false;
  }

  // initialize simulated time and step length
  Time::initialize();
  simStepLength = static_cast<float>(is2D ? scene2D->getStepLength() : scene->getStepLength()) * 1000.f;
  delayTime = simStepLength;

  // Load test state if available
  std::string scenePath = application->getFilePath().toStdString();
  gameController.setTestDirPath(TestUtils::generateTestDirPath(scenePath));
  if(application->isSimResetting())
  {
    gameController.loadTest();
  }

  // Get colors of first and second team
  std::array<GameController::TeamInfo, 2> teamInfos{{{5, Settings::TeamColor::black, Settings::TeamColor::purple}, {70, Settings::TeamColor::red, Settings::TeamColor::blue}}};
  SimRobot::Object* teamInfosObject = application->resolveObject("RoboCup.teams", is2D ? static_cast<int>(SimRobotCore2D::compound) : static_cast<int>(SimRobotCore3::compound));
  if(teamInfosObject)
  {
    ASSERT(application->getObjectChildCount(*teamInfosObject) == 2);
    for(unsigned i = 0; i < 2; ++i)
    {
      SimRobot::Object* teamObject = application->getObjectChild(*teamInfosObject, i);
      ASSERT(application->getObjectChildCount(*teamObject) == 3);

      const auto getChildName = [teamObject](int i)
      {
        const QString& fullName = application->getObjectChild(*teamObject, i)->getFullName();
        return fullName.mid(fullName.lastIndexOf('.') + 1).toStdString();
      };

      const auto parseTeamNumber = [](const std::string& name)
      {
        const unsigned long number = std::strtoul(name.c_str(), nullptr, 10);
        ASSERT(number >= 0 && number < 256);
        return static_cast<uint8_t>(number);
      };

      const auto parseTeamColor = [](const std::string& name)
      {
        const int color = TypeRegistry::getEnumValue(typeid(Settings::TeamColor).name(), name);
        ASSERT(color != -1);
        return static_cast<Settings::TeamColor>(color);
      };
      teamInfos[i].number = parseTeamNumber(getChildName(0));
      teamInfos[i].fieldPlayerColor = parseTeamColor(getChildName(1));
      teamInfos[i].goalkeeperColor = parseTeamColor(getChildName(2));
    }
  }

  if(gameController.isTestActive())
  {
    auto teamNumbers = gameController.getTestParameters().teamNumbers;
    if(teamNumbers[0] == teamNumbers[1])
    {
      FAIL("Both teams have the same team number: " << teamNumbers[0]);
      teamNumbers[1] = (teamNumbers[0] + 1) % 256;
    }
    teamInfos[0].number = static_cast<uint8_t>(teamNumbers[0]);
    teamInfos[1].number = static_cast<uint8_t>(teamNumbers[1]);
  }

  gameController.setTeamInfos(teamInfos);

  std::string location = "Default";
  SimRobot::Object* locationObject = application->resolveObject("RoboCup.location", is2D ? static_cast<int>(SimRobotCore2D::compound) : static_cast<int>(SimRobotCore3::compound));
  if(locationObject)
  {
    ASSERT(application->getObjectChildCount(*locationObject) == 1);
    const QString& fullNameLocation = application->getObjectChild(*locationObject, 0u)->getFullName();
    location = fullNameLocation.mid(fullNameLocation.lastIndexOf('.') + 1).toUtf8().constData();
    if(!QDir(QString(File::getBHDir()) + "/Config/Locations/" + QString::fromStdString(location)).exists())
    {
      FAIL("Location \"" << location << "\" does not exist.");
      location = "Default";
    }
  }

  const std::string defaultScenario = is2D ? "2D" : "Default";
  std::array<std::string, 2> scenarios = {defaultScenario, defaultScenario};
  SimRobot::Object* scenariosObject = application->resolveObject("RoboCup.scenarios", is2D ? static_cast<int>(SimRobotCore2D::compound) : static_cast<int>(SimRobotCore3::compound));
  if(scenariosObject)
  {
    const unsigned count = application->getObjectChildCount(*scenariosObject);
    ASSERT(count == 1 || count == 2);
    for(unsigned i = 0; i < count; ++i)
    {
      const QString& fullNameScenario = application->getObjectChild(*scenariosObject, i)->getFullName();
      scenarios[i] = fullNameScenario.mid(fullNameScenario.lastIndexOf('.') + 1).toUtf8().constData();
      if(!QDir(QString(File::getBHDir()) + "/Config/Scenarios/" + QString::fromStdString(scenarios[i])).exists())
      {
        FAIL("Scenario \"" << scenarios[i] << "\" does not exist.");
        scenarios[i] = defaultScenario;
      }
    }
    if(count != 2)
      scenarios[1] = scenarios[0];
  }

  // get interfaces to simulated objects
  SimRobot::Object* group = application->resolveObject("RoboCup.robots", is2D ? static_cast<int>(SimRobotCore2D::compound) : static_cast<int>(SimRobotCore3::compound));

  const Settings::RobotType robotType = getRobotType();
  std::string robotName = TypeRegistry::getEnumName(robotType);
  robotName[0] &= ~0x20;
  robotName = "Simulated" + robotName;
  for(unsigned currentRobot = 0, count = application->getObjectChildCount(*group); currentRobot < count; ++currentRobot)
  {
    SimRobot::Object* robot = application->getObjectChild(*group, currentRobot);
    const QString& fullName = robot->getFullName();
    const bool firstTeam = SimulatedRobot::isFirstTeam(robot);
    robots.push_back(new ControllerRobot(Settings(robotName, robotName,
                                                  simStepLength * 0.001f,
                                                  robotType,
                                                  teamInfos[firstTeam ? 0 : 1].number,
                                                  teamInfos[firstTeam ? 0 : 1].fieldPlayerColor,
                                                  teamInfos[firstTeam ? 0 : 1].goalkeeperColor,
                                                  SimulatedRobot::getNumber(robot) - (firstTeam ? 0 : SimulatedRobot::robotsPerTeam),
                                                  location, scenarios[firstTeam ? 0 : 1],
                                                  0),
                                         fullName.mid(fullName.lastIndexOf('.') + 1).toUtf8().constData(),
                                         static_cast<ConsoleRoboCupCtrl*>(this)));
  }
  const SimRobot::Object* balls = application->resolveObject("RoboCup.balls", is2D ? static_cast<int>(SimRobotCore2D::compound) : static_cast<int>(SimRobotCore3::compound));
  if(balls)
  {
    SimulatedRobot::setBall(application->getObjectChild(*balls, 0));
    if(is2D)
    {
      SimRobotCore2D::Geometry* ballGeom = static_cast<SimRobotCore2D::Geometry*>(application->resolveObject("RoboCup.balls.ball.DiskGeometry", SimRobotCore2D::geometry));
      if(ballGeom)
        ballGeom->registerCollisionCallback(*this);
    }
    else
    {
      SimRobotCore3::Geometry* ballGeom = static_cast<SimRobotCore3::Geometry*>(application->resolveObject("RoboCup.balls.ball.SphereGeometry", SimRobotCore3::geometry));
      if(ballGeom)
        ballGeom->registerCollisionCallback(*this);
    }
  }
  gameController.loadBallSpecification();

  return true;
}

RoboCupCtrl::~RoboCupCtrl()
{
  qDeleteAll(views);
  SimulatedRobot::setBall(nullptr);
  Time::deinitialize();
  delete paintMethods3D;
  controller = nullptr;
  application = nullptr;
}

QBrush RoboCupCtrl::getAlternateBackgroundColor()
{
#ifdef MACOS
  return getAlternateBase();
#else
  return QApplication::palette().alternateBase();
#endif
}

void RoboCupCtrl::addView(SimRobot::Object* object, const SimRobot::Object* parent, int flags)
{
  views.append(object);
  application->registerObject(*this, *object, parent, flags | SimRobot::Flag::showParent);
}

void RoboCupCtrl::addView(SimRobot::Object* object, const QString& categoryName, int flags)
{
  SimRobot::Object* category = application->resolveObject(categoryName);
  if(!category)
  {
    qsizetype p = categoryName.lastIndexOf('.');
    category = addCategory(categoryName.mid(p + 1), categoryName.left(p));
  }
  addView(object, category, flags);
}

void RoboCupCtrl::removeView(SimRobot::Object* object)
{
  views.removeOne(object);

  // Determine parent category
  const QString fullName = object->getFullName();
  const auto lio = fullName.lastIndexOf('.');
  const QString parentName = fullName.left(lio);
  SimRobot::Object* parent = nullptr;
  for(SimRobot::Object* object : views)
    if(object->getFullName() == parentName && dynamic_cast<Category*>(object))
      parent = object;

  // Has parent category still children?
  if(parent)
  {
    const QString siblingStart = parentName + ".";
    for(SimRobot::Object* object : views)
      if(object->getFullName().startsWith(siblingStart))
      {
        parent = nullptr;
        break;
      }
  }

  application->unregisterObject(*object);
  delete object;

  // If parent category does not have children anymore, it is removed as well.
  if(parent)
    removeView(parent);
}

SimRobot::Object* RoboCupCtrl::addCategory(const QString& name, const SimRobot::Object* parent, const char* icon)
{
  SimRobot::Object* category = new Category(name, parent ? parent->getFullName() + "." + name : name, icon ? icon : ":/Icons/icons8-folder-50.png");
  views.append(category);
  application->registerObject(*this, *category, parent, SimRobot::Flag::windowless | SimRobot::Flag::hidden);
  return category;
}

SimRobot::Object* RoboCupCtrl::addCategory(const QString& name, const QString& parentName)
{
  SimRobot::Object* parent = application->resolveObject(parentName);
  if(!parent)
  {
    const auto lio = parentName.lastIndexOf('.');
    QString subParentName = parentName.left(lio);
    QString name = parentName.mid(lio + 1);
    parent = addCategory(name, subParentName);
  }
  return addCategory(name, parent);
}

Settings::RobotType RoboCupCtrl::getRobotType() const
{
  SimRobot::Object* robotTypeObject = application->resolveObject("RoboCup.robot type", is2D ? static_cast<int>(SimRobotCore2D::compound) : static_cast<int>(SimRobotCore3::compound));
  if(robotTypeObject)
  {
    ASSERT(application->getObjectChildCount(*robotTypeObject) == 1);
    const QString& robotTypeFullName = application->getObjectChild(*robotTypeObject, 0)->getFullName();
    const std::string robotTypeName = robotTypeFullName.mid(robotTypeFullName.lastIndexOf('.') + 1).toUtf8().constData();
    int robotTypeValue = TypeRegistry::getEnumValue(typeid(Settings::RobotType).name(), robotTypeName.c_str());
    if(robotTypeValue != -1)
      return static_cast<Settings::RobotType>(robotTypeValue);
    else
      FAIL("Robot type \"" << robotTypeName << "\" does not exist.");
  }
  return Settings::nao;
}

void RoboCupCtrl::start()
{
#ifdef WINDOWS
  VERIFY(timeBeginPeriod(1) == TIMERR_NOERROR);
#endif
  DebugSenderBase::terminating = false;
  for(ControllerRobot* robot : robots)
    robot->start();
}

void RoboCupCtrl::stop()
{
  DebugSenderBase::terminating = true;
  for(ControllerRobot* robot : robots)
    robot->announceStop();
  for(ControllerRobot* robot : robots)
  {
    robot->stop();
    delete robot;
  }
  controller = nullptr;
#ifdef WINDOWS
  VERIFY(timeEndPeriod(1) == TIMERR_NOERROR);
#endif
}

void RoboCupCtrl::update()
{
  if(delayTime != 0.f)
  {
    float t = static_cast<float>(Time::getRealSystemTime());
    lastTime += delayTime;
    if(lastTime > t) // simulation is running faster then rt
    {
      if(lastTime > t + TOLERANCE)
        Thread::sleep(int(lastTime - t - TOLERANCE));
    }
    else if(t > lastTime + TOLERANCE) // slower then rt
      lastTime = t - TOLERANCE;
  }

  SimulatedRobot::applyBallFriction(ballFriction);

  gameController.update();

  for(ControllerRobot* robot : robots)
    robot->update();

  Time::addSimulatedTime(static_cast<int>(simStepLength + 0.5f));
}

void RoboCupCtrl::collided(SimRobotCore3::Geometry&, SimRobotCore3::Geometry& geom2)
{
  SimRobotCore3::Body* body = geom2.getParentBody();
  if(!body)
    return;
  body = body->getRootBody();
  controller->gameController.setLastBallContactRobot(body);
}

void RoboCupCtrl::collided(SimRobotCore2D::Geometry&, SimRobotCore2D::Geometry& geom2)
{
  SimRobotCore2D::Body* body = geom2.getParentBody();
  if(!body)
    return;
  body = body->getRootBody();
  controller->gameController.setLastBallContactRobot(body);
}
