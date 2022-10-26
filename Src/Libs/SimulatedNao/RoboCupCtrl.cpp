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

#include <QApplication>
#include <QIcon>

#ifdef MACOS
#include "AppleHelper/Helper.h"
#define TOLERANCE (3.f * simStepLength)
#else
#define TOLERANCE simStepLength
#endif

RoboCupCtrl* RoboCupCtrl::controller = nullptr;
SimRobot::Application* RoboCupCtrl::application = nullptr;

RoboCupCtrl::RoboCupCtrl(SimRobot::Application& application)
{
  Thread::nameCurrentThread("Main");

  controller = this;
  RoboCupCtrl::application = &application;
  Q_INIT_RESOURCE(Controller);
}

bool RoboCupCtrl::compile()
{
  static_assert(static_cast<int>(SimRobotCore2::scene) != static_cast<int>(SimRobotCore2D::scene),
                "The kinds 'scene' must be different to distinguish between simulation cores.");

  // find simulation object
  SimRobotCore2D::Scene* scene2D = nullptr;
  SimRobotCore2::Scene* scene = static_cast<SimRobotCore2::Scene*>(application->resolveObject("RoboCup", SimRobotCore2::scene));
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

  // Get colors of first and second team
  // If there is no team colors compound, fall back to default (black, blue)
  std::array<uint8_t, 2> teamNumbers = {89, 90};
  std::array<Settings::TeamColor, 2> teamColors = {static_cast<Settings::TeamColor>(-1), static_cast<Settings::TeamColor>(-1)};
  SimRobot::Object* teamColorsObject = application->resolveObject("RoboCup.teamColors", is2D ? static_cast<int>(SimRobotCore2D::compound) : static_cast<int>(SimRobotCore2::compound));
  if(teamColorsObject)
  {
    ASSERT(application->getObjectChildCount(*teamColorsObject) == 2);
    for(unsigned i = 0; i < 2; ++i)
    {
      const QString& fullNameTeamColor = application->getObjectChild(*teamColorsObject, i)->getFullName();
      const std::string baseNameTeamColor = fullNameTeamColor.mid(fullNameTeamColor.lastIndexOf('.') + 1).toUtf8().constData();
      teamColors[i] = static_cast<Settings::TeamColor>(TypeRegistry::getEnumValue(typeid(Settings::TeamColor).name(), baseNameTeamColor));
    }
  }
  if(teamColors[0] == static_cast<Settings::TeamColor>(-1))
    teamColors[0] = Settings::TeamColor::black;
  if(teamColors[1] == static_cast<Settings::TeamColor>(-1))
    teamColors[1] = Settings::TeamColor::blue;
  gameController.setTeamInfos(teamNumbers, teamColors);

  std::string location = "Default";
  SimRobot::Object* locationObject = application->resolveObject("RoboCup.location", is2D ? static_cast<int>(SimRobotCore2D::compound) : static_cast<int>(SimRobotCore2::compound));
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
  SimRobot::Object* scenariosObject = application->resolveObject("RoboCup.scenarios", is2D ? static_cast<int>(SimRobotCore2D::compound) : static_cast<int>(SimRobotCore2::compound));
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
  SimRobot::Object* group = application->resolveObject("RoboCup.robots", is2D ? static_cast<int>(SimRobotCore2D::compound) : static_cast<int>(SimRobotCore2::compound));

  for(unsigned currentRobot = 0, count = application->getObjectChildCount(*group); currentRobot < count; ++currentRobot)
  {
    SimRobot::Object* robot = application->getObjectChild(*group, currentRobot);
    const QString& fullName = robot->getFullName();
    const bool firstTeam = SimulatedRobot::isFirstTeam(robot);
    robots.push_back(new ControllerRobot(Settings("Nao", "Nao",
                                                  teamNumbers[firstTeam ? 0 : 1],
                                                  teamColors[firstTeam ? 0 : 1],
                                                  SimulatedRobot::getNumber(robot) - (firstTeam ? 0 : SimulatedRobot::robotsPerTeam),
                                                  location, scenarios[firstTeam ? 0 : 1],
                                                  0),
                                         fullName.mid(fullName.lastIndexOf('.') + 1).toUtf8().constData(),
                                         static_cast<ConsoleRoboCupCtrl*>(this)));
  }
  const SimRobot::Object* balls = application->resolveObject("RoboCup.balls", is2D ? static_cast<int>(SimRobotCore2D::compound) : static_cast<int>(SimRobotCore2::compound));
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
      SimRobotCore2::Geometry* ballGeom = static_cast<SimRobotCore2::Geometry*>(application->resolveObject("RoboCup.balls.ball.SphereGeometry", SimRobotCore2::geometry));
      if(ballGeom)
        ballGeom->registerCollisionCallback(*this);
    }
  }

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
    const auto lio = categoryName.lastIndexOf('.');
    QString subParentName = categoryName.mid(0, lio);
    QString name = categoryName.mid(lio + 1);
    category = addCategory(name, subParentName);
  }
  addView(object, category, flags);
}

void RoboCupCtrl::removeView(SimRobot::Object* object)
{
  views.removeOne(object);
  application->unregisterObject(*object);
}

void RoboCupCtrl::removeCategory(SimRobot::Object* object)
{
  views.removeOne(object);
  application->unregisterObject(*object);
}

SimRobot::Object* RoboCupCtrl::addCategory(const QString& name, const SimRobot::Object* parent, const char* icon)
{
  class Category : public SimRobot::Object
  {
    QString name;
    QString fullName;
    QIcon icon;

  public:
    Category(const QString& name, const QString& fullName, const char* icon) : name(name), fullName(fullName), icon(icon) {}

  private:
    const QString& getFullName() const override { return fullName; }
    const QIcon* getIcon() const override { return &icon; }
  };

  SimRobot::Object* category = new Category(name, parent ? parent->getFullName() + "." + name : name, icon ? icon : ":/Icons/folder.png");
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
    QString subParentName = parentName.mid(0, lio);
    QString name = parentName.mid(lio + 1);
    parent = addCategory(name, subParentName);
  }
  return addCategory(name, parent);
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

void RoboCupCtrl::collided(SimRobotCore2::Geometry&, SimRobotCore2::Geometry& geom2)
{
  SimRobotCore2::Body* body = geom2.getParentBody();
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
