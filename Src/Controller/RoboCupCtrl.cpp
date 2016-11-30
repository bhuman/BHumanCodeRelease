/**
 * @file Controller/RoboCupCtrl.cpp
 *
 * This file implements the class RoboCupCtrl.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 * @author <A href="mailto:kspiess@tzi.de">Kai Spiess</A>
 * @author Colin Graf
 */

#include "RobotConsole.h"
#include "RoboCupCtrl.h"
#include "Platform/Time.h"
#include "Platform/SimulatedNao/Robot.h"

#include <QIcon>

#ifdef MACOS
#define TOLERANCE 30.f
#else
#define TOLERANCE 10.f
#endif

RoboCupCtrl* RoboCupCtrl::controller = nullptr;
SimRobot::Application* RoboCupCtrl::application = nullptr;

RoboCupCtrl::RoboCupCtrl(SimRobot::Application& application) : robotName(nullptr)
{
  Thread::nameThread("Main");

  this->controller = this;
  this->application = &application;
  Q_INIT_RESOURCE(Controller);
}

bool RoboCupCtrl::compile()
{
  // find simulation object
  SimRobotCore2::Scene* scene = (SimRobotCore2::Scene*)application->resolveObject("RoboCup", SimRobotCore2::scene);
  if(!scene)
    return false;

  // initialize simulated time and step length
  time = 10000 - Time::getRealSystemTime();
  simStepLength = int(scene->getStepLength() * 1000.f + 0.5f);
  if(simStepLength > 20)
    simStepLength = 20;
  delayTime = (float)simStepLength;

  // get interfaces to simulated objects
  SimRobot::Object* group = application->resolveObject("RoboCup.robots", SimRobotCore2::compound);
  for(unsigned currentRobot = 0, count = application->getObjectChildCount(*group); currentRobot < count; ++currentRobot)
  {
    SimRobot::Object* robot = (SimRobot::Object*)application->getObjectChild(*group, currentRobot);
    const QString& fullName = robot->getFullName();
    std::string robotName = fullName.toUtf8().constData();
    this->robotName = robotName.c_str();
    robots.push_back(new Robot(fullName.mid(fullName.lastIndexOf('.') + 1).toUtf8().constData()));
  }
  this->robotName = nullptr;
  const SimRobot::Object* balls = (SimRobotCore2::Object*)RoboCupCtrl::application->resolveObject("RoboCup.balls", SimRobotCore2::compound);
  if(balls)
  {
    SimulatedRobot::setBall(RoboCupCtrl::application->getObjectChild(*balls, 0));
    SimRobotCore2::Geometry* ballGeom = (SimRobotCore2::Geometry*)application->resolveObject("RoboCup.balls.ball.SphereGeometry", SimRobotCore2::geometry);
    if(ballGeom)
      ballGeom->registerCollisionCallback(*this);
  }

  return true;
}

RoboCupCtrl::~RoboCupCtrl()
{
  qDeleteAll(views);
  SimulatedRobot::setBall(nullptr);
  controller = nullptr;
  application = nullptr;
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
    int lio = categoryName.lastIndexOf('.');
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
    virtual const QString& getDisplayName() const { return name; }
    virtual const QString& getFullName() const { return fullName; }
    virtual const QIcon* getIcon() const { return &icon; }
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
    int lio = parentName.lastIndexOf('.');
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
  for(Robot* robot : robots)
    robot->start();
}

void RoboCupCtrl::stop()
{
  DebugSenderBase::terminating = true;
  for(Robot* robot : robots)
    robot->announceStop();
  for(Robot* robot : robots)
  {
    robot->stop();
    delete robot;
  }
  controller = 0;
#ifdef WINDOWS
  VERIFY(timeEndPeriod(1) == TIMERR_NOERROR);
#endif
}

void RoboCupCtrl::update()
{
  if(delayTime != 0.f)
  {
    float t = (float)Time::getRealSystemTime();
    lastTime += delayTime;
    if(lastTime > t) // simulation is running faster then rt
    {
      if(lastTime > t + TOLERANCE)
        Thread::sleep(int(lastTime - t - TOLERANCE));
    }
    else if(t > lastTime + TOLERANCE) // slower then rt
      lastTime = t - TOLERANCE;
  }

  gameController.referee();

  statusText = "";
  for(Robot* robot : robots)
    robot->update();
  if(simTime)
    time += simStepLength;
}

void RoboCupCtrl::collided(SimRobotCore2::Geometry& geom1, SimRobotCore2::Geometry& geom2)
{
  SimRobotCore2::Body* body = geom2.getParentBody();
  if(!body)
    return;
  body = body->getRootBody();
  GameController::setLastBallContactRobot(body);
}

std::string RoboCupCtrl::getRobotName() const
{
  std::thread::id threadId = Thread::getCurrentId();
  for(const Robot* robot : robots)
    for(const ProcessBase* process : *robot)
      if(process->getId() == threadId)
        return robot->getName();
  if(!this->robotName)
    return "Robot1";
  std::string robotName(this->robotName);
  return robotName.substr(robotName.rfind('.') + 1);
}

unsigned RoboCupCtrl::getTime() const
{
  if(simTime)
    return unsigned(time);
  else
    return unsigned(Time::getRealSystemTime() + time);
}
