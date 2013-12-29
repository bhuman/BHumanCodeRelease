/**
 * @file Controller/RoboCupCtrl.cpp
 *
 * This file implements the class RoboCupCtrl.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 * @author <A href="mailto:kspiess@tzi.de">Kai Spiess</A>
 * @author Colin Graf
 */

#include <QIcon>

#include "RoboCupCtrl.h"
#include "Platform/SimRobotQt/Robot.h"
#include "RobotConsole.h"

RoboCupCtrl* RoboCupCtrl::controller = 0;
SimRobot::Application* RoboCupCtrl::application = 0;

RoboCupCtrl::RoboCupCtrl(SimRobot::Application& application) : robotName(0), simTime(false), dragTime(true), lastTime(0)
{
  NAME_THREAD("Main");

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
  time = 10000 - SystemCall::getRealSystemTime();
  simStepLength =  int(scene->getStepLength() * 1000. + 0.5);
  if(simStepLength > 20)
    simStepLength = 20;

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
  this->robotName = 0;
  if(!robots.empty())
    Oracle::init();
  const SimRobot::Object* balls = (SimRobotCore2::Object*)RoboCupCtrl::application->resolveObject("RoboCup.balls", SimRobotCore2::compound);
  if(balls)
  {
    Oracle::setBall(RoboCupCtrl::application->getObjectChild(*balls, 0));
    SimRobotCore2::Geometry* ballGeom = (SimRobotCore2::Geometry*)application->resolveObject("RoboCup.balls.ball.SphereGeometry", SimRobotCore2::geometry);
    if(ballGeom)
      ballGeom->registerCollisionCallback(*this);
  }

  return true;
}

RoboCupCtrl::~RoboCupCtrl()
{
  qDeleteAll(views);
  Oracle::setBall(0);
  controller = 0;
  application = 0;
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

SimRobot::Object* RoboCupCtrl::addCategory(const QString& name, const SimRobot::Object* parent, const char* icon)
{
  class Category : public SimRobot::Object
  {
  public:
    Category(const QString& name, const QString& fullName, const char* icon) : name(name), fullName(fullName), icon(icon) {}

  private:
    QString name;
    QString fullName;
    QIcon icon;

    virtual const QString& getDisplayName() const {return name;};
    virtual const QString& getFullName() const {return fullName;};
    virtual const QIcon* getIcon() const {return &icon;};
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
#ifdef WIN32
  VERIFY(timeBeginPeriod(1) == TIMERR_NOERROR);
#endif
  for(std::list<Robot*>::iterator i = robots.begin(); i != robots.end(); ++i)
    (*i)->start();
}

void RoboCupCtrl::stop()
{
  for(std::list<Robot*>::iterator i = robots.begin(); i != robots.end(); ++i)
    (*i)->announceStop();
  for(std::list<Robot*>::iterator i = robots.begin(); i != robots.end(); ++i)
  {
    (*i)->stop();
    delete *i;
  }
  controller = 0;
#ifdef WIN32
  VERIFY(timeEndPeriod(1) == TIMERR_NOERROR);
#endif
}

void RoboCupCtrl::update()
{
  if(dragTime)
  {
    unsigned int t = SystemCall::getRealSystemTime();
    lastTime += simStepLength;
    if(lastTime > t) // simulation is running faster then rt
    {
      if(lastTime - t > (unsigned int)simStepLength)
        SystemCall::sleep(lastTime - t - simStepLength);
    }
    else // slower then rt
    {
      if(t - lastTime > (unsigned int)simStepLength)
        lastTime += simStepLength * ((t - lastTime) / simStepLength);
    }
  }

  gameController.referee();

  statusText = "";
  for(std::list<Robot*>::iterator i = robots.begin(); i != robots.end(); ++i)
    (*i)->update();
  if(simTime)
    time += simStepLength;
}

void RoboCupCtrl::collided(SimRobotCore2::Geometry& geom1, SimRobotCore2::Geometry& geom2)
{
  SimRobotCore2::Body* body = geom2.getParentBody();
  if(!body)
    return;
  body = body->getRootBody();
  Oracle::setLastBallContactRobot(body);
}

std::string RoboCupCtrl::getRobotName() const
{
  size_t threadId = Thread<ProcessBase>::getCurrentId();
  for(std::list<Robot*>::const_iterator i = robots.begin(); i != robots.end(); ++i)
    for(ProcessList::const_iterator j = (*i)->begin(); j != (*i)->end(); ++j)
      if((*j)->getId() == threadId)
        return (*i)->getName();
  if(!this->robotName)
    return "Robot1";
  std::string robotName(this->robotName);
  return robotName.substr(robotName.rfind('.') + 1);
}

std::string RoboCupCtrl::getModelName() const
{
  return "Nao";
}

unsigned RoboCupCtrl::getTime() const
{
  if(simTime)
    return unsigned(time);
  else
    return unsigned(SystemCall::getRealSystemTime() + time);
}
