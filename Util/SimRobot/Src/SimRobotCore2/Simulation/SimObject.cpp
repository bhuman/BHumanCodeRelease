/**
* @file Simulation/SimObject.cpp
* Implementation of class SimObject
* @author Colin Graf
*/

#include <typeinfo>
#ifdef __GNUC__
#include <cctype>
#endif

#include "Simulation/SimObject.h"
#include "Platform/Assert.h"
#include "CoreModule.h"
#include "SimObjectWidget.h"
#include "SimObjectRenderer.h"

SimObject::SimObject() : translation(0), rotation(0) {}

void SimObject::addParent(Element& parent)
{
  dynamic_cast<SimObject*>(&parent)->children.push_back(this);
}

SimObject::~SimObject()
{
  if(translation)
    delete translation;
  if(rotation)
    delete rotation;
}

void SimObject::registerObjects()
{
  for(std::list<SimObject*>::const_iterator iter = children.begin(), end = children.end(); iter != end; ++iter)
  {
    SimObject* simObject = *iter;
    if(simObject->name.empty())
    {
      const char* typeName = typeid(*simObject).name();
#ifdef __GNUC__
      while(isdigit(*typeName))
        ++typeName;
#else
      const char* str = strchr(typeName, ' ');
      if(str)
        typeName = str + 1;
#endif
      simObject->fullName = fullName + "." + typeName;
    }
    else
      simObject->fullName = fullName + "." + simObject->name.c_str();
    CoreModule::application->registerObject(*CoreModule::module, dynamic_cast<SimRobot::Object&>(*simObject), dynamic_cast<SimRobot::Object*>(this));
    simObject->registerObjects();
  }
}

SimRobot::Widget* SimObject::createWidget()
{
  return new SimObjectWidget(*this);
}

const QIcon* SimObject::getIcon() const
{
  return &CoreModule::module->objectIcon;
}

SimRobotCore2::Renderer* SimObject::createRenderer()
{
  return new SimObjectRenderer(*this);
}
