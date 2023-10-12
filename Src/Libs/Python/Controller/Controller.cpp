/**
 * @file Controller.cpp
 *
 * This file implements a class that controls a set of robots.
 *
 * @author Arne Hasselbring
 */

#include "Controller.h"
#include "Framework/Communication.h"
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include "Platform/Time.h"
#include <stdexcept>
#ifdef WINDOWS
#include <Windows.h> // not used directly, but needed for timeapi.h
#include <timeapi.h>
#endif

Controller* Controller::controller = nullptr;

Controller::Controller()
{
  if(controller)
    throw std::runtime_error("Only one controller instance can exist at a time.");
  controller = this;
  Time::initialize();
  Time::setSimulatedTime(true);
}

Controller::~Controller()
{
  Time::deinitialize();
  controller = nullptr;
}

void Controller::start()
{
#ifdef WINDOWS
  VERIFY(timeBeginPeriod(1) == TIMERR_NOERROR);
#endif
  DebugSenderBase::terminating = false;
  for(auto& robot : robots)
    robot->start();
}

void Controller::stop()
{
  DebugSenderBase::terminating = true;
  for(auto& robot : robots)
    robot->announceStop();
  for(auto& robot : robots)
    robot->stop();
#ifdef WINDOWS
  VERIFY(timeEndPeriod(1) == TIMERR_NOERROR);
#endif
}

void Controller::update()
{
  for(auto& robot : robots)
    robot->update();
}

void Controller::addPlayer(const std::string& name, int teamNumber, Settings::TeamColor fieldPlayerColor, Settings::TeamColor goalkeeperColor, int playerNumber, const std::string& location, const std::string& scenario)
{
  robots.emplace_back(std::make_unique<PythonRobot>(Settings("Nao", "Nao", teamNumber, fieldPlayerColor, goalkeeperColor, playerNumber, location, scenario, 0), name));
}

SystemCall::Mode SystemCall::getMode()
{
  return SystemCall::simulatedRobot;
}
