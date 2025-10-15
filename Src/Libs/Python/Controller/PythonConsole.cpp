/**
 * @file PythonConsole.cpp
 *
 * This file implements a class that controls a particular robot instance.
 *
 * @author Arne Hasselbring
 */

#include "PythonConsole.h"
#include "Framework/Communication.h"
#include "Framework/Debug.h"

PythonConsole::PythonConsole(const Settings& settings, const std::string& robotName, Debug* debug) :
  ThreadFrame(settings, robotName, connectReceiverWithRobot(debug), connectSenderWithRobot(debug))
{
}

PythonConsole::~PythonConsole()
{
}

void PythonConsole::update()
{
}

void PythonConsole::init()
{
}

bool PythonConsole::main()
{
  return true;
}

DebugReceiver<MessageQueue>* PythonConsole::connectReceiverWithRobot(Debug* debug)
{
  ASSERT(!debug->debugSender);
  DebugReceiver<MessageQueue>* receiver = new DebugReceiver<MessageQueue>(this, debug->getName());
  debug->debugSender = new DebugSender<MessageQueue>(*receiver, "PythonConsole");
  return receiver;
}

DebugSender<MessageQueue>* PythonConsole::connectSenderWithRobot(Debug* debug) const
{
  ASSERT(!debug->debugReceiver);
  debug->debugReceiver = new DebugReceiver<MessageQueue>(debug, "PythonConsole");
  return new DebugSender<MessageQueue>(*debug->debugReceiver, debug->getName());
}
