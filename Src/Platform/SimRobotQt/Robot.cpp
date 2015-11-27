/**
 * @file Platform/SimRobotQt/Robot.cpp
 *
 * This file implements the class Robot.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include <QString>
#include <QFileInfo>

#include "Controller/LocalRobot.h"
#include "Robot.h"
#include "Controller/ConsoleRoboCupCtrl.h"

extern "C" DLL_EXPORT SimRobot::Module* createModule(SimRobot::Application& simRobot)
{
  QFileInfo info(simRobot.getFilePath());
  QString baseName = info.baseName();
  return new ConsoleRoboCupCtrl(simRobot);
}

MAKE_PROCESS(LocalRobot);

Robot::Robot(const char* name)
  : name(name)
{
  InMapFile cm("Processes/connect.cfg");
  ASSERT(cm.exists());

  // attach receivers to senders
  ConnectionParameter cp;
  cm >> cp;
  for(std::vector<ConnectionParameter::ProcessConnection>::const_iterator it = cp.processConnections.begin(); it != cp.processConnections.end(); ++it)
  {
    connect(getSender(it->sender.c_str()), getReceiver(it->receiver.c_str()));
  }

  // connect additional senders and receivers
  connect(getSender("Debug.Sender.MessageQueue.S"), getReceiver("LocalRobot.Receiver.MessageQueue.O"));
  connect(getSender("LocalRobot.Sender.MessageQueue.S"), getReceiver("Debug.Receiver.MessageQueue.O"));

  robotProcess = 0;
  for(ProcessList::const_iterator i = begin(); i != end() && !robotProcess; ++i)
    robotProcess = (LocalRobot*)(*i)->getProcess("LocalRobot");
  ASSERT(robotProcess);
}

void Robot::update()
{
  robotProcess->update();
}

void Robot::connect(SenderList* sender, ReceiverList* receiver)
{
  if(sender && receiver)
    sender->add(receiver);
}

SenderList* Robot::getSender(const std::string& senderName)
{
  for(ProcessList::const_iterator i = begin(); i != end(); ++i)
  {
    std::string name;
    if(senderName[0] == '.')
      name = (*i)->getName();
    else
      name = "";
    name += senderName;
    SenderList* sender = (*i)->lookupSender(name);
    if(sender)
      return sender;
  }
  TRACE("%s not found", senderName.c_str());
  return 0;
}

ReceiverList* Robot::getReceiver(const std::string& receiverName)
{
  for(ProcessList::const_iterator i = begin(); i != end(); ++i)
  {
    ReceiverList* receiver = (*i)->lookupReceiver(receiverName);
    if(receiver)
      return receiver;
  }
  TRACE("%s not found", receiverName.c_str());
  return 0;
}
