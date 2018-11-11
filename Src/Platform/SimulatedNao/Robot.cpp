/**
 * @file Platform/SimulatedNao/Robot.cpp
 *
 * This file implements the class Robot.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include <QString>
#include <QFileInfo>

#include "Controller/ConsoleRoboCupCtrl.h"
#include "Controller/LocalRobot.h"
#include "Robot.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/FunctionList.h"
#include "Tools/Streams/TypeRegistry.h"

extern "C" DLL_EXPORT SimRobot::Module* createModule(SimRobot::Application& simRobot)
{
  FunctionList::execute();
  //TypeRegistry::print(); // @todo Remove later
  QFileInfo info(simRobot.getFilePath());
  QString baseName = info.baseName();
  return new ConsoleRoboCupCtrl(simRobot);
}

MAKE_PROCESS(LocalRobot);

Robot::Robot(const std::string& name) :
  name(name)
{
  InMapFile cm("processes.cfg");
  ASSERT(cm.exists());

  // attach receivers to senders
  ConnectionParameter cp;
  cm >> cp;
  for(std::vector<ConnectionParameter::ProcessConnection>::const_iterator it = cp.processConnections.begin(); it != cp.processConnections.end(); ++it)
  {
    connect(getSender(it->sender.c_str()), getReceiver(it->receiver.c_str()));
  }

  // connect additional senders and receivers
  connect(getSender("Debug.MessageQueue"), getReceiver("LocalRobot.MessageQueue"));
  connect(getSender("LocalRobot.MessageQueue"), getReceiver("Debug.MessageQueue"));

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
  OUTPUT_ERROR(senderName.c_str() << " not found");
  return nullptr;
}

ReceiverList* Robot::getReceiver(const std::string& receiverName)
{
  for(ProcessList::const_iterator i = begin(); i != end(); ++i)
  {
    ReceiverList* receiver = (*i)->lookupReceiver(receiverName);
    if(receiver)
      return receiver;
  }
  OUTPUT_ERROR(receiverName.c_str() << " not found");
  return nullptr;
}
