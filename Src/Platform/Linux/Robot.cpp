/**
* @file Platform/linux/Robot.cpp
* This file implements a class that implements a robot as a list of processes.
* @author <a href="mailto:Bernd.Gersdorf@dfki.de">Bernd Gersdorf</a>
*/

#include "Robot.h"
#include "Tools/Streams/InStreams.h"

Robot::Robot()
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
}

void Robot::connect(SenderList* sender, ReceiverList* receiver)
{
  if(sender && receiver)
    sender->add(receiver);
}

SenderList* Robot::getSender(const std::string& senderName)
{
  std::string name;
  for(ProcessList::const_iterator i = begin(); i != end(); ++i)
  {
    if(senderName[0] == '.')
      name = (*i)->getName();
    else
      name = "";
    name += senderName;
    SenderList* sender = (*i)->lookupSender(name);
    if(sender)
      return sender;
  }
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
  return 0;
}
