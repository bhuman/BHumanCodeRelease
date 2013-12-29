#include "Utils/bush/agents/RemoteRobotAgent.h"
#include "Utils/bush/models/Robot.h"
#include "Utils/bush/bhwrapper/Framework.h"
#include "Utils/bush/tools/StringTools.h"
#include "Utils/bush/Session.h"
#include "Tools/Debugging/TcpConnection.h"
#include "Tools/MessageQueue/MessageQueue.h"
#include <cstring>

void communicate(TcpConnection& connection, MessageQueue& theDebugReceiver, MessageQueue& theDebugSender)
{
  // code from Controller/RemoteRobot.cpp
  MessageQueue temp;
  unsigned char* sendData = 0, * receivedData;
  int sendSize = 0, receivedSize = 0;

  // If there is something to send, prepare a package
  if(!theDebugSender.isEmpty())
  {
    OutBinarySize size;
    size << theDebugSender;
    sendSize = size.getSize();
    sendData = new unsigned char[sendSize];
    OutBinaryMemory stream(sendData);
    stream << theDebugSender;
    // make backup
    theDebugSender.moveAllMessages(temp);
  }

  // exchange data with the router
  if(!connection.sendAndReceive(sendData, sendSize, receivedData, receivedSize) && sendSize)
  {
    // sending failed, restore theDebugSender
    // move all messages since cleared (if any)
    theDebugSender.moveAllMessages(temp);
    // restore
    temp.moveAllMessages(theDebugSender);
  }

  // If a package was prepared, remove it
  if(sendSize)
  {
    delete [] sendData;
    sendSize = 0;
  }

  // If a package was received from the router program, add it to receiver queue
  if(receivedSize > 0)
  {
    InBinaryMemory stream(receivedData, receivedSize);
    stream >> theDebugReceiver;
    delete [] receivedData;
  }
}

class RemoteMessageHandler : public MessageHandler
{
  RemoteRobotAgent* parent;
public:
  RemoteMessageHandler(RemoteRobotAgent* parent) : parent(parent) {}

  virtual bool handleMessage(InMessage& message)
  {
    Session::getInstance().log(WARN, toString(message.getMessageID()));
    switch(message.getMessageID())
    {
    case idText:
    {
      std::string answer;
      message.bin >> answer;
      parent->appendAnswer(answer);
      return true;
    }
    default:
      return false;
    }
  }
};

void RemoteRobotAgent::appendAnswer(const std::string& answer)
{
  answers.push_back(answer);
}

std::vector<std::string> RemoteRobotAgent::sendDebugRequestToRobot(const Robot* robot, const std::string& command)
{
  bool reachable = Session::getInstance().isReachable(robot);
  char ip[16]; // xxx.xxx.xxx.xxx\0
  if(reachable)
  {
    strcpy(ip, Session::getInstance().getBestIP(robot).c_str());
  }
  else
  {
    std::vector<std::string> result(1, "Robot " + robot->name + " not reachable.");
    return result;
  }

  std::vector<std::string> cmdWithArgs = split(command);
  if(cmdWithArgs.size() < 2)
  {
    std::vector<std::string> result(1, "Too few arguments for debug request.");
    return result;
  }

  bool enable = false;

  if(cmdWithArgs.size() == 2 || cmdWithArgs[2] == "on")
  {
    enable = true;
  }
  else if(cmdWithArgs[2] == "off")
  {
    enable = false;
  }
  else
  {
    std::vector<std::string> result(1, "Syntax error: " + command);
    return result;
  }

  // TODO currently we cannot read any answers, maybe we should wait longer?
  DebugRequest dr(cmdWithArgs[1], enable);

  std::string processName = "bush.RemoteRobotAgent." + robot->name;
  Framework* bhFramework = Framework::getInstance(processName);
  TcpConnection connection;
  connection.connect(ip, 0xA1BD, TcpConnection::sender);
  MessageQueue theDebugReceiver, theDebugSender;
  RemoteMessageHandler handler(this);
  communicate(connection, theDebugReceiver, theDebugSender);
  theDebugReceiver.handleAllMessages(handler);

  theDebugSender.out.bin << dr;
  theDebugSender.out.finishMessage(idDebugRequest);
  communicate(connection, theDebugReceiver, theDebugSender);
  theDebugReceiver.handleAllMessages(handler);

  theDebugSender.out.bin << DebugRequest("disableAll");
  theDebugSender.out.finishMessage(idDebugRequest);
  communicate(connection, theDebugReceiver, theDebugSender);
  theDebugReceiver.handleAllMessages(handler);

  Framework::destroy(processName);
  bhFramework = 0;

  appendAnswer("Sent debug request.");

  std::vector<std::string> result = answers;
  answers.clear();
  return result;
}
