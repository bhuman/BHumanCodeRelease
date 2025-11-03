/**
 * @file SimulatedNao/RemoteConsole.cpp
 * Implementation of the base class of threads that communicate with a remote robot.
 * @author Thomas Röfer
 */

#include "RemoteConsole.h"
#include "ConsoleRoboCupCtrl.h"
#include "SimulatedRobot3D.h"
#include "Math/Constants.h"
#include "Platform/Time.h"

RemoteConsole::RemoteConsole(const std::string& robotName, const std::string& ip, ConsoleRoboCupCtrl* ctrl, const Settings& settings) :
  RobotConsole(settings, robotName, ctrl, SystemCall::remoteRobot, nullptr, nullptr), ip(ip)
{
  if(!ctrl->is2D)
  {
    SimRobot::Object* puppet = RoboCupCtrl::application->resolveObject("RoboCup.puppets." + QString::fromStdString(robotName), SimRobotCore3::body);
    if(puppet)
    {
      simulatedRobot = std::make_unique<SimulatedRobot3D>(puppet);
      simulatedRobot->enableGravity(false);
    }
  }

  // try to connect for one second
  Thread::start(this, &RemoteConsole::connect);
  Thread::stop(1000);
}

void RemoteConsole::connect()
{
  Thread::nameCurrentThread(robotName + ".RemoteConsole.connect");
  TcpConnection::connect(ip.c_str(), 9999, TcpConnection::sender);
}

bool RemoteConsole::main()
{
  // This method is the counterpart of DebugHandler::communicate.
  unsigned char* sendData = nullptr;
  unsigned char* receivedData;
  int sendSize = 0;
  int receivedSize = 0;

  // If there is something to send, prepare a packet
  if(!numOfDataMessageIDsRequestSent)
  {
    sendSize = sizeof(MessageQueue::QueueHeader) + sizeof(MessageQueue::MessageHeader);
    sendData = new unsigned char[sendSize]();
    *sendData = sizeof(MessageQueue::MessageHeader);
  }
  else if(numOfDataMessageIDsReceived && !debugSender->empty())
  {
    {
      SYNC;
      sendSize = static_cast<int>(sizeof(MessageQueue::QueueHeader) + debugSender->size());
      OutBinaryMemory stream(sendSize);
      stream << *debugSender;
      sendData = reinterpret_cast<unsigned char*>(stream.obtainData());
    }
    if(messageIdOffset)
      correctMessageIDs(sendData, sendSize, -messageIdOffset);
  }

  // exchange data with the robot
  if(sendAndReceive(sendData, sendSize, receivedData, receivedSize) && sendSize)
  {
    if(numOfDataMessageIDsRequestSent)
    {
      // Remove messages from queue that were sent.
      SYNC;
      if(sendSize == static_cast<int>(debugSender->size() + sizeof(MessageQueue::QueueHeader)))
        debugSender->clear();
      else
        debugSender->filter([this, originalSize = sendSize - sizeof(MessageQueue::QueueHeader)](MessageQueue::const_iterator i)
        {
          return i - debugSender->begin() >= originalSize;
        });
    }
    else
      numOfDataMessageIDsRequestSent = true;
  }

  // If a packet was prepared, remove it
  if(sendSize)
    delete[] sendData;

  // If a packet was received from the robot, add it to receiver queue
  if(receivedSize > 0)
  {
    if(messageIdOffset)
      correctMessageIDs(receivedData, receivedSize, messageIdOffset);
    InBinaryMemory stream(receivedData, receivedSize);
    {
      SYNC;
      stream >> *debugReceiver;
    }
    delete[] receivedData;
  }

  Thread::sleep(receivedSize > 0 ? 1 : 20);
  return false;
}

bool RemoteConsole::handleMessage(MessageQueue::Message message)
{
  if(message.id() == idNumOfDataMessageIDs)
  {
    MessageID otherNumOfDataMessageIDs;
    message.bin() >> otherNumOfDataMessageIDs;
    messageIdOffset = numOfDataMessageIDs - otherNumOfDataMessageIDs;
    numOfDataMessageIDsReceived = true;
    return true;
  }
  return RobotConsole::handleMessage(message);
}

void RemoteConsole::announceStop()
{
  {
    SYNC;
    debugSender->bin(idDebugRequest) << DebugRequest("disableAll");
  }
  Thread::sleep(1000);
  Thread::announceStop();
}

void RemoteConsole::update()
{
  RobotConsole::update();

  if(simulatedRobot)
  {
    SYNC;
    if(RobotConsole::jointSensorData.timestamp)
      simulatedRobot->setJointRequest(reinterpret_cast<JointRequest&>(RobotConsole::jointSensorData), true);
    else
      simulatedRobot->getAndSetJointData(jointRequest, jointSensorData);
    simulatedRobot->moveRobot({ 0, 0, -1000.f }, {0, 0, 0}, true, true);
  }
  if(Time::getTimeSince(timestamp) >= 2000)
  {
    int bytes = this->getOverallBytesSent() + this->getOverallBytesReceived() - bytesTransferred;
    bytesTransferred += bytes;
    timestamp = Time::getCurrentSystemTime();
    transferSpeed = bytes / 2000.0f;
  }

  QString statusText = QString::fromStdString(robotName) +
                       (isConnected()
                        ? QString(": connected to %1, %2 kb/s").arg(ip.c_str()).arg(transferSpeed, 0, 'f', 1)
                        : QString(": connection lost from %1").arg(ip.c_str()));

  {
    SYNC;
    if(!logPlayer.empty())
      statusText += QString(", recorded %1 mb").arg(static_cast<float>(logPlayer.size()) / 1000000.f, 0, 'f', 1);
  }

  if(pollingFor)
  {
    statusText += statusText != "" ? ", polling for " : "polling for ";
    statusText += pollingFor;
  }

  ctrl->printStatusText(statusText);
}

void RemoteConsole::correctMessageIDs(unsigned char* buffer, size_t size, char offset)
{
  for(unsigned char* p = buffer + sizeof(MessageQueue::QueueHeader), *end = buffer + size; p < end;
      p += sizeof(MessageQueue::MessageHeader) + reinterpret_cast<MessageQueue::MessageHeader*>(p)->size)
    if(static_cast<unsigned char>(*p + offset) >= numOfDataMessageIDs)
      *p += offset;
}
