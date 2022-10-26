/**
 * @file SimulatedNao/RemoteConsole.cpp
 * Implementation of the base class of threads that communicate with a remote robot.
 * @author Thomas Röfer
 */

#include "RemoteConsole.h"
#include "ConsoleRoboCupCtrl.h"
#include "SimulatedRobot3D.h"
#include "Platform/Time.h"

RemoteConsole::RemoteConsole(const std::string& robotName, const std::string& ip, ConsoleRoboCupCtrl* ctrl) :
  RobotConsole(Settings("Nao", "Nao"), robotName, ctrl, SystemCall::remoteRobot, nullptr, nullptr), ip(ip)
{
  if(!ctrl->is2D)
  {
    SimRobot::Object* puppet = RoboCupCtrl::application->resolveObject("RoboCup.puppets." + QString::fromStdString(robotName), SimRobotCore2::body);
    if(puppet)
      simulatedRobot = std::make_unique<SimulatedRobot3D>(puppet);
  }

  // try to connect for one second
  Thread::start(this, &RemoteConsole::connect);
  Thread::stop();
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
  MessageQueue temp;

  // If there is something to send, prepare a packet
  if(!debugSender->isEmpty())
  {
    SYNC;
    sendSize = static_cast<int>(debugSender->getStreamedSize());
    OutBinaryMemory stream(sendSize);
    stream << *debugSender;
    sendData = reinterpret_cast<unsigned char*>(stream.obtainData());
    // make backup
    debugSender->moveAllMessages(temp);
  }

  // exchange data with the robot
  if(!sendAndReceive(sendData, sendSize, receivedData, receivedSize) && sendSize)
  {
    // sending failed, restore theDebugSender
    SYNC;
    // move all messages since cleared (if any)
    debugSender->moveAllMessages(temp);
    // restore
    temp.moveAllMessages(*debugSender);
  }

  // If a packet was prepared, remove it
  if(sendSize)
    delete[] sendData;

  // If a packet was received from the robot, add it to receiver queue
  if(receivedSize > 0)
  {
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

void RemoteConsole::announceStop()
{
  {
    SYNC;
    debugSender->out.bin << DebugRequest("disableAll");
    debugSender->out.finishMessage(idDebugRequest);
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
      simulatedRobot->setJointRequest(reinterpret_cast<JointRequest&>(RobotConsole::jointSensorData));
    else
      simulatedRobot->setJointRequest(jointRequest);
  }
  if(Time::getTimeSince(timestamp) >= 2000)
  {
    int bytes = this->getOverallBytesSent() + this->getOverallBytesReceived() - bytesTransferred;
    bytesTransferred += bytes;
    timestamp = Time::getCurrentSystemTime();
    transferSpeed = bytes / 2000.0f;
  }

  char buf[33];
  sprintf(buf, "%.1lf kb/s", transferSpeed);
  QString statusText = QString::fromStdString(robotName) +
                       (isConnected() ? QString(": connected to ") + QString::fromStdString(ip) + ", " + buf
                        : QString(": connection lost from ") + QString::fromStdString(ip));

  {
    SYNC;
    if(logPlayer.getNumberOfMessages() != 0)
    {
      sprintf(buf, "%u", logPlayer.numberOfFrames);
      statusText += QString(", recorded ") + buf;
    }
  }

  if(pollingFor)
  {
    statusText += statusText != "" ? ", polling for " : "polling for ";
    statusText += pollingFor;
  }

  ctrl->printStatusText(statusText);
}
