/**
 * @file Controller/RemoteRobot.cpp
 * Implementation of the base class of threads that communicate with a remote robot.
 * @author Thomas Röfer
 */

#include "RemoteRobot.h"
#include "ConsoleRoboCupCtrl.h"
#include "Platform/Time.h"

RemoteRobot::RemoteRobot(const std::string& name, const std::string& ip) :
  RobotConsole(nullptr, nullptr), name(name), ip(ip)
{
  mode = SystemCall::remoteRobot;
  puppet = (SimRobotCore2::Body*)RoboCupCtrl::application->resolveObject("RoboCup.puppets." + robotName, SimRobotCore2::body);
  if(puppet)
    simulatedRobot.init(puppet);

  // try to connect for one second
  Thread::start(this, &RemoteRobot::connect);
  Thread::stop();
}

void RemoteRobot::connect()
{
  Thread::nameCurrentThread(name + ".RemoteRobot.connect");
  TcpConnection::connect(ip.c_str(), 9999, TcpConnection::sender);
}

bool RemoteRobot::main()
{
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

  // exchange data with the router
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

  // If a packet was received from the router program, add it to receiver queue
  if(receivedSize > 0)
  {
    SYNC;
    InBinaryMemory stream(receivedData, receivedSize);
    stream >> *debugReceiver;
    delete[] receivedData;
  }

  Thread::sleep(receivedSize > 0 ? 1 : 20);
  return false;
}

void RemoteRobot::announceStop()
{
  {
    SYNC;
    debugSender->out.bin << DebugRequest("disableAll");
    debugSender->out.finishMessage(idDebugRequest);
  }
  Thread::sleep(1000);
  Thread::announceStop();
}

void RemoteRobot::update()
{
  RobotConsole::update();

  if(puppet)
  {
    if(RobotConsole::jointSensorData.timestamp)
      simulatedRobot.setJointRequest(reinterpret_cast<JointRequest&>(RobotConsole::jointSensorData));
    else
      simulatedRobot.setJointRequest(jointRequest);
    if(moveOp != noMove)
    {
      if(moveOp == moveBoth)
        simulatedRobot.moveRobot(movePos, moveRot * 1_deg, true);
      else if(moveOp == movePosition)
        simulatedRobot.moveRobot(movePos, Vector3f::Zero(), false);
      else if(moveOp == moveBallPosition)
        simulatedRobot.moveBall(movePos);
      moveOp = noMove;
    }
  }
  if(Time::getTimeSince(timestamp) >= 2000)
  {
    int bytes = this->getOverallBytesSent() + this->getOverallBytesReceived() - bytesTransfered;
    bytesTransfered += bytes;
    timestamp = Time::getCurrentSystemTime();
    transferSpeed = bytes / 2000.0f;
  }

  char buf[33];
  sprintf(buf, "%.1lf kb/s", transferSpeed);
  QString statusText = robotName.mid(robotName.lastIndexOf(".") + 1).toUtf8().constData() +
                       (isConnected() ? QString(": connected to ") + QString::fromStdString(ip) + ", " + buf
                        : QString(": connection lost from ") + QString::fromStdString(ip));

  if(logPlayer.getNumberOfMessages() != 0)
  {
    sprintf(buf, "%u", logPlayer.numberOfFrames);
    statusText += QString(", recorded ") + buf;
  }

  if(pollingFor)
  {
    statusText += statusText != "" ? ", polling for " : "polling for ";
    statusText += pollingFor;
  }

  ctrl->printStatusText(statusText);
}
