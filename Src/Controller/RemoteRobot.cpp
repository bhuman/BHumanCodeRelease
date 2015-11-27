/**
* @file Controller/RemoteRobot.cpp
* Implementation of the base class of processes that communicate with a remote robot.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include "RemoteRobot.h"
#include "ConsoleRoboCupCtrl.h"

RemoteRobot::RemoteRobot(const char* name, const char* ip) :
  RobotConsole((setGlobals(), theDebugReceiver), theDebugSender),
  theDebugReceiver(this, "Receiver.MessageQueue.O"),
  theDebugSender(this, "Sender.MessageQueue.S"),
  bytesTransfered(0),
  transferSpeed(0),
  timeStamp(0)
{
  strcpy(this->name, name);
  strcpy(this->ip, ip);
  mode = SystemCall::remoteRobot;
  puppet = (SimRobotCore2::Body*)RoboCupCtrl::application->resolveObject("RoboCup.puppets." + robotName, SimRobotCore2::body);
  if(puppet)
    simulatedRobot.init(puppet);

  // try to connect for one second
  Thread<RemoteRobot>::start(this, &RemoteRobot::connect);
  Thread<RemoteRobot>::stop();
}

void RemoteRobot::connect()
{
  NAME_THREAD((std::string(name) + ".RemoteRobot.connect").c_str());
  TcpConnection::connect(*ip ? ip : 0, 0xA1BD, TcpConnection::sender);
}

void RemoteRobot::run()
{
  NAME_THREAD((std::string(name) + ".RemoteRobot").c_str());
  setGlobals();
  while(isRunning())
    processMain();
}

bool RemoteRobot::main()
{
  unsigned char* sendData = 0,
               * receivedData;
  int sendSize = 0,
      receivedSize = 0;
  MessageQueue temp;

  // If there is something to send, prepare a package
  if(!theDebugSender.isEmpty())
  {
    SYNC;
    OutBinarySize size;
    size << theDebugSender;
    sendSize = (int) size.getSize();
    sendData = new unsigned char[sendSize];
    OutBinaryMemory stream(sendData);
    stream << theDebugSender;
    // make backup
    theDebugSender.moveAllMessages(temp);
  }

  // exchange data with the router
  if(!sendAndReceive(sendData, sendSize, receivedData, receivedSize) && sendSize)
  {
    // sending failed, restore theDebugSender
    SYNC;
    // move all messages since cleared (if any)
    theDebugSender.moveAllMessages(temp);
    // restore
    temp.moveAllMessages(theDebugSender);
  }

  // If a package was prepared, remove it
  if(sendSize)
    delete [] sendData;

  // If a package was received from the router program, add it to receiver queue
  if(receivedSize > 0)
  {
    SYNC;
    InBinaryMemory stream(receivedData, receivedSize);
    stream >> theDebugReceiver;
    delete [] receivedData;
  }

  SystemCall::sleep(receivedSize > 0 ? 1 : 20);
  return false;
}

void RemoteRobot::announceStop()
{
  {
    SYNC;
    debugOut.out.bin << DebugRequest("disableAll");
    debugOut.out.finishMessage(idDebugRequest);
  }
  SystemCall::sleep(1000);
  Thread<RemoteRobot>::announceStop();
}

void RemoteRobot::update()
{
  RobotConsole::update();

  if(puppet)
  {
    simulatedRobot.setJointRequest(jointRequest);
    if(moveOp != noMove)
    {
      if(moveOp == moveBoth)
        simulatedRobot.moveRobot(movePos, moveRot * 1_deg,true);
      else if(moveOp == movePosition)
        simulatedRobot.moveRobot(movePos, Vector3f::Zero(),false);
      else if(moveOp == moveBallPosition)
        simulatedRobot.moveBall(movePos);
      moveOp = noMove;
    }
  }
  if(SystemCall::getTimeSince(timeStamp) >= 2000)
  {
    int bytes = this->getOverallBytesSent() + this->getOverallBytesReceived() - bytesTransfered;
    bytesTransfered += bytes;
    timeStamp = SystemCall::getCurrentSystemTime();
    transferSpeed = bytes / 2000.0f;
  }

  char buf[33];
  sprintf(buf, "%.1lf kb/s", transferSpeed);
  std::string statusText = robotName.mid(robotName.lastIndexOf(".") + 1).toUtf8().constData() +
                           (isConnected() ? std::string(": connected to ") + ip + ", " + buf
                                          : std::string(": connection lost from ") + ip);

  if(logPlayer.getNumberOfMessages() != 0)
  {
    sprintf(buf, "%u", logPlayer.numberOfFrames);
    statusText += std::string(", recorded ") + buf;
  }

  if(pollingFor)
  {
    statusText += statusText != "" ? ", polling for " : "polling for ";
    statusText += pollingFor;
  }

  ctrl->printStatusText(statusText);
}
