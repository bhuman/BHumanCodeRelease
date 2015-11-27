/**
* @file Processes/Debug.cpp
*
* Implementation of class Debug.
*
* @author Martin Lötzsch
*/

#include "Debug.h"
#include "Tools/Debugging/Debugging.h"
#include "Platform/SystemCall.h"

Debug::Debug() :
  INIT_EXTERNAL_DEBUGGING,
  INIT_DEBUG_RECEIVER(Cognition),
  INIT_DEBUG_RECEIVER(Motion),
  INIT_DEBUG_SENDER(Cognition),
  INIT_DEBUG_SENDER(Motion)
{
  theDebugSender.setSize(MAX_PACKAGE_SEND_SIZE - 2000);
  theDebugReceiver.setSize(MAX_PACKAGE_RECEIVE_SIZE - 2000);
  theCognitionReceiver.setSize(5200000);
  theCognitionSender.setSize(2800000);

  theMotionReceiver.setSize(70000);
  theMotionSender.setSize(200000);
  if(SystemCall::getMode() == SystemCall::physicalRobot)
    setPriority(5);
}

bool Debug::main()
{
  // Copying messages from debug queues from cognition and motion
  switch(outQueueMode.behavior)
  {
    case QueueFillRequest::sendCollected:
    case QueueFillRequest::discardNew:
    case QueueFillRequest::discardAll:
      // Discard new messages
      theCognitionReceiver.clear();
      theMotionReceiver.clear();
      break;

    default:
      // Move the messages from other processes' debug queues to the outgoing queue
      if(!theCognitionReceiver.isEmpty())
      {
        theCognitionReceiver.moveAllMessages(theDebugSender);
      }
      if(!theMotionReceiver.isEmpty())
      {
        OUTPUT(idProcessBegin, bin, 'm');
        theMotionReceiver.moveAllMessages(theDebugSender);
      }
  }

  // Handing behaviour
  bool sendNow = false;
  bool sendToGUI = false;
  switch(outQueueMode.behavior)
  {
    case QueueFillRequest::sendAfter:
      if(SystemCall::getCurrentSystemTime() > sendTime)
      {
        // Send messages that are in the queue (now matter how long it takes), but don't take new messages
        sendNow = true;
        outQueueMode.behavior = QueueFillRequest::sendCollected;
      }
      break;

    case QueueFillRequest::sendEvery:
      if(SystemCall::getCurrentSystemTime() > sendTime)
      {
        // Send now (if the network is busy, this send time is effectively skipped)
        sendNow = true;

        // Compute time for next sending
        sendTime = SystemCall::getCurrentSystemTime() + outQueueMode.timingMilliseconds;
      }
      break;

    case QueueFillRequest::collect:
    case QueueFillRequest::discardNew:
      // Don't send now
      break;

    case QueueFillRequest::discardAll:
      // Clear output queue
      theDebugSender.clear();
      break;

    case QueueFillRequest::sendImmediately:
    case QueueFillRequest::sendCollected:
    default:
      sendNow = true;
  }

  if(sendNow)
  {
    // Apply filter
    switch(outQueueMode.filter)
    {
      case QueueFillRequest::latestOnly:
        // Send only latest of each type
        theDebugSender.removeRepetitions();
        break;

      case QueueFillRequest::sendEverything:
        ; // Do nothing
    }

    // Send or save
    switch(outQueueMode.target)
    {
      case QueueFillRequest::writeToStick:
        if(!theDebugSender.isEmpty())
        {
          if(!fout)
          {
            fout = new OutBinaryFile("logfile.log");
            theDebugSender.writeAppendableHeader(*fout);
          }
          // Append the outgoing queue to the file on the memory stick
          theDebugSender.append(*fout);
          theDebugSender.clear();
        }
        break;

      case QueueFillRequest::sendViaNetwork:
        sendToGUI = true;
        break;
    }
  }

  // Send messages to the processes
#ifdef TARGET_SIM
  theCognitionSender.send(true);
  theMotionSender.send(true);
#else
  theCognitionSender.send(false);
  theMotionSender.send(false);
#endif

  DO_EXTERNAL_DEBUGGING(sendToGUI);
  return true;
}

void Debug::init()
{
  // read requests.dat
  InBinaryFile stream("requests.dat");
  if(stream.exists() && !stream.eof())
    stream >> theDebugReceiver;

  theDebugReceiver.handleAllMessages(*this);
  theDebugReceiver.clear();
}

bool Debug::handleMessage(InMessage& message)
{
  switch(message.getMessageID())
  {
    case idText: // loop back to GUI
      message >> theDebugSender;
      return true;

    // messages to Cognition
    case idColorCalibration:
      message >> theCognitionSender;
      return true;

    // messages to Motion
    case idMotionNet:
    case idWalkingEngineKick:
      message >> theMotionSender;
      return true;

    // messages to Debug
    case idQueueFillRequest:
      // Read message queue settings and compute time when next to send (if in a timed mode)
      message.bin >> outQueueMode;
      sendTime = SystemCall::getCurrentSystemTime() + outQueueMode.timingMilliseconds;
      if(fout)
      {
        delete fout;
        fout = 0;
      }
      return true;

    // messages to Cognition and Motion
    case idModuleRequest:
    case idDebugDataChangeRequest:
    case idStreamSpecification:
      message >> theCognitionSender;
      message >> theMotionSender;
      return true;

    // messages to all processes
    case idDebugRequest:
      message >> theCognitionSender;
      message >> theMotionSender;
      return Process::handleMessage(message);

    case idProcessBegin:
      message.bin >> processIdentifier;
      message.resetReadPosition();
      // no break

    default:
      if(processIdentifier == 'm')
        message >> theMotionSender;
      else
        message >> theCognitionSender;

      return true;
  }
}

MAKE_PROCESS(Debug);
