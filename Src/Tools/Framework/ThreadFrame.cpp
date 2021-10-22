/**
 * @file Tools/Framework/ThreadFrame.cpp
 *
 * This file implements classes corresponding to the thread framework.
 *
 * @author Thomas Röfer
 * @author Jan Fiedler
 */

#include "ThreadFrame.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Global.h"
#include <asmjit/asmjit.h>

ThreadFrame::ThreadFrame(const Settings& settings, const std::string& robotName) :
  settings(settings),
  asmjitRuntime(new asmjit::JitRuntime()),
  robotName(robotName)
{
  // Set settings as soon as possible for file access.
  Global::theSettings = &this->settings;
}

ThreadFrame::ThreadFrame(const Settings& settings, const std::string& robotName, DebugReceiver<MessageQueue>* debugReceiver, DebugSender<MessageQueue>* debugSender) :
  debugReceiver(debugReceiver),
  debugSender(debugSender),
  settings(settings),
  asmjitRuntime(new asmjit::JitRuntime()),
  robotName(robotName)
{
  // Set settings as soon as possible for file access and debugOut for debugging.
  Global::theSettings = &this->settings;

  // Initialize MessageQueues if no messaging is required.
  if(!debugReceiver && !debugSender)
  {
    this->debugReceiver = new DebugReceiver<MessageQueue>(nullptr, Communication::dummy);
    this->debugSender = new DebugSender<MessageQueue>(*this->debugReceiver, Communication::dummy);
  }

  Global::theDebugOut = &this->debugSender->out;
}

ThreadFrame::~ThreadFrame()
{
  setGlobals();
  delete debugReceiver;
  delete debugSender;
  delete asmjitRuntime;
}

void ThreadFrame::setGlobals()
{
  Global::theAnnotationManager = &annotationManager;
  Global::theDebugOut = &debugSender->out;
  Global::theSettings = &settings;
  Global::theDebugRequestTable = &debugRequestTable;
  Global::theDebugDataTable = &debugDataTable;
  Global::theDrawingManager = &drawingManager;
  Global::theDrawingManager3D = &drawingManager3D;
  Global::theTimingManager = &timingManager;
  Global::theAsmjitRuntime = asmjitRuntime;

  Blackboard::setInstance(blackboard); // blackboard is NOT globally accessible
}

void ThreadFrame::threadMain()
{
  Thread::nameCurrentThread(robotName.empty() ? getName() : (robotName + "." + getName()));

  if(SystemCall::getMode() == SystemCall::physicalRobot)
    setPriority(getPriority());
  else
    setPriority(0);
  Thread::yield(); // always leave processing time to other threads
  setGlobals();
  init();
  while(isRunning())
  {
    debugReceiver->checkForPacket();
    handleAllMessages(*debugReceiver);
    debugReceiver->clear();

    const bool shouldWait = main();

    if(Global::getDebugRequestTable().pollCounter > 0 &&
       --Global::getDebugRequestTable().pollCounter == 0)
      OUTPUT(idDebugResponse, text, "pollingFinished");

    if(shouldWait)
      wait();
  }
  terminate();
}

bool ThreadFrame::handleMessage(InMessage& message)
{
  switch(message.getMessageID())
  {
    case idDebugRequest:
    {
      DebugRequest debugRequest;
      message.bin >> debugRequest;
      Global::getDebugRequestTable().addRequest(debugRequest);
      return true;
    }
    case idDebugDataChangeRequest:
      Global::getDebugDataTable().processChangeRequest(message);
      return true;
    default:
      return false;
  }
}

void ThreadFrame::handleAllMessages(MessageQueue& messageQueue)
{
  messageQueue.handleAllMessages(*this);
}
