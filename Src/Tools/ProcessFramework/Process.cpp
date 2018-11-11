/**
 * @file Process.cpp
 *
 * Implementation of class Process.
 */

#include "Process.h"
#include "Tools/Global.h"
#include <asmjit/asmjit.h>

bool DebugSenderBase::terminating = false;

Process::Process(MessageQueue& debugIn, MessageQueue& debugOut) :
  debugIn(debugIn), debugOut(debugOut), asmjitRuntime(new asmjit::JitRuntime())
{
  setGlobals();
  initialized = false;
}

Process::~Process()
{
  delete asmjitRuntime;
}

bool Process::processMain()
{
  if(!initialized)
  {
    init();
    initialized = true;
  }

  handleAllMessages(debugIn);
  debugIn.clear();

  bool wait = main();

  if(Global::getDebugRequestTable().pollCounter > 0 &&
     --Global::getDebugRequestTable().pollCounter == 0)
    OUTPUT(idDebugResponse, text, "pollingFinished");
  return wait;
}

void Process::handleAllMessages(MessageQueue& messageQueue)
{
  debugIn.handleAllMessages(*this);
}

void Process::setGlobals()
{
  Global::theAnnotationManager = &annotationManager;
  Global::theDebugOut = &debugOut.out;
  Global::theSettings = &settings;
  Global::theDebugRequestTable = &debugRequestTable;
  Global::theDebugDataTable = &debugDataTable;
  Global::theDrawingManager = &drawingManager;
  Global::theDrawingManager3D = &drawingManager3D;
  Global::theTimingManager = &timingManager;
  Global::theAsmjitRuntime = asmjitRuntime;

  Blackboard::setInstance(blackboard); // blackboard is NOT globally accessible
}

bool Process::handleMessage(InMessage& message)
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
