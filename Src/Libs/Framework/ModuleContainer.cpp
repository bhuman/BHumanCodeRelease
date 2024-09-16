/**
 * @file Framework/ModuleContainer.cpp
 *
 * This file implements a thread that executes a container of modules.
 *
 * @author Jan Fiedler
 */

#include "ModuleContainer.h"
#include "Debugging/AnnotationManager.h"
#include "Debugging/Debugging.h"
#include "Debugging/Stopwatch.h"
#include "Framework/Blackboard.h"
#include "Framework/Debug.h"
#include "Framework/FrameExecutionUnit.h"
#include "Framework/Logger.h"
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include "Streaming/Global.h"
#include "Streaming/Output.h"

thread_local std::list<std::function<bool(MessageQueue::Message message)>> ModuleContainer::messageHandlers;

ModuleContainer::ModuleContainer(const Settings& settings, const std::string& robotName, const Configuration& config, const std::size_t index, Logger* logger) :
  ThreadFrame(settings, robotName),
  name(config()[index].name),
  priority(config()[index].priority),
  moduleGraphRunner(config().size()),
  logger(logger)
{
  for(ExecutionUnitCreatorBase* i = ExecutionUnitCreatorBase::first; i; i = i->next)
  {
    if(config()[index].executionUnit == i->getName())
    {
      executionUnit = i->create();
      break;
    }
  }
  ASSERT(executionUnit);

  loggingController = executionUnit->initLogging(config, index);
}

ModuleContainer::~ModuleContainer()
{
  setGlobals();
  delete loggingController;
  delete executionUnit;
}

void ModuleContainer::connectWithSender(ModuleContainer* sender, const Configuration& config)
{
  receivers.emplace_back(this, sender->getName());
  receivers.back().moduleGraphRunner = &moduleGraphRunner;
  for(std::size_t i = 0; i < config().size(); i++)
    if(sender->getName() == config()[i].name)
    {
      receivers.back().index = i;
      break;
    }
  sender->senders.emplace_back(receivers.back(), getName());
  sender->senders.back().moduleGraphRunner = &sender->moduleGraphRunner;
  for(std::size_t i = 0; i < config().size(); i++)
    if(getName() == config()[i].name)
    {
      sender->senders.back().index = i;
      break;
    }
}

void ModuleContainer::connectWithDebug(Debug* debug, const Configuration::Thread& config)
{
  ASSERT(!debugSender && !debugReceiver);
  debug->receivers.emplace_back(debug, getName(), config.debugSenderSize);
  debugSender = new DebugSender<MessageQueue>(debug->receivers.back(), debug->getName(), config.debugSenderSize, config.debugSenderInfrastructureSize);

  debugReceiver = new DebugReceiver<MessageQueue>(this, debug->getName(), config.debugReceiverSize);
  debug->senders.emplace_back(*debugReceiver, getName(), config.debugReceiverSize);
}

void ModuleContainer::init()
{
  BH_TRACE_INIT(getName().c_str());

  // Prepare first frame
  originalSize = debugSender->size();
  OUTPUT(idFrameBegin, bin, getName());
  sizeAfterFrameBegin = debugSender->size();
}

bool ModuleContainer::main()
{
  for(Receiver<ModulePacket>& receiver : receivers)
    if(!moduleGraphRunner.receiverEmpty(receiver.index))
      receiver.receivePacket();

  if((executionUnit->beforeFrame() || moduleGraphRunner.hasChanged()) && moduleGraphRunner.isValid())
  {
    Global::getTimingManager().signalThreadStart();

    executionUnit->beforeModules();
    STOPWATCH("AllModules") moduleGraphRunner.execute();
    executionUnit->afterModules();

    DEBUG_RESPONSE_ONCE("automated requests:DrawingManager") OUTPUT(idDrawingManager, bin, Global::getDrawingManager());
    DEBUG_RESPONSE_ONCE("automated requests:DrawingManager3D") OUTPUT(idDrawingManager3D, bin, Global::getDrawingManager3D());

    for(Sender<ModulePacket>& sender : senders)
      if(!moduleGraphRunner.senderEmpty(sender.index))
      {
        BH_TRACE_MSG("before sender.send() to: " + sender.receiverThreadName);
        sender.send();
      }

    if(logger && loggingController)
      logger->update(*loggingController);

    const bool keepAnnotations = logger && !logger->execute(getName());

    DEBUG_RESPONSE("timing") *debugSender << Global::getTimingManager().getData();

    DEBUG_RESPONSE("annotation")
    {
      *debugSender << Global::getAnnotationManager().getOut();
      Global::getAnnotationManager().getOut().clear();
    }
    // If there is a logger that is not logging now (e.g. during INITIAL),
    // annotations should not be cleared so that they appear later in the log.
    else if(!keepAnnotations)
      Global::getAnnotationManager().getOut().clear();

    if(debugSender->size() > sizeAfterFrameBegin)
    {
      // messages were sent in this frame -> send thread finished
      OUTPUT(idFrameFinished, bin, getName());
    }
    else
      debugSender->resize(originalSize);

    BH_TRACE_MSG("debugSender->send()");
    debugSender->send();

    // Prepare next frame
    originalSize = debugSender->size();
    OUTPUT(idFrameBegin, bin, getName());
    sizeAfterFrameBegin = debugSender->size();
  }
  else
  {
    // If data was not sent in the previous frame or new data was appended
    // (idLogResponse or "pollingFinished"), try to send it now.
    if(originalSize > 0 || debugSender->size() > sizeAfterFrameBegin)
    {
      debugSender->send();

      if(debugSender->size() == 0)
      {
        // Prepare next frame
        originalSize = 0;
        OUTPUT(idFrameBegin, bin, getName());
        sizeAfterFrameBegin = debugSender->size();
      }
    }
    if(Global::getDebugRequestTable().pollCounter > 0)
      // If the frame is not executed during polling, the poll counter should not become smaller.
      ++Global::getDebugRequestTable().pollCounter;
  }

  if(executionUnit->afterFrame())
  {
    if(Global::getDebugRequestTable().pollCounter == 0
       && (Global::getDebugOut().size() == 0 || ++Global::getDebugOut().begin() == Global::getDebugOut().end()))
      return true;
    else
    {
      Thread::yield();
      return false;
    }
  }
  else
    return false;
}

void ModuleContainer::terminate()
{
  if(SystemCall::getMode() == SystemCall::physicalRobot)
    setPriority(0);
  moduleGraphRunner.destroy();
}

bool ModuleContainer::handleMessage(MessageQueue::Message message)
{
  BH_TRACE_MSG("before" + getName() + ":handleMessage");
  switch(message.id())
  {
    case idModuleRequest:
    {
      auto stream = message.bin();
      moduleGraphRunner.update(stream);
      return true;
    }
    default:
      for(const std::function<bool(MessageQueue::Message message)>& messageHandler : messageHandlers)
        if(messageHandler(message))
          return true;
      return ThreadFrame::handleMessage(message);
  }
}
