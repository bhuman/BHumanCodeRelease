/**
 * @file Tools/Framework/ModuleContainer.cpp
 *
 * This file implements a thread that executes a container of modules.
 *
 * @author Jan Fiedler
 */

#include "ModuleContainer.h"
#include "Platform/SystemCall.h"
#include "Platform/Time.h"
#include "Threads/Debug.h"
#include "Tools/Framework/FrameExecutionUnit.h"
#include "Tools/Logging/Logger.h"
#include "Tools/Math/Constants.h"

#include "Representations/Infrastructure/CameraInfo.h"

thread_local std::list<std::function<bool(InMessage& message)>> ModuleContainer::messageHandlers;

ModuleContainer::ModuleContainer(const Configuration& config, const std::size_t index, Logger* logger) :
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
}

ModuleContainer::~ModuleContainer()
{
  setGlobals();
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
  numberOfMessages = debugSender->getNumberOfMessages();
  OUTPUT(idFrameBegin, bin, getName());
}

bool ModuleContainer::main()
{
  for(Receiver<ModulePacket>& receiver : receivers)
    if(!moduleGraphRunner.receiverEmpty(receiver.index))
      receiver.checkForPacket();

  if((executionUnit->beforeFrame() || moduleGraphRunner.hasChanged()) && moduleGraphRunner.isValid())
  {
    Global::getTimingManager().signalThreadStart();
    Global::getAnnotationManager().signalThreadStart();

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

    Global::getTimingManager().signalThreadStop();
    logger->execute(getName());

    DEBUG_RESPONSE("timing") Global::getTimingManager().getData().copyAllMessages(*debugSender);

    DEBUG_RESPONSE("annotation") Global::getAnnotationManager().getOut().copyAllMessages(*debugSender);
    Global::getAnnotationManager().clear();

    if(debugSender->getNumberOfMessages() > numberOfMessages + 1)
    {
      // messages were sent in this frame -> send thread finished
      OUTPUT(idFrameFinished, bin, getName());
    }
    else
      debugSender->removeLastMessage();

    BH_TRACE_MSG("debugSender->send()");
    debugSender->send(SystemCall::getMode() == SystemCall::logFileReplay);

    // Prepare next frame
    numberOfMessages = debugSender->getNumberOfMessages();
    OUTPUT(idFrameBegin, bin, getName());
  }
  else if(Global::getDebugRequestTable().pollCounter > 0)
    // If the frame is not executed during polling, the pollcounter should not become smaller.
    ++Global::getDebugRequestTable().pollCounter;

  return executionUnit->afterFrame();
}

void ModuleContainer::terminate()
{
  if(SystemCall::getMode() == SystemCall::physicalRobot)
    setPriority(0);
  moduleGraphRunner.destroy();
}

bool ModuleContainer::handleMessage(InMessage& message)
{
  BH_TRACE_MSG("before" + getName() + ":handleMessage");
  if(message.getMessageID() == idModuleRequest)
  {
    moduleGraphRunner.update(message.bin);
    message.resetReadPosition();
  }
  for(const std::function<bool(InMessage& message)>& messageHandler : messageHandlers)
    if(messageHandler(message))
      return true;
  return message.getMessageID() == idModuleRequest || ThreadFrame::handleMessage(message);
}
