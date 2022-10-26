/**
 * @file Tools/Framework/ModuleContainer.cpp
 *
 * This file implements a thread that executes a container of modules.
 *
 * @author Jan Fiedler
 */

#include "ModuleContainer.h"
#include "Debugging/Annotation.h"
#include "Framework/Blackboard.h"
#include "Framework/Debug.h"
#include "Framework/FrameExecutionUnit.h"
#include "Framework/Logger.h"
#include "Math/Constants.h"
#include "Platform/SystemCall.h"
#include "Platform/Time.h"
#include "Representations/Infrastructure/GameState.h"
#include "Streaming/TypeRegistry.h"

thread_local std::list<std::function<bool(InMessage& message)>> ModuleContainer::messageHandlers;

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

  // If this module container is the (first) in which the GameState is provided, it is the "logging master".
  // If no thread provides the GameState (e.g. because it is a default representation), it doesn't help anyway.
  // Changing the GameState-providing thread at runtime doesn't make sense either.
  for(const auto& representationProvider : config()[index].representationProviders)
    if(representationProvider.representation == "GameState")
    {
      isLoggingMaster = true;
      for(std::size_t i = 0; i < index; ++i)
      {
        for(const auto& otherRepresentationProvider : config()[i].representationProviders)
          if(otherRepresentationProvider.representation == "GameState")
          {
            isLoggingMaster = false;
            break;
          }
      }
      break;
    }
  if(isLoggingMaster)
  {
    InMapFile stream("teamList.cfg");
    if(stream.exists())
    {
      TeamList teamList;
      stream >> teamList;
      for(const Team& team : teamList.teams)
        teams[team.number] = team.name;
    }
  }
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
      receiver.receivePacket();

  if((executionUnit->beforeFrame() || moduleGraphRunner.hasChanged()) && moduleGraphRunner.isValid())
  {
    Global::getTimingManager().signalThreadStart();
    if(Blackboard::getInstance().exists("GameState"))
    {
      const GameState& gameState = static_cast<const GameState&>(Blackboard::getInstance()["GameState"]);
      if(gameState.state != lastGameState)
        ANNOTATION("GameState", "Switched to " << TypeRegistry::getEnumName(gameState.state) << " state.");
      lastGameState = gameState.state;
    }

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

    if(logger && isLoggingMaster)
    {
      const GameState& gameState = static_cast<const GameState&>(Blackboard::getInstance()["GameState"]);
      logger->update(!(gameState.isInitial() || gameState.isFinished()), [this, &gameState]{ return getLogDescription(gameState); });
    }

    const bool keepAnnotations = logger && !logger->execute(getName());

    DEBUG_RESPONSE("timing") Global::getTimingManager().getData().copyAllMessages(*debugSender);

    DEBUG_RESPONSE("annotation")
    {
      Global::getAnnotationManager().getOut().copyAllMessages(*debugSender);
      Global::getAnnotationManager().getOut().clear();
    }
    // If there is a logger that is not logging now (e.g. during INITIAL),
    // annotations should not be cleared so that they appear later in the log.
    else if(!keepAnnotations)
      Global::getAnnotationManager().getOut().clear();

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
  else if(SystemCall::getMode() == SystemCall::logFileReplay && debugSender->getNumberOfMessages() > numberOfMessages + 1)
  {
    // idLogResponse was added in a delayed frame (no idFrameFinished to avoid flickering drawings)
    debugSender->send(true);

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
  switch(message.getMessageID())
  {
    case idModuleRequest:
      moduleGraphRunner.update(message.bin);
      return true;
    default:
      for(const std::function<bool(InMessage& message)>& messageHandler : messageHandlers)
        if(messageHandler(message))
          return true;
      return ThreadFrame::handleMessage(message);
  }
}

std::string ModuleContainer::getLogDescription(const GameState& gameState) const
{
  if(gameState.gameControllerActive)
  {
    auto team = teams.find(static_cast<uint8_t>(gameState.opponentTeam.number));
    if(team != teams.end())
      return team->second + "_"
             + (gameState.phase == GameState::penaltyShootout ? "ShootOut"
                : gameState.phase == GameState::firstHalf ? "1stHalf" : "2ndHalf");
  }
  return "Testing";
}
