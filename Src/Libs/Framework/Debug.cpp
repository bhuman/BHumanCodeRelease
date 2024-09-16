/**
 * @file Debug.cpp
 *
 * Implementation of class Debug.
 *
 * @author Martin Lötzsch
 * @author Jan Fiedler
 */

#include "Debug.h"
#include "Debugging/Debugging.h"
#include "Platform/Time.h"
#include "Streaming/TypeInfo.h"

Debug::Debug(const Settings& settings, const std::string& robotName, const Configuration& config) :
#ifdef TARGET_ROBOT
  ThreadFrame(settings, robotName, nullptr, nullptr), // Initializes the MessageQueues used from the DebugHandler.
  debugHandler(*debugReceiver, *debugSender, MAX_PACKAGE_SEND_SIZE, 0),
#else
  ThreadFrame(settings, robotName),
#endif
  config(config)
{
#ifdef TARGET_ROBOT
  debugSender->reserve(MAX_PACKAGE_SEND_SIZE - 2000);
  debugReceiver->reserve(MAX_PACKAGE_RECEIVE_SIZE - 2000);
#endif
}

void Debug::init()
{
#ifndef TARGET_ROBOT
  debugSender->reserve(MAX_PACKAGE_SEND_SIZE - 2000);
  debugReceiver->reserve(MAX_PACKAGE_RECEIVE_SIZE - 2000);
#endif

  for(DebugSender<MessageQueue>& sender : senders)
    senderMap[sender.receiverThreadName] = &sender;

  BH_TRACE_INIT("Debug");

  moduleGraphCreator = std::make_unique<ModuleGraphCreator>(config);

  for(MessageQueue::Message message : *debugReceiver)
    handleMessage(message);
  debugReceiver->clear();

  // Send initial module config
  OutBinaryMemory out(20000);
  out << config;
  InBinaryMemory stream(out.data());
  if(moduleGraphCreator->update(stream))
  {
    for(DebugSender<MessageQueue>& sender : senders)
    {
      std::size_t i = 0;
      for(; i < config().size(); i++)
        if(config()[i].name == sender.receiverThreadName)
          break;
      auto stream = sender.bin(idModuleRequest);
      stream << moduleGraphCreator->getExecutionValues(i);
      stream << -1;
    }
  }
}

bool Debug::main()
{
  for(Receiver<MessageQueue>& receiver : receivers)
    receiver.receivePacket();

  DEBUG_RESPONSE_ONCE("automated requests:TypeInfo") OUTPUT(idTypeInfo, bin, *TypeInfo::current);

  DEBUG_RESPONSE_ONCE("automated requests:ModuleTable")
  {
    std::size_t size = 0;
    for(ModuleBase* i = ModuleBase::first; i; i = i->next)
      size++;

    auto stream = Global::getDebugOut().bin(idModuleTable);
    stream << static_cast<unsigned>(size);
    for(ModuleBase* m = ModuleBase::first; m; m = m->next)
    {
      stream << m->name;

      unsigned requirementsSize = 0;
      unsigned providersSize = 0;
      for(const ModuleBase::Info& j : m->getModuleInfo())
        if(j.update)
          ++providersSize;
        else
          ++requirementsSize;

      stream << requirementsSize;
      for(const ModuleBase::Info& j : m->getModuleInfo())
        if(!j.update)
          stream << j.representation;

      stream << providersSize;
      for(const ModuleBase::Info& j : m->getModuleInfo())
        if(j.update)
          stream << j.representation;
    }

    stream << moduleGraphCreator->config;
  }

  DEBUG_RESPONSE_ONCE("moduleGraph:moduleOrder")
  {
    // Shows the execution order of all threads.
    std::string text;
    for(std::size_t i = 0; i < moduleGraphCreator->config.threads.size(); i++)
    {
      text.append(moduleGraphCreator->config.threads[i].name + ":");
      for(const ModuleGraphCreator::Provider& provider : moduleGraphCreator->providers[i])
        text.append(std::string("\n  ") + provider.representation + ":" + provider.moduleBase->name);
      text.append("\n");
    }
    OUTPUT_TEXT(text);
  }

  DEBUG_RESPONSE_ONCE("moduleGraph:dataExchanged")
  {
    // Shows which representations which threads receives.
    std::string text;
    for(std::size_t i = 0; i < moduleGraphCreator->config.threads.size(); i++)
    {
      text.append(moduleGraphCreator->config.threads[i].name + " receives from:");
      for(std::size_t j = 0; j < moduleGraphCreator->received[i].size(); j++)
      {
        text.append("\n  " + moduleGraphCreator->config.threads[j].name + ":");
        for(const std::string& rep : moduleGraphCreator->received[i][j])
          text.append("\n    " + rep);
      }
      text.append("\n");
    }
    OUTPUT_TEXT(text);
  }

  // Move the messages from other threads' debug queues to the outgoing queue
  for(Receiver<MessageQueue>& receiver : receivers)
  {
    if(!receiver.empty())
    {
      *debugSender << receiver;
      receiver.clear();
    }
  }

  // If not requested otherwise, send only latest of each type
  DEBUG_RESPONSE_NOT("debug:keepAllMessages")
    removeRepetitions();

  // Send messages to the threads
#ifndef TARGET_ROBOT
  for(DebugSender<MessageQueue>& sender : senders)
  {
    DEBUG_RESPONSE("async")
      sender.send(false);
    else
      sender.send(true);
  }
  debugSender->send();
#else
  for(DebugSender<MessageQueue>& sender : senders)
    sender.send();
  debugHandler.communicate(true);
#ifdef NDEBUG
  // Stop debug in release after sending the module configuration
  printf("Stopping Debug\n");
  announceStop();
#endif // NDEBUG
#endif // !TARGET_ROBOT

  // If there is still data to be sent, immediately start a new cycle.
  // Otherwise, wait for the next packet to arrive.
  if(debugSender->size() > 0)
  {
    Thread::yield();
    return false;
  }
  else
    return true;
}

void Debug::removeRepetitions()
{
  std::unordered_map<std::string, std::array<size_t, numOfMessageIDs>> threads;
  std::string thread = "unknown";

  size_t* messagesPerType = threads[thread].data();
  for(MessageQueue::Message message : *debugSender)
  {
    if(message.id() == idFrameBegin)
    {
      message.bin() >> thread;
      messagesPerType = threads[thread].data();
    }
    ++messagesPerType[message.id()];
  }

  messagesPerType = threads["unknown"].data();
  size_t originalSize = 0;
  size_t sizeAfterFrameBegin = 0;

  debugSender->filter([&](MessageQueue::const_iterator i)
  {
    MessageQueue::Message message = *i;
    switch(message.id())
    {
      case idFrameBegin:
        originalSize = debugSender->size();
        message.bin() >> thread;
        sizeAfterFrameBegin = debugSender->size();
        messagesPerType = threads[thread].data();
        return true;

      case idFrameFinished:
        --messagesPerType[idFrameFinished];
        if(debugSender->size() == sizeAfterFrameBegin)
        {
          debugSender->resize(originalSize);
          return false;
        }
        else
          return true;

      case idText:
        return --messagesPerType[idText] <= 20;

      // accept always, thread id is not important
      case idNumOfDataMessageIDs:
      case idDebugRequest:
      case idDebugResponse:
      case idDebugDataResponse:
      case idPlot:
      case idConsole:
      case idAudioData:
      case idAnnotation:
      case idLogResponse:
      case idThread:
        return true;

      // only the latest messages for infrastructure
      default:
        if(message.id() >= numOfDataMessageIDs)
          return --messagesPerType[message.id()] == 0;
        [[fallthrough]];

      // data only from latest frame
      case idStopwatch:
      case idDebugImage:
      case idDebugDrawing:
      case idDebugDrawing3D:
        return messagesPerType[idFrameFinished] == 1;
    }
  });
}

bool Debug::handleMessage(MessageQueue::Message message)
{
  switch(message.id())
  {
    case idNumOfDataMessageIDs:
    {
      if(SystemCall::getMode() == SystemCall::physicalRobot)
        Global::getDebugOut().bin(idNumOfDataMessageIDs) << numOfDataMessageIDs;
      return true;
    }
    case idText: // loop back to GUI
      *debugSender << message;
      return true;

    case idThread:
      message.bin() >> currentThreadName;
      return true;

    case idModuleRequest:
    {
      auto stream = message.bin();
      unsigned timestamp;
      stream >> timestamp;
      if(moduleGraphCreator->update(stream))
      {
        // Sending the new module configuration if one could be determined
        for(DebugSender<MessageQueue>& sender : senders)
        {
          std::size_t i = 0;
          for(; i < config().size(); i++)
            if(config()[i].name == sender.receiverThreadName)
              break;
          sender.bin(idModuleRequest) << moduleGraphCreator->getExecutionValues(i) << timestamp;
        }
      }
      return true;
    }

    // messages to the threads
    case idDebugDataChangeRequest:
      if(!currentThreadName.empty())
      {
        *senderMap[currentThreadName] << message;
        currentThreadName = "";
        return true;
      }
      [[fallthrough]];

    case idTypeInfo:
      for(DebugSender<MessageQueue>& sender : senders)
        sender << message;
      return true;

    // messages to all threads
    case idDebugRequest:
      if(!currentThreadName.empty())
      {
        *senderMap[currentThreadName] << message;
        currentThreadName = "";
        return true;
      }
      for(DebugSender<MessageQueue>& sender : senders)
        sender << message;
      return ThreadFrame::handleMessage(message);

    case idFrameBegin:
      message.bin() >> threadName;
      [[fallthrough]];

    default:
      *senderMap[currentThreadName.empty() ? threadName : currentThreadName] << message;
      currentThreadName = "";
      return true;
  }
}
