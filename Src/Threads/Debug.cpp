/**
 * @file Threads/Debug.cpp
 *
 * Implementation of class Debug.
 *
 * @author Martin Lötzsch
 * @author Jan Fiedler
 */

#include "Debug.h"
#include "Platform/Time.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Streams/TypeInfo.h"

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
  debugSender->setSize(MAX_PACKAGE_SEND_SIZE - 2000);
  debugReceiver->setSize(MAX_PACKAGE_RECEIVE_SIZE - 2000);
#endif
}

void Debug::init()
{
#ifndef TARGET_ROBOT
  debugSender->setSize(MAX_PACKAGE_SEND_SIZE - 2000);
  debugReceiver->setSize(MAX_PACKAGE_RECEIVE_SIZE - 2000);
#endif

  for(DebugSender<MessageQueue>& sender : senders)
    senderMap[sender.receiverThreadName] = &sender;

  BH_TRACE_INIT("Debug");

  moduleGraphCreator = std::make_unique<ModuleGraphCreator>(config);

  // read requests.dat
  InBinaryFile file("requests.dat");
  if(file.exists() && !file.eof())
    file >> *debugReceiver;

  debugReceiver->handleAllMessages(*this);
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
      sender.out.bin << moduleGraphCreator->getExecutionValues(i);
      unsigned timestamp = 0xffffffff;
      sender.out.bin << timestamp;
      sender.out.finishMessage(idModuleRequest);
    }
  }
}

bool Debug::main()
{
  for(Receiver<MessageQueue>& receiver : receivers)
    receiver.checkForPacket();

  DEBUG_RESPONSE_ONCE("automated requests:TypeInfo") OUTPUT(idTypeInfo, bin, *TypeInfo::current);

  DEBUG_RESPONSE_ONCE("automated requests:ModuleTable")
  {
    std::size_t size = 0;
    for(ModuleBase* i = ModuleBase::first; i; i = i->next)
      size++;

    Global::getDebugOut().bin << static_cast<unsigned>(size);
    for(ModuleBase* m = ModuleBase::first; m; m = m->next)
    {
      Global::getDebugOut().bin << m->name << static_cast<unsigned char>(m->category);

      unsigned requirementsSize = 0;
      unsigned providersSize = 0;
      for(const ModuleBase::Info& j : m->getModuleInfo())
        if(j.update)
          ++providersSize;
        else
          ++requirementsSize;

      Global::getDebugOut().bin << requirementsSize;
      for(const ModuleBase::Info& j : m->getModuleInfo())
        if(!j.update)
          Global::getDebugOut().bin << j.representation;

      Global::getDebugOut().bin << providersSize;
      for(const ModuleBase::Info& j : m->getModuleInfo())
        if(j.update)
          Global::getDebugOut().bin << j.representation;
    }

    Global::getDebugOut().bin << moduleGraphCreator->config;
    Global::getDebugOut().finishMessage(idModuleTable);
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
    if(!receiver.isEmpty())
      receiver.moveAllMessages(*debugSender);
  }

  // If not requested otherwise, send only latest of each type
  DEBUG_RESPONSE_NOT("debug:keepAllMessages")
    debugSender->removeRepetitions();

  // Send messages to the threads
#ifndef TARGET_ROBOT
  for(DebugSender<MessageQueue>& sender : senders)
    sender.send(true);
  debugSender->send(true);
#else
  for(DebugSender<MessageQueue>& sender : senders)
    sender.send(false);
  debugHandler.communicate(true);
#ifdef NDEBUG
  // Stop debug in release after sending the module configuration
  printf("Stopping Debug\n");
  announceStop();
#endif // NDEBUG
#endif // !TARGET_ROBOT

  return true;
}

bool Debug::handleMessage(InMessage& message)
{
  switch(message.getMessageID())
  {
    case idText: // loop back to GUI
      message >> *debugSender;
      return true;

    case idModuleRequest:
      unsigned timestamp;
      message.bin >> timestamp;
      if(moduleGraphCreator->update(message.bin))
      {
        // Sending the new module configuration if one could be determined
        for(DebugSender<MessageQueue>& sender : senders)
        {
          std::size_t i = 0;
          for(; i < config().size(); i++)
            if(config()[i].name == sender.receiverThreadName)
              break;
          sender.out.bin << moduleGraphCreator->getExecutionValues(i);
          sender.out.bin << timestamp; // pass timestamp
          sender.out.finishMessage(idModuleRequest);
        }
      }
      return true;

    // messages to the threads
    case idDebugDataChangeRequest:
    case idTypeInfo:
      for(DebugSender<MessageQueue>& sender : senders)
        message >> sender;
      return true;

    // messages to all threads
    case idDebugRequest:
      for(DebugSender<MessageQueue>& sender : senders)
        message >> sender;
      return ThreadFrame::handleMessage(message);

    case idFrameBegin:
      threadIdentifier = message.readThreadIdentifier();
      message.resetReadPosition();
    // no break

    default:
      message >> *senderMap[threadIdentifier];
      DEBUG_RESPONSE("legacy")
      {
        if((threadIdentifier == "Upper" || threadIdentifier == "Lower"))
          message >> *senderMap["Cognition"];
      }
      return true;
  }
}
