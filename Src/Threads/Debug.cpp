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

Debug::Debug(const Configuration& config) :
#ifdef TARGET_ROBOT
  ThreadFrame(nullptr, nullptr), // Initializes the MessageQueues used from the DebugHandler.
  debugHandler(*debugReceiver, *debugSender, MAX_PACKAGE_SEND_SIZE, 0),
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

  DEBUG_RESPONSE_ONCE("automated requests:TypeInfo") OUTPUT(idTypeInfo, bin, moduleGraphCreator->typeInfo);

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
#ifdef TARGET_SIM
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
#endif // TARGET_SIM

  return true;
}

bool Debug::handleMessage(InMessage& message)
{
  switch(message.getMessageID())
  {
    case idText: // loop back to GUI
      message >> *debugSender;
      return true;

    // messages to Cognition
    case idColorCalibration:
      message >> *senderMap["Lower"];
      message >> *senderMap["Upper"];
      return true;

    // messages to Motion
    case idMotionNet:
      message >> *senderMap["Motion"];
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
      return true;
  }
}
