/**
 * @file Processes/Cognition.cpp
 * Implementation of a class that represents a process that receives data from the robot at about 60 Hz.
 */

#include "Tools/Math/Eigen.h" // include first to avoid conflicts between Cabsl defines and some clang headers
#include "Cognition.h" // include second header conflicts on Windows
#include "Modules/Configuration/ConfigurationDataProvider/CognitionConfigurationDataProvider.h"
#include "Modules/Infrastructure/CameraProvider/CameraProvider.h"
#include "Modules/Infrastructure/LogDataProvider/CognitionLogDataProvider.h"
#include "Platform/BHAssert.h"
#include "Platform/Time.h"

#include "Representations/Communication/TeamData.h"
#include "Representations/Communication/BHumanMessage.h"

Cognition::Cognition() :
  Process(theDebugReceiver, theDebugSender),
  theDebugReceiver(this),
  theDebugSender(this),
  theMotionReceiver(this),
  theMotionSender(this),
  theSPLMessageHandler(inTeamMessages, outTeamMessage),
  moduleManager({ModuleBase::cognitionInfrastructure, ModuleBase::communication,
                 ModuleBase::perception, ModuleBase::modeling, ModuleBase::behaviorControl}),
  logger("Cognition", 'c', 60)
{
  theDebugSender.setSize(5200000, 100000);
  theDebugReceiver.setSize(2800000);
  theMotionSender.moduleManager = theMotionReceiver.moduleManager = &moduleManager;
}

void Cognition::init()
{
#ifdef TARGET_SIM
  theSPLMessageHandler.startLocal(Global::getSettings().teamPort, static_cast<unsigned>(Global::getSettings().playerNumber));
#else
  std::string bcastAddr = UdpComm::getWifiBroadcastAddress();
  theSPLMessageHandler.start(Global::getSettings().teamPort, bcastAddr.c_str());
#endif
  moduleManager.load();
  BH_TRACE_INIT("Cognition");

  // Prepare first frame
  numberOfMessages = theDebugSender.getNumberOfMessages();
  OUTPUT(idProcessBegin, bin, 'c');
}

void Cognition::terminate()
{
  moduleManager.destroy();
  Process::terminate();
}

bool Cognition::main()
{
  // read from team comm udp socket
  static_cast<void>(theSPLMessageHandler.receive());

  if(CognitionLogDataProvider::isFrameDataComplete() && CameraProvider::isFrameDataComplete())
  {
    timingManager.signalProcessStart();
    annotationManager.signalProcessStart();

    BH_TRACE_MSG("before TeamData");
    // push teammate data in our system
    if(Blackboard::getInstance().exists("TeamData") &&
       static_cast<const TeamData&>(Blackboard::getInstance()["TeamData"]).generate.operator bool())
      while(!inTeamMessages.empty())
        static_cast<const TeamData&>(Blackboard::getInstance()["TeamData"]).generate(inTeamMessages.takeBack());

    // Reset coordinate system for debug field drawing
    DECLARE_DEBUG_DRAWING("origin:Reset", "drawingOnField"); // Set the origin to the (0,0,0)
    ORIGIN("origin:Reset", 0.0f, 0.0f, 0.0f);

    STOPWATCH("Cognition") moduleManager.execute();

    DEBUG_RESPONSE_ONCE("automated requests:DrawingManager") OUTPUT(idDrawingManager, bin, Global::getDrawingManager());
    DEBUG_RESPONSE_ONCE("automated requests:DrawingManager3D") OUTPUT(idDrawingManager3D, bin, Global::getDrawingManager3D());

    theMotionSender.timeStamp = Time::getCurrentSystemTime();
    BH_TRACE_MSG("before theMotionSender.send()");
    theMotionSender.send();

    if(Blackboard::getInstance().exists("BHumanMessageOutputGenerator")
       && static_cast<const BHumanMessageOutputGenerator&>(Blackboard::getInstance()["BHumanMessageOutputGenerator"]).generate.operator bool()
       && static_cast<const BHumanMessageOutputGenerator&>(Blackboard::getInstance()["BHumanMessageOutputGenerator"]).sendThisFrame)
    {
      static_cast<const BHumanMessageOutputGenerator&>(Blackboard::getInstance()["BHumanMessageOutputGenerator"]).generate(&outTeamMessage);

      BH_TRACE_MSG("before theTeamHandler.send()");
      theSPLMessageHandler.send();
    }

    timingManager.signalProcessStop();
    logger.execute();

    DEBUG_RESPONSE("timing") timingManager.getData().copyAllMessages(theDebugSender);

    DEBUG_RESPONSE("annotation") annotationManager.getOut().copyAllMessages(theDebugSender);
    annotationManager.clear();

    if(theDebugSender.getNumberOfMessages() > numberOfMessages + 1)
    {
      // Send process finished message
      if(Blackboard::getInstance().exists("CameraInfo") &&
         static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]).camera == CameraInfo::lower)
      {
        // lower camera -> process called 'd'
        // Send completion notification
        theDebugSender.patchMessage(numberOfMessages, 0, 'd');
        OUTPUT(idProcessFinished, bin, 'd');
      }
      else
      {
        OUTPUT(idProcessFinished, bin, 'c');
      }
    }
    else
      theDebugSender.removeLastMessage();

    BH_TRACE_MSG("theDebugSender.send()");
    theDebugSender.send();

    // Prepare next frame
    numberOfMessages = theDebugSender.getNumberOfMessages();
    OUTPUT(idProcessBegin, bin, 'c');
  }
  else if(Global::getDebugRequestTable().pollCounter > 0)
    ++Global::getDebugRequestTable().pollCounter;

  if(Blackboard::getInstance().exists("Image"))
  {
    if(SystemCall::getMode() == SystemCall::physicalRobot)
      setPriority(10);
    Thread::sleep(1);
    BH_TRACE_MSG("before waitForFrameData");
    CameraProvider::waitForFrameData();
    if(SystemCall::getMode() == SystemCall::physicalRobot)
      setPriority(0);
  }
  else
    Thread::sleep(33);

  return SystemCall::getMode() != SystemCall::physicalRobot;
}

bool Cognition::handleMessage(InMessage& message)
{
  BH_TRACE_MSG("before Cognition:handleMessage");
  switch(message.getMessageID())
  {
    case idModuleRequest:
    {
      unsigned timeStamp;
      message.bin >> timeStamp;
      moduleManager.update(message.bin, timeStamp);
      return true;
    }
    default:
      return CognitionLogDataProvider::handleMessage(message) ||
             CognitionConfigurationDataProvider::handleMessage(message) ||
             Process::handleMessage(message);
  }
}

MAKE_PROCESS(Cognition);
