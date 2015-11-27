/**
 * @file Processes/Cognition.cpp
 * Implementation of a class that represents a process that receives data from the robot at about 60 Hz.
 */

#include "Tools/Math/Eigen.h" // include first to avoid conflicts between Cabsl defines and some clang headers
#include "Cognition.h" // include second header conflicts on Windows
#include "Modules/Configuration/CognitionConfigurationDataProvider.h"
#include "Modules/Infrastructure/CameraProvider.h"
#include "Modules/Infrastructure/CognitionLogDataProvider.h"
#include "Modules/Infrastructure/TeammateDataProvider.h"
#include "Platform/BHAssert.h"
#include "Representations/Infrastructure/TeammateData.h"

Cognition::Cognition() :
  INIT_DEBUGGING,
  INIT_RECEIVER(MotionToCognition),
  INIT_SENDER(CognitionToMotion),
  INIT_TEAM_COMM,
  moduleManager({ModuleBase::cognitionInfrastructure, ModuleBase::perception, ModuleBase::modeling, ModuleBase::behaviorControl})
{
  theDebugSender.setSize(5200000, 100000);
  theDebugReceiver.setSize(2800000);
  const unsigned size = SPL_STANDARD_MESSAGE_DATA_SIZE - SPLStandardMessage::bhumanHeaderSize - 2 * sizeof(unsigned);
  theTeamSender.setSize(size);
  theTeamReceiver.setSize(5 * size); // more than 4 because of additional data
  theCognitionToMotionSender.moduleManager = theMotionToCognitionReceiver.moduleManager = &moduleManager;
}

void Cognition::init()
{
  Global::theTeamOut = &theTeamSender.out;
  START_TEAM_COMM;
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
  RECEIVE_TEAM_COMM;

  if(CognitionLogDataProvider::isFrameDataComplete() && CameraProvider::isFrameDataComplete())
  {
    timingManager.signalProcessStart();
    annotationManager.signalProcessStart();

    BH_TRACE_MSG("before TeammateDataProvider");
    TeammateDataProvider::handleMessages(theTeamReceiver);

    // Reset coordinate system for debug field drawing
    DECLARE_DEBUG_DRAWING("origin:Reset", "drawingOnField"); // Set the origin to the (0,0,0)
    ORIGIN("origin:Reset", 0.0f, 0.0f, 0.0f);

    STOPWATCH_WITH_PLOT("Cognition") moduleManager.execute();

    DEBUG_RESPONSE_ONCE("automated requests:DrawingManager") OUTPUT(idDrawingManager, bin, Global::getDrawingManager());
    DEBUG_RESPONSE_ONCE("automated requests:DrawingManager3D") OUTPUT(idDrawingManager3D, bin, Global::getDrawingManager3D());
    DEBUG_RESPONSE_ONCE("automated requests:StreamSpecification") OUTPUT(idStreamSpecification, bin, Global::getStreamHandler());

    theCognitionToMotionSender.timeStamp = SystemCall::getCurrentSystemTime();
    BH_TRACE_MSG("before cognition2Motion.send");
    theCognitionToMotionSender.send();

    BH_TRACE_MSG("before SEND_TEAM_COMM");
    if(!theTeamSender.isEmpty())
    {
      if(Blackboard::getInstance().exists("TeammateData") &&
         static_cast<const TeammateData&>(Blackboard::getInstance()["TeammateData"]).sendThisFrame)
      {
        SEND_TEAM_COMM;
      }
      theTeamSender.clear(); // team messages are purged even when not sent.
    }

    timingManager.signalProcessStop();
    logger.execute();

    DEBUG_RESPONSE("timing") timingManager.getData().copyAllMessages(theDebugSender);

    DEBUG_RESPONSE("annotation") annotationManager.getOut().copyAllMessages(theDebugSender);
    annotationManager.clear();

    if(theDebugSender.getNumberOfMessages() > numberOfMessages + 1)
    {
      // messages were sent in this frame -> send process finished
      if(Blackboard::getInstance().exists("CameraInfo") &&
         static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]).camera == CameraInfo::lower)
      { // lower camera -> process called 'd'
        theDebugSender.patchMessage(numberOfMessages, 0, 'd');
        OUTPUT(idProcessFinished, bin, 'd');
      }
      else
        OUTPUT(idProcessFinished, bin, 'c');
    }
    else
      theDebugSender.removeLastMessage();

    BH_TRACE_MSG("theDebugSender.send()");
    theDebugSender.send();

    // Prepare next frame
    numberOfMessages = theDebugSender.getNumberOfMessages();
    OUTPUT(idProcessBegin, bin, 'c');
  }
  else if(Global::getDebugRequestTable().poll)
    --Global::getDebugRequestTable().pollCounter;

  if(Blackboard::getInstance().exists("Image"))
  {
    if(SystemCall::getMode() == SystemCall::physicalRobot)
      setPriority(10);
    SystemCall::sleep(1);
    BH_TRACE_MSG("before waitForFrameData");
    CameraProvider::waitForFrameData();
    if(SystemCall::getMode() == SystemCall::physicalRobot)
      setPriority(0);
  }
  else
    SystemCall::sleep(33);

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
  BH_TRACE_MSG("after Cognition:handleMessage");
}

MAKE_PROCESS(Cognition);
