/**
* @file Processes/Cognition.cpp
* Implementation of a class that represents a process that receives data from the robot at about 60 Hz.
*/

#include "Cognition.h"
#include "Modules/Infrastructure/CameraProvider.h"
#include "Modules/Configuration/CognitionConfigurationDataProvider.h"
#include "Modules/Infrastructure/CognitionLogDataProvider.h"
#include "Modules/Infrastructure/TeamDataProvider.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Platform/BHAssert.h"
#include "Tools/Settings.h"
#include "Tools/Team.h"

static const char* categories[] = {"Cognition Infrastructure", "Perception", "Modeling", "Behavior Control"};

Cognition::Cognition() :
  INIT_DEBUGGING,
  INIT_RECEIVER(MotionToCognition),
  INIT_SENDER(CognitionToMotion),
  INIT_TEAM_COMM,
  moduleManager(categories, sizeof(categories) / sizeof(*categories)),
  logger(moduleManager)
{
  theDebugSender.setSize(10000000);
  theDebugReceiver.setSize(1400000);
  theTeamSender.setSize(1384); // 1 package without timestamp, size, localId, and message queue header
  theTeamReceiver.setSize(4 * 1450); // >= 4 packages
  theCognitionToMotionSender.moduleManager = theMotionToCognitionReceiver.moduleManager = &moduleManager;
}

void Cognition::init()
{
  Global::theTeamOut = &theTeamSender.out;
  START_TEAM_COMM;
  moduleManager.load();
  BH_TRACE_INIT("Cognition");
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

  // required to detect whether any messages are sent in this frame
  int numberOfMessages = theDebugSender.getNumberOfMessages();

  OUTPUT(idProcessBegin, bin, 'c');

  if(CognitionLogDataProvider::isFrameDataComplete() && CameraProvider::isFrameDataComplete() && CameraProvider::isFrameDataComplete())
  {
    timingManager.signalProcessStart();

    // There must not be any TEAM_OUTPUT before this in each frame.
    BH_TRACE_MSG("before TeamDataProvider");
    TeamDataProvider::handleMessages(theTeamReceiver);

    // Reset coordinate system for debug field drawing
    DECLARE_DEBUG_DRAWING("origin:Reset", "drawingOnField"); // Set the origin to the (0,0,0)
    ORIGIN("origin:Reset", 0.0f, 0.0f, 0.0f);

    STOP_TIME_ON_REQUEST_WITH_PLOT("Cognition", moduleManager.execute(););

    DEBUG_RESPONSE("process:Cognition:jointDelay",
    {
      if(&Blackboard::theInstance->theFrameInfo && &Blackboard::theInstance->theFilteredJointData)
      {
        OUTPUT(idText, text, Blackboard::theInstance->theFrameInfo.getTimeSince(Blackboard::theInstance->theFilteredJointData.timeStamp));
      }
    });

    DEBUG_RESPONSE_ONCE("automated requests:DrawingManager", OUTPUT(idDrawingManager, bin, Global::getDrawingManager()););
    DEBUG_RESPONSE_ONCE("automated requests:DrawingManager3D", OUTPUT(idDrawingManager3D, bin, Global::getDrawingManager3D()););
    DEBUG_RESPONSE_ONCE("automated requests:StreamSpecification", OUTPUT(idStreamSpecification, bin, Global::getStreamHandler()););

    theCognitionToMotionSender.timeStamp = SystemCall::getCurrentSystemTime();
    BH_TRACE_MSG("before cognition2Motion.send");
    theCognitionToMotionSender.send();

    BH_TRACE_MSG("before SEND_TEAM_COMM");
    if(!theTeamSender.isEmpty())
    {
      if(&Blackboard::theInstance->theTeamMateData && Blackboard::theInstance->theTeamMateData.sendThisFrame)
      {
        SEND_TEAM_COMM;
      }
      theTeamSender.clear(); // team messages are purged even when not sent.
    }

    timingManager.signalProcessStop();

    DEBUG_RESPONSE("timing",
      timingManager.getData().copyAllMessages(theDebugSender);
    );

    logger.run();

    if(theDebugSender.getNumberOfMessages() > numberOfMessages + 1)
    {
      // messages were sent in this frame -> send process finished
      if(&Blackboard::theInstance->theCameraInfo &&
         Blackboard::theInstance->theCameraInfo.camera == CameraInfo::lower)
      { // lower camera -> process called 'd'
        theDebugSender.patchMessage(numberOfMessages, 0, 'd');
        OUTPUT(idProcessFinished, bin, 'd');
      }
      else
        OUTPUT(idProcessFinished, bin, 'c');
    }
    else if(theDebugSender.getNumberOfMessages() == numberOfMessages + 1)
      theDebugSender.removeLastMessage();

    BH_TRACE_MSG("theDebugSender.send()");
    theDebugSender.send();
  }
  else
  {
    if(theDebugSender.getNumberOfMessages() == numberOfMessages + 1)
      theDebugSender.removeLastMessage();

#ifndef RELEASE
  else if(Global::getDebugRequestTable().poll)
    --Global::getDebugRequestTable().pollCounter;
#endif
  }

  if(&Blackboard::theInstance->theImage)
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
           Process::handleMessage(message);
  }
  BH_TRACE_MSG("after Cognition:handleMessage");
}

MAKE_PROCESS(Cognition);
