/**
* @file Processes/Motion.cpp
* Implementation of a class that represents the process that sends commands to the robot at 50Hz.
*/
#include "Motion.h"
#include "Modules/Infrastructure/NaoProvider.h"
#include "Modules/Infrastructure/MotionLogDataProvider.h"
#include "Modules/MotionControl/SpecialActions.h"
#include "Modules/MotionControl/WalkingEngine/WalkingEngine.h"
#include "Modules/MotionControl/MotionSelector.h"

static const char* categories[] = {"Motion Infrastructure", "Motion Control", "Sensing"};

Motion::Motion() :
  INIT_DEBUGGING,
  INIT_RECEIVER(CognitionToMotion),
  INIT_SENDER(MotionToCognition),
  moduleManager(categories, sizeof(categories) / sizeof(*categories))
{
  theDebugReceiver.setSize(200000);
  theDebugSender.setSize(40000);

  theMotionToCognitionSender.moduleManager = theCognitionToMotionReceiver.moduleManager = &moduleManager;

  if(SystemCall::getMode() == SystemCall::physicalRobot)
    setPriority(50);
}

void Motion::init()
{
  moduleManager.load();
  BH_TRACE_INIT("Motion");
}

void Motion::terminate()
{
  moduleManager.destroy();
  Process::terminate();
}

bool Motion::main()
{
  // there has been no new package from Cognition in more than 500ms and thus we let the robot stand.
  if(SystemCall::getTimeSince(theCognitionToMotionReceiver.timeStamp) > 500)
    MotionSelector::stand();

  // required to detect whether any messages are sent in this frame
  int numberOfMessages = theDebugSender.getNumberOfMessages();

  if(MotionLogDataProvider::isFrameDataComplete() && NaoProvider::isFrameDataComplete())
  {
    timingManager.signalProcessStart();

    STOP_TIME_ON_REQUEST_WITH_PLOT("Motion", moduleManager.execute(););
    NaoProvider::finishFrame();

    DEBUG_RESPONSE_ONCE("automated requests:DrawingManager", OUTPUT(idDrawingManager, bin, Global::getDrawingManager()););
    DEBUG_RESPONSE_ONCE("automated requests:DrawingManager3D", OUTPUT(idDrawingManager3D, bin, Global::getDrawingManager3D()););
    DEBUG_RESPONSE_ONCE("automated requests:StreamSpecification", OUTPUT(idStreamSpecification, bin, Global::getStreamHandler()););

    theMotionToCognitionSender.timeStamp = SystemCall::getCurrentSystemTime();
    theMotionToCognitionSender.send();

    timingManager.signalProcessStop();
    DEBUG_RESPONSE("timing",
      timingManager.getData().copyAllMessages(theDebugSender);
    );


    if(theDebugSender.getNumberOfMessages() != numberOfMessages)
    {
      // messages were sent in this frame -> send process finished
      OUTPUT(idProcessFinished, bin, 'm');
    }
    theDebugSender.send();
  }

  if(&Blackboard::theInstance->theJointData)
    NaoProvider::waitForFrameData();
  else
    SystemCall::sleep(10);

  return SystemCall::getMode() != SystemCall::physicalRobot;
}

bool Motion::handleMessage(InMessage& message)
{
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
    return MotionLogDataProvider::handleMessage(message) ||
           SpecialActions::handleMessage(message) ||
           WalkingEngine::handleMessage(message) ||
           Process::handleMessage(message);
  }
}

MAKE_PROCESS(Motion);
