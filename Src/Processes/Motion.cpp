/**
 * @file Processes/Motion.cpp
 * Implementation of a class that represents the process that sends commands to the robot at 50Hz.
 */

#include "Motion.h"
#include "Modules/Infrastructure/LogDataProvider/MotionLogDataProvider.h"
#include "Modules/Infrastructure/NaoProvider/NaoProvider.h"
#include "Modules/MotionControl/MotionSelector/MotionSelector.h"
#include "Modules/MotionControl/SpecialActions/SpecialActions.h"
#include "Platform/Time.h"

Motion::Motion() :
  Process(theDebugReceiver, theDebugSender),
  theDebugReceiver(this),
  theDebugSender(this),
  theCognitionReceiver(this),
  theCognitionSender(this),
  moduleManager({ModuleBase::motionInfrastructure, ModuleBase::motionControl, ModuleBase::sensing}),
  logger("Motion", 'm', 100)
{
  theDebugReceiver.setSize(500000);
  theDebugSender.setSize(50000, 20000);

  theCognitionSender.moduleManager = theCognitionReceiver.moduleManager = &moduleManager;
  
  if(SystemCall::getMode() == SystemCall::physicalRobot)
    setPriority(20);
}

void Motion::init()
{
  moduleManager.load();
  BH_TRACE_INIT("Motion");

  // Prepare first frame
  numberOfMessages = theDebugSender.getNumberOfMessages();
  OUTPUT(idProcessBegin, bin, 'm');
}

void Motion::terminate()
{
  if(SystemCall::getMode() == SystemCall::physicalRobot)
    setPriority(0);
  moduleManager.destroy();
  Process::terminate();
}

bool Motion::main()
{
  // there has been no new package from Cognition in more than 3000ms and thus we let the robot sit down.
  if(theCognitionReceiver.timeStamp != 0 && Time::getTimeSince(theCognitionReceiver.timeStamp) > 3000)
  {
    MotionSelector::sitDown();
  }

  if(MotionLogDataProvider::isFrameDataComplete())
  {
    timingManager.signalProcessStart();
    annotationManager.signalProcessStart();

    STOPWATCH("Motion") moduleManager.execute();
    NaoProvider::finishFrame();

    DEBUG_RESPONSE_ONCE("automated requests:DrawingManager") OUTPUT(idDrawingManager, bin, Global::getDrawingManager());
    DEBUG_RESPONSE_ONCE("automated requests:DrawingManager3D") OUTPUT(idDrawingManager3D, bin, Global::getDrawingManager3D());

    theCognitionSender.timeStamp = Time::getCurrentSystemTime();
    theCognitionSender.send();

    timingManager.signalProcessStop();
    logger.execute();

    DEBUG_RESPONSE("timing") timingManager.getData().copyAllMessages(theDebugSender);

    DEBUG_RESPONSE("annotation") annotationManager.getOut().copyAllMessages(theDebugSender);
    annotationManager.clear();

    if(theDebugSender.getNumberOfMessages() > numberOfMessages + 1)
    {
      // messages were sent in this frame -> send process finished
      OUTPUT(idProcessFinished, bin, 'm');
    }
    else
      theDebugSender.removeLastMessage();

    theDebugSender.send();

    // Prepare next frame
    numberOfMessages = theDebugSender.getNumberOfMessages();
    OUTPUT(idProcessBegin, bin, 'm');
  }

  if(Blackboard::getInstance().exists("JointSensorData"))
    NaoProvider::waitForFrameData();
  else
    Thread::sleep(10);

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
             Process::handleMessage(message);
  }
}

MAKE_PROCESS(Motion);

#if (defined LINUX || defined MACOS)
#include "Modules/Sensing/FallDownStateDetector/FallDownStateProvider.h"
#include "Modules/Sensing/InertialDataProvider/InertialDataProvider.h"
#include "Modules/MotionControl/WalkingEngine/Walk2014Generator.h"

extern Module<FallDownStateProvider> theFallDownStateProviderModule;
auto linkFallDownStateProvider = &theFallDownStateProviderModule;
extern Module<InertialDataProvider> theInertialDataProviderModule;
auto linkInertialDataProvider = &theInertialDataProviderModule;
extern Module<Walk2014Generator> theWalk2014GeneratorModule;
auto linkWalk2014Generator = &theWalk2014GeneratorModule;

#endif
