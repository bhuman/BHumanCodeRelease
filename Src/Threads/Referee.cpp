/**
 * @file Referee.cpp
 *
 * This file implements a thread that handles the referee pose detection.
 *
 * @author Ayleen Lührsen
 */

#include "Referee.h"
#include "Framework/Blackboard.h"
#include "Framework/ModuleContainer.h"
#include "Modules/Infrastructure/LogDataProvider/LogDataProvider.h"
#include "Platform/Thread.h"
#include "Representations/Perception/ImagePreprocessing/OptionalCameraImage.h"

REGISTER_EXECUTION_UNIT(Referee)

bool Referee::beforeFrame()
{
  // Register handler. This cannot be done in the constructor, because it
  // runs in a different thread.
  if(!handlerRegistered)
  {
    ModuleContainer::addMessageHandler([this](MessageQueue::Message) -> bool
    {
      receivedDebugData = true;
      return false;
    });
    handlerRegistered = true;
  }

  // This thread should only run if images are to process. However, it must also
  // run if the framework expects feedback. This is the case if log data is available,
  // debug data was received or it is currently polling for debug requests.
  bool shouldRun = (LogDataProvider::isFrameDataComplete() && LogDataProvider::exists())
                   || receivedDebugData
                   || Global::getDebugRequestTable().pollCounter > 0;
  receivedDebugData = false;

  // If a new image was received, the thread should run as well.
  if(Blackboard::getInstance().exists("OptionalCameraImage"))
  {
    const OptionalCameraImage& image = static_cast<OptionalCameraImage&>(Blackboard::getInstance()["OptionalCameraImage"]);
    const unsigned currentTimeStamp = image.image.has_value() ? image.image.value().timestamp : 0;
    shouldRun |= lastImageTimestamp != currentTimeStamp;
    lastImageTimestamp = currentTimeStamp;
  }

  return shouldRun;
}

bool Referee::afterFrame()
{
  return true;
}
