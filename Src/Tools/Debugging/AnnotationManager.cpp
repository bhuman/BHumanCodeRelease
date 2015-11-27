/**
 * @file AnnotationManager.cpp
 * @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
 */

#include "AnnotationManager.h"

#include "Platform/BHAssert.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Module/Blackboard.h"

#include <cstring>
#include <stdarg.h>

AnnotationManager::AnnotationManager() : lastGameState(STATE_INITIAL)
{
  outData.setSize(100000);
}

void AnnotationManager::signalProcessStart()
{
  if(Blackboard::getInstance().exists("GameInfo"))
  {
    const GameInfo& gameInfo = static_cast<const GameInfo&>(Blackboard::getInstance()["GameInfo"]);
    if(gameInfo.state == STATE_READY || gameInfo.state == STATE_SET || gameInfo.state == STATE_PLAYING)
      ++currentFrame;

    if(gameInfo.state != lastGameState)
    {
      addAnnotation();
      outData.out.text << "GameState" << gameInfo.getStateAsString() << " state.";
      outData.out.finishMessage(idAnnotation);
    }
    lastGameState = gameInfo.state;
  }
}

void AnnotationManager::clear()
{
  DEBUG_RESPONSE("annotation")
  {
    outData.clear();
  }
  else
  {
    if(Blackboard::getInstance().exists("GameInfo"))
    {
      const GameInfo& gameInfo = static_cast<const GameInfo&>(Blackboard::getInstance()["GameInfo"]);
      if(gameInfo.state == STATE_READY || gameInfo.state == STATE_SET || gameInfo.state == STATE_PLAYING)
        outData.clear();
    }
    else
      outData.clear();
  }
}

void AnnotationManager::addAnnotation()
{
  outData.out.bin << annotationCounter++;
  outData.out.bin << currentFrame;
}

MessageQueue& AnnotationManager::getOut()
{
  return outData;
}
