/**
 * @file AnnotationManager.cpp
 * @author Andreas Stolpmann
 */

#include "AnnotationManager.h"

#include "Platform/BHAssert.h"
#include "Representations/Communication/GameInfo.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Module/Blackboard.h"

#include <cstring>
#include <cstdarg>

AnnotationManager::AnnotationManager() : lastGameState(STATE_INITIAL), lastSetPlay(SET_PLAY_NONE)
{
  outData.setSize(100000);
}

void AnnotationManager::signalThreadStart()
{
  if(Blackboard::getInstance().exists("GameInfo"))
  {
    const GameInfo& gameInfo = static_cast<const GameInfo&>(Blackboard::getInstance()["GameInfo"]);
    if(gameInfo.state != lastGameState)
    {
      addAnnotation();
      outData.out.text << "GameState" << gameInfo.getStateAsString() << " state.";
      outData.out.finishMessage(idAnnotation);
    }
    else if(gameInfo.setPlay != lastSetPlay && gameInfo.setPlay != SET_PLAY_NONE)
    {
      addAnnotation();
      outData.out.text << "GameState" << gameInfo.getStateAsString() << " for team " << gameInfo.kickingTeam << ".";
      outData.out.finishMessage(idAnnotation);
    }
    lastGameState = gameInfo.state;
    lastSetPlay = gameInfo.setPlay;
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
  outData.out.bin << (0x80000000 | annotationCounter++);
}

MessageQueue& AnnotationManager::getOut()
{
  return outData;
}
