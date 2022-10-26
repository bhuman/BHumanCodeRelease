/**
 * @file AnnotationInfo.cpp
 * @author Andreas Stolpmann
 */

#include "AnnotationInfo.h"
#include "Platform/Time.h"
#include "Streaming/InMessage.h"
#include <algorithm>

void AnnotationInfo::clear()
{
  SYNC;
  timeOfLastMessage = Time::getCurrentSystemTime();
  newAnnotations.emplace_back();
  newAnnotations.back().name = "CLEAR";
}

void AnnotationInfo::addMessage(InMessage& message, unsigned currentFrame)
{
  SYNC;
  timeOfLastMessage = Time::getCurrentSystemTime();
  newAnnotations.emplace_back();
  AnnotationData& data = newAnnotations.back();

  message.bin >> data.annotationNumber;
  if(!(data.annotationNumber & 0x80000000))
    message.bin >> data.frame; // Compatibility with old annotations
  data.annotationNumber &= ~0x80000000;
  data.frame = currentFrame;
  message.text >> data.name;
  data.annotation = message.text.readAll();

  for(auto* listener : listeners)
    listener->handleAnnotation(data);
}

void AnnotationInfo::registerListener(Listener* listener)
{
  SYNC;
  listeners.push_back(listener);
}

void AnnotationInfo::unregisterListener(Listener* listener)
{
  SYNC;
  for(auto it = listeners.rbegin(); it != listeners.rend(); ++it)
    if(*it == listener)
    {
      listeners.erase(std::next(it).base());
      break;
    }
}
