/**
 * @file AnnotationInfo.cpp
 * @author Andreas Stolpmann
 */

#include "AnnotationInfo.h"
#include "Platform/Time.h"
#include <algorithm>

void AnnotationInfo::clear()
{
  SYNC;
  timeOfLastMessage = Time::getCurrentSystemTime();
  newAnnotations.emplace_back();
  newAnnotations.back().name = "CLEAR";
}

void AnnotationInfo::addMessage(MessageQueue::Message message, unsigned currentFrame)
{
  SYNC;
  timeOfLastMessage = Time::getCurrentSystemTime();
  newAnnotations.emplace_back();
  Annotation& data = newAnnotations.back();
  message.bin() >> data;
  data.frame = currentFrame;

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
