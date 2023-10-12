/**
 * @file AnnotationInfo.cpp
 * @author Andreas Stolpmann
 */

#include "AnnotationInfo.h"
#include "Platform/Time.h"
#include <algorithm>

void AnnotationInfo::AnnotationData::read(MessageQueue::Message message)
{
  auto stream = message.bin();
  stream >> annotationNumber;
  if(!(annotationNumber & 0x80000000))
    stream >> frame; // Compatibility with old annotations
  const size_t size = stream.getSize() - stream.getPosition();
  std::string text;
  text.resize(size);
  stream.read(text.data(), size);
  InTextMemory textStream(text.data(), size);
  textStream >> name;
  annotation = textStream.readAll();
  annotationNumber &= ~0x80000000;
}

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
  AnnotationData& data = newAnnotations.back();
  data.read(message);
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
