/**
 * @file AnnotationInfo.h
 * @author Andreas Stolpmann
 */

#pragma once

#include "LogPlayback/Annotation.h"
#include "Platform/Thread.h"
#include "Streaming/MessageQueue.h"
#include <string>
#include <vector>

class AnnotationInfo
{
public:
  struct Listener
  {
    virtual void handleAnnotation(const Annotation& data) = 0;
  };

  void clear();
  void addMessage(MessageQueue::Message message, unsigned currentFrame);
  void registerListener(Listener* listener);
  void unregisterListener(Listener* listener);

  DECLARE_SYNC;
  std::vector<Annotation> newAnnotations;
  unsigned timeOfLastMessage = 0;

private:
  std::vector<Listener*> listeners;
};
