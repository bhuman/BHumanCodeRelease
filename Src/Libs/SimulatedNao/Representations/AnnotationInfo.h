/**
 * @file AnnotationInfo.h
 * @author Andreas Stolpmann
 */

#pragma once

#include "Platform/Thread.h"
#include "Streaming/MessageQueue.h"
#include <string>
#include <vector>

class AnnotationInfo
{
public:
  struct AnnotationData
  {
    unsigned annotationNumber;
    unsigned frame;
    std::string name;
    std::string annotation;

    void read(MessageQueue::Message message);
  };

  struct Listener
  {
    virtual void handleAnnotation(const AnnotationData& data) = 0;
  };

  void clear();
  void addMessage(MessageQueue::Message message, unsigned currentFrame);
  void registerListener(Listener* listener);
  void unregisterListener(Listener* listener);

  DECLARE_SYNC;
  std::vector<AnnotationData> newAnnotations;
  unsigned timeOfLastMessage = 0;

private:
  std::vector<Listener*> listeners;
};
