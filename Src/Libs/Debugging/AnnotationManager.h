/**
 * @file AnnotationManager.h
 * @author Andreas Stolpmann
 */

#pragma once

#include "Streaming/MessageQueue.h"

class AnnotationManager final
{
public:
  AnnotationManager();
  AnnotationManager(const AnnotationManager&) = delete;

  MessageQueue::OutBinary add();
  MessageQueue& getOut();

private:
  MessageQueue outData;
  unsigned annotationCounter = 0;
};
