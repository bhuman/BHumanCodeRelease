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

  void addAnnotation();
  MessageQueue& getOut();

private:
  MessageQueue outData;
  unsigned annotationCounter = 0;
};
