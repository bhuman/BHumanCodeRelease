/**
 * @file AnnotationManager.h
 * @author Andreas Stolpmann
 */

#pragma once

#include "Tools/MessageQueue/MessageQueue.h"

class AnnotationManager final
{
public:
  AnnotationManager();
  AnnotationManager(const AnnotationManager&) = delete;

  void signalThreadStart();
  void clear();

  void addAnnotation();
  MessageQueue& getOut();

private:
  MessageQueue outData;
  unsigned annotationCounter = 0;
  unsigned lastGameState;
  unsigned lastSetPlay;
};
