/**
 * @file AnnotationManager.h
 * @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
 */

#pragma once

#include "Tools/MessageQueue/MessageQueue.h"

#include <vector>
#include <unordered_map>

class AnnotationManager
{
private:
  MessageQueue outData;
  unsigned currentFrame = 0;
  unsigned annotationCounter = 0;
  unsigned lastGameState;

  friend class Process;

  AnnotationManager(); // private so only Process can access it.

public:
  void signalProcessStart();
  void clear();

  void addAnnotation();
  MessageQueue& getOut();
};
