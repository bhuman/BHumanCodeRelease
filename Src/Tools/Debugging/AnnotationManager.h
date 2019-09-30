/**
 * @file AnnotationManager.h
 * @author Andreas Stolpmann
 */

#pragma once

#include "Tools/MessageQueue/MessageQueue.h"

#include <vector>
#include <unordered_map>

class AnnotationManager
{
private:
  MessageQueue outData;
  unsigned annotationCounter = 0;
  unsigned lastGameState;
  unsigned lastSetPlay;

  friend class ThreadFrame; /**< A thread is allowed to create the instance. */

  /**
   * Default constructor.
   * No other instance of this class is allowed except the one accessible via Global::getAnnotationManager.
   * Therefore the constructor is private.
   */
  AnnotationManager();

public:
  void signalThreadStart();
  void clear();

  void addAnnotation();
  MessageQueue& getOut();
};
