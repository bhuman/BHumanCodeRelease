/**
 * @file AnnotationInfo.h
 * @author Andreas Stolpmann
 */

#pragma once

#include "Platform/Thread.h"
#include "Tools/MessageQueue/InMessage.h"

#include <vector>

class AnnotationView;

class AnnotationInfo
{
  struct AnnotationData
  {
    unsigned annotationNumber;
    unsigned frame;
    std::string name;
    std::string annotation;
  };

  DECLARE_SYNC;
  std::vector<AnnotationData> newAnnotations;
  unsigned timeOfLastMessage = 0;
  const AnnotationView* view = nullptr;

public:
  void clear();
  void addMessage(InMessage& message, unsigned currentFrame);

  friend class AnnotationView;
  friend class AnnotationWidget;
};
