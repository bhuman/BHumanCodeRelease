/**
 * @file AnnotationInfo.cpp
 * @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
 */

#pragma once

#include "Platform/Thread.h"
#include "Tools/MessageQueue/InMessage.h"

#include <vector>

class AnnotationView;

class AnnotationInfo : public MessageHandler
{
public:
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

  bool handleMessage(InMessage& message);
};
