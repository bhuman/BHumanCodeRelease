/**
 * @file AnnotationInfo.cpp
 * @author Andreas Stolpmann
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

  struct ProcessAnnotationInfo
  {
    DECLARE_SYNC;
    std::vector<AnnotationData> newAnnotations;
    unsigned timeOfLastMessage = 0;
    const AnnotationView* view = nullptr;
  };

  ProcessAnnotationInfo* currentProcess = nullptr;

  std::unordered_map<char, ProcessAnnotationInfo> annotationProcesses;

  AnnotationInfo();

  bool handleMessage(InMessage& message);
};
