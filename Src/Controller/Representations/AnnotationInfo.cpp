/**
 * @file AnnotationInfo.cpp
 * @author Andreas Stolpmann
 */

#include "AnnotationInfo.h"

#include "Platform/BHAssert.h"
#include "Platform/Time.h"
#include "Tools/Global.h"
#include "Tools/MessageQueue/InMessage.h"
#include "Tools/MessageQueue/OutMessage.h"
#include "Controller/Views/AnnotationView/AnnotationView.h"

#include <QRegExp>

AnnotationInfo::AnnotationInfo()
{
  annotationProcesses['c'];
  annotationProcesses['m'];
}

bool AnnotationInfo::handleMessage(InMessage& message)
{
  if(message.getMessageID() == idProcessBegin)
  {
    char current;
    message.bin >> current;
    switch(current)
    {
      case 'c':
        currentProcess = &annotationProcesses['c'];
        break;
      case 'm':
        currentProcess = &annotationProcesses['m'];
        break;
      default:
        ASSERT(false);
    }
  }
  else if(message.getMessageID() == idProcessFinished)
  {
    currentProcess = nullptr;
  }
  else if(message.getMessageID() == idAnnotation && currentProcess != nullptr)
  {
    currentProcess->timeOfLastMessage = Time::getCurrentSystemTime();
    {
      SYNC_WITH(*currentProcess);
      currentProcess->newAnnotations.push_back(AnnotationData());
      AnnotationData& data = currentProcess->newAnnotations.back();

      message.bin >> data.annotationNumber;
      message.bin >> data.frame;
      message.text >> data.name;
      data.annotation = message.text.readAll();

      if(currentProcess->view && currentProcess->view->stopOnFilter)
      {
        const QString name = QString(data.name.c_str()).toLower();
        const QString annotation = QString(data.annotation.c_str()).toLower();

        if(currentProcess->view->filterIsRegEx)
        {
          QRegExp regex(currentProcess->view->filter);
          if(regex.exactMatch(name) || regex.exactMatch(annotation))
            currentProcess->view->application->simStop();
        }
        else
        {
          if(name.contains(currentProcess->view->filter) || annotation.contains(currentProcess->view->filter))
            currentProcess->view->application->simStop();
        }
      }
    }
  }
  return true;
}
