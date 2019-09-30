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
#include "Controller/Views/AnnotationView.h"

#include <QRegExp>

void AnnotationInfo::clear()
{
  SYNC;
  timeOfLastMessage = Time::getCurrentSystemTime();
  newAnnotations.push_back(AnnotationData());
  newAnnotations.back().name = "CLEAR";
}

void AnnotationInfo::addMessage(InMessage& message, unsigned currentFrame)
{
  SYNC;
  timeOfLastMessage = Time::getCurrentSystemTime();
  newAnnotations.push_back(AnnotationData());
  AnnotationData& data = newAnnotations.back();

  message.bin >> data.annotationNumber;
  if(!(data.annotationNumber & 0x80000000))
    message.bin >> data.frame; // Compatibility with old annotations
  data.annotationNumber &= ~0x80000000;
  data.frame = currentFrame;
  message.text >> data.name;
  data.annotation = message.text.readAll();

  if(view && view->stopOnFilter)
  {
    const QString name = QString(data.name.c_str()).toLower();
    const QString annotation = QString(data.annotation.c_str()).toLower();

    if(view->filterIsRegEx)
    {
      QRegExp regex(view->filter);
      if(regex.exactMatch(name) || regex.exactMatch(annotation))
        view->application->simStop();
    }
    else if(name.contains(view->filter) || annotation.contains(view->filter))
      view->application->simStop();
  }
}
