/**
 * @file AnnotationInfo.cpp
 * @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
 */

#include "AnnotationInfo.h"

#include "Platform/BHAssert.h"
#include "Platform/Time.h"
#include "Tools/Global.h"
#include "Tools/MessageQueue/InMessage.h"
#include "Tools/MessageQueue/OutMessage.h"
#include "Controller/Views/AnnotationView/AnnotationView.h"

#include <QRegExp>

bool AnnotationInfo::handleMessage(InMessage& message)
{
  if(message.getMessageID() == idAnnotation)
  {
    timeOfLastMessage = Time::getCurrentSystemTime();
    {
      SYNC;
      newAnnotations.push_back(AnnotationData());
      AnnotationData& data = newAnnotations.back();

      message.bin >> data.annotationNumber;
      message.bin >> data.frame;
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
        else
        {
          if(name.contains(view->filter) || annotation.contains(view->filter))
            view->application->simStop();
        }
      }
    }
  }
  return true;
}
