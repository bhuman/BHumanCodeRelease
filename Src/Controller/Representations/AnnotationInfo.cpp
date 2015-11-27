/**
* @file AnnotationInfo.cpp
* @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
*/

#include "AnnotationInfo.h"

#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include "Tools/Global.h"
#include "Tools/MessageQueue/InMessage.h"
#include "Tools/MessageQueue/OutMessage.h"

AnnotationInfo::AnnotationInfo() : timeOfLastMessage(0)
{}

bool AnnotationInfo::handleMessage(InMessage& message)
{
  if(message.getMessageID() == idAnnotation)
  {
    timeOfLastMessage = SystemCall::getCurrentSystemTime();
    {
      SYNC;
      newAnnotations.push_back(AnnotationData());
      AnnotationData& data = newAnnotations.back();

      message.bin >> data.annotationNumber;
      message.bin >> data.frame;
      message.text >> data.name;
      data.annotation = message.text.readAll();
    }
  }
  return true;
}
