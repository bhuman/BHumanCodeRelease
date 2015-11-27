/**
* @file AnnotationView.cpp
* @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
*/

#include "AnnotationView.h"
#include "AnnotationWidget.h"
#include "Platform/SystemCall.h"
#include "Controller/Representations/AnnotationInfo.h"

AnnotationView::AnnotationView(const QString& fullName, AnnotationInfo& info, LogPlayer& logPlayer, SimRobot::Application* application)
  : fullName(fullName), icon(":/Icons/tag_green.png"), info(info), logPlayer(logPlayer), application(application){}

SimRobot::Widget* AnnotationView::createWidget()
{
  return new AnnotationWidget(*this);
}
