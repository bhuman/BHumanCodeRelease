/**
 * @file AnnotationView.cpp
 * @author Andreas Stolpmann
 */

#include "AnnotationView.h"
#include "AnnotationWidget.h"
#include "Platform/SystemCall.h"

AnnotationView::AnnotationView(const QString& fullName, AnnotationInfo::ProcessAnnotationInfo& info, LogPlayer& logPlayer, SystemCall::Mode mode, SimRobot::Application* application) :
  fullName(fullName), icon(":/Icons/tag_green.png"), info(info), logPlayer(logPlayer), mode(mode), application(application)
{}

SimRobot::Widget* AnnotationView::createWidget()
{
  return new AnnotationWidget(*this, mode);
}
