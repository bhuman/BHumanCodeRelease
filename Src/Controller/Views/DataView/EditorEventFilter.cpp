/**
 * EditorEventFilter.cpp
 *
 *  Created on: May 3, 2012
 *      Author: arne
 */

#include "EditorEventFilter.h"
#include <QEvent>
#include "DataView.h"

EditorEventFilter::EditorEventFilter(QObject* pParent, DataView* pView, QWidget* pSource, QtProperty* pProperty) :
  QObject(pParent), pTheView(pView), pTheSource(pSource), pTheProperty(pProperty)
{}

bool EditorEventFilter::eventFilter(QObject* obj, QEvent* event)
{
  if(nullptr != pTheView &&
     (event->type() == QEvent::FocusIn || event->type() == QEvent::Paint ||
      event->type() == QEvent::FocusOut || event->type() == QEvent::LayoutRequest)) // prevent sending events after death
  {
    return pTheView->handlePropertyEditorEvent(pTheSource, pTheProperty, event);
  }
  else
    return false;
}
