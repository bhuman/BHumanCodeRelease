/**
 * EditorEventFilter.h
 *
 *  Created on: May 3, 2012
 *      Author: arne
 */

#pragma once

#include <QObject>

class QEvent;
class DataView;
class QWidget;
class QtProperty;

class EditorEventFilter : public QObject
{
public:
  /**
   * @param The editor will forward all events to the DataView.
   * @param pSource the editor which is the source of all events.
   * @param pProperty The property which belongs to the editor.
   */
  EditorEventFilter(QObject* pParent, DataView* pView, QWidget* pSource, QtProperty* pProperty);

protected:
  bool eventFilter(QObject* obj, QEvent* event);

private:
  DataView* pTheView;
  QWidget* pTheSource;
  QtProperty* pTheProperty;
};
