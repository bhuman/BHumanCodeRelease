/**
 * @file Controller/Views/KickView/TabWidget.cpp
 *
 * Implementation of class TabWiget
 *
 * @author <a href="mailto:judy@tzi.de">Judith Müller</a>
 */

#include <QMouseEvent>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QMimeData>
#include <QDrag>
#include <QApplication>
#include "TabWidget.h"

TabBar::TabBar(QWidget* parent) : QTabBar(parent)
{
  setUsesScrollButtons(true);
  setAcceptDrops(true);
}

void TabBar::mousePressEvent(QMouseEvent* event)
{
  if(event->button() == Qt::LeftButton)
    m_dragStartPos = event->pos();
  QTabBar::mousePressEvent(event);
}

void TabBar::mouseMoveEvent(QMouseEvent* event)
{
  if(!(event->buttons() & Qt::LeftButton))
    return;

  if(tabAt(m_dragStartPos) == 0 ||
     ((event->pos() - m_dragStartPos).manhattanLength()
      < QApplication::startDragDistance()))
    return;

  QDrag* drag = new QDrag(this);
  QMimeData* mimeData = new QMimeData;
  mimeData->setData("action", "tab-reordering");
  drag->setMimeData(mimeData);
  drag->exec();
}

void TabBar::dragEnterEvent(QDragEnterEvent* event)
{
  if(tabAt(m_dragStartPos) == 0)
    return;

  const QMimeData* m = event->mimeData();
  QStringList formats = m->formats();
  if(formats.contains("action") && (m->data("action") == "tab-reordering"))
    event->acceptProposedAction();
}

void TabBar::dropEvent(QDropEvent* event)
{
  int fromIndex = tabAt(m_dragStartPos);
  int toIndex = tabAt(event->pos());
  if(fromIndex != 0 && toIndex != 0)
  {
    if(fromIndex != toIndex)
      emit tabMoveRequested(fromIndex, toIndex);
  }
  event->acceptProposedAction();
}

TabWidget::TabWidget(QWidget* parent) : QTabWidget(parent)
{
  tb = new TabBar();
  setTabBar(tb);
}
