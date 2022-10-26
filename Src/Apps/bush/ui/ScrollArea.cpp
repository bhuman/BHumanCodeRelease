#include "ScrollArea.h"
#include <QEvent>
#include <QScrollBar>

ScrollArea::ScrollArea(QWidget* parent) :
  QScrollArea(parent)
{
  connect(verticalScrollBar(), &QScrollBar::valueChanged, this, &ScrollArea::updateScrollEnabled);
}

bool ScrollArea::viewportEvent(QEvent* event)
{
  bool ret = QScrollArea::viewportEvent(event);
  if(event->type() == QEvent::LayoutRequest && widget() && scrollEnabled)
    ensureVisible(0, widget()->size().height());
  return ret;
}

void ScrollArea::updateScrollEnabled()
{
  scrollEnabled = verticalScrollBar()->value() == verticalScrollBar()->maximum();
}
