#pragma once

#include <QScrollArea>

class QEvent;
class QWidget;

class ScrollArea : public QScrollArea
{
  Q_OBJECT

  bool scrollEnabled = true;

public:
  ScrollArea(QWidget* parent);

  bool viewportEvent(QEvent* event) override;

public slots:
  void updateScrollEnabled();
};
