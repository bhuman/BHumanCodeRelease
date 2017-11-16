#pragma once

#include "SimRobot.h"

#include <QPainter>
#include <QWidget>

class FootViewWidget;

class FootWidget : public QWidget
{
private:
  const FootViewWidget& footViewWidget;
  QPainter painter;
  QPolygonF leftFootPoly;
  QPolygonF rightFootPoly;

public:
  FootWidget(const FootViewWidget& footViewWidget);

  void paintEvent(QPaintEvent* event);
};
