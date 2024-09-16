/**
 * @file SimulatedNao/Views/FieldView.cpp
 *
 * Implementation of class FieldView
 *
 * @author Thomas Röfer
 * @author Colin Graf
 */

#include <QMenu>
#include "FieldView.h"
#include "Representations/Configuration/FieldDimensions.h"

FieldView::FieldView(const QString& fullName, RobotConsole& console,
                     const std::string& name, const std::string& threadName)
  : DrawingView(fullName, console, name, threadName, ":/Icons/icons8-stadium-50.png", -1.f) {}

SimRobot::Widget* FieldView::createWidget()
{
  return new FieldWidget(*this);
}

FieldWidget::FieldWidget(FieldView& view)
  : DrawingWidget(view, QPointF(0.f, 0.f), view.console.fieldViews[view.name])
{
  FieldDimensions fieldDimensions;
  fieldDimensions.load();
  viewSize = QSize(static_cast<int>(fieldDimensions.xPosOpponentFieldBorder * 2.f),
                   static_cast<int>(fieldDimensions.yPosLeftFieldBorder * 2.f));
}

void FieldWidget::paint(QPainter& painter)
{
  updateTransform(painter);
  SYNC_WITH(view.console);
  paintDrawings(painter);
}

std::vector<std::pair<std::string, const DebugDrawing*>> FieldWidget::getDrawings(const std::string& name) const
{
  return DrawingWidget::getDrawings(name, [](const RobotConsole::ThreadData& data) -> const RobotConsole::Drawings&
  {
    return data.fieldDrawings;
  });
}

QMenu* FieldWidget::createUserMenu() const
{
  return new QMenu(tr("&Field"));
}
