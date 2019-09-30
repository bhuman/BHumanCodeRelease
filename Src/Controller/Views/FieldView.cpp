/**
 * @file Controller/Views/FieldView.cpp
 *
 * Implementation of class FieldView
 *
 * @author Thomas Röfer
 * @author Colin Graf
 */

#include <QApplication>
#include <QWidget>
#include <QMenu>
#include <QMouseEvent>
#include <QPainter>
#include <QPinchGesture>
#include <QResizeEvent>
#include <QSettings>

#include "FieldView.h"
#include "Controller/RoboCupCtrl.h"
#include "Controller/RobotConsole.h"
#include "Controller/Visualization/PaintMethods.h"

FieldView::FieldView(const QString& fullName, RobotConsole& console, const std::string& name) :
  fullName(fullName), icon(":/Icons/tag_green.png"), console(console), name(name)
{}

SimRobot::Widget* FieldView::createWidget()
{
  return new FieldWidget(*this);
}

FieldWidget::FieldWidget(FieldView& fieldView) :
  fieldView(fieldView), dragStart(-1, -1), offset(0, 0)
{
  setFocusPolicy(Qt::StrongFocus);
  setMouseTracking(true);
  grabGesture(Qt::PinchGesture);
  setAttribute(Qt::WA_AcceptTouchEvents);
  fieldDimensions.load();

  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(fieldView.fullName);
  zoom = static_cast<float>(settings.value("Zoom", 1.).toDouble());
  offset = settings.value("Offset", QPointF()).toPoint();
  settings.endGroup();
}

FieldWidget::~FieldWidget()
{
  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(fieldView.fullName);
  settings.setValue("Zoom", static_cast<double>(zoom));
  settings.setValue("Offset", offset);
  settings.endGroup();
}

void FieldWidget::paintEvent(QPaintEvent* event)
{
  painter.begin(this);
  paint(painter);
  painter.end();
}

void FieldWidget::paint(QPainter& painter)
{
  const QSize& size = painter.window().size();
  float xScale = float(size.width()) / (fieldDimensions.xPosOpponentFieldBorder * 2.f);
  float yScale = float(size.height()) / (fieldDimensions.yPosLeftFieldBorder * 2.f);
  scale = xScale < yScale ? xScale : yScale;
  scale *= zoom;
  const float imageXOffset = float(size.width()) * 0.5f - float(offset.x()) * scale;
  const float imageYOffset = float(size.height()) * 0.5f - float(offset.y()) * scale;
  painter.setTransform(QTransform(scale, 0, 0, -scale, imageXOffset, imageYOffset));

  {
    SYNC_WITH(fieldView.console);
    paintDrawings(painter);
  }
}

void FieldWidget::paintDrawings(QPainter& painter)
{
  const QTransform baseTrans(painter.transform());
  std::unordered_map<std::string, QTransform> transforms;
  for(const std::string& drawing : fieldView.console.fieldViews[fieldView.name])
    for(auto& pair : getDrawings(drawing))
    {
      auto transform = transforms.find(pair.first);
      if(transform == transforms.end())
      {
        transforms.emplace(pair.first, baseTrans);
        transform = transforms.find(pair.first);
      }
      painter.setTransform(transform->second);
      PaintMethods::paintDebugDrawing(painter, *pair.second, baseTrans);
      transform->second = painter.transform();
      if(pair.second->timestamp > lastDrawingsTimestamp)
        lastDrawingsTimestamp = pair.second->timestamp;
    }

  painter.setTransform(baseTrans);
}

bool FieldWidget::needsRepaint() const
{
  SYNC_WITH(fieldView.console);
  std::string threadName;
  for(const std::string& drawing : fieldView.console.fieldViews[fieldView.name])
  {
    for(const auto& pair : getDrawings(drawing))
      if(pair.second->timestamp > lastDrawingsTimestamp)
        return true;
  }
  return false;
}

void FieldWidget::window2viewport(QPointF& point)
{
  const QSize& size(this->size());
  const float xScale = float(size.width()) / (fieldDimensions.xPosOpponentFieldBorder * 2.f);
  const float yScale = float(size.height()) / (fieldDimensions.yPosLeftFieldBorder * 2.f);
  float scale = xScale < yScale ? xScale : yScale;
  scale *= zoom;
  const float xOffset = (float(size.width())) * 0.5f - float(offset.x()) * scale;
  const float yOffset = (float(size.height())) * 0.5f - float(offset.y()) * scale;
  point = QPointF((point.x() - xOffset) / scale, -(point.y() - yOffset) / scale);
}

void FieldWidget::mouseMoveEvent(QMouseEvent* event)
{
  QWidget::mouseMoveEvent(event);
  QPointF pos(event->pos());
  mousePos = pos;

  if(dragStart != QPointF(-1, -1))
  {
    offset = dragStartOffset + (dragStart - pos) / scale;
    QWidget::update();
    return;
  }

  window2viewport(pos);

  // Update tool tip
  SYNC_WITH(fieldView.console);
  const char* text = 0;
  for(auto& data : fieldView.console.threadData)
  {
    Pose2f origin;
    const std::list<std::string>& drawings(fieldView.console.fieldViews[fieldView.name]);
    for(const std::string& drawing : drawings)
    {
      data.second.fieldDrawings[drawing].updateOrigin(origin);
      int x = static_cast<int>(pos.x());
      int y = static_cast<int>(pos.y());
      text = data.second.fieldDrawings[drawing].getTip(x, y, origin);
      if(text)
      {
        setToolTip(QString(text));
        return;
      }
    }
  }
  setToolTip(QString());
}

void FieldWidget::mousePressEvent(QMouseEvent* event)
{
  QWidget::mousePressEvent(event);

  if(event->button() == Qt::LeftButton || event->button() == Qt::MidButton)
  {
    dragStart = event->pos();
    dragStartOffset = offset;
  }
}

void FieldWidget::mouseReleaseEvent(QMouseEvent* event)
{
  QWidget::mouseReleaseEvent(event);

  dragStart = QPointF(-1, -1);
}

#define ZOOM_MAX_VALUE 20.f
#define ZOOM_MIN_VALUE 0.1f
void FieldWidget::keyPressEvent(QKeyEvent* event)
{
  switch(event->key())
  {
    case Qt::Key_PageUp:
    case Qt::Key_Plus:
      event->accept();
      if(zoom >= 1.f)
        zoom *= 1.1f;
      else
        zoom += ZOOM_MIN_VALUE;
      if(zoom > ZOOM_MAX_VALUE)
        zoom = ZOOM_MAX_VALUE;
      QWidget::update();
      break;
    case Qt::Key_PageDown:
    case Qt::Key_Minus:
      event->accept();
      if(zoom >= 1.f)
        zoom /= 1.1f;
      else if(zoom > ZOOM_MIN_VALUE)
        zoom -= 0.1f;
      if(zoom < ZOOM_MIN_VALUE)
        zoom = ZOOM_MIN_VALUE;
      QWidget::update();
      break;
    case Qt::Key_Down:
      event->accept();
      offset += QPointF(0, 100);
      QWidget::update();
      break;
    case Qt::Key_Up:
      event->accept();
      offset += QPointF(0, -100);
      QWidget::update();
      break;
    case Qt::Key_Left:
      event->accept();
      offset += QPointF(-100, 0);
      QWidget::update();
      break;
    case Qt::Key_Right:
      event->accept();
      offset += QPointF(100, 0);
      QWidget::update();
      break;
    default:
      QWidget::keyPressEvent(event);
      break;
  }
}

bool FieldWidget::event(QEvent* event)
{
  if(event->type() == QEvent::Gesture)
  {
    QPinchGesture* pinch = static_cast<QPinchGesture*>(static_cast<QGestureEvent*>(event)->gesture(Qt::PinchGesture));
    if(pinch && (pinch->changeFlags() & QPinchGesture::ScaleFactorChanged))
    {
#ifdef FIX_MACOS_NO_CENTER_IN_PINCH_GESTURE_BUG
      QPoint center = mapFromGlobal(QCursor::pos());
#else
      QPointF center(pinch->centerPoint().x(),
                     pinch->centerPoint().y());
#endif
      QPointF before(center);
      window2viewport(before);
      scale /= zoom;
#ifdef FIX_MACOS_PINCH_SCALE_RELATIVE_BUG
      pinch->setLastScaleFactor(1.f);
#endif
      zoom *= static_cast<float>(pinch->scaleFactor() / pinch->lastScaleFactor());
      if(zoom > ZOOM_MAX_VALUE)
        zoom = ZOOM_MAX_VALUE;
      else if(zoom < ZOOM_MIN_VALUE)
        zoom = ZOOM_MIN_VALUE;
      scale *= zoom;
      QPointF after(center);
      window2viewport(after);
      QPointF diff = before - after;
      diff.ry() *= -1;
      offset += diff;
      QWidget::update();
      return true;
    }
  }
  return QWidget::event(event);
}

void FieldWidget::wheelEvent(QWheelEvent* event)
{
  QWidget::wheelEvent(event);
#ifndef MACOS
  QPointF beforeZoom = mousePos;
  window2viewport(beforeZoom);

  zoom += 0.1f * event->delta() / 120.f;
  if(zoom > ZOOM_MAX_VALUE)
    zoom = ZOOM_MAX_VALUE;
  else if(zoom < ZOOM_MIN_VALUE)
    zoom = ZOOM_MIN_VALUE;

  QPointF afterZoom = mousePos;
  window2viewport(afterZoom);
  QPointF diff = beforeZoom - afterZoom;
  diff.ry() *= -1;
  offset += diff;

  QWidget::update();
#else
  float step = -event->delta() / (scale * 2.f);
  offset += event->orientation() == Qt::Horizontal ? QPointF(step, 0) : QPointF(0, step);
  if(step != 0.f)
    QWidget::update();
#endif
}

void FieldWidget::mouseDoubleClickEvent(QMouseEvent* event)
{
  QWidget::mouseDoubleClickEvent(event);
  zoom = 1.f;
  offset.setX(0);
  offset.setY(0);
  QWidget::update();
}

QMenu* FieldWidget::createUserMenu() const
{
  return new QMenu(tr("&Field"));
}

std::vector<std::pair<std::string, const DebugDrawing*>> FieldWidget::getDrawings(const std::string& name) const
{
  std::vector<std::pair<std::string, const DebugDrawing*>> drawings;
  for(auto& pair : fieldView.console.threadData)
  {
    auto debugDrawing = pair.second.fieldDrawings.find(name);
    if(debugDrawing != pair.second.fieldDrawings.end())
      drawings.emplace_back(pair.first, &debugDrawing->second);
  }
  return drawings;
}
