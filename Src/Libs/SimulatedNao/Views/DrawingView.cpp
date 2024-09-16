/**
 * @file DrawingView.cpp
 *
 * This file implements a class that is the base for 2D views that display drawings.
 *
 * @author Thomas Röfer
 */

#include <QPinchGesture>
#include <QSettings>
#include "DrawingView.h"
#include "SimulatedNao/ConsoleRoboCupCtrl.h"
#include "SimulatedNao/Visualization/PaintMethods.h"

DrawingView::DrawingView(const QString& fullName, RobotConsole& console,
                         const std::string& name, const std::string& threadName,
                         const char* iconName, float ySign)
  : fullName(fullName), icon(iconName), console(console),
    name(name), threadName(threadName), ySign(ySign)
{
  icon.setIsMask(true);
}

DrawingWidget::DrawingWidget(DrawingView& view, const QPointF& origin,
                             const std::list<std::string>& drawings)
  : view(view), origin(origin), drawings(drawings), dragStart(-1, -1), offset(0, 0)
{
  setFocusPolicy(Qt::StrongFocus);
  setMouseTracking(true);
  grabGesture(Qt::PinchGesture);

  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(view.fullName);
  zoom = static_cast<float>(settings.value("Zoom", 1.).toDouble());
  offset = settings.value("Offset", QPointF()).toPoint();
  settings.endGroup();
}

DrawingWidget::~DrawingWidget()
{
  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(view.fullName);
  settings.setValue("Zoom", static_cast<double>(zoom));
  settings.setValue("Offset", offset);
  settings.endGroup();
}

void DrawingWidget::update()
{
  if(needsRepaint())
    QWidget::update();
}

void DrawingWidget::paintEvent(QPaintEvent*)
{
  painter.begin(this);
  paint(painter);
  painter.end();
}

void DrawingWidget::mousePressEvent(QMouseEvent* event)
{
  QWidget::mousePressEvent(event);

  if(event->button() == Qt::LeftButton || event->button() == Qt::MiddleButton)
  {
    dragStart = event->pos();
    dragStartOffset = offset;
  }
}

void DrawingWidget::mouseMoveEvent(QMouseEvent* event)
{
  QWidget::mouseMoveEvent(event);

  if(dragStart != QPointF(-1, -1))
  {
    offset = dragStartOffset + (event->pos() - dragStart) / scale;
    QWidget::update();
  }
  else
  {
    QPointF pos(event->pos());
    window2viewport(pos);
    pos.setY(view.ySign * pos.y());

    // Update tool tip
    std::unordered_map<std::string, Pose2f> transforms;
    {
      SYNC_WITH(view.console);
      for(const std::string& drawing : drawings)
        for(auto& [threadName, debugDrawing] : getDrawings(drawing))
        {
          auto transform = transforms.find(threadName);
          if(transform == transforms.end())
          {
            transforms.emplace(threadName, Pose2f());
            transform = transforms.find(threadName);
          }
          debugDrawing->updateOrigin(transform->second);
          int x = static_cast<int>(pos.x());
          int y = static_cast<int>(pos.y());
          const char* text = debugDrawing->getTip(x, y, transform->second);
          if(text)
          {
            setToolTip(QString(text));
            return;
          }
        }
    }
    setToolTip(QString());
  }
}

void DrawingWidget::mouseReleaseEvent(QMouseEvent* event)
{
  QWidget::mouseReleaseEvent(event);

  dragStart = QPointF(-1, -1);
  QWidget::update();
}

void DrawingWidget::mouseDoubleClickEvent(QMouseEvent* event)
{
  QWidget::mouseDoubleClickEvent(event);
  zoom = 1.f;
  offset.setX(0);
  offset.setY(0);
  QWidget::update();
}

void DrawingWidget::wheelEvent(QWheelEvent* event)
{
  QWidget::wheelEvent(event);
  QPointF beforeZoom = event->position();
  window2viewport(beforeZoom);
  scale /= zoom;
  zoom += zoom * 0.1f * event->angleDelta().y() / 120.f;
  zoomRange.clamp(zoom);
  scale *= zoom;
  QPointF afterZoom = event->position();
  window2viewport(afterZoom);
  offset += afterZoom - beforeZoom;

  QWidget::update();
}

void DrawingWidget::keyPressEvent(QKeyEvent* event)
{
  switch(event->key())
  {
    case Qt::Key_PageUp:
    case Qt::Key_Plus:
      zoom += 0.1f * zoom;
      break;
    case Qt::Key_PageDown:
    case Qt::Key_Minus:
      zoom -= 0.1f * zoom;
      break;
    case Qt::Key_Up:
      offset -= QPointF(0, viewSize.width() * offsetStepRatio / zoom);
      break;
    case Qt::Key_Down:
      offset += QPointF(0, viewSize.width() * offsetStepRatio / zoom);
      break;
    case Qt::Key_Left:
      offset -= QPointF(viewSize.height() * offsetStepRatio / zoom, 0);
      break;
    case Qt::Key_Right:
      offset += QPointF(viewSize.height() * offsetStepRatio / zoom, 0);
      break;
    default:
      QWidget::keyPressEvent(event);
      return;
  }
  event->accept();
  zoomRange.clamp(zoom);
  QWidget::update();
}

bool DrawingWidget::event(QEvent* event)
{
  if(event->type() == QEvent::Gesture)
  {
    QPinchGesture* pinch = dynamic_cast<QPinchGesture*>(dynamic_cast<QGestureEvent*>(event)->gesture(Qt::PinchGesture));
    if(pinch && (pinch->changeFlags() & QPinchGesture::ScaleFactorChanged))
    {
      QPointF center(pinch->centerPoint().x(),
                     pinch->centerPoint().y());
      QPointF before(center);
      window2viewport(before);
      scale /= zoom;
#ifdef FIX_MACOS_PINCH_SCALE_RELATIVE_BUG
      pinch->setLastScaleFactor(1.f);
#endif
      zoom *= static_cast<float>(pinch->scaleFactor() / pinch->lastScaleFactor());
      zoomRange.clamp(zoom);
      scale *= zoom;
      QPointF after(center);
      window2viewport(after);
      offset -= before - after;
      QWidget::update();
      return true;
    }
  }
  return QWidget::event(event);
}

void DrawingWidget::updateTransform(QPainter& painter)
{
  const QSize& size = painter.window().size();
  const float xScale = static_cast<float>(size.width()) / static_cast<float>(viewSize.width());
  const float yScale = static_cast<float>(size.height()) / static_cast<float>(viewSize.height());
  scale = xScale < yScale ? xScale : yScale;
  scale *= zoom;
  const float xOffset = static_cast<float>(size.width()) * 0.5f + static_cast<float>(offset.x() + origin.x() * viewSize.width()) * scale;
  const float yOffset = static_cast<float>(size.height()) * 0.5f + static_cast<float>(offset.y() + origin.y() * viewSize.height()) * scale;
  painter.setTransform(QTransform(scale, 0, 0, view.ySign * scale, xOffset, yOffset));
}

void DrawingWidget::paintDrawings(QPainter& painter)
{
  const QTransform baseTrans(painter.transform());
  std::unordered_map<std::string, QTransform> transforms;
  for(const std::string& drawing : drawings)
    for(auto& [threadName, debugDrawing] : getDrawings(drawing))
    {
      auto transform = transforms.find(threadName);
      if(transform == transforms.end())
      {
        transforms.emplace(threadName, baseTrans);
        transform = transforms.find(threadName);
      }
      painter.setTransform(transform->second);
      PaintMethods::paintDebugDrawing(painter, *debugDrawing, baseTrans);
      transform->second = painter.transform();
      if(debugDrawing->timestamp > lastDrawingsTimestamp)
        lastDrawingsTimestamp = debugDrawing->timestamp;
    }

  painter.setTransform(baseTrans);
}

bool DrawingWidget::needsRepaint() const
{
  SYNC_WITH(view.console);

  for(const std::string& drawing : drawings)
    for(const auto& [_, debugDrawing] : getDrawings(drawing))
      if(debugDrawing && debugDrawing->timestamp > lastDrawingsTimestamp)
        return true;

  return false;
}

void DrawingWidget::window2viewport(QPointF& point)
{
  const QSize& size(this->size());
  const float xScale = float(size.width()) / float(viewSize.width());
  const float yScale = float(size.height()) / float(viewSize.height());
  float scale = xScale < yScale ? xScale : yScale;
  scale *= zoom;
  const float xOffset = float(size.width()) * 0.5f + float(offset.x() + origin.x() * viewSize.width()) * scale;
  const float yOffset = float(size.height()) * 0.5f + float(offset.y() + origin.y() * viewSize.height()) * scale;
  point = QPointF((point.x() - xOffset) / scale, (point.y() - yOffset) / scale);
}

std::vector<std::pair<std::string, const DebugDrawing*>> DrawingWidget::getDrawings(const std::string& name,
  const std::function<const RobotConsole::Drawings&(const RobotConsole::ThreadData&)>& thread2Drawings) const
{
  std::vector<std::pair<std::string, const DebugDrawing*>> found;
  const RobotConsole::ThreadData* thisData = nullptr;
  if(!view.threadName.empty())
  {
    thisData = &view.console.threadData[view.threadName];

    // First search for the thread belonging to this view
    auto debugDrawing = thread2Drawings(*thisData).find(name);
    if(debugDrawing != thread2Drawings(*thisData).end())
    {
      found.emplace_back(view.threadName, &debugDrawing->second);
      return found;
    }
  }

  // Search other threads, but consider only drawings that did not use THREAD
  for(auto& [threadName, data] : view.console.threadData)
  {
    auto debugDrawing = thread2Drawings(data).find(name);
    if(debugDrawing != thread2Drawings(data).end()
       && (debugDrawing->second.threadName == threadName // drawing was forwarded to other thread
           || !thisData // view is not associated with a thread
           || std::find_if(thisData->drawingManager.drawings.begin(), thisData->drawingManager.drawings.end(),
                           [this, name](const auto& pair){return name == view.console.ctrl->translate(pair.first);})
                           == thisData->drawingManager.drawings.end())) // associated thread cannot provide this drawing
      found.emplace_back(threadName, &debugDrawing->second);
  }

  return found;
}
