/**
 * @file Controller/Views/FieldView.cpp
 *
 * Implementation of class FieldView
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 * @author Colin Graf
 */

#include <QWidget>
#include <QPainter>
#include <QApplication>
#include <QMouseEvent>
#include <QPinchGesture>
#include <QResizeEvent>
#include <QSettings>
#include <QMenu>

#include "Controller/RobotConsole.h"
#include "Controller/RoboCupCtrl.h"
#include "Controller/Visualization/PaintMethods.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Platform/Thread.h"
#include "FieldView.h"

#define MAXZOOM 20.f
#define MINZOOM 0.1f

class FieldWidget : public QWidget, public SimRobot::Widget
{
public:
  FieldWidget(FieldView& fieldView) :
    fieldView(fieldView)
  {
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);
    grabGesture(Qt::PinchGesture);
    setAttribute(Qt::WA_AcceptTouchEvents);
    fieldDimensions.load();

    QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
    settings.beginGroup(fieldView.fullName);
    zoom = (float)settings.value("Zoom", 1.).toDouble();
    offset = settings.value("Offset", QPoint()).toPoint();
    settings.endGroup();
  }

  virtual ~FieldWidget()
  {
    QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
    settings.beginGroup(fieldView.fullName);
    settings.setValue("Zoom", (double)zoom);
    settings.setValue("Offset", offset);
    settings.endGroup();
  }

private:
  FieldView& fieldView;
  unsigned lastDrawingsTimeStamp = 0;
  QPainter painter;
  FieldDimensions fieldDimensions; /**< The field dimensions. */
  float scale;
  float zoom = 1.f;
  QPoint offset;
  QPoint dragStart;
  QPoint dragStartOffset;

  void paintEvent(QPaintEvent* event)
  {
    painter.begin(this);
    paint(painter);
    painter.end();
  }

  virtual void paint(QPainter& painter)
  {
    const QSize& size = painter.window().size();
    int viewWidth = int(fieldDimensions.xPosOpponentFieldBorder) * 2;
    int viewHeight = int(fieldDimensions.yPosLeftFieldBorder) * 2;
    float xScale = float(size.width()) / viewWidth;
    float yScale = float(size.height()) / viewHeight;
    scale = xScale < yScale ? xScale : yScale;
    scale *= zoom;
    painter.setTransform(QTransform(scale, 0, 0, -scale, size.width() / 2. - offset.x() * scale, size.height() / 2. - offset.y() * scale));

    {
      SYNC_WITH(fieldView.console);
      paintDrawings(painter);
    }
  }

  void paintDrawings(QPainter& painter)
  {
    const QTransform baseTrans(painter.transform());
    const std::list<std::string>& drawings = fieldView.console.fieldViews[fieldView.name];
    for(const std::string& drawing : drawings)
    {
      const DebugDrawing& debugDrawingLowerCam = fieldView.console.lowerCamFieldDrawings[drawing];
      PaintMethods::paintDebugDrawing(painter, debugDrawingLowerCam, baseTrans);
      if(debugDrawingLowerCam.timeStamp > lastDrawingsTimeStamp)
        lastDrawingsTimeStamp = debugDrawingLowerCam.timeStamp;

      const DebugDrawing& debugDrawingUpperCam = fieldView.console.upperCamFieldDrawings[drawing];
      PaintMethods::paintDebugDrawing(painter, debugDrawingUpperCam, baseTrans);
      if(debugDrawingUpperCam.timeStamp > lastDrawingsTimeStamp)
        lastDrawingsTimeStamp = debugDrawingUpperCam.timeStamp;

      const DebugDrawing& debugDrawingMotion = fieldView.console.motionFieldDrawings[drawing];
      PaintMethods::paintDebugDrawing(painter, debugDrawingMotion, baseTrans);
      if(debugDrawingMotion.timeStamp > lastDrawingsTimeStamp)
        lastDrawingsTimeStamp = debugDrawingMotion.timeStamp;
    }

    painter.setTransform(baseTrans);
  }

  bool needsRepaint() const
  {
    SYNC_WITH(fieldView.console);
    const std::list<std::string>& drawings = fieldView.console.fieldViews[fieldView.name];
    for(const std::string& drawing : drawings)
    {
      const DebugDrawing& debugDrawingLowerCam(fieldView.console.lowerCamFieldDrawings[drawing]);
      const DebugDrawing& debugDrawingUpperCam(fieldView.console.upperCamFieldDrawings[drawing]);
      const DebugDrawing& debugDrawingMotion(fieldView.console.motionFieldDrawings[drawing]);

      if(debugDrawingLowerCam.timeStamp > lastDrawingsTimeStamp
         || debugDrawingUpperCam.timeStamp > lastDrawingsTimeStamp
         || debugDrawingMotion.timeStamp > lastDrawingsTimeStamp)
        return true;
    }
    return false;
  }

  void window2viewport(QPoint& point)
  {
    const QSize& size(this->size());
    point = QPoint((int)((point.x() - size.width() / 2) / scale), (int)((point.y() - size.height() / 2) / scale));
    point += offset;
    point.ry() = -point.ry();
  }

  void mouseMoveEvent(QMouseEvent* event)
  {
    QPoint pos(event->pos());
    if(dragStart.x() > 0)
    {
      offset = dragStartOffset + (dragStart - pos) / scale;
      QWidget::update();
      return;
    }

    window2viewport(pos);

    // Update tool tip
    SYNC_WITH(fieldView.console);
    const char* text = 0;
    RobotConsole::Drawings* debugDrawings[] =
    {
      &fieldView.console.lowerCamFieldDrawings,
      &fieldView.console.upperCamFieldDrawings,
      &fieldView.console.motionFieldDrawings
    };
    for(int i = 0; i < 3 && !text; ++i)
    {
      RobotConsole::Drawings& debugDrawing = *debugDrawings[i];
      Pose2f origin;
      const std::list<std::string>& drawings(fieldView.console.fieldViews[fieldView.name]);
      for(const std::string& drawing : drawings)
      {
        debugDrawing[drawing].updateOrigin(origin);
        text = debugDrawing[drawing].getTip(pos.rx(), pos.ry(), origin);
        if(text)
          break;
      }
    }

    if(text)
      setToolTip(QString(text));
    else
      setToolTip(QString());
  }

  void keyPressEvent(QKeyEvent* event)
  {
    switch(event->key())
    {
      case Qt::Key_PageUp:
      case Qt::Key_Plus:
        event->accept();
        if(zoom >= 1.f)
          zoom *= 1.1f;
        else
          zoom += MINZOOM;
        if(zoom >= MAXZOOM)
          zoom = MAXZOOM;
        QWidget::update();
        break;
      case Qt::Key_PageDown:
      case Qt::Key_Minus:
        event->accept();
        if(zoom >= 1.f)
          zoom /= 1.1f;
        else if(zoom > MINZOOM)
          zoom -= 0.1f;
        if(zoom <= MINZOOM)
          zoom = MINZOOM;
        QWidget::update();
        break;
      case Qt::Key_Down:
        event->accept();
        offset += QPoint(0, 100);
        QWidget::update();
        break;
      case Qt::Key_Up:
        event->accept();
        offset += QPoint(0, -100);
        QWidget::update();
        break;
      case Qt::Key_Left:
        event->accept();
        offset += QPoint(-100, 0);
        QWidget::update();
        break;
      case Qt::Key_Right:
        event->accept();
        offset += QPoint(100, 0);
        QWidget::update();
        break;
      default:
        QWidget::keyPressEvent(event);
        break;
    }
  }

  bool event(QEvent* event)
  {
    if(event->type() == QEvent::Gesture)
    {
      QPinchGesture* pinch = static_cast<QPinchGesture*>(static_cast<QGestureEvent*>(event)->gesture(Qt::PinchGesture));
      if(pinch && (pinch->changeFlags() & QPinchGesture::ScaleFactorChanged))
      {
#ifdef FIX_MACOS_NO_CENTER_IN_PINCH_GESTURE_BUG
        QPoint center = mapFromGlobal(QCursor::pos());
#else
        QPoint center(static_cast<int>(pinch->centerPoint().x()),
                      static_cast<int>(pinch->centerPoint().y()));
#endif
        QPoint before(center);
        window2viewport(before);
        scale /= zoom;
#ifdef FIX_MACOS_PINCH_SCALE_RELATIVE_BUG
        pinch->setLastScaleFactor(1.f);
#endif
        zoom *= pinch->scaleFactor() / pinch->lastScaleFactor();
        if(zoom >= MAXZOOM)
          zoom = MAXZOOM;
        else if(zoom <= MINZOOM)
          zoom = MINZOOM;
        scale *= zoom;
        QPoint after(center);
        window2viewport(after);
        QPoint diff = before - after;
        diff.ry() *= -1;
        offset += diff;
        QWidget::update();
        return true;
      }
    }
    return QWidget::event(event);
  }

  void wheelEvent(QWheelEvent* event)
  {
    QWidget::wheelEvent(event);

#ifndef MACOS
    zoom += 0.1 * event->delta() / 120;
    if(zoom >= MAXZOOM)
      zoom = MAXZOOM;
    else if(zoom <= MINZOOM)
      zoom = MINZOOM;
    QWidget::update();
#else
    int step = -static_cast<int>(event->delta() / (scale * 2.f));
    offset += event->orientation() == Qt::Horizontal ? QPoint(step, 0) : QPoint(0, step);
    if(step)
      QWidget::update();
#endif
  }

  void mousePressEvent(QMouseEvent* event)
  {
    QWidget::mousePressEvent(event);

    if(event->button() == Qt::LeftButton || event->button() == Qt::MidButton)
    {
      dragStart = event->pos();
      dragStartOffset = offset;
    }
  }

  void mouseReleaseEvent(QMouseEvent* event)
  {
    QWidget::mouseReleaseEvent(event);

    dragStart = QPoint(-1, -1);
  }

  void mouseDoubleClickEvent(QMouseEvent* event)
  {
    QWidget::mouseDoubleClickEvent(event);
    zoom = 1;
    offset = QPoint(0, 0);
    QWidget::update();
  }

  QSize sizeHint() const { return QSize(int(fieldDimensions.xPosOpponentFieldBorder * 0.2f), int(fieldDimensions.yPosLeftFieldBorder * 0.2f)); }

  virtual QWidget* getWidget() { return this; }

  void update()
  {
    if(needsRepaint())
      QWidget::update();
  }
  virtual QMenu* createUserMenu() const { return new QMenu(tr("&Field")); }

  friend class FieldView;
};

FieldView::FieldView(const QString& fullName, RobotConsole& console, const std::string& name) :
  fullName(fullName), icon(":/Icons/tag_green.png"), console(console), name(name)
{}

SimRobot::Widget* FieldView::createWidget()
{
  return new FieldWidget(*this);
}
