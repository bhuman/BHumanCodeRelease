/**
 * @file Controller/Views/FieldView.cpp
 *
 * Implementation of class FieldView
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
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
#include "Representations/Configuration/FieldDimensions.h"

#define MAXZOOM 20.f
#define MINZOOM 0.1f

class FieldWidget : public WIDGET2D, public SimRobot::Widget
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

  ~FieldWidget()
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

  void paintEvent(QPaintEvent* event) override
  {
    painter.begin(this);
    paint(painter);
    painter.end();
  }

  void paint(QPainter& painter) override
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
    QTransform lowerTrans(painter.transform());
    QTransform upperTrans(painter.transform());
    QTransform motionTrans(painter.transform());
    const std::list<std::string>& drawings = fieldView.console.fieldViews[fieldView.name];
    for(const std::string& drawing : drawings)
    {
      const DebugDrawing& debugDrawingLowerCam = fieldView.console.lowerCamFieldDrawings[drawing];
      painter.setTransform(lowerTrans);
      PaintMethods::paintDebugDrawing(painter, debugDrawingLowerCam, baseTrans);
      lowerTrans = painter.transform();
      if(debugDrawingLowerCam.timeStamp > lastDrawingsTimeStamp)
        lastDrawingsTimeStamp = debugDrawingLowerCam.timeStamp;

      const DebugDrawing& debugDrawingUpperCam = fieldView.console.upperCamFieldDrawings[drawing];
      painter.setTransform(upperTrans);
      PaintMethods::paintDebugDrawing(painter, debugDrawingUpperCam, baseTrans);
      upperTrans = painter.transform();
      if(debugDrawingUpperCam.timeStamp > lastDrawingsTimeStamp)
        lastDrawingsTimeStamp = debugDrawingUpperCam.timeStamp;

      const DebugDrawing& debugDrawingMotion = fieldView.console.motionFieldDrawings[drawing];
      painter.setTransform(motionTrans);
      PaintMethods::paintDebugDrawing(painter, debugDrawingMotion, baseTrans);
      motionTrans = painter.transform();
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

  void mouseMoveEvent(QMouseEvent* event) override
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
    for(size_t i = 0; i < sizeof(debugDrawings) / sizeof(RobotConsole::Drawings*) && !text; ++i)
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

  void keyPressEvent(QKeyEvent* event) override
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

  bool event(QEvent* event) override
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

  void wheelEvent(QWheelEvent* event) override
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

  void mousePressEvent(QMouseEvent* event) override
  {
    QWidget::mousePressEvent(event);

    if(event->button() == Qt::LeftButton || event->button() == Qt::MidButton)
    {
      dragStart = event->pos();
      dragStartOffset = offset;
    }
  }

  void mouseReleaseEvent(QMouseEvent* event) override
  {
    QWidget::mouseReleaseEvent(event);

    dragStart = QPoint(-1, -1);
  }

  void mouseDoubleClickEvent(QMouseEvent* event) override
  {
    QWidget::mouseDoubleClickEvent(event);
    zoom = 1;
    offset = QPoint(0, 0);
    QWidget::update();
  }

  QSize sizeHint() const override { return QSize(int(fieldDimensions.xPosOpponentFieldBorder * 0.2f), int(fieldDimensions.yPosLeftFieldBorder * 0.2f)); }

  QWidget* getWidget() override { return this; }

  void update() override
  {
    if(needsRepaint())
      QWidget::update();
  }
  QMenu* createUserMenu() const override { return new QMenu(tr("&Field")); }

  friend class FieldView;
};

FieldView::FieldView(const QString& fullName, RobotConsole& console, const std::string& name) :
  fullName(fullName), icon(":/Icons/tag_green.png"), console(console), name(name)
{}

SimRobot::Widget* FieldView::createWidget()
{
  return new FieldWidget(*this);
}
