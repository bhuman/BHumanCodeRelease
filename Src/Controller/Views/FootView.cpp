/**
* @file Controller/Views/FootView.cpp
*
* Implementation of class FootView
* @author Felix Wenk
*/

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wconversion"
#endif
#include <QWidget>
#include <QPainter>
#include <QApplication>
#include <QMouseEvent>
#include <QResizeEvent>
#include <QSettings>
#include <QMenu>
#ifdef __clang__
#pragma clang diagnostic pop
#endif

#include "Controller/RobotConsole.h"
#include "Controller/RoboCupCtrl.h"
#include "Controller/Visualization/PaintMethods.h"
#include "Platform/Thread.h"
#include "FootView.h"

class FootWidget : public QWidget, public SimRobot::Widget
{
public:
  FootWidget(FootView& footView, const RobotBalance& robotBalance)
    : footView(footView), robotBalance(robotBalance),
      ankleMark(-5.0f, -5.0f, 10.0f, 10.0f), maxFootLength(175.0f), maxFootWidth(110.0f)
  {
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);

    QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
    settings.beginGroup(footView.fullName);
    // Load settings here.
    settings.endGroup();
  }

  virtual ~FootWidget()
  {
    QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
    settings.beginGroup(footView.fullName);
    // Write settings here.
    settings.endGroup();
  }

private:
  FootView& footView;
  const RobotBalance& robotBalance;
  QPainter painter;
  float scale;
  float pointRadius;
  float largePointRadius;
  QRectF leftFootShape;
  QRectF rightFootShape;
  QTransform leftAnkle;
  QTransform rightAnkle;
  const QRect ankleMark;
  const float maxFootLength;
  const float maxFootWidth;

  void paintEvent(QPaintEvent* event)
  {
    painter.begin(this);
    paint(painter);
    painter.end();
  }

  virtual void paint(QPainter& painter)
  {
    const QSize& size = painter.window().size();
    pointRadius = maxFootLength / 200.0f;
    largePointRadius = pointRadius * 2.0f;
    const float footSpacing = 0.1f * maxFootWidth;
    float xScale = static_cast<float>(size.width()) / (2.f * maxFootWidth + 2.f * footSpacing);
    float yScale = static_cast<float>(size.height()) / (maxFootLength + footSpacing);
    scale = xScale < yScale ? xScale : yScale;
    const QTransform scaleTransform(scale, 0, 0, -scale, 0, size.height());
    painter.setTransform(scaleTransform);
    leftFootShape = QRectF(0.5f * footSpacing, 0.5f * footSpacing, maxFootWidth, maxFootLength);
    rightFootShape = QRectF(1.5f * footSpacing + maxFootWidth, 0.5f * footSpacing,
                           maxFootWidth, maxFootLength);
    const float alx = (footSpacing + maxFootWidth) / 2.0f;
    const float aly = footSpacing / 2.0f + maxFootLength / 3.0f;
    const float arx = (footSpacing + maxFootWidth) * 3.0f / 2.0f;
    leftAnkle = QTransform::fromTranslate(alx, aly);
    rightAnkle = QTransform::fromTranslate(arx, aly); // imaginary 'ary' == aly
    leftAnkle.rotate(90.0);
    rightAnkle.rotate(90.0);

    {
      SYNC_WITH(footView.console);
      paintDrawings(painter);
    }
  }

  void paintDrawings(QPainter& painter)
  {
    // Retain current painter properties.
    painter.save();

    // Paint foot shapes.
    painter.drawRect(leftFootShape);
    painter.drawRect(rightFootShape);

    // Paint ankle projections
    painter.drawRect(leftAnkle.mapRect(ankleMark));
    painter.drawRect(rightAnkle.mapRect(ankleMark));

    // Draw dot for zmp.
    const QTransform& ankle = robotBalance.leftSupport ? leftAnkle : rightAnkle;
    painter.setBrush(QBrush(Qt::blue));
    QRectF zmp(robotBalance.zmp.x - largePointRadius,
               robotBalance.zmp.y - largePointRadius,
               2.0f * largePointRadius, 2.0f * largePointRadius);
    painter.drawEllipse(ankle.mapRect(zmp));

    // Draw wanted zmp and preview.
    const QPen savedpen = painter.pen();
    const int colorStep = 255 / robotBalance.numZmpPreview;
    for(int i = robotBalance.numZmpPreview - 1; i >= 1; --i)
    {
      const QRectF pzmp(robotBalance.zmpPreview[i].x - largePointRadius,
                        robotBalance.zmpPreview[i].y - largePointRadius,
                        2.0f * largePointRadius, 2.0f * largePointRadius);
      const int color = i * colorStep;
      const QColor red(255, color, color);
      painter.setBrush(QBrush(red));
      painter.setPen(red);
      painter.drawEllipse(ankle.mapRect(pzmp));
    }
    painter.setPen(savedpen);


    // Restore painter properties.
    painter.restore();
  }

  QSize sizeHint() const { return QSize(static_cast<int>(maxFootWidth * 1.5f), static_cast<int>(maxFootLength * 1.5f)); }

  virtual QWidget* getWidget() {return this;}

  void update()
  {
    QWidget::update();
  }

  virtual QMenu* createUserMenu() const {return new QMenu(tr("&Foot"));}

  friend class FootView;
};

FootView::FootView(const QString& fullName, RobotConsole& console, const RobotBalance& robotBalance, const std::string& name) :
  fullName(fullName), icon(":/Icons/tag_green.png"), console(console),
  robotBalance(robotBalance), name(name) {}

SimRobot::Widget* FootView::createWidget()
{
  return new FootWidget(*this, robotBalance);
}
