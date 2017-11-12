#include "FootWidget.h"
#include "FootViewWidget.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Sensing/FootGroundContactState.h"
#include "Tools/RobotParts/FootShape.h"

#include <algorithm>

FootWidget::FootWidget(const FootViewWidget& footViewWidget) : footViewWidget(footViewWidget)
{
  for(size_t i = 0; i < FootShape::polygon.size(); ++i)
  {
    const Vector2f& p = FootShape::polygon[i];
    leftFootPoly.append(QPointF(p.x(), p.y()));
    rightFootPoly.append(QPointF(p.x(), -p.y()));
  }
  setMinimumSize(QSize(200, 150));
}

void FootWidget::paintEvent(QPaintEvent* event)
{
  const FootGroundContactState& footGroundContactState = footViewWidget.footGroundContactState;
  const JointSensorData& jointSensorData = footViewWidget.jointSensorData;
  const RobotDimensions& robotDimensions = footViewWidget.robotDimensions;

  if(jointSensorData.timestamp == 0)
    return;

  painter.begin(this);
  painter.setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing);
  auto maxXElem = std::max_element(FootShape::polygon.begin(), FootShape::polygon.end(), [](const Vector2f& p1, const Vector2f& p2)
  {
    return p1.x() < p2.x();
  });
  auto minXElem = std::max_element(FootShape::polygon.begin(), FootShape::polygon.end(), [](const Vector2f& p1, const Vector2f& p2)
  {
    return p1.x() > p2.x();
  });
  auto maxYElem = std::max_element(FootShape::polygon.begin(), FootShape::polygon.end(), [](const Vector2f& p1, const Vector2f& p2)
  {
    return p1.y() < p2.y();
  });

  const float paintWidth = (robotDimensions.yHipOffset + maxYElem->y()) * 2.f;
  const float paintHeight = (maxXElem->x() - minXElem->x());
  const float paintOffset = (maxXElem->x() + minXElem->x());

  const QSize windowSize = painter.window().size();
  const float xScale = float(windowSize.width()) / paintWidth;
  const float yScale = float(windowSize.height()) / paintHeight;
  const float scale = (xScale < yScale ? xScale : yScale) * 0.95f;

  const QPen defaultPen(QColor(0, 0, 0), static_cast<int>(std::max(1.f, std::sqrt(scale))));
  const QPen leftContactPen(QColor(255, 0, 0), static_cast<int>(std::max(1.f, std::sqrt(scale))));
  const QPen rightContactPen(QColor(0, 0, 255), static_cast<int>(std::max(1.f, std::sqrt(scale))));

  painter.setPen(defaultPen);

  QTransform trans;
  trans.translate(windowSize.width() / 2.f, windowSize.height() / 2.f);
  trans.scale(-1, 1);
  painter.setTransform(trans);

  QTransform midTrans;
  midTrans.scale(scale, scale);
  midTrans.rotate(-90);
  midTrans.translate(-paintOffset / 2.f, 0);

  painter.drawLine(midTrans.map(QLineF(0, 0, 20, 0)));
  painter.drawText(midTrans.map(QPointF(20, 0)), "x");
  painter.drawLine(midTrans.map(QLineF(0, 0, 0, 20)));
  painter.drawText(midTrans.map(QPointF(0, 20)), "y");

  bool contactLeft = footGroundContactState.contact == FootGroundContactState::both || footGroundContactState.contact == FootGroundContactState::left;
  bool contactRight = footGroundContactState.contact == FootGroundContactState::both || footGroundContactState.contact == FootGroundContactState::right;

  QTransform leftTrans = midTrans;
  leftTrans.translate(0, robotDimensions.yHipOffset);

  // draw left fsr position circles
  const QRectF circle(5, 5, -10, -10);
  for(unsigned i = 0; i < FsrSensors::numOfFsrSensors; ++i)
  {
    painter.setPen(footGroundContactState.leftSensorContacts[i] ? leftContactPen : defaultPen);
    const Vector2f& pos = robotDimensions.leftFsrPositions[i];
    painter.drawEllipse(leftTrans.mapRect(circle.translated(pos.x(), pos.y())));
  }

  // draw left foot
  painter.setPen(contactLeft ? leftContactPen : defaultPen);
  painter.drawPoint(leftTrans.map(QPointF(0, 0)));
  painter.drawPolygon(leftTrans.map(leftFootPoly));

  const QLineF l1(5, 5, -5, -5);
  const QLineF l2(5, -5, -5, 5);

  // draw left CoP
  if(contactLeft)
  {
    const Vector2f& leftCoP = footGroundContactState.centerOfPressure[Legs::left];
    painter.drawLine(leftTrans.map(l1.translated(QPointF(leftCoP.x(), leftCoP.y()))));
    painter.drawLine(leftTrans.map(l2.translated(QPointF(leftCoP.x(), leftCoP.y()))));
  }

  //-----------------------------

  QTransform rightTrans = midTrans;
  rightTrans.translate(0, -robotDimensions.yHipOffset);

  // draw left fsr position circles
  for(unsigned i = 0; i < FsrSensors::numOfFsrSensors; ++i)
  {
    painter.setPen(footGroundContactState.rightSensorContacts[i] ? rightContactPen : defaultPen);
    const Vector2f& pos = robotDimensions.rightFsrPositions[i];
    painter.drawEllipse(rightTrans.mapRect(circle.translated(pos.x(), pos.y())));
  }

  // draw right foot
  painter.setPen(contactRight ? rightContactPen : defaultPen);
  painter.drawPoint(rightTrans.map(QPointF(0, 0)));
  painter.drawPolygon(rightTrans.map(rightFootPoly));

  // draw right CoP
  if(contactRight)
  {
    const Vector2f& rightCoP = footGroundContactState.centerOfPressure[Legs::right];
    painter.drawLine(rightTrans.map(l1.translated(QPointF(rightCoP.x(), rightCoP.y()))));
    painter.drawLine(rightTrans.map(l2.translated(QPointF(rightCoP.x(), rightCoP.y()))));
  }

  // draw CoP of both foot when existent
  if(contactLeft && contactRight)
  {
    Vector2f leftPosSum = footGroundContactState.centerOfPressure[Legs::left] * footGroundContactState.totalMass[FootGroundContactState::left];
    Vector2f rightPosSum = footGroundContactState.centerOfPressure[Legs::right] * footGroundContactState.totalMass[FootGroundContactState::right];
    leftPosSum.y() += robotDimensions.yHipOffset * footGroundContactState.totalMass[FootGroundContactState::left];
    rightPosSum.y() -= robotDimensions.yHipOffset * footGroundContactState.totalMass[FootGroundContactState::right];

    const Vector2f CoP = (leftPosSum + rightPosSum) / footGroundContactState.totalMass[FootGroundContactState::both];

    const QPen greenPen(QColor(0, 255, 0), static_cast<int>(std::max(1.f, std::sqrt(scale))));
    painter.setPen(greenPen);
    painter.drawLine(midTrans.map(l1.translated(QPointF(CoP.x(), CoP.y()))));
    painter.drawLine(midTrans.map(l2.translated(QPointF(CoP.x(), CoP.y()))));
  }

  painter.end();
}
