/**
 * @file SimulatedNao/Views/SACFieldView.cpp
 *
 * Implementation of class SACFieldView
 *
 * @author Tatjana Thielke
 */

#include "SACFieldView.h"

SimRobot::Widget* SACFieldView::createWidget()
{
  return new SACFieldWidget(*this);
}

void SACFieldWidget::paint(QPainter& painter)
{
  updateTransform(painter);
  SYNC_WITH(view.console);
  paintDrawings(painter);
}

void SACFieldWidget::mousePressEvent(QMouseEvent* event)
{
  if(event->button() == Qt::LeftButton)
  {
    QPointF pos(event->pos());
    window2viewport(pos);
    Vector2f point(pos.x(), pos.y());
    point.rotate(90_deg);
    pos = QPointF(point.x(), point.y());
    pos.setY(view.ySign * pos.y());
    view.console.sharedAutonomyRequest.targetPose = Pose2f(static_cast<float>(pos.x()), static_cast<float>(pos.y()));
    view.console.sharedAutonomyRequest.arrowPosition = view.console.sharedAutonomyRequest.targetPose.translation;
    view.console.sharedAutonomyRequest.controlOperations = view.console.sharedAutonomyRequest.requestedOperations;
    view.console.sharedAutonomyRequest.mousePressed = true;
    QWidget::update();
  }
}

void SACFieldWidget::mouseMoveEvent(QMouseEvent* event)
{
  if(view.console.sharedAutonomyRequest.mousePressed)
  {
    QPointF pos(event->pos());
    window2viewport(pos);
    Vector2f point(pos.x(), pos.y());
    point.rotate(90_deg);
    pos = QPointF(point.x(), point.y());
    pos.setY(view.ySign * pos.y());
    view.console.sharedAutonomyRequest.arrowPosition = Vector2f(static_cast<float>(pos.x()), static_cast<float>(pos.y()));
    QWidget::update();
  }
}

void SACFieldWidget::mouseReleaseEvent(QMouseEvent*)
{
  if(view.console.sharedAutonomyRequest.mousePressed)
  {
    view.console.sharedAutonomyRequest.mousePressed = false;
    view.console.sharedAutonomyRequest.targetPose.rotation = (view.console.sharedAutonomyRequest.arrowPosition - view.console.sharedAutonomyRequest.targetPose.translation).angle();
    QWidget::update();
  }
}

void SACFieldWidget::mouseDoubleClickEvent(QMouseEvent*)
{
}

void SACFieldWidget::wheelEvent(QWheelEvent*)
{
}

void SACFieldWidget::keyPressEvent(QKeyEvent* event)
{
  event->ignore();
}

bool SACFieldWidget::event(QEvent* event)
{
  return QWidget::event(event);
}
