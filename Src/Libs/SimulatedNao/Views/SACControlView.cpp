#include "SACControlView.h"
#include "SimulatedNao/RobotConsole.h"
#include "Theme.h"

#include <QBoxLayout>
#include <QEvent>
#include <QHelpEvent>
#include <QLabel>
#include <QRadioButton>
#include <QShortcut>

SACControlWidget::SACControlWidget(SACControlView& sacControlView) :
  sacControlView(sacControlView)
{
  setFocusPolicy(Qt::StrongFocus);
  sacControlView.console.sharedAutonomyRequest.isValid = true;

  tactic = new QCheckBox(QString("allow goals"));
  QFont font = tactic->font();
  puts(font.family().toStdString().c_str());
  font.setPixelSize(25);
  tactic->setFont(font);
  tactic->setShortcut(Qt::Key_Q);

  QVBoxLayout* controlBlock = new QVBoxLayout;
  QLabel* controls = new QLabel(
    QString(
      "<b>Controls:</b> <br> "
      "<b>Arrows:</b> head control <br> "
      "<b>Space:</b>  deactivate head control <br>"
      "<b>W + Click:</b> kick to target <br> "
      "<b>E + Click:</b> dribble to target <br> "
      "<b>R:</b> pass to teammate <br> "
      "<b>S:</b> stand <br> "
      "<b>Q:</b> switch tactic <br> "
    )
  );
  font = controls->font();
  font.setPixelSize(18);
  controls->setFont(font);
  controlBlock->addWidget(controls);

  QVBoxLayout* rightPanel = new QVBoxLayout;
  rightPanel->addWidget(tactic);
  rightPanel->addLayout(controlBlock);

  QWidget* restrictor = new QWidget;
  restrictor->setLayout(rightPanel);
  restrictor->setMaximumWidth(275);
  restrictor->setContentsMargins(0, 0, 0, 0);

  connect(tactic, &QCheckBox::stateChanged, this, &SACControlWidget::tacticChanged);

  QHBoxLayout* mainWindow = new QHBoxLayout();
  mainWindow->setContentsMargins(0, 0, 0, 0);
  setLayout(mainWindow);
  setEnabled(sacControlView.console.sharedAutonomyRequest.isValid);
  fieldWidget = static_cast<SACFieldWidget*>(sacControlView.field.createWidget());

  mainWindow->addWidget(fieldWidget->getWidget());
  mainWindow->addWidget(restrictor);
}

void SACControlWidget::update()
{
  if(fieldWidget->needsRepaint())
    fieldWidget->update();
  if(holdHeadPosition)
  {
    sacControlView.console.sharedAutonomyRequest.headRequest.angle.x() = JointAngles::ignore;
    sacControlView.console.sharedAutonomyRequest.headRequest.angle.y() = JointAngles::ignore;
    holdHeadPosition = false;
  }
}

void SACControlWidget::tacticChanged()
{
  sacControlView.console.sharedAutonomyRequest.allowGoalKicks = tactic->isChecked();
}

SimRobot::Widget* SACControlView::createWidget()
{
  return new SACControlWidget(*this);
}

void SACControlWidget::paint(QPainter& painter)
{
  fieldWidget->paint(painter);
}

void SACControlWidget::keyPressEvent(QKeyEvent* event)
{
  switch(event->key())
  {
    case Qt::Key_Up:
      sacControlView.console.sharedAutonomyRequest.headRequest.manual = true;
      sacControlView.console.sharedAutonomyRequest.headRequest.angle.y() = -38.5_deg;
      break;
    case Qt::Key_Down:
      sacControlView.console.sharedAutonomyRequest.headRequest.manual = true;
      sacControlView.console.sharedAutonomyRequest.headRequest.angle.y() = 29.5_deg;
      break;
    case Qt::Key_Left:
      sacControlView.console.sharedAutonomyRequest.headRequest.manual = true;
      sacControlView.console.sharedAutonomyRequest.headRequest.angle.x() = 119.5_deg;
      break;
    case Qt::Key_Right:
      sacControlView.console.sharedAutonomyRequest.headRequest.manual = true;
      sacControlView.console.sharedAutonomyRequest.headRequest.angle.x() = -119.5_deg;
      break;
    case Qt::Key_Space:
      sacControlView.console.sharedAutonomyRequest.headRequest.manual = false;
      break;
    case Qt::Key_W:
      sacControlView.console.sharedAutonomyRequest.requestedOperations = SharedAutonomyRequest::Operations::playBallKickToPoint;
      break;
    case Qt::Key_E:
      sacControlView.console.sharedAutonomyRequest.requestedOperations = SharedAutonomyRequest::Operations::playBallDribbleToPoint;
      break;
    case Qt::Key_R:
      sacControlView.console.sharedAutonomyRequest.controlOperations = SharedAutonomyRequest::Operations::playBallPassToMate;
      break;
    case Qt::Key_S:
      sacControlView.console.sharedAutonomyRequest.controlOperations = SharedAutonomyRequest::Operations::stand;
      break;
    default:
      QWidget::keyPressEvent(event);
      return;
  }
  event->accept();
  QWidget::update();
}

void SACControlWidget::keyReleaseEvent(QKeyEvent* event)
{
  if(event->isAutoRepeat())
  {
    QWidget::keyReleaseEvent(event);
    return;
  }
  switch(event->key())
  {
    case Qt::Key_Up:
    case Qt::Key_Down:
    case Qt::Key_Left:
    case Qt::Key_Right:
      holdHeadPosition = true;
      break;
    case Qt::Key_W:
    case Qt::Key_E:
      sacControlView.console.sharedAutonomyRequest.requestedOperations = SharedAutonomyRequest::Operations::walkToTarget;
      break;
    default:
      QWidget::keyReleaseEvent(event);
      return;
  }
  event->accept();
  QWidget::update();
}
