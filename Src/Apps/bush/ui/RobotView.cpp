#include <QDrag>
#include <QDropEvent>
#include <QDragEnterEvent>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QMimeData>
#include <QMouseEvent>
#include <QPalette>
#include <QProgressBar>

#include "agents/PingAgent.h"
#include "models/Robot.h"
#include "models/Team.h"
#include "ui/RobotView.h"
#include "ui/TeamSelector.h"
#include "ui/RobotPool.h"
#include "Session.h"
#include "ui/SizeManager.h"
#ifdef MACOS
#include "AppleHelper/Helper.h"
#endif

RobotView::RobotView(TeamSelector* teamSelector,
                     Robot* robot,
                     unsigned short playerNumber,
                     unsigned short pos) :
  QGroupBox(teamSelector),
  teamSelector(teamSelector),
  robot(robot),
  playerNumber(playerNumber),
  pos(pos)
{
  QFormLayout* layout = new QFormLayout();
  QFont font;
  font.setBold(true);
  this->setFont(font);

  if(playerNumber)
    cPlayerNumber = new QLabel(QString("<font size=5><b>") + QString::number(playerNumber) + QString("</b></font>"));
  SizeManager sizeManager;

  statusWidget = new QWidget(this);
  statusWidget->setMaximumSize(sizeManager.statusWidgetWidth, sizeManager.statusWidgetHeight);
  this->setMaximumWidth(sizeManager.robotViewWidth);
  this->setMaximumHeight(sizeManager.robotViewHeight);
  QGridLayout* statusLayout = new QGridLayout(statusWidget);
  statusLayout->setContentsMargins(0, 0, 0, 0);
  statusLayout->setVerticalSpacing(2);

  QLabel* pingLabelWLAN = new QLabel("<font size=2><b>WLAN</b></font>", statusWidget);
  pingBarWLAN = new QLabel(this);
  pingBarWLAN->setMaximumSize(sizeManager.statusBarWidth, sizeManager.statusBarHeight);
  pingBarWLAN->setAlignment(Qt::AlignCenter);
  setPing(WLAN, robot, PingAgent::pingTimeout);
  statusLayout->addWidget(pingLabelWLAN, 0, 0, Qt::AlignLeft);
  statusLayout->addWidget(pingBarWLAN, 0, 1);

  QLabel* pingLabelLAN = new QLabel("<font size=2><b>LAN</b></font>", statusWidget);
  pingBarLAN = new QLabel(this);
  pingBarLAN->setMaximumSize(sizeManager.statusBarWidth, sizeManager.statusBarHeight);
  pingBarLAN->setAlignment(Qt::AlignCenter);
  setPing(LAN, robot, PingAgent::pingTimeout);
  statusLayout->addWidget(pingLabelLAN, 1, 0, Qt::AlignLeft);
  statusLayout->addWidget(pingBarLAN, 1, 1);

  QLabel* powerLabel = new QLabel("<font size=2><b>Power</b></font>", statusWidget);
  powerBar = new QProgressBar(this);
  powerBar->setMaximumSize(sizeManager.statusBarWidth, static_cast<int>(sizeManager.statusBarHeight * 0.8f));
  powerBar->setRange(0, 100);
  powerBar->setValue(0);
  powerBar->setAlignment(Qt::AlignCenter);
  statusLayout->addWidget(powerLabel, 2, 0, Qt::AlignLeft);
  statusLayout->addWidget(powerBar, 2, 1);

  QLabel* logLabel = new QLabel("<font size=2><b>Logs</b></font>", statusWidget);
  logStatus = new QLabel("--");
  statusLayout->addWidget(logLabel, 3, 0, Qt::AlignLeft);
  statusLayout->addWidget(logStatus, 3, 1);

  QLabel* dumpLabel = new QLabel("<font size=2><b>Crashed?</b></font>", statusWidget);
  dumpStatus = new QLabel("no");
  statusLayout->addWidget(dumpLabel, 4, 0, Qt::AlignLeft);
  statusLayout->addWidget(dumpStatus, 4, 1);

  layout->addRow(cPlayerNumber, statusWidget);

  setLayout(layout);
  connect(this, &RobotView::toggled, this, &RobotView::setSelected);

  if(robot)
  {
    Session::getInstance().registerPingListener(this);
    Session::getInstance().registerStatusListener(this, robot);
  }

  setStyleSheet("QGroupBox::title { \
                subcontrol-origin: margin;\
                left: 10px; \
                margin-top: 0px; \
                } \
                QGroupBox { \
                border: 1px solid gray; \
                border-radius: 2px; \
                margin-top: 0.5em; \
                } \
                ");
  update();
  setAcceptDrops(true);
}

void RobotView::update()
{
  if(playerNumber)
    cPlayerNumber->setVisible(false);
  statusWidget->setVisible(false);
  setCheckable(false);
  if(robot)
  {
    Robot* r = robot;
    robot = 0;
    setCheckable(playerNumber);
    robot = r;
    if(playerNumber)
      setChecked(isSelected());
    std::string ipPostfix = robot->wlan.substr(robot->wlan.length() - 2);
    setTitle(QString::fromStdString(robot->name + " (." + ipPostfix + ")"));
    if(playerNumber)
    {
      cPlayerNumber->setEnabled(isSelected());
      cPlayerNumber->setVisible(true);
    }
    statusWidget->setVisible(true);
  }
  else
  {
    setTitle(QString("Empty"));
    cPlayerNumber->setVisible(true);
    cPlayerNumber->setEnabled(false);
  }
}

QString RobotView::getRobotName() const
{
  if(!robot)
    return "";
  return QString::fromStdString(robot->name);
}

bool RobotView::isSelected() const
{
  if(!robot)
    return false;
  return teamSelector->getSelectedTeam()->isPlayerSelected(robot);
}

void RobotView::setRobot(Robot* robot)
{
  if(this->robot)
  {
    Session::getInstance().removePingListener(this);
    Session::getInstance().removeStatusListener(this, this->robot);
  }
  this->robot = robot;
  if(playerNumber)
  {
    Team* team = teamSelector->getSelectedTeam();
    team->changePlayer(playerNumber, pos, robot);
  }
  if(robot)
  {
    Session::getInstance().registerPingListener(this);
    Session::getInstance().registerStatusListener(this, robot);
  }
  emit robotChanged();
}

void RobotView::setPing(ENetwork network, const Robot* robot, double ping)
{
  if(robot != this->robot)
    return;
  QLabel* bar = network == LAN ? pingBarLAN : pingBarWLAN;

  QString sheet;
  if(ping >= PingAgent::pingTimeout)
#ifdef MACOS
    sheet = "QLabel { background-color : " + getAlternateBase().color().name(QColor::HexArgb) + "; border: 1px solid silver; }";
#else
    sheet = "QLabel { background-color : #e6e6e6; border: 1px solid silver; }";
#endif
  else if(ping >= 500)
    sheet = "QLabel { background-color : red; border: 1px solid silver; color : black; }";
  else if(ping >= 250)
    sheet = "QLabel { background-color : yellow; border: 1px solid silver; color : black; }";
  else
    sheet = "QLabel { background-color : lime; border: 1px solid silver; color : black; }";

  if(sheet != bar->styleSheet())
    bar->setStyleSheet(sheet);

  if(ping < PingAgent::pingTimeout)
    bar->setText(QString::number(static_cast<int>(ping)) + " ms");
  else
    bar->setText("n/a");
}

void RobotView::setStatus(const Robot* robot, const Status* status)
{
  if(robot != this->robot)
    return;

  powerBar->setValue(status->isPowerValid() ? status->batteryLevel : 0);

#ifdef MACOS
  QString sheet = "QProgressBar::chunk { background-color: lime; } QProgressBar { background-color : "
                  + getAlternateBase().color().name(QColor::HexArgb) + "; border: 1px solid silver; }";
#else
  QString sheet = "QProgressBar::chunk { background-color: lime; } QProgressBar { background-color : "
                  "#e6e6e6; border: 1px solid silver; }";
#endif
  if(status->isPowerValid())
  {
    if(status->batteryCharging)
      sheet = "QProgressBar::chunk { background-color: lime; } QProgressBar { color: black; }";
    else
      sheet = "QProgressBar::chunk { background-color: red; } QProgressBar { color: black; }";
  }
  if(sheet != powerBar->styleSheet())
    powerBar->setStyleSheet(sheet);

  if(status->logs != 0)
    logStatus->setText(QString("<font color='red'><b>") + QString::number(status->logs) + QString("</b></font>"));
  else
    logStatus->setText("--");

  if(status->hasDump)
    dumpStatus->setText("<font color='red'><b>yes</b></font>");
  else
    dumpStatus->setText("no");
}

void RobotView::mouseMoveEvent(QMouseEvent* me)
{
  if(!robot)
    return;
  QDrag* d = new QDrag(this);
  QPixmap pm = QWidget::grab(rect());
  d->setPixmap(pm);
  d->setHotSpot(me->pos());
  QMimeData* data = new QMimeData();
  data->setText(QString::fromStdString(robot->name));
  d->setMimeData(data);
  d->exec(Qt::MoveAction);
  me->accept();
}

void RobotView::dragEnterEvent(QDragEnterEvent* e)
{
  if(e->source() && e->source() != this && e->source()->inherits("RobotView"))
    e->acceptProposedAction();
}

void RobotView::dropEvent(QDropEvent* e)
{
  e->accept();
  QString robotName = e->mimeData()->text();
  Robot* r = Session::getInstance().robotsByName[robotName.toStdString()];
  RobotView* source = dynamic_cast<RobotView*>(e->source());
  if(source->playerNumber)
  {
    bool selected = source->isSelected();
    if(source->robot)
      source->setSelected(false);
    if(robot)
      source->setRobot(robot);
    else
      source->setRobot(nullptr);
    source->setSelected(selected);
    source->update();
    setRobot(r);
    update();
  }
  else
  {
    bool selected = isSelected();
    setSelected(false);
    setRobot(r);
    setSelected(selected);
    update();
    source->setRobot(r);
  }
}

void RobotView::setSelected(bool selected)
{
  if(robot)
  {
    teamSelector->getSelectedTeam()->setSelectPlayer(robot, selected);
    if(cPlayerNumber)
      cPlayerNumber->setEnabled(selected);
  }
}
