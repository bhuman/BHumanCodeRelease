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
#include <QDesktopWidget>

#include "Utils/bush/models/Robot.h"
#include "Utils/bush/models/Team.h"
#include "Utils/bush/ui/RobotView.h"
#include "Utils/bush/ui/TeamSelector.h"
#include "Utils/bush/ui/RobotPool.h"
#include "Utils/bush/tools/StringTools.h"
#include "Utils/bush/Session.h"
#include "Utils/bush/ui/SizeManager.h"
#ifdef MACOS
#include "../Src/Controller/Visualization/Helper.h"
#endif

void RobotView::init()
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
  statusLayout->setMargin(0);
  statusLayout->setVerticalSpacing(2);

  QLabel* pingLabelWLAN = new QLabel("<font size=2><b>Wlan</b></font>", statusWidget);
  pingBarWLAN = new QLabel(this);
  pingBarWLAN->setMaximumSize(sizeManager.statusBarWidth, sizeManager.statusBarHeight);
  pingBarWLAN->setAlignment(Qt::AlignCenter);
  setPings(WLAN, 0);
  statusLayout->addWidget(pingLabelWLAN, 0, 0, Qt::AlignLeft);
  statusLayout->addWidget(pingBarWLAN, 0, 1);

  QLabel* pingLabelLAN = new QLabel("<font size=2><b>Lan</b></font>", statusWidget);
  pingBarLAN = new QLabel(this);
  pingBarLAN->setMaximumSize(sizeManager.statusBarWidth, sizeManager.statusBarHeight);
  pingBarLAN->setAlignment(Qt::AlignCenter);
  setPings(LAN, 0);
  statusLayout->addWidget(pingLabelLAN, 1, 0, Qt::AlignLeft);
  statusLayout->addWidget(pingBarLAN, 1, 1);

  QLabel* powerLabel = new QLabel("<font size=2><b>Power</b></font>", statusWidget);
  powerBar = new QProgressBar(this);
  powerBar->setMaximumSize(sizeManager.statusBarWidth, static_cast<int>(sizeManager.statusBarHeight * 0.8));
  powerBar->setRange(0, 100);
  powerBar->setValue(0);
  powerBar->setAlignment(Qt::AlignCenter);
  statusLayout->addWidget(powerLabel, 2, 0, Qt::AlignLeft);
  statusLayout->addWidget(powerBar, 2, 1);

  QLabel* logLabel = new QLabel("<font size=2><b>Logs</b></font>", statusWidget);
  logStatus = new QLabel("--");
  statusLayout->addWidget(logLabel, 3, 0, Qt::AlignLeft);
  statusLayout->addWidget(logStatus, 3, 1);

  layout->addRow(cPlayerNumber, statusWidget);

  setLayout(layout);
  connect(this, SIGNAL(toggled(bool)), this, SLOT(setSelected(bool)));

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
    setTitle(fromString(robot->name + " (." + ipPostfix + ")"));
    if(playerNumber)
    {
      cPlayerNumber->setEnabled(isSelected());
      cPlayerNumber->setVisible(true);
    }
    statusWidget->setVisible(true);
  }
  else
  {
    setTitle(fromString("Empty"));
    cPlayerNumber->setVisible(true);
    cPlayerNumber->setEnabled(false);
  }
}

RobotView::RobotView(TeamSelector* teamSelector,
                     Robot* robot,
                     unsigned short playerNumber,
                     unsigned short pos)
  : QGroupBox(teamSelector),
    teamSelector(teamSelector),
    robot(robot),
    playerNumber(playerNumber),
    pos(pos),
    cPlayerNumber(0)
{
  init();
}

RobotView::RobotView(TeamSelector* teamSelector,
                     Robot* robot)
  : QGroupBox(teamSelector),
    teamSelector(teamSelector),
    robot(robot),
    playerNumber(0),
    pos(0),
    cPlayerNumber(0)
{
  init();
}

QString RobotView::getRobotName() const
{
  if(!robot)
    return "";
  return fromString(robot->name);
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

void RobotView::setPings(ENetwork network, std::map<std::string, double>* pings)
{
  QLabel* bar = network == LAN ? pingBarLAN : pingBarWLAN;
  int value = 2000;
  if(pings)
    value = static_cast<int>((*pings)[robot->name]);

  QString sheet;
  if(value >= 2000)
#ifdef MACOS
    sheet = "QLabel { background-color : " + getAlternateBase().color().name(QColor::HexArgb) + "; border: 1px solid silver; }";
#else
    sheet = "QLabel { background-color : #e6e6e6; border: 1px solid silver; }";
#endif
  else if(value >= 500)
    sheet = "QLabel { background-color : red; border: 1px solid silver; color : black; }";
  else if(value >= 250)
    sheet = "QLabel { background-color : yellow; border: 1px solid silver; color : black; }";
  else
    sheet = "QLabel { background-color : lime; border: 1px solid silver; color : black; }";

  if(sheet != bar->styleSheet())
    bar->setStyleSheet(sheet);

  if(value < 2000)
    bar->setText(QString::number(value) + " ms");
  else
    bar->setText("n/a");
}

void RobotView::setPower(std::map<std::string, Power>* power)
{
  int value = 0;
  bool charging = false;
  if(power && (*power)[robot->name].isValid())
  {
    value = ((*power)[robot->name]).value;
    charging = ((*power)[robot->name]).batteryCharging;
  }

  powerBar->setValue(value);

#ifdef MACOS
  QString sheet = "QProgressBar::chunk { background-color: lime; } QProgressBar { background-color : "
                  + getAlternateBase().color().name(QColor::HexArgb) + "; border: 1px solid silver; }";
#else
  QString sheet = "QProgressBar::chunk { background-color: lime; } QProgressBar { background-color : "
                  "#e6e6e6; border: 1px solid silver; }";
#endif
  if(power && (*power)[robot->name].isValid())
  {
    if(charging)
      sheet = "QProgressBar::chunk { background-color: lime; } QProgressBar { color: black; }";
    else
      sheet = "QProgressBar::chunk { background-color: red; } QProgressBar { color: black; }";
  }
  if(sheet != powerBar->styleSheet())
    powerBar->setStyleSheet(sheet);
}

void RobotView::setLogs(std::map<std::string, int>* logs)
{
  int numOfLogs = (*logs)[robot->name];
  if(numOfLogs != 0)
    logStatus->setText(QString("<font color='red'><b>") + QString::number(numOfLogs) + QString("<b/></font>"));
  else
    logStatus->setText("--");
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
  data->setText(fromString(robot->name));
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
  Robot* r = Session::getInstance().robotsByName[toString(robotName)];
  RobotView* source = dynamic_cast<RobotView*>(e->source());
  if(source->playerNumber)
  {
    bool selected = source->isSelected();
    if(source->robot)
      source->setSelected(false);
    if(robot)
      source->setRobot(robot);
    else
      source->setRobot(0);
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
