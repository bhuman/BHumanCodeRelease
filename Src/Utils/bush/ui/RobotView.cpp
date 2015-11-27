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

#include "Utils/bush/models/Robot.h"
#include "Utils/bush/models/Team.h"
#include "Utils/bush/ui/RobotView.h"
#include "Utils/bush/ui/TeamSelector.h"
#include "Utils/bush/ui/RobotPool.h"
#include "Utils/bush/tools/StringTools.h"
#include "Utils/bush/Session.h"

void RobotView::init()
{
  QFormLayout* layout = new QFormLayout();

  if(playerNumber)
    cPlayerNumber = new QLabel(QString("<font size=5><b>") + QString::number(playerNumber) + QString("</b></font>"));

  statusWidget = new QWidget(this);
  statusWidget->setMaximumSize(200, 75);
  this->setMaximumWidth(170);
  this->setMaximumHeight(125);
  QGridLayout* statusLayout = new QGridLayout(statusWidget);

  QLabel* pingLabelWLAN = new QLabel("<font size=2><b>Wlan</b></font>", statusWidget);
  pingBarWLAN = new QLabel(this);
  pingBarWLAN->setMaximumSize(50, 13);
  pingBarWLAN->setAlignment(Qt::AlignCenter);
  setPings(WLAN, 0);
  statusLayout->addWidget(pingLabelWLAN, 0, 0, Qt::AlignLeft);
  statusLayout->addWidget(pingBarWLAN, 0, 1);

  QLabel* pingLabelLAN = new QLabel("<font size=2><b>Lan</b></font>", statusWidget);
  pingBarLAN = new QLabel(this);
  pingBarLAN->setMaximumSize(50, 13);
  pingBarLAN->setAlignment(Qt::AlignCenter);
  setPings(LAN, 0);
  statusLayout->addWidget(pingLabelLAN, 1, 0, Qt::AlignLeft);
  statusLayout->addWidget(pingBarLAN, 1, 1);

  QLabel* powerLabel = new QLabel("<font size=2><b>Power</b></font>", statusWidget);
  powerBar = new QProgressBar(this);
  powerBar->setMaximumSize(50, 13);
  powerBar->setRange(0, 100);
  powerBar->setValue(0);
  powerBar->setAlignment(Qt::AlignCenter);
  statusLayout->addWidget(powerLabel, 2, 0, Qt::AlignLeft);
  statusLayout->addWidget(powerBar, 2, 1);

  layout->addRow(cPlayerNumber, statusWidget);

  setLayout(layout);
  connect(this, SIGNAL(toggled(bool)), this, SLOT(setSelected(bool)));

  if(robot)
  {
    Session::getInstance().registerPingListener(this);
    Session::getInstance().registerPowerListener(this, robot);
  }

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
    Session::getInstance().removePowerListener(this, this->robot);
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
    Session::getInstance().registerPowerListener(this, robot);
  }
  emit robotChanged();
}

void RobotView::setPings(ENetwork network, std::map<std::string, double>* pings)
{
  QLabel* bar = network == LAN ? pingBarLAN : pingBarWLAN;
  int value = 2000;
  if(pings)
    value = static_cast<int>((*pings)[robot->name]);

  if(value >= 2000)
    bar->setStyleSheet("QLabel { background-color : #e6e6e6; border: 1px solid silver; }");
  else if(value >= 500)
    bar->setStyleSheet("QLabel { background-color : red; border: 1px solid silver; }");
  else if(value >= 250)
    bar->setStyleSheet("QLabel { background-color : yellow; border: 1px solid silver; }");
  else
    bar->setStyleSheet("QLabel { background-color : lime; border: 1px solid silver; }");

  if(value < 2000)
    bar->setText(QString::number(value) + " ms");
  else
    bar->setText("n/a");

}

void RobotView::setPower(std::map<std::string, Power>* power)
{
  int value = 0;
  if(power && (*power)[robot->name].isValid())
    value = ((*power)[robot->name]).value;
  powerBar->setValue(value);
}

void RobotView::mouseMoveEvent(QMouseEvent* me)
{
  if(!robot)
    return;
  QDrag* d = new QDrag(this);
  QPixmap pm = QPixmap::grabWidget(this, rect());
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
    cPlayerNumber->setEnabled(selected);
  }
}
