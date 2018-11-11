/**
 * @file Controller/Views/Evaluation/StatisticRobotView.cpp
 *
 * Implementation of class StatisticRobotView
 *
 * @author <a href="mailto:jan_fie@uni-bremen.de">Jan Fiedler</a>
 */

#include "StatisticRobotView.h"
#include "Controller/RoboCupCtrl.h"
#include "Controller/Visualization/HeaderedWidget.h"

#include <algorithm>

class StatisticRobotHeaderedWidget : public HeaderedWidget, public SimRobot::Widget
{
private:
  StatisticRobotWidget* statisticRobotWidget;

public:
  StatisticRobotHeaderedWidget(StatisticRobotView& statisticRobotView);

private:
  QWidget* getWidget() override { return this; }
  void update() override { statisticRobotWidget->update(); }
};

StatisticRobotWidget::StatisticRobotWidget(StatisticRobotView& statisticRobotView, QHeaderView* headerView, QWidget* parent) :
  QWidget(parent), statisticRobotView(statisticRobotView), headerView(headerView), noPen(Qt::NoPen)
{
  setFocusPolicy(Qt::StrongFocus);
  setBackgroundRole(QPalette::Base);
  const QFontMetrics& fontMetrics(QApplication::fontMetrics());
  lineSpacing = fontMetrics.lineSpacing() + 2;
  textOffset = fontMetrics.descent() + 1;

  font = QApplication::font();

  const QPalette& pal(QApplication::palette());
  altBrush = pal.alternateBase();
  fontPen.setColor(pal.text().color());

  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(statisticRobotView.fullName);
  headerView->restoreState(settings.value("HeaderState").toByteArray());
  settings.endGroup();
}

StatisticRobotWidget::~StatisticRobotWidget()
{
  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(statisticRobotView.fullName);
  settings.setValue("HeaderState", headerView->saveState());
  settings.endGroup();
}

void StatisticRobotWidget::update()
{
  if(statisticRobotView.timeStamp == lastUpdateTimeStamp)
    return;
  else
    lastUpdateTimeStamp = statisticRobotView.timeStamp;

  QWidget::update();
}

void StatisticRobotWidget::forceUpdate()
{
  QWidget::update();
}

void StatisticRobotWidget::paintEvent(QPaintEvent* event)
{
  painter.begin(this);
  painter.setFont(font);
  painter.setBrush(altBrush);
  painter.setPen(fontPen);
  fillBackground = false;

  paintRect = painter.window();
  paintRectField = QRect(headerView->sectionViewportPosition(0) + textOffset, 0, headerView->sectionSize(0) + headerView->sectionSize(1) - textOffset * 2, lineSpacing);
  paintRectField0 = QRect(headerView->sectionViewportPosition(0) + textOffset, 0, headerView->sectionSize(0) - textOffset * 2, lineSpacing);
  paintRectField1 = QRect(headerView->sectionViewportPosition(1) + textOffset, 0, headerView->sectionSize(1) - textOffset * 2, lineSpacing);
  {
    const Statistic::StatisticRobot& robot = statisticRobotView.robots[statisticRobotView.robotIndex];
    print("Robot" + (robot.logCount > 1 ? '(' + QString::number(robot.logCount) + ')' : ""), robot.name.c_str());
    print("Configuration", robot.scenario.empty() ? "" : TypeRegistry::getEnumName(robot.configuration));
    print("Location", robot.location.c_str());
    print("Scenario", robot.scenario.c_str());
    print("Number", QString::number(robot.number));
    print("Number of Frames", QString::number(robot.frames));
    newSection();
    print("Pickups", QString::number(robot.pickedUps));
    print("Penalizes", QString::number(robot.penaltys));
    newSection();
    print("Number of Fallen", QString::number(robot.fallen));
    printList(robot.fallenFrames);
    print("Failed getUps", QString::number(robot.failedGetUps));
    printList(robot.failedGetUpFrames);
    print("walked distance (in m)", QString::number(robot.walkedDistance));
    print("Bad localisation", QString::number(robot.badLocalisations));
    newSection();
    printLine("Joints of interrest:");
    printJoints();
    newSection();
    printLine("Number of:");
    print("WalkRequests", QString::number(robot.walkRequests));
    print("StandRequests", QString::number(robot.standRequests));
    print("SpecialActionRequests", QString::number(robot.specialActionRequests));
    print("GetUpRequests", QString::number(robot.getUpRequests));
    print("KickRequests", QString::number(robot.kickRequests));
    print("BallKicked", QString::number(robot.ballKicked));
    print("In-walk kicks", QString::number(robot.inWalkKicks));
    print("Seen balls", QString::number(robot.seenBalls));
    print("Guessed balls", QString::number(robot.guessedBalls));
    newSection();
    printLine("Communication:");
    print("Received messages", QString::number(robot.receivedMessages));
    for(size_t i = 0; i < robot.receivedMessagesRobots.size(); i++)
    {
      print(QString::fromStdString(getNamebyNumber(statisticRobotView.robots, static_cast<int>(i) + 1)), QString::number(robot.receivedMessagesRobots[i]));
    }
  }
  painter.end();
  setMinimumHeight(paintRectField1.top());
}

void StatisticRobotWidget::printJoints()
{
  const Statistic::StatisticRobot& robot = statisticRobotView.robots[statisticRobotView.robotIndex];

  FOREACH_ENUM(Joints::Joint, joint)
  {
    if(robot.maxJointTemperatures[joint] != 0)
      print(TypeRegistry::getEnumName(joint), QString::number(robot.maxJointTemperatures[joint]));
  }
}

void StatisticRobotWidget::print(const QString& name, const QString& value)
{
  if(fillBackground)
  {
    painter.setPen(noPen);
    painter.drawRect(paintRect.left(), paintRectField1.top(), paintRect.width(), paintRectField1.height());
    painter.setPen(fontPen);
  }
  painter.drawText(paintRectField0, Qt::TextSingleLine | Qt::AlignVCenter, name);
  painter.drawText(paintRectField1, Qt::TextSingleLine | Qt::AlignVCenter | Qt::AlignRight, value);
  paintRectField.moveTop(paintRectField.top() + lineSpacing);
  paintRectField0.moveTop(paintRectField0.top() + lineSpacing);
  paintRectField1.moveTop(paintRectField1.top() + lineSpacing);

  fillBackground = !fillBackground;
}

void StatisticRobotWidget::printLine(const QString& value, const bool changeColor)
{
  if(fillBackground)
  {
    painter.setPen(noPen);
    painter.drawRect(paintRect.left(), paintRectField1.top(), paintRect.width(), paintRectField1.height());
    painter.setPen(fontPen);
  }
  painter.drawText(paintRectField, Qt::TextSingleLine | Qt::AlignVCenter, value);
  paintRectField.moveTop(paintRectField.top() + lineSpacing);
  paintRectField0.moveTop(paintRectField0.top() + lineSpacing);
  paintRectField1.moveTop(paintRectField1.top() + lineSpacing);

  if(changeColor)
    fillBackground = !fillBackground;
}

void StatisticRobotWidget::printList(const std::vector<unsigned>& list, const size_t lineElements)
{
  QString string;
  for(size_t i = 0; i < list.size(); i++)
  {
    string.append(QString::number(list[i]));
    if(i + 1 != list.size())
      string.append(", ");
    if(i != 0 && i % lineElements == 0)
    {
      printLine(string, (i != list.size() ? true : false));
      string.clear();
    }
  }
  if(!string.isEmpty())
    printLine(string);
}

void StatisticRobotWidget::newSection()
{
  painter.drawLine(paintRect.left(), paintRectField1.top(), paintRect.width(), paintRectField1.top());
  paintRectField.moveTop(paintRectField.top() + 1);
  paintRectField0.moveTop(paintRectField0.top() + 1);
  paintRectField1.moveTop(paintRectField1.top() + 1);
}

StatisticRobotHeaderedWidget::StatisticRobotHeaderedWidget(StatisticRobotView& statisticRobotView)
{
  QStringList headerLabels;
  headerLabels << "Property" << "Value";
  setHeaderLabels(headerLabels, "ll");
  QHeaderView* headerView = getHeaderView();
  headerView->setMinimumSectionSize(70);
  headerView->resizeSection(0, 130);
  headerView->resizeSection(1, 70);
  statisticRobotWidget = new StatisticRobotWidget(statisticRobotView, headerView, this);
  setWidget(statisticRobotWidget);
  connect(getHeaderView(), SIGNAL(sectionResized(int, int, int)), statisticRobotWidget, SLOT(forceUpdate()));
}

StatisticRobotView::StatisticRobotView(const QString& fullName, const std::vector<Statistic::StatisticRobot>& robots, const size_t robotIndex, const unsigned& timeStamp):
  fullName(fullName), icon(":/Icons/tag_green.png"), robots(robots), robotIndex(robotIndex), timeStamp(timeStamp)
{}

SimRobot::Widget* StatisticRobotView::createWidget()
{
  return new StatisticRobotHeaderedWidget(*this);
}

std::string StatisticRobotWidget::getNamebyNumber(const std::vector<Statistic::StatisticRobot>& robots, int number)
{
  for(const Statistic::StatisticRobot& robot : robots)
  {
    if(robot.number == number)
      return robot.name;
  }
  return std::to_string(number);
}

