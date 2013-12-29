/**
* @file Controller/Views/TeamComm3DView.cpp
* Implementation of class TeamComm3DView
* @author Colin Graf
*/

#include "TeamComm3DView.h"
#include "Controller/TeamComm3DCtrl.h"
#include "Controller/Visualization/HeaderedWidget.h"
#include <algorithm>

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wconversion"
#endif
#include <QHeaderView>
#include <QApplication>
#include <QPainter>
#include <QKeyEvent>
#include <QSettings>
#ifdef __clang__
#pragma clang diagnostic pop
#endif

class TeamComm3DWidget : public QWidget
{
public:
  TeamComm3DWidget(TeamComm3DView& view, QHeaderView* headerView, QWidget* parent) : QWidget(parent),
    view(view), headerView(headerView), baseBrush(QPalette().base()), noPen(Qt::NoPen)
  {
    setFocusPolicy(Qt::StrongFocus);
    setBackgroundRole(QPalette::Base);

    font = QApplication::font();
    boldFont = font;
    boldFont.setBold(true);
    setFontPointSize(getFontPointSize());

    const QPalette& pal(QApplication::palette());
    altBrush = pal.alternateBase();
    fontPen.setColor(pal.text().color());

    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    QSettings& settings = TeamComm3DCtrl::application->getLayoutSettings();
    settings.beginGroup(view.fullName);
    headerView->restoreState(settings.value("HeaderState").toByteArray());
    settings.endGroup();
  }

  virtual ~TeamComm3DWidget()
  {
    QSettings& settings = TeamComm3DCtrl::application->getLayoutSettings();
    settings.beginGroup(view.fullName);
    settings.setValue("HeaderState", headerView->saveState());
    settings.endGroup();
  }

  int getFontPointSize()
  {
    return font.pointSize();
  }

  void setFontPointSize(int size)
  {
    font.setPointSize(size);
    boldFont.setPointSize(size);
    QFontMetrics me(font);
    lineSpacing = me.lineSpacing() + 2;
    textOffset = me.descent() + 1;
  }

  void paintEvent(QPaintEvent* event)
  {
    painter.begin(this);
    painter.setFont(font);
    painter.setBrush(altBrush);
    painter.setPen(fontPen);
    fillBackground = false;

    char formatedVar[257];
    char formatedValue[257];

    paintRect = painter.window();
    defaultLeft = headerView->sectionViewportPosition(0) + textOffset;
    paintRectField0 = QRect(defaultLeft, 0, headerView->sectionSize(0) - textOffset * 2, lineSpacing);
    paintRectField1 = QRect(headerView->sectionViewportPosition(1) + textOffset, 0, headerView->sectionSize(1) - textOffset * 2, lineSpacing);

    TeamComm3DCtrl::TeamListener& teamListener = TeamComm3DCtrl::controller->teamListener[view.listenerIndex];
    unsigned int now = TeamComm3DCtrl::controller->now;
    float totalTraffic = 0;
    for(std::map<unsigned int, TeamComm3DCtrl::RobotData>::iterator iter = teamListener.robotData.begin(), end = teamListener.robotData.end(); iter != end; ++iter)
    {
      TeamComm3DCtrl::RobotData& robotData = iter->second;
      unsigned int ipAddress = iter->first;
      bool bold = robotData.puppetData && robotData.puppetData->selected;

      indent(0);
      if(robotData.robotNumber == 0)
        sprintf(formatedVar,  "Listener");
      else
        sprintf(formatedVar, "Robot %d", robotData.robotNumber);
      if(!robotData.robotHealth.robotName.empty())
        sprintf(formatedValue, "%u.%u.%u.%u (%s)", (ipAddress >> 24) & 0xff, (ipAddress >> 16) & 0xff, (ipAddress >> 8) & 0xff, ipAddress & 0xff, robotData.robotHealth.robotName.c_str());
      else
        sprintf(formatedValue, "%u.%u.%u.%u", (ipAddress >> 24) & 0xff, (ipAddress >> 16) & 0xff, (ipAddress >> 8) & 0xff, ipAddress & 0xff);
      print(formatedVar, formatedValue, 1.f, bold);
      indent(1);
      sprintf(formatedValue, "%u ms, %u ms", robotData.ping, now - robotData.timeStamp);
      float expectedPacketDelay = robotData.robotNumber != 0 ? 100.f : 2000.f;
      print("Ping, Packet Last Received", formatedValue, std::min(computeRegularity(robotData.ping, 100.f, 1000.f), computeRegularity(now - robotData.timeStamp, expectedPacketDelay + 50.f, expectedPacketDelay + 100.f)));
      //sprintf(formatedValue, "%d ms", int(robotData.lastPacketLatency));
      //print("Packet Latency", formatedValue);
      unsigned int oldestPacketTimeStamp = robotData.packetTimeStamps.getEntry(robotData.packetTimeStamps.getNumberOfEntries() - 1);
      float traffic = now != oldestPacketTimeStamp ? float(robotData.packetSizes.getSum() * 1000) / float(now - oldestPacketTimeStamp) : 0;
      totalTraffic += traffic;
      float averagePackageSize = float(robotData.packetSizes.getSum()) / float(robotData.packetSizes.getNumberOfEntries()) - 28.f;
      sprintf(formatedValue, "%.0f bytes/s, 28 + %.1f bytes", traffic, averagePackageSize);
      print("Traffic, Average Packet Size", formatedValue, std::min(computeRegularity(traffic, 5000.f, 10000.f), computeRegularity(averagePackageSize, 500.f, 1000.f)));
      //sprintf(formatedValue, "", averagePackageSize);
      //print("Average Packet Size", formatedValue, computeRegularity(averagePackageSize, 500.f, 1000.f));
      //sprintf(formatedValue, "%u ms", now - robotData.timeStamp);
      //float expectedPacketDelay = robotData.robotNumber != 0 ? 100.f : 2000.f;
      //print("Packet Last Received", formatedValue, computeRegularity(now - robotData.timeStamp, expectedPacketDelay + 50.f, expectedPacketDelay + 100.f));
      newBlock();

      if(robotData.robotNumber != 0)
      {
        if(!robotData.robotHealth.robotName.empty())
        {
          //print("Name", robotData.robotHealth.robotName.c_str(), 1.f);
          //newBlock();

          float loadAverage1 = float(robotData.robotHealth.load[0]) / 10.f;
          sprintf(formatedValue, "%.1f %.1f %.1f, %hhu%%", loadAverage1, float(robotData.robotHealth.load[1]) / 10.f, float(robotData.robotHealth.load[2]) / 10.f, robotData.robotHealth.memoryUsage);
          print("Load Average, Memory Usage", formatedValue, std::min(computeRegularity(loadAverage1, 1.f, 1.1f), computeRegularity(robotData.robotHealth.memoryUsage, 50.f, 90.f)));
          newBlock();

          sprintf(formatedValue, "%.1f fps, %.1f fps", robotData.robotHealth.cognitionFrameRate, robotData.robotHealth.motionFrameRate);
          print("Cognition, Motion Rate", formatedValue, std::min(computeRegularity(robotData.robotHealth.cognitionFrameRate, 29.f, 28.f), computeRegularity(robotData.robotHealth.motionFrameRate, 99.f, 98.f)));
          newBlock();

          sprintf(formatedValue, "%hhu%%, %hhu°C", robotData.robotHealth.batteryLevel, robotData.robotHealth.maxJointTemperature);
          print("Battery Level, Max Temperature", formatedValue, std::min(computeRegularity(robotData.robotHealth.batteryLevel, 30.f, 10.f),  computeRegularity(robotData.robotHealth.maxJointTemperature, 60.f, 80.f)));
          newBlock();


          if(robotData.robotHealthTimeStamps.getNumberOfEntries() > 0)
          {
            unsigned int oldestRobotHealthTimeStamp = robotData.robotHealthTimeStamps.getEntry(robotData.robotHealthTimeStamps.getNumberOfEntries() - 1);
            unsigned int newestRobotHealthTimeStamp = robotData.robotHealthTimeStamps.getEntry(0);
            unsigned int oldestBallPerceptCount = robotData.ballPercepts.getEntry(robotData.ballPercepts.getNumberOfEntries() - 1);
            unsigned int oldestLinePerceptCount = robotData.linePercepts.getEntry(robotData.linePercepts.getNumberOfEntries() - 1);
            unsigned int oldestGoalPerceptCount = robotData.goalPercepts.getEntry(robotData.goalPercepts.getNumberOfEntries() - 1);
            sprintf(formatedValue, "%.1f, %.1f, %.1f",
                    float(robotData.robotHealth.ballPercepts - oldestBallPerceptCount) / (float(newestRobotHealthTimeStamp - oldestRobotHealthTimeStamp) * 0.001f / 60.f),
                    float(robotData.robotHealth.goalPercepts - oldestGoalPerceptCount) / (float(newestRobotHealthTimeStamp - oldestRobotHealthTimeStamp) * 0.001f / 60.f),
                    float(robotData.robotHealth.linePercepts - oldestLinePerceptCount) / (float(newestRobotHealthTimeStamp - oldestRobotHealthTimeStamp) * 0.001f / 60.f));
            print("Ball-, Goal-, LinePercepts / min", formatedValue, 1.f);
            newBlock();
          }
        }

        int ballLastSeen = robotData.ballModel.timeWhenLastSeen ? int(now - robotData.ballModel.timeWhenLastSeen) : 0;
        if(ballLastSeen)
          sprintf(formatedValue,  "%d ms", ballLastSeen);
        else
          sprintf(formatedValue, "never");
        print("Ball Last Seen", formatedValue, computeRegularity(ballLastSeen, 10000.f, 12000.f));
        //sprintf(formatedValue, robotData.goalPercept.timeWhenOwnGoalLastSeen || robotData.goalPercept.timeWhenOppGoalLastSeen ? "%d ms" : "never", std::min(int(now - robotData.goalPercept.timeWhenOwnGoalLastSeen), int(now - robotData.goalPercept.timeWhenOppGoalLastSeen)));
        //print("Goal Last Seen", formatedValue);
        newBlock();

        if(robotData.robotPose.deviation == RobotPose::unknownDeviation)
          sprintf(formatedValue, "unknown");
        else
          sprintf(formatedValue,  "%.1f mm", robotData.robotPose.deviation);
        print("Pose deviation", formatedValue, computeRegularity(robotData.robotPose.deviation, 100.f, RobotPose::unknownDeviation));
        newBlock();

        sprintf(formatedValue, "%s%s", BehaviorStatus::getName(robotData.behaviorStatus.role), robotData.isPenalized ? " (Penalized)" : "");
        print("Behavior Role", formatedValue, 1.f);
        newBlock();
      }
      newSection();
    }
    sprintf(formatedValue, "%.0f bytes/s", totalTraffic);
    print("Total Traffic", formatedValue, computeRegularity(totalTraffic, 150000, 200000));

    painter.end();
    setMinimumHeight(paintRectField1.top());
  }

private:
  TeamComm3DView& view;
  QHeaderView* headerView;
  QPainter painter;
  int lineSpacing;
  int textOffset;
  int defaultLeft;

  QFont font;
  QFont boldFont;
  QBrush baseBrush;
  QBrush altBrush;
  QPen fontPen;
  QPen noPen;
  bool fillBackground;

  QRect paintRect;
  QRect paintRectField0;
  QRect paintRectField1;

  void indent(int count)
  {
    paintRectField0.setLeft(defaultLeft + 10 * count);
  }

  float computeRegularity(float value, float good, float bad)
  {
    if(good < bad)
      return 1.f - (value - good) / (bad - good);
    else
      return (value - bad) / (good - bad);
  }

  void print(const char* name, const char* value, float regularity, bool bold = false, bool rightAlign = false)
  {
    painter.setPen(noPen);
    if(fillBackground)
      painter.drawRect(paintRect.left(), paintRectField1.top(), paintRect.width(), paintRectField1.height());
    QLinearGradient bgColorGradient(paintRectField1.left(), 0.f, paintRectField1.right(), 0.f);
    bgColorGradient.setColorAt(0.f, fillBackground ? altBrush.color() : baseBrush.color());
    bgColorGradient.setColorAt(1.f, QColor(std::max(std::min(int(255.f * (1.f - regularity)), 255), 0), std::max(std::min(int(255.f * regularity), 255), 0), 0));
    painter.setBrush(bgColorGradient);
    painter.drawRect(paintRectField1);
    painter.setBrush(altBrush);
    painter.setPen(fontPen);
    if(name)
    {
      if(bold)
        painter.setFont(boldFont);
      painter.drawText(paintRectField0, ((!value /*|| rightAlign*/) ? Qt::TextDontClip : 0) | Qt::TextSingleLine | Qt::AlignVCenter, tr(name));
      if(bold)
        painter.setFont(font);
    }
    if(value)
      painter.drawText(paintRectField1, (rightAlign ? Qt::AlignRight : Qt::AlignLeft) | Qt::TextSingleLine | Qt::AlignVCenter, tr(value));
    paintRectField0.moveTop(paintRectField0.top() + lineSpacing);
    paintRectField1.moveTop(paintRectField1.top() + lineSpacing);
  }

  void newBlock()
  {
    fillBackground = fillBackground ? false : true;
  }

  void newSection()
  {
    painter.drawLine(paintRect.left(), paintRectField1.top(), paintRect.width(), paintRectField1.top());
    paintRectField0.moveTop(paintRectField0.top() + 1);
    paintRectField1.moveTop(paintRectField1.top() + 1);
    fillBackground = false;
  }

  void keyPressEvent(QKeyEvent* event)
  {
    switch(event->key())
    {
    case Qt::Key_PageUp:
    case Qt::Key_Plus:
      event->accept();
      if(getFontPointSize() < 48)
        setFontPointSize(getFontPointSize() + 1);
      QWidget::update();
      break;
    case Qt::Key_PageDown:
    case Qt::Key_Minus:
      event->accept();
      if(getFontPointSize() > 3)
        setFontPointSize(getFontPointSize() - 1);
      QWidget::update();
      break;
    default:
      QWidget::keyPressEvent(event);
      break;
    }
  }

  QSize sizeHint() const { return QSize(200, 500); }
};

class TeamComm3DHeaderedWidget : public HeaderedWidget, public SimRobot::Widget
{
public:
  TeamComm3DHeaderedWidget(TeamComm3DView& view)
  {
    QStringList headerLabels;
    headerLabels << "Name" << "Value";
    setHeaderLabels(headerLabels);
    QHeaderView* headerView = getHeaderView();
    headerView->setMinimumSectionSize(30);
    headerView->resizeSection(0, 100);
    headerView->resizeSection(1, 100);
    widget = new TeamComm3DWidget(view, headerView, this);
    setWidget(widget);
  }

private:
  TeamComm3DWidget* widget;
  virtual QWidget* getWidget() {return this;}
  virtual void update() {widget->update();}
};

TeamComm3DView::TeamComm3DView(const QString& fullName, int listenerIndex) :
  fullName(fullName), icon(":/Icons/tag_green.png"), listenerIndex(listenerIndex) {}

SimRobot::Widget* TeamComm3DView::createWidget()
{
  return new TeamComm3DHeaderedWidget(*this);
}
