/**
* @file Controller/Views/JointView.cpp
* Implementation of class JointView
* @author Colin Graf
*/

#include <QHeaderView>
#include <QApplication>
#include <QPainter>
#include <QFontMetrics>
#include <QSettings>

#include "JointView.h"
#include "Platform/Thread.h"
#include "Controller/RobotConsole.h"
#include "Controller/RoboCupCtrl.h"
#include "Controller/Visualization/HeaderedWidget.h"

class JointWidget : public QWidget
{
public:
  JointWidget(JointView& jointView, QHeaderView* headerView, QWidget* parent) : QWidget(parent),
    jointView(jointView), headerView(headerView), noPen(Qt::NoPen)
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
    settings.beginGroup(jointView.fullName);
    headerView->restoreState(settings.value("HeaderState").toByteArray());
    settings.endGroup();
  }

  virtual ~JointWidget()
  {
    QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
    settings.beginGroup(jointView.fullName);
    settings.setValue("HeaderState", headerView->saveState());
    settings.endGroup();
  }

  void update()
  {
    {
      SYNC_WITH(jointView.console);
      if(jointView.jointData.timeStamp == lastUpdateTimeStamp)
        return;
      lastUpdateTimeStamp = jointView.jointData.timeStamp;
    }

    QWidget::update();
  }

  void paintEvent(QPaintEvent* event)
  {
    painter.begin(this);
    painter.setFont(font);
    painter.setBrush(altBrush);
    painter.setPen(fontPen);
    fillBackground = false;

    paintRect = painter.window();
    paintRectField0 = QRect(headerView->sectionViewportPosition(0) + textOffset, 0, headerView->sectionSize(0) - textOffset * 2, lineSpacing);
    paintRectField1 = QRect(headerView->sectionViewportPosition(1) + textOffset, 0, headerView->sectionSize(1) - textOffset * 2, lineSpacing);
    paintRectField2 = QRect(headerView->sectionViewportPosition(2) + textOffset, 0, headerView->sectionSize(2) - textOffset * 2, lineSpacing);
    paintRectField3 = QRect(headerView->sectionViewportPosition(3) + textOffset, 0, headerView->sectionSize(3) - textOffset * 2, lineSpacing);
    paintRectField4 = QRect(headerView->sectionViewportPosition(4) + textOffset, 0, headerView->sectionSize(4) - textOffset * 2, lineSpacing);
    {
      char request[32], sensor[32], load[32], temp[32];
      SYNC_WITH(jointView.console);
      const JointData& jointData(jointView.jointData);
      const SensorData& sensorData(jointView.sensorData);
      const JointData& jointRequest(jointView.jointRequest);
      for(int i = 0; i < JointData::numOfJoints; ++i)
      {
        if(i == JointData::LShoulderPitch || i == JointData::RShoulderPitch || i == JointData::LHipYawPitch || i == JointData::RHipYawPitch)
          newSection();
        jointRequest.angles[i] == JointData::off ? (void)strcpy(request, "off") : (void)sprintf(request, "%.1f°", toDegrees(jointRequest.angles[i]));
        jointData.angles[i] == JointData::off ? (void)strcpy(sensor, "?") : (void)sprintf(sensor, "%.1f°", toDegrees(jointData.angles[i]));
        sensorData.temperatures[i] == 0 ? (void)strcpy(load, "?") : (void)sprintf(load, "%dmA", sensorData.currents[i]);
        sensorData.temperatures[i] == 0 ? (void)strcpy(temp, "?") : (void)sprintf(temp, "%d°C", sensorData.temperatures[i]);
        print(JointData::getName(JointData::Joint(i)), request, sensor, load, temp);
        newBlock();
      }
    }
    painter.end();
    setMinimumHeight(paintRectField1.top());
  }

private:
  JointView& jointView;
  unsigned int lastUpdateTimeStamp; /**< Timestamp of the last painted joint angles. */

  QHeaderView* headerView;

  QPainter painter;
  int lineSpacing;
  int textOffset;

  QFont font;
  QBrush altBrush;
  QPen fontPen;
  QPen noPen;
  bool fillBackground;

  QRect paintRect;
  QRect paintRectField0;
  QRect paintRectField1;
  QRect paintRectField2;
  QRect paintRectField3;
  QRect paintRectField4;

  void print(const char* name, const char* value1, const char* value2, const char* value3, const char* value4)
  {
    if(fillBackground)
    {
      painter.setPen(noPen);
      painter.drawRect(paintRect.left(), paintRectField1.top(), paintRect.width(), paintRectField1.height());
      painter.setPen(fontPen);
    }
    painter.drawText(paintRectField0, Qt::TextSingleLine | Qt::AlignVCenter, tr(name));
    painter.drawText(paintRectField1, Qt::TextSingleLine | Qt::AlignVCenter | Qt::AlignRight, tr(value1));
    painter.drawText(paintRectField2, Qt::TextSingleLine | Qt::AlignVCenter | Qt::AlignRight, tr(value2));
    painter.drawText(paintRectField3, Qt::TextSingleLine | Qt::AlignVCenter | Qt::AlignRight, tr(value3));
    painter.drawText(paintRectField4, Qt::TextSingleLine | Qt::AlignVCenter | Qt::AlignRight, tr(value4));
    paintRectField0.moveTop(paintRectField0.top() + lineSpacing);
    paintRectField1.moveTop(paintRectField1.top() + lineSpacing);
    paintRectField2.moveTop(paintRectField2.top() + lineSpacing);
    paintRectField3.moveTop(paintRectField3.top() + lineSpacing);
    paintRectField4.moveTop(paintRectField4.top() + lineSpacing);
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
    paintRectField2.moveTop(paintRectField2.top() + 1);
    paintRectField3.moveTop(paintRectField3.top() + 1);
    paintRectField4.moveTop(paintRectField4.top() + 1);
    fillBackground = false;
  }

  QSize sizeHint() const { return QSize(260, 400); }
};


class JointHeaderedWidget : public HeaderedWidget, public SimRobot::Widget
{
public:
  JointHeaderedWidget(JointView& sensorView, RobotConsole& console)
  {
    QStringList headerLabels;
    headerLabels << "Joint" << "Request" << "Sensor" << "Load" << "Temp";
    setHeaderLabels(headerLabels, "lrrrr");
    QHeaderView* headerView = getHeaderView();
    headerView->setMinimumSectionSize(30);
    headerView->resizeSection(0, 60);
    headerView->resizeSection(1, 50);
    headerView->resizeSection(2, 50);
    headerView->resizeSection(3, 50);
    headerView->resizeSection(4, 50);
    jointWidget = new JointWidget(sensorView, headerView, this);
    setWidget(jointWidget);
  }

private:
  JointWidget* jointWidget;
  virtual QWidget* getWidget() {return this;}
  virtual void update() {jointWidget->update();}
};



JointView::JointView(const QString& fullName, RobotConsole& robotConsole, const JointData& jointData, const SensorData& sensorData, const JointData& jointRequest) :
  fullName(fullName), icon(":/Icons/tag_green.png"), console(robotConsole), jointData(jointData), sensorData(sensorData), jointRequest(jointRequest) {}

SimRobot::Widget* JointView::createWidget()
{
  return new JointHeaderedWidget(*this, console);
}
