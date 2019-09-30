/**
 * @file Controller/Views/JointView.cpp
 * Implementation of class JointView
 * @author Colin Graf
 */

#include "Controller/RobotConsole.h"

#include "JointView.h"
#include "Controller/RoboCupCtrl.h"
#include "Controller/Visualization/HeaderedWidget.h"
#include "Tools/Motion/SensorData.h"

class JointHeaderedWidget : public HeaderedWidget, public SimRobot::Widget
{
private:
  JointWidget* jointWidget;

public:
  JointHeaderedWidget(JointView& sensorView, RobotConsole& console);

private:
  QWidget* getWidget() override { return this; }
  void update() override { jointWidget->update(); }
};

JointWidget::JointWidget(JointView& jointView, QHeaderView* headerView, QWidget* parent) :
  QWidget(parent), jointView(jointView), headerView(headerView), noPen(Qt::NoPen)
{
  setFocusPolicy(Qt::StrongFocus);
  setBackgroundRole(QPalette::Base);
  const QFontMetrics& fontMetrics(QApplication::fontMetrics());
  lineSpacing = fontMetrics.lineSpacing() + 2;
  textOffset = fontMetrics.descent() + 1;

  font = QApplication::font();

  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(jointView.fullName);
  headerView->restoreState(settings.value("HeaderState").toByteArray());
  settings.endGroup();
}

JointWidget::~JointWidget()
{
  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(jointView.fullName);
  settings.setValue("HeaderState", headerView->saveState());
  settings.endGroup();
}

void JointWidget::update()
{
  {
    SYNC_WITH(jointView.console);
    if(jointView.jointSensorData.timestamp == lastUpdateTimestamp)
      return;
    lastUpdateTimestamp = jointView.jointSensorData.timestamp;
  }

  QWidget::update();
}

void JointWidget::forceUpdate()
{
  QWidget::update();
}

void JointWidget::paintEvent(QPaintEvent* event)
{
  painter.begin(this);
  painter.setFont(font);
  painter.setBrush(RoboCupCtrl::controller->getAlternateBackgroundColor());
  painter.setPen(QApplication::palette().text().color());
  fillBackground = false;

  paintRect = painter.window();
  paintRectField0 = QRect(headerView->sectionViewportPosition(0) + textOffset, 0, headerView->sectionSize(0) - textOffset * 2, lineSpacing);
  paintRectField1 = QRect(headerView->sectionViewportPosition(1) + textOffset, 0, headerView->sectionSize(1) - textOffset * 2, lineSpacing);
  paintRectField2 = QRect(headerView->sectionViewportPosition(2) + textOffset, 0, headerView->sectionSize(2) - textOffset * 2, lineSpacing);
  paintRectField3 = QRect(headerView->sectionViewportPosition(3) + textOffset, 0, headerView->sectionSize(3) - textOffset * 2, lineSpacing);
  paintRectField4 = QRect(headerView->sectionViewportPosition(4) + textOffset, 0, headerView->sectionSize(4) - textOffset * 2, lineSpacing);
  paintRectField5 = QRect(headerView->sectionViewportPosition(5) + textOffset, 0, headerView->sectionSize(5) - textOffset * 2, lineSpacing);
  {
    char request[32], sensor[32], load[32], temp[32], stiffness[32];
    SYNC_WITH(jointView.console);
    const JointSensorData& jointSensorData(jointView.jointSensorData);
    const JointRequest& jointRequest(jointView.jointRequest);
    for(int i = 0; i < Joints::numOfJoints; ++i)
    {
      if(i == Joints::lShoulderPitch || i == Joints::rShoulderPitch || i == Joints::lHipYawPitch || i == Joints::rHipYawPitch)
        newSection();
      if(i == Joints::lHand || i == Joints::rHand)
      {
        jointRequest.angles[i] == JointAngles::off ? static_cast<void>(strcpy(request, "off")) : static_cast<void>(sprintf(request, "%.1f %%", static_cast<float>(jointRequest.angles[i] * 100)));
        jointSensorData.angles[i] == JointAngles::off ? static_cast<void>(strcpy(sensor, "?")) : static_cast<void>(sprintf(sensor, "%.1f %%", static_cast<float>(jointSensorData.angles[i] * 100)));
      }
      else
      {
        jointRequest.angles[i] == JointAngles::off ? static_cast<void>(strcpy(request, "off")) : static_cast<void>(sprintf(request, "%.1f°", jointRequest.angles[i].toDegrees()));
        jointSensorData.angles[i] == JointAngles::off ? static_cast<void>(strcpy(sensor, "?")) : static_cast<void>(sprintf(sensor, "%.1f°", jointSensorData.angles[i].toDegrees()));
      }
      jointSensorData.currents[i] == SensorData::off ? static_cast<void>(strcpy(load, "off")) : static_cast<void>(sprintf(load, "%d mA", jointSensorData.currents[i]));
      jointSensorData.temperatures[i] == 0 ? static_cast<void>(strcpy(temp, "off")) : static_cast<void>(sprintf(temp, "%d °C", jointSensorData.temperatures[i]));
      jointRequest.stiffnessData.stiffnesses[i] == StiffnessData::useDefault ? static_cast<void>(strcpy(stiffness, "?")) : static_cast<void>(sprintf(stiffness, "%d %%", jointRequest.stiffnessData.stiffnesses[i]));
      print(TypeRegistry::getEnumName(static_cast<Joints::Joint>(i)), request, sensor, load, temp, stiffness);
    }
  }
  painter.end();
  setMinimumHeight(paintRectField1.top());
}

void JointWidget::print(const char* name, const char* value1, const char* value2, const char* value3, const char* value4, const char* value5)
{
  if(fillBackground)
  {
    painter.setPen(noPen);
    painter.drawRect(paintRect.left(), paintRectField1.top(), paintRect.width(), paintRectField1.height());
    painter.setPen(QApplication::palette().text().color());
  }
  painter.drawText(paintRectField0, Qt::TextSingleLine | Qt::AlignVCenter, tr(name));
  painter.drawText(paintRectField1, Qt::TextSingleLine | Qt::AlignVCenter | Qt::AlignRight, tr(value1));
  painter.drawText(paintRectField2, Qt::TextSingleLine | Qt::AlignVCenter | Qt::AlignRight, tr(value2));
  painter.drawText(paintRectField3, Qt::TextSingleLine | Qt::AlignVCenter | Qt::AlignRight, tr(value3));
  painter.drawText(paintRectField4, Qt::TextSingleLine | Qt::AlignVCenter | Qt::AlignRight, tr(value4));
  painter.drawText(paintRectField5, Qt::TextSingleLine | Qt::AlignVCenter | Qt::AlignRight, tr(value5));
  paintRectField0.moveTop(paintRectField0.top() + lineSpacing);
  paintRectField1.moveTop(paintRectField1.top() + lineSpacing);
  paintRectField2.moveTop(paintRectField2.top() + lineSpacing);
  paintRectField3.moveTop(paintRectField3.top() + lineSpacing);
  paintRectField4.moveTop(paintRectField4.top() + lineSpacing);
  paintRectField5.moveTop(paintRectField5.top() + lineSpacing);

  fillBackground = !fillBackground;
}

void JointWidget::newSection()
{
  painter.drawLine(paintRect.left(), paintRectField1.top(), paintRect.width(), paintRectField1.top());
  paintRectField0.moveTop(paintRectField0.top() + 1);
  paintRectField1.moveTop(paintRectField1.top() + 1);
  paintRectField2.moveTop(paintRectField2.top() + 1);
  paintRectField3.moveTop(paintRectField3.top() + 1);
  paintRectField4.moveTop(paintRectField4.top() + 1);
  paintRectField5.moveTop(paintRectField5.top() + 1);
  fillBackground = false;
}

JointHeaderedWidget::JointHeaderedWidget(JointView& sensorView, RobotConsole& console)
{
  QStringList headerLabels;
  headerLabels << "Joint" << "Request" << "Sensor" << "Load" << "Temp" << "Stiffness";
  setHeaderLabels(headerLabels, "lrrrrr");
  QHeaderView* headerView = getHeaderView();
  headerView->setMinimumSectionSize(30);
  headerView->resizeSection(0, 60);
  headerView->resizeSection(1, 50);
  headerView->resizeSection(2, 50);
  headerView->resizeSection(3, 50);
  headerView->resizeSection(4, 50);
  headerView->resizeSection(5, 50);
  jointWidget = new JointWidget(sensorView, headerView, this);
  setWidget(jointWidget);
  connect(getHeaderView(), SIGNAL(sectionResized(int, int, int)), jointWidget, SLOT(forceUpdate()));
}

JointView::JointView(const QString& fullName, RobotConsole& robotConsole, const JointSensorData& jointSensorData, const JointRequest& jointRequest) :
  fullName(fullName), icon(":/Icons/tag_green.png"), console(robotConsole), jointSensorData(jointSensorData), jointRequest(jointRequest)
{}

SimRobot::Widget* JointView::createWidget()
{
  return new JointHeaderedWidget(*this, console);
}
