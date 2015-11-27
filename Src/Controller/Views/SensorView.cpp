/**
* @file Controller/Views/SensorView.cpp
*
* Implementation of class SensorView
*
* @author of the original sensorview <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
* @author Jeff
* @author Colin Graf
*/

#include <QHeaderView>
#include <QApplication>
#include <QPainter>
#include <QFontMetrics>
#include <QSettings>

#include "SensorView.h"
#include "Controller/RobotConsole.h"
#include "Controller/RoboCupCtrl.h"
#include "Controller/Visualization/HeaderedWidget.h"
#include "Platform/BHAssert.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Infrastructure/SensorData/SystemSensorData.h"
#include "Representations/Infrastructure/SensorData/UsSensorData.h"

#include <algorithm>


class SensorWidget : public QWidget
{
private:
  enum class ValueType
  {
    acceleration, // m/s^2
    angle,        // °
    angularSpeed, // °/s^2
    current,      // A
    pressure,     // g
    ratio,        // %
    temperatur    // °C
  };

  SensorView& sensorView;
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

public:
  SensorWidget(SensorView& sensorView, QHeaderView* headerView, QWidget* parent);
  virtual ~SensorWidget();

  void update();
  void paintEvent(QPaintEvent* event);

private:
  void paintFsrSensorData();
  void paintInertialSensorData();
  void paintKeyStates();
  void paintSystemSensorData();
  QString printValue(ValueType valueType, float value) const;
  QString printCoordinate(Vector2f val) const;
  QString printButton(bool pressed) const;
  void print(const QString& name, const QString& value);
  void newSection();
  QSize sizeHint() const { return QSize(250, 400); }
};

class SensorHeaderedWidget : public HeaderedWidget, public SimRobot::Widget
{
private:
  SensorWidget* sensorWidget;

public:
  SensorHeaderedWidget(SensorView& sensorView, RobotConsole& console);

private:
  virtual QWidget* getWidget() {return this;}
  virtual void update() {sensorWidget->update();}
};

SensorView::SensorView(const QString& fullName, RobotConsole& robotConsole, const FsrSensorData& fsrSensorData,
                       const InertialSensorData& inertialSensorData, const KeyStates& keyStates,
                       const SystemSensorData& systemSensorData) :
  fullName(fullName), icon(":/Icons/tag_green.png"), console(robotConsole), fsrSensorData(fsrSensorData),
  inertialSensorData(inertialSensorData), keyStates(keyStates), systemSensorData(systemSensorData)
{}

SimRobot::Widget* SensorView::createWidget()
{
  return new SensorHeaderedWidget(*this, console);
}

SensorWidget::SensorWidget(SensorView& sensorView, QHeaderView* headerView, QWidget* parent) :
  QWidget(parent), sensorView(sensorView), headerView(headerView), noPen(Qt::NoPen)
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
  settings.beginGroup(sensorView.fullName);
  headerView->restoreState(settings.value("HeaderState").toByteArray());
  settings.endGroup();
}

SensorWidget::~SensorWidget()
{
  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(sensorView.fullName);
  settings.setValue("HeaderState", headerView->saveState());
  settings.endGroup();
}

void SensorWidget::update()
{
  // TODO The old implementation refused an update without a change in the sensor data. That may be a good idea
  /*{
    SYNC_WITH(sensorView.console);
    if(sensorView.sensorData.timeStamp == lastUpdateTimeStamp)
      return;
    else
      lastUpdateTimeStamp = sensorView.sensorData.timeStamp;
  }*/

  QWidget::update();
}

void SensorWidget::paintEvent(QPaintEvent* event)
{
  painter.begin(this);
  painter.setFont(font);
  painter.setBrush(altBrush);
  painter.setPen(fontPen);
  fillBackground = false;

  paintRect = painter.window();
  paintRectField0 = QRect(headerView->sectionViewportPosition(0) + textOffset, 0, headerView->sectionSize(0) - textOffset * 2, lineSpacing);
  paintRectField1 = QRect(headerView->sectionViewportPosition(1) + textOffset, 0, headerView->sectionSize(1) - textOffset * 2, lineSpacing);
  {
    SYNC_WITH(sensorView.console);
    paintInertialSensorData();
    newSection();
    paintSystemSensorData();
    newSection();
    paintFsrSensorData();
    newSection();
    paintKeyStates();
  }
  painter.end();
  setMinimumHeight(paintRectField1.top());
}

void SensorWidget::paintFsrSensorData()
{
  const FsrSensorData& data = sensorView.fsrSensorData;
  print("Fsr sensor data:", "");
  print(" Fsr lfl", printValue(ValueType::pressure, data.left[FsrSensorData::fl]));
  print(" Fsr lfr", printValue(ValueType::pressure, data.left[FsrSensorData::fr]));
  print(" Fsr lbl", printValue(ValueType::pressure, data.left[FsrSensorData::bl]));
  print(" Fsr lbr", printValue(ValueType::pressure, data.left[FsrSensorData::br]));
  print(" Fsr rfl", printValue(ValueType::pressure, data.right[FsrSensorData::fl]));
  print(" Fsr rfr", printValue(ValueType::pressure, data.right[FsrSensorData::fr]));
  print(" Fsr rbl", printValue(ValueType::pressure, data.right[FsrSensorData::bl]));
  print(" Fsr rbr", printValue(ValueType::pressure, data.right[FsrSensorData::br]));
  print(" Fsr total left", printValue(ValueType::pressure, data.leftTotal));
  print(" Fsr total right", printValue(ValueType::pressure, data.rightTotal));
}

void SensorWidget::paintInertialSensorData()
{
  const InertialSensorData& data = sensorView.inertialSensorData;
  print("Inertial sensor data:", "");
  print(" Gyro x", printValue(ValueType::angularSpeed, data.gyro.x()));
  print(" Gyro y", printValue(ValueType::angularSpeed, data.gyro.y()));
  print(" Gyro z", printValue(ValueType::angularSpeed, data.gyro.z()));
  print(" Acc x", printValue(ValueType::acceleration, data.acc.x()));
  print(" Acc y", printValue(ValueType::acceleration, data.acc.y()));
  print(" Acc z", printValue(ValueType::acceleration, data.acc.z()));
  print(" Angle x", printValue(ValueType::angle, data.angle.x()));
  print(" Angle y", printValue(ValueType::angle, data.angle.y()));
}

void SensorWidget::paintKeyStates()
{
  const KeyStates& data = sensorView.keyStates;
  print("Key states:", "");
  print(" Head front", printButton(data.pressed[KeyStates::headFront]));
  print(" Head middle", printButton(data.pressed[KeyStates::headMiddle]));
  print(" Head rear", printButton(data.pressed[KeyStates::headRear]));
  print(" Left hand back", printButton(data.pressed[KeyStates::lHandBack]));
  print(" Left hand left", printButton(data.pressed[KeyStates::lHandLeft]));
  print(" Left hand right", printButton(data.pressed[KeyStates::lHandRight]));
  print(" Right hand back", printButton(data.pressed[KeyStates::rHandBack]));
  print(" Right hand left", printButton(data.pressed[KeyStates::rHandLeft]));
  print(" Right hand right", printButton(data.pressed[KeyStates::rHandRight]));
  print(" Left foot left", printButton(data.pressed[KeyStates::leftFootLeft]));
  print(" Left foot right", printButton(data.pressed[KeyStates::leftFootRight]));
  print(" Right foot left", printButton(data.pressed[KeyStates::rightFootLeft]));
  print(" Right foot right", printButton(data.pressed[KeyStates::rightFootRight]));
  print(" Chest", printButton(data.pressed[KeyStates::chest]));
}

void SensorWidget::paintSystemSensorData()
{
  const SystemSensorData& data = sensorView.systemSensorData;
  print("System sensor data:", "");
  print(" Cpu temperatur", printValue(ValueType::temperatur, data.cpuTemperature));
  print(" Battery current", printValue(ValueType::current, data.batteryCurrent));
  print(" Battery level", printValue(ValueType::ratio, data.batteryLevel));
  print(" Battery temperatur", printValue(ValueType::ratio, data.batteryTemperature));
}

QString SensorWidget::printValue(ValueType valueType, float value) const
{
  if(value == SensorData::off)
    return "off";
  else
  {
    QString text;
    switch(valueType)
    {
      case ValueType::acceleration:
        text = QString::number(value, 'f', 2) + QString::fromUtf8(" m/s²");
        break;
      case ValueType::angle:
        text = QString::number(toDegrees(value), 'f', 1) + "°";
        break;
      case ValueType::angularSpeed:
        text = QString::number(toDegrees(value), 'f', 1) + " °/s";
        break;
      case ValueType::current:
        text = QString::number(value, 'f', 2) + " A";
        break;
      case ValueType::pressure:
        text = QString::number(value * 1000.f, 'f', 0) + " g";
        break;
      case ValueType::ratio:
        text = QString::number(value * 100.f, 'f', 1) + " %";
        break;
      case ValueType::temperatur:
        text = QString::number(value, 'f', 1) + " °C";
        break;
    }
    return text;
  }
}

inline QString SensorWidget::printCoordinate(Vector2f val) const
{
  return "x: " + QString::number(val.x(), 'f', 0) + " mm, y: " + QString::number(val.y(), 'f', 0) + " mm";
}

QString SensorWidget::printButton(bool pressed) const
{
  return pressed ? "on" : "off";
}

void SensorWidget::print(const QString& name, const QString& value)
{
  if(fillBackground)
  {
    painter.setPen(noPen);
    painter.drawRect(paintRect.left(), paintRectField1.top(), paintRect.width(), paintRectField1.height());
    painter.setPen(fontPen);
  }
  painter.drawText(paintRectField0, Qt::TextSingleLine | Qt::AlignVCenter, name);
  painter.drawText(paintRectField1, Qt::TextSingleLine | Qt::AlignVCenter | Qt::AlignRight, value);
  paintRectField0.moveTop(paintRectField0.top() + lineSpacing);
  paintRectField1.moveTop(paintRectField1.top() + lineSpacing);

  fillBackground = !fillBackground;
}

void SensorWidget::newSection()
{
  painter.drawLine(paintRect.left(), paintRectField1.top(), paintRect.width(), paintRectField1.top());
  paintRectField0.moveTop(paintRectField0.top() + 1);
  paintRectField1.moveTop(paintRectField1.top() + 1);
  fillBackground = false;
}

SensorHeaderedWidget::SensorHeaderedWidget(SensorView& sensorView, RobotConsole& console)
{
  QStringList headerLabels;
  headerLabels << "Sensor" << "Value";
  setHeaderLabels(headerLabels, "lr");
  QHeaderView* headerView = getHeaderView();
  headerView->setMinimumSectionSize(50);
  headerView->resizeSection(0, 80);
  headerView->resizeSection(1, 50);
  sensorWidget = new SensorWidget(sensorView, headerView, this);
  setWidget(sensorWidget);
}