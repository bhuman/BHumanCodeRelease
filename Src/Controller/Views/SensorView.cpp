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

#include <algorithm>

#include "SensorView.h"
#include "Controller/RobotConsole.h"
#include "Controller/RoboCupCtrl.h"
#include "Controller/Visualization/HeaderedWidget.h"

class SensorWidget : public QWidget
{
public:
  SensorWidget(SensorView& sensorView, QHeaderView* headerView, QWidget* parent) : QWidget(parent),
    sensorView(sensorView), headerView(headerView), noPen(Qt::NoPen)
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

    for(int j = 0; j < 2; ++j)
      for(int i = 0; i < SensorData::numOfUsActuatorModes; ++i)
      {
        usTimeStamps[j][i] = 0;
        for(int k = 0; k < 10; ++k)
        {
          usLDistances[j][i][k] = SensorData::off;
          usRDistances[j][i][k] = SensorData::off;
        }
      }

    QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
    settings.beginGroup(sensorView.fullName);
    headerView->restoreState(settings.value("HeaderState").toByteArray());
    settings.endGroup();
  }

  virtual ~SensorWidget()
  {
    QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
    settings.beginGroup(sensorView.fullName);
    settings.setValue("HeaderState", headerView->saveState());
    settings.endGroup();
  }

  void update()
  {
    {
      SYNC_WITH(sensorView.console);
      if(sensorView.sensorData.timeStamp == lastUpdateTimeStamp)
        return;
      lastUpdateTimeStamp = sensorView.sensorData.timeStamp;
    }

    QWidget::update();
  }

  enum Type
  {
    distance,
    pressure,
    acceleration,
    batteryLevel,
    angularSpeed,
    ratio,
    unknown,
    angle2,
    usFromLeftDistance,
    usFromRightDistance,
    end
  };
  struct ViewInfo
  {
    SensorData::Sensor sensor; /**< The id of the sensor. */
    Type type;  /**< The type of measurements the sensor takes. */
    bool endOfSection; /**< Is this entry the last in a group of sensors? */
  };

  bool formatSensor(Type type, float inValue, bool filtered, char* value, SensorData::Sensor sensorType) const
  {
    if(sensorType == SensorData::usL || sensorType == SensorData::usR)
    {
      const float* inValue = 0;
      const int f = filtered ? 1 : 0;
      if(sensorType == SensorData::usL)
      {
        if(type == usFromLeftDistance)
        {
          if(usTimeStamps[f][SensorData::leftToLeft] > usTimeStamps[f][SensorData::bothToSame])
            inValue = usRDistances[f][SensorData::leftToLeft]; // note: stored in right!
          else
            inValue = usLDistances[f][SensorData::bothToSame];
        }
        else if(usTimeStamps[f][SensorData::rightToLeft] > usTimeStamps[f][SensorData::bothToOther])
          inValue = usRDistances[f][SensorData::rightToLeft]; // note: stored in right!
        else
          inValue = usLDistances[f][SensorData::bothToOther];
      }
      else if(type == usFromLeftDistance)
      {
        if(usTimeStamps[f][SensorData::leftToRight] > usTimeStamps[f][SensorData::bothToOther])
          inValue = usRDistances[f][SensorData::leftToRight];
        else
          inValue = usRDistances[f][SensorData::bothToOther];
      }
      else if(usTimeStamps[f][SensorData::rightToRight] > usTimeStamps[f][SensorData::bothToSame])
        inValue = usRDistances[f][SensorData::rightToRight];
      else
        inValue = usRDistances[f][SensorData::bothToSame];

      if(*inValue == SensorData::off)
      {
        strcpy(value, "?");
        return true;
      }
      char* p = value;
      for(int i = 0; i < 10 && (!i || inValue[i] != 2550.f); ++i)
      {
        sprintf(p, "%.0f, ", inValue[i]);
        p += strlen(p);
      }
      p[-2] = p[-1] = 'm';
      return true;
    }

    if(inValue == SensorData::off)
    {
      strcpy(value, "?");
      return true;
    }
    switch(type)
    {
    case SensorWidget::distance:
      sprintf(value, "%.1f mm", inValue);
      break;
    case SensorWidget::pressure:
      sprintf(value, "%.1f g", inValue * 1000);
      break;
    case SensorWidget::acceleration:
      sprintf(value, "%.2f m/s²", inValue);
      break;
    case SensorWidget::batteryLevel:
      sprintf(value, "%.1f V", inValue);
      break;
    case SensorWidget::angularSpeed:
      sprintf(value, "%.1f °/s", toDegrees(inValue));
      break;
    case SensorWidget::ratio:
      sprintf(value, "%.1f %%", inValue * 100);
      break;
    case SensorWidget::angle2:
      sprintf(value, "%.1f °", toDegrees(inValue));
      break;
    case SensorWidget::usFromLeftDistance:
    case SensorWidget::usFromRightDistance:
      sprintf(value, "%.0fmm", inValue);
      break;
    default:
      return false;
    }
    return true;
  }

  const char* getSensorName(const ViewInfo& vi) const
  {
    if(vi.sensor == SensorData::usL)
    {
      if(vi.type == usFromLeftDistance)
        return "usLeftToLeft";
      else
        return "usRightToLeft";
    }
    else if(vi.sensor == SensorData::usR)
    {
      if(vi.type == usFromLeftDistance)
        return "usLeftToRight";
      else
        return "usRightToRight";
    }
    else
      return SensorData::getName(vi.sensor);
  }

  void paintEvent(QPaintEvent* event)
  {
    static const ViewInfo viewInfoNao[] =
    {
      {SensorData::gyroX,    angularSpeed, false},
      {SensorData::gyroY,    angularSpeed, true},

      {SensorData::accX,    acceleration, false},
      {SensorData::accY,    acceleration, false},
      {SensorData::accZ,    acceleration, true},

      {SensorData::batteryLevel, ratio, true},

      {SensorData::fsrLFL,  pressure, false},
      {SensorData::fsrLFR,  pressure, false},
      {SensorData::fsrLBL,  pressure, false},
      {SensorData::fsrLBR,  pressure, false},
      {SensorData::fsrRFL,  pressure, false},
      {SensorData::fsrRFR,  pressure, false},
      {SensorData::fsrRBL,  pressure, false},
      {SensorData::fsrRBR,  pressure, true},

      {SensorData::usL,  usFromLeftDistance, false},
      {SensorData::usL,  usFromRightDistance, false},
      {SensorData::usR,  usFromLeftDistance, false},
      {SensorData::usR,  usFromRightDistance, true},

      {SensorData::angleX,  angle2, false},
      {SensorData::angleY,  angle2, false},
      {SensorData::numOfSensors, end, true}
    };

    painter.begin(this);
    painter.setFont(font);
    painter.setBrush(altBrush);
    painter.setPen(fontPen);
    fillBackground = false;

    paintRect = painter.window();
    paintRectField0 = QRect(headerView->sectionViewportPosition(0) + textOffset, 0, headerView->sectionSize(0) - textOffset * 2, lineSpacing);
    paintRectField1 = QRect(headerView->sectionViewportPosition(1) + textOffset, 0, headerView->sectionSize(1) - textOffset * 2, lineSpacing);
    paintRectField2 = QRect(headerView->sectionViewportPosition(2) + textOffset, 0, headerView->sectionSize(2) - textOffset * 2, lineSpacing);
    {
      char value[200];
      char filtered[200];
      SYNC_WITH(sensorView.console);
      const SensorData& sensorData(sensorView.sensorData);
      const SensorData& filteredSensorData(sensorView.filteredSensorData);
      int mode = std::min<int>(sensorData.usActuatorMode, SensorData::numOfUsActuatorModes - 1);

      if(sensorData.data[SensorData::usL] != SensorData::off)
        for(int i = 0; i < 10; ++i)
          usLDistances[0][mode][i] = sensorData.data[SensorData::usL + i];
      if(sensorData.data[SensorData::usR] != SensorData::off)
        for(int i = 0; i < 10; ++i)
          usRDistances[0][mode][i] = sensorData.data[SensorData::usR + i];
      if(sensorData.data[SensorData::usL] != SensorData::off || sensorData.data[SensorData::usR] != SensorData::off)
        usTimeStamps[0][mode] = sensorData.usTimeStamp;

      int filteredMode = std::min<int>(filteredSensorData.usActuatorMode, SensorData::numOfUsActuatorModes - 1);
      if(filteredSensorData.data[SensorData::usL] != SensorData::off)
        for(int i = 0; i < 10; ++i)
          usLDistances[1][filteredMode][i] = filteredSensorData.data[SensorData::usL + i];
      if(filteredSensorData.data[SensorData::usR] != SensorData::off)
        for(int i = 0; i < 10; ++i)
          usRDistances[1][filteredMode][i] = filteredSensorData.data[SensorData::usR + i];
      if(filteredSensorData.data[SensorData::usL] != SensorData::off || filteredSensorData.data[SensorData::usR] != SensorData::off)
        usTimeStamps[1][filteredMode] = filteredSensorData.usTimeStamp;

      for(int i = 0;; ++i)
      {
        const ViewInfo& vi = viewInfoNao[i];
        if(vi.type == SensorWidget::end)
          break;

        formatSensor(vi.type, sensorData.data[vi.sensor], false, value, vi.sensor);
        formatSensor(vi.type, filteredSensorData.data[vi.sensor], true, filtered, vi.sensor);

        print(getSensorName(vi), value, filtered);
        newBlock();
        if(vi.endOfSection)
          newSection();
      }
    }
    painter.end();
    setMinimumHeight(paintRectField1.top());
  }

private:
  SensorView& sensorView;
  unsigned int lastUpdateTimeStamp; /**< Timestamp of the last painted sensor data. */
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

  float usLDistances[2][SensorData::numOfUsActuatorModes][10];
  float usRDistances[2][SensorData::numOfUsActuatorModes][10];
  unsigned usTimeStamps[2][SensorData::numOfUsActuatorModes];

  void print(const char* name, const char* value1, const char* value2)
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
    paintRectField0.moveTop(paintRectField0.top() + lineSpacing);
    paintRectField1.moveTop(paintRectField1.top() + lineSpacing);
    paintRectField2.moveTop(paintRectField2.top() + lineSpacing);
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
    fillBackground = false;
  }

  QSize sizeHint() const { return QSize(160, 350); }
};

class SensorHeaderedWidget : public HeaderedWidget, public SimRobot::Widget
{
public:
  SensorHeaderedWidget(SensorView& sensorView, RobotConsole& console)
  {
    QStringList headerLabels;
    headerLabels << "Sensor" << "Value" << "Filtered";
    setHeaderLabels(headerLabels, "lrr");
    QHeaderView* headerView = getHeaderView();
    headerView->setMinimumSectionSize(30);
    headerView->resizeSection(0, 60);
    headerView->resizeSection(1, 50);
    headerView->resizeSection(2, 50);
    sensorWidget = new SensorWidget(sensorView, headerView, this);
    setWidget(sensorWidget);
  }

private:
  SensorWidget* sensorWidget;
  virtual QWidget* getWidget() {return this;}
  virtual void update() {sensorWidget->update();}
};

SensorView::SensorView(const QString& fullName, RobotConsole& console, const SensorData& sensorData, const SensorData& filteredSensorData) :
  fullName(fullName), icon(":/Icons/tag_green.png"), console(console), sensorData(sensorData), filteredSensorData(filteredSensorData) {};

SimRobot::Widget* SensorView::createWidget()
{
  return new SensorHeaderedWidget(*this, console);
}
