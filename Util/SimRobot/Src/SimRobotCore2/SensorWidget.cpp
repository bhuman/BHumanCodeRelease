/**
* @file SensorWidget.cpp
* Implementation of class SensorWidget
*/

#include <sstream>
#include <iomanip>

#include <QMenu>
#include <QApplication>
#include <QClipboard>
#include <QMimeData>
#include <QLocale>

#include "SensorWidget.h"
#include "CoreModule.h"

#include "Simulation/Simulation.h"
#include "Tools/Math/Constants.h"

static inline float toDeg(float angleInRad)
{ return (angleInRad * 180.f / pi);}

SensorWidget::SensorWidget(SimRobotCore2::SensorPort* sensor) : pen(QColor::fromRgb(255, 0, 0)), sensor(sensor), mineData(0)
{
  setFocusPolicy(Qt::StrongFocus);

  sensorDimensions = sensor->getDimensions();
  sensorType = sensor->getSensorType();;
}

SensorWidget::~SensorWidget()
{
  if(mineData && QApplication::clipboard()->mimeData() == mineData)
    QApplication::clipboard()->clear(); // hack, prevents crash on exit
}

void SensorWidget::paintEvent(QPaintEvent *event)
{
  float minValue, maxValue;
  bool hasMinAndMax = sensor->getMinAndMax(minValue, maxValue);
  const QList<int>& dimensions = sensorDimensions;
  painter.begin(this);
  switch(sensorType)
  {
    case SimRobotCore2::SensorPort::floatSensor:
    {
      float sensorValue = sensor->getValue().floatValue;
      char str_val[32];
      sprintf(str_val, "%.03f", sensorValue);
      if(hasMinAndMax)
      {
        sensorValue -= minValue;
        sensorValue /= (maxValue-minValue);
        QBrush brush(QColor::fromRgb(static_cast<int>(sensorValue * 255), static_cast<int>(sensorValue * 255), static_cast<int>(sensorValue * 255)));
        painter.setBrush(brush);
      }
      else
      {
        QBrush brush(QColor::fromRgb(0,0,0));
        painter.setBrush(brush);
      }
      painter.drawRect(0,0, this->width(), this->height());
      painter.setPen(pen);
      painter.drawText(this->width()/2, this->height()/2, tr(str_val));
      break;
    }
    case SimRobotCore2::SensorPort::floatArraySensor:
    {
      const QStringList& descriptions = sensor->getDescriptions();
      // Accelerometer, Gyroscope, PositionSensor, ...
      if(descriptions.size() && dimensions.size() == 1)
        paintFloatArrayWithDescriptionsSensor();
      // Laser Range Finder
      else if(descriptions.size() == 0 && dimensions.size() == 1 && hasMinAndMax)
        paintFloatArrayWithLimitsAndWithoutDescriptions();
      else if(descriptions.size() == 0 && dimensions.size() == 2 && hasMinAndMax)
        paint2DFloatArrayWithLimitsAndWithoutDescriptions();
      // Other stuff
      else
        painter.drawText(0, 0, this->width(), this->height(), Qt::AlignCenter, "Not implemented yet!");
      break;
    }
    case SimRobotCore2::SensorPort::boolSensor:
    {
      paintBoolSensor();
      break;
    }
    case SimRobotCore2::SensorPort::cameraSensor:
    {
      int xsize = dimensions[0], ysize = dimensions[1];
      const unsigned char *vals = sensor->getValue().byteArray;
      //const unsigned char* end = vals + xsize * ysize * 3;
      unsigned char* buffer = new unsigned char[xsize * ysize * 4];
      unsigned char* pDest = buffer;
      for(int y = ysize - 1; y >= 0; --y)
        for(const unsigned char* pSrc = vals + xsize * 3 * y, * end = pSrc + xsize * 3; pSrc < end; pSrc += 3)
        {
          *pDest++ = pSrc[2];
          *pDest++ = pSrc[1];
          *pDest++ = pSrc[0];
          *pDest++ = 0xff;
        }
      QImage img(buffer, xsize, ysize, QImage::Format_RGB32);
      painter.drawImage(0,0, img.scaled(this->width(), this->height()));
      delete [] buffer;
      break;
    }
    case SimRobotCore2::SensorPort::noSensor:
      break; // do nothing
  }
  painter.end();
}

void SensorWidget::paintBoolSensor()
{
  const bool value = sensor->getValue().boolValue;
  QBrush brush(value ? QColor::fromRgb(255,255,255) : QColor::fromRgb(0,0,0));
  pen.setColor(value ? QColor::fromRgb(0,0,0) : QColor::fromRgb(255,255,255));
  painter.setPen(pen);
  painter.fillRect(0, 0, this->width(), this->height(), brush);
  painter.drawText(0, 0, this->width(), this->height(), Qt::AlignCenter, (value ? "true" : "false"));
}

void SensorWidget::paintFloatArrayWithDescriptionsSensor()
{
  QBrush brush(QColor::fromRgb(255, 255, 255));
  painter.setBrush(brush);
  painter.fillRect(0, 0, this->width(), this->height(), brush);
  pen.setColor(QColor::fromRgb(0, 0, 0));
  painter.setPen(pen);
  const QString& unitForDisplay = sensor->getUnit();
  SimRobotCore2::SensorPort::Data data = sensor->getValue();
  const QStringList& descriptions = sensor->getDescriptions();
  bool conversionToDegreesNeeded = unitForDisplay.indexOf(tr("°")) != -1;
  for(int i = 0; i < sensorDimensions[0]; i++)
  {
    float value = conversionToDegreesNeeded ?
      toDeg(data.floatArray[i]) : data.floatArray[i];
    std::stringstream valueStream;
    valueStream << std::fixed << std::setprecision(4) << value;
    painter.drawText(0, 20*i, this->width(), 20, Qt::AlignLeft,
      " " + descriptions[i]);
    painter.drawText(0, 20*i, this->width(), 20, Qt::AlignRight,
      tr((QString(valueStream.str().c_str()) + " " + unitForDisplay + " ").toUtf8().data()));
  }
}

void SensorWidget::paintFloatArrayWithLimitsAndWithoutDescriptions()
{
  float minValue, maxValue;
  if(!sensor->getMinAndMax(minValue, maxValue))
    maxValue = 1;
  QBrush brush(QColor::fromRgb(255,255,255));
  QBrush brush2(QColor::fromRgb(0,255,0));
  painter.fillRect(0, 0, this->width(), this->height(), brush);
  pen.setColor(QColor::fromRgb(0,0,0));
  painter.setPen(pen);
  const float* valueArray = sensor->getValue().floatArray;
  float widthPerValue = this->width() / static_cast<float>(sensorDimensions[0]);
  if(widthPerValue < 1.0)
    widthPerValue = 1.0;
  for(int i = 0; i < sensorDimensions[0]; i++)
  {
    float factor = valueArray[i] / maxValue;
    painter.fillRect(QRectF(i*widthPerValue, this->height()*(1-factor), widthPerValue, this->height()*factor), brush2);
    painter.drawRect(QRectF(i*widthPerValue, this->height()*(1-factor), widthPerValue, this->height()*factor));
  }
}

void SensorWidget::paint2DFloatArrayWithLimitsAndWithoutDescriptions()
{
  float minValue, maxValue;
  if(!sensor->getMinAndMax(minValue, maxValue))
  {
    minValue = 0;
    maxValue = 1;
  }
  double scale = (6 << 8) / (maxValue - minValue);
  int xsize = sensorDimensions[0], ysize = sensorDimensions[1];
  const float *vals = sensor->getValue().floatArray;
  unsigned char* buffer = new unsigned char[xsize * ysize * 4];
  unsigned char* pDest = buffer;
  for(int y = ysize - 1; y >= 0; --y)
    for(const float* pSrc = vals + xsize * y, * end = pSrc + xsize; pSrc < end; ++pSrc)
    {
      unsigned value = unsigned((*pSrc - minValue) * scale);
      unsigned char c = static_cast<unsigned char>(value);
      switch(value >> 8)
      {
        case 0:
          *pDest++ = 255;
          *pDest++ = 255 - c;
          *pDest++ = 255 - c;
          break;
        case 1:
          *pDest++ = 255;
          *pDest++ = c;
          *pDest++ = 0;
          break;
        case 2:
          *pDest++ = 255 - c;
          *pDest++ = 255;
          *pDest++ = 0;
          break;
        case 3:
          *pDest++ = 0;
          *pDest++ = 255;
          *pDest++ = c;
          break;
        case 4:
          *pDest++ = 0;
          *pDest++ = 255 - c;
          *pDest++ = 255;
          break;
        case 5:
          *pDest++ = 0;
          *pDest++ = 0;
          *pDest++ = 255 - c;
          break;
        default:
          *pDest++ = 0;
          *pDest++ = 0;
          *pDest++ = 0;
          break;
      }
      *pDest++ = 0xff;
    }
  QImage img(buffer, xsize, ysize, QImage::Format_RGB32);
  painter.drawImage(0,0, img.scaled(this->width(), this->height()));
  delete [] buffer;
}

QSize SensorWidget::sizeHint () const
{
  if(sensorType != SimRobotCore2::SensorPort::cameraSensor)
    return QSize(200, 200); // some dummy default
  return QSize(sensorDimensions[0], sensorDimensions[1]);
}

void SensorWidget::update()
{
  QWidget::update();
}

QMenu* SensorWidget::createEditMenu() const
{
  QMenu* menu = new QMenu(tr("&Edit"));
  QAction* action;
  action = menu->addAction(QIcon(":/Icons/page_copy.png"), tr("&Copy"));
  action->setShortcut(QKeySequence(QKeySequence::Copy));
  action->setStatusTip(tr("Copy the current selection's contents or view to the clipboard"));
  connect(action, SIGNAL(triggered()), this, SLOT(copy()));
  return menu;
}

void SensorWidget::copy()
{
  QApplication::clipboard()->clear();
  mineData = new QMimeData;
  setClipboardGraphics(*mineData);
  int count = 1;
  for(int i = 0; i < int(sensorDimensions.size()); i++)
    count *= sensorDimensions[i];
  if(count < 10000)
    setClipboardText(*mineData);
  QApplication::clipboard()->setMimeData(mineData);
}

void SensorWidget::setClipboardGraphics(QMimeData& mimeData)
{
  mimeData.setImageData(QVariant(grab().toImage()));
}

void SensorWidget::setClipboardText(QMimeData& mimeData)
{
  float floatValue;
  float* pDouble = 0;
  bool deletePDouble = false;
  int dimSize[3] = {1, 1, 1};
  {
    for(int i = 0; i < 3 && i < int(sensorDimensions.size()); i++)
      if(sensorDimensions[i] > 0)
        dimSize[i] = sensorDimensions[i];
  }
  switch(sensorType)
  {
    case SimRobotCore2::SensorPort::boolSensor:
    {
      floatValue = sensor->getValue().boolValue ? 1 : 0;
      pDouble = &floatValue;
      break;
    }
    case SimRobotCore2::SensorPort::floatSensor:
      floatValue = sensor->getValue().floatValue;
      pDouble = &floatValue;
      break;
    case SimRobotCore2::SensorPort::cameraSensor:
    {
      pDouble = new float[dimSize[0] * dimSize[1] * dimSize[2]];
      deletePDouble = true;
      const unsigned char *vals = sensor->getValue().byteArray;
      for(int i = dimSize[0] * dimSize[1] * dimSize[2] - 1; i >= 0; --i)
        pDouble[i] = vals[i];
      break;
    }
    case SimRobotCore2::SensorPort::floatArraySensor:
    {
      pDouble = const_cast<float*>(sensor->getValue().floatArray);
      break;
    }
    case SimRobotCore2::SensorPort::noSensor:
      break;
  }

  if(!pDouble)
    return;

  QString text;
  const QLocale& locale(QLocale::system());
  float* pData = pDouble;
  char sBuffer[30];
  for(int y = 0; y < dimSize[1] * dimSize[2]; y++)
    for(int x = 0; x < dimSize[0]; x++)
    {
      float d = *pData;
      sprintf(sBuffer, "%g", d);
      for(char* str = sBuffer; *str; ++str)
        if(*str == '.')
          text += locale.decimalPoint();
        else
          text += *str;
      text += x + 1 == dimSize[0] ? '\n' : '\t';
      pData++;
    }
  if(deletePDouble)
    delete[] pDouble;

  mimeData.setText(text);
}
