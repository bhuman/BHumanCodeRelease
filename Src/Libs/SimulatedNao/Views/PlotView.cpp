/**
 * @file SimulatedNao/Views/PlotView.cpp
 *
 * Implementation of class PlotView
 *
 * @author Thomas Röfer
 * @author Colin Graf
 */

#include <QMouseEvent>
#include <QMenu>
#include <QSettings>
#include <QFileDialog>
#include <QTextStream>

#include "SimulatedNao/ConsoleRoboCupCtrl.h"
#include "SimulatedNao/RobotConsole.h"
#include "PlotView.h"
#include <algorithm>

PlotWidget::PlotWidget(PlotView& view) :
  view(view), grayPen(QColor(0xbb, 0xbb, 0xbb))
{
  grayPen.setWidth(0);
  setFocusPolicy(Qt::StrongFocus);
  setBackgroundRole(QPalette::Base);
  setAutoFillBackground(true);

  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(view.fullName);
  drawUnits = settings.value("DrawUnits", true).toBool();
  drawLegend = settings.value("DrawLegend", true).toBool();
  antialiasing = settings.value("Antialiasing", false).toBool();
  settings.endGroup();
}

PlotWidget::~PlotWidget()
{
  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(view.fullName);
  settings.setValue("DrawUnits", drawUnits);
  settings.setValue("DrawLegend", drawLegend);
  settings.setValue("Antialiasing", antialiasing);
  settings.endGroup();

  view.widget = nullptr;
}

bool PlotWidget::needsRepaint() const
{
  SYNC_WITH(view.console);

  for(const RobotConsole::Layer& layer : view.console.plotViews[view.name])
    for(const RobotConsole::Plot* plot : getPlots(layer.layer))
      if(plot && plot->timestamp > lastTimestamp)
        return true;
  return false;
}

void PlotWidget::paintEvent(QPaintEvent*)
{
  painter.begin(this);
  if(antialiasing)
    painter.setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing);
  paint(painter);
  painter.end();
}

void PlotWidget::paint(QPainter& painter)
{
  // calculate margin sizes
  float plotSizeF = std::max(1.f, static_cast<float>(view.plotSize));
  const float space = 3;
  float leftMargin;
  float bottomMargin;
  float topMargin;
  float rightMargin;
  float textHeight;
  QPen blackPen(palette().text().color());
  blackPen.setWidth(0);
  const QFontMetrics& fontMetrics = painter.fontMetrics();
  {
    char buf[32];
    sprintf(buf, "%g", (view.maxValue - view.minValue) < 8.f ? (view.minValue / 4.f) : view.minValue);
    const QSize& bufSize = fontMetrics.size(Qt::TextSingleLine, buf);
    textHeight = bufSize.height();
    leftMargin = bufSize.width();
    bottomMargin = textHeight + space * 2;
    topMargin = space + textHeight / 2.f;
    sprintf(buf, "%g", (view.maxValue - view.minValue) < 8.f ? (view.maxValue / 4.f) : view.maxValue);
    const QSize& bufSize2 = fontMetrics.size(Qt::TextSingleLine, buf);
    if(bufSize2.width() > leftMargin)
      leftMargin = bufSize2.width();
    rightMargin = fontMetrics.size(Qt::TextSingleLine, "0").width() / 2.f + space;
    leftMargin += space * 2;
    if(drawUnits)
    {
      if(!view.yUnit.empty())
        topMargin += textHeight + space;
      if(!view.xUnit.empty())
        rightMargin += fontMetrics.size(Qt::TextSingleLine, view.xUnit.c_str()).width() + space;
    }
  }

  // calculate size of the plot area
  const QRect& windowRect = painter.window();
  QRect plotRect(QPoint(static_cast<int>(windowRect.x() + leftMargin), static_cast<int>(windowRect.y() + topMargin)),
                 QPoint(static_cast<int>(windowRect.right() - rightMargin), static_cast<int>(windowRect.bottom() - bottomMargin)));
  if(!plotRect.isValid())
    return; // window too small

  // calculate step sizes to regrade the axes
  float stepY = std::pow(10.f, std::ceil(std::log10(view.valueLength * 20.f / plotRect.height())));
  float stepX = std::pow(10.f, std::ceil(std::log10(plotSizeF * 25.f / plotRect.width())));
  if(stepY * plotRect.height() / view.valueLength >= 40.f)
    stepY /= 2.f;
  if(stepY * plotRect.height() / view.valueLength >= 40.f)
    stepY /= 2.f;
  if(stepY > std::max<>(view.maxValue, -view.minValue))
    stepY = std::max<>(view.maxValue, -view.minValue);
  if(plotRect.width() * stepX / plotSizeF >= 50.f)
    stepX /= 2.f;
  if(plotRect.width() * stepX / plotSizeF >= 50.f)
    stepX /= 2.f;
  if(stepX > plotSizeF)
    stepX = plotSizeF;

  if((view.maxValue - view.minValue) < 8.f)
  {
    char buf[32];
    sprintf(buf, "%g", stepY < std::abs(view.maxValue) ? (view.maxValue - stepY) : view.maxValue);
    int width = fontMetrics.size(Qt::TextSingleLine, buf).width();
    sprintf(buf, "%g", stepY < std::abs(view.minValue) ? (view.minValue + stepY) : view.minValue);
    int width2 = fontMetrics.size(Qt::TextSingleLine, buf).width();
    if(width2 > width)
      width = width2;
    width += static_cast<int>(space) * 2;
    if(width > leftMargin)
    {
      plotRect.setLeft(static_cast<int>(plotRect.left() - (leftMargin - width)));
      leftMargin = width;
    }
  }

  // draw axis labels
  {
    char buf[32];
    painter.setPen(blackPen);

    // y
    {
      QRectF rect(0, 0, leftMargin - space, textHeight);
      sprintf(buf, "%g", view.maxValue);
      rect.moveTop((view.valueLength - (view.maxValue - view.minValue)) * plotRect.height() / view.valueLength + topMargin - textHeight / 2.f);
      painter.drawText(rect, Qt::AlignRight, tr(buf));
      sprintf(buf, "%g", view.minValue);
      rect.moveTop((view.valueLength - (view.minValue - view.minValue)) * plotRect.height() / view.valueLength + topMargin - textHeight / 2.f);
      painter.drawText(rect, Qt::AlignRight, tr(buf));
      rect.moveTop((view.valueLength - (0.f - view.minValue)) * plotRect.height() / view.valueLength + topMargin - textHeight / 2.f);
      painter.drawText(rect, Qt::AlignRight, QString("0"));
      const float minOffset = view.valueLength * 20.f / plotRect.height();
      for(float pos = stepY; pos <= view.maxValue - minOffset; pos += stepY)
      {
        sprintf(buf, "%g", pos);
        rect.moveTop((view.valueLength - (pos - view.minValue)) * plotRect.height() / view.valueLength + topMargin - textHeight / 2.f);
        painter.drawText(rect, Qt::AlignRight, tr(buf));
      }
      for(float pos = -stepY; pos >= view.minValue + minOffset; pos -= stepY)
      {
        sprintf(buf, "%g", pos);
        rect.moveTop(((view.maxValue - view.minValue) - (pos - view.minValue)) * plotRect.height() / view.valueLength + topMargin - textHeight / 2.f);
        painter.drawText(rect, Qt::AlignRight, tr(buf));
      }
    }

    // x
    {
      QRectF rect(0, plotRect.bottom() + space, leftMargin - space, textHeight);
      sprintf(buf, "%.2f", plotSizeF * view.xScale);
      rect.setWidth(fontMetrics.size(Qt::TextSingleLine, buf).width());
      sprintf(buf, "%g", plotSizeF * view.xScale);
      rect.moveLeft(leftMargin - rect.width() / 2.f);
      painter.drawText(rect, Qt::AlignCenter, tr(buf));
      rect.moveLeft(plotRect.width() + leftMargin - rect.width() / 2.f);
      painter.drawText(rect, Qt::AlignCenter, QString("0"));
      const float minOffset = plotSizeF * 25.f / plotRect.width();
      for(float pos = stepX; pos <= plotSizeF - minOffset; pos += stepX)
      {
        sprintf(buf, "%g", pos * view.xScale);
        rect.moveLeft((plotSizeF - pos) * plotRect.width() / plotSizeF + leftMargin - rect.width() / 2.f);
        painter.drawText(rect, Qt::AlignCenter, tr(buf));
      }
    }
  }

  // draw units
  if(drawUnits)
  {
    if(!view.yUnit.empty())
    {
      QRect rect(static_cast<int>(space), static_cast<int>(space),
                 static_cast<int>(windowRect.width() - space), static_cast<int>(textHeight));
      painter.drawText(rect, Qt::AlignLeft, tr(view.yUnit.c_str()));
    }
    if(!view.xUnit.empty())
    {
      QRect rect(0, static_cast<int>(plotRect.bottom() + space),
                 static_cast<int>(windowRect.width() - space), static_cast<int>(textHeight));
      painter.drawText(rect, Qt::AlignRight, tr(view.xUnit.c_str()));
    }
  }

  // setup plot-paint-transformation
  {
    QTransform transform;
    transform.translate(plotRect.right(), plotRect.top() + static_cast<float>(plotRect.height()) * view.maxValue / view.valueLength);
    transform.scale(-static_cast<float>(plotRect.width()) / plotSizeF, -static_cast<float>(plotRect.height()) / view.valueLength);
    painter.setTransform(transform);
  }

  // draw axes and regrade-lines
  painter.setPen(grayPen);
  float twoPxX = plotSizeF * 2.f / plotRect.width();
  float twoPxY = view.valueLength * 2.f / plotRect.height();
  for(float pos = stepY; pos < view.maxValue; pos += stepY)
    painter.drawLine(QPointF(0.f, pos), QPointF(plotSizeF + twoPxX, pos));
  for(float pos = -stepY; pos > view.minValue; pos -= stepY)
    painter.drawLine(QPointF(0.f, pos), QPointF(plotSizeF + twoPxX, pos));
  for(float pos = stepX; pos < plotSizeF; pos += stepX)
    painter.drawLine(QPointF(pos, view.minValue - twoPxY), QPointF(pos, view.maxValue));
  painter.setPen(blackPen);
  painter.drawLine(QPointF(0.f, 0.f), QPointF(plotSizeF + twoPxX, 0.f));
  painter.drawLine(QPointF(0.f, view.minValue - twoPxY), QPointF(0.f, view.maxValue));
  painter.drawLine(QPointF(plotSizeF, view.minValue - twoPxY), QPointF(plotSizeF, view.maxValue));
  if(view.minValue != 0.f)
    painter.drawLine(QPointF(0.f, view.minValue), QPointF(plotSizeF + twoPxX, view.minValue));
  if(view.maxValue != 0.f)
    painter.drawLine(QPointF(0.f, view.maxValue), QPointF(plotSizeF + twoPxX, view.maxValue));

  {
    SYNC_WITH(view.console);

    // draw plots
    if(antialiasing)
      painter.setRenderHints(QPainter::Antialiasing);
    int legendWidth = 0;
    const std::list<RobotConsole::Layer>& plotList = view.console.plotViews[view.name];
    for(const RobotConsole::Layer& layer : plotList)
      for(const RobotConsole::Plot* plot : getPlots(layer.layer))
      {
        size_t numOfPoints = std::min(plot->points.size(), static_cast<size_t>(view.plotSize));
        if(numOfPoints > 1)
        {
          std::list<float>::const_iterator k = plot->points.end();
          for(size_t i = 0; i < numOfPoints; ++i)
            view.points[i].ry() = *(--k);

          const ColorRGBA& color = layer.color;
          QPen pen = color == ColorRGBA::black ? blackPen : QPen(QColor(color.r, color.g, color.b));
          pen.setWidth(0);
          painter.setPen(pen);
          painter.drawPolyline(view.points, static_cast<int>(numOfPoints));
        }
        lastTimestamp = std::max(lastTimestamp, plot->timestamp);
        if(drawLegend)
        {
          int width = fontMetrics.size(Qt::TextSingleLine, layer.description.c_str()).width();
          if(width > legendWidth)
            legendWidth = width;
        }
      }
    if(antialiasing)
      painter.setRenderHints(QPainter::Antialiasing, false);

    // draw legend
    if(drawLegend && plotList.size() > 0)
    {
      QRect legendRect(static_cast<int>(plotRect.left() + space),
                       static_cast<int>(plotRect.top() + space),
                       static_cast<int>(legendWidth + space * 3 + 10),
                       static_cast<int>(space + plotList.size() * (textHeight + space)));
      QRect rect(static_cast<int>(legendRect.left() + space + 10 + space),
                 static_cast<int>(legendRect.top() + space),
                 static_cast<int>(legendRect.width() - (space + 10 + space)),
                 static_cast<int>(textHeight));
      QLine line(static_cast<int>(legendRect.left() + space),
                 static_cast<int>(legendRect.top() + space + textHeight / 2 + 1),
                 static_cast<int>(legendRect.left() + space + 10),
                 static_cast<int>(legendRect.top() + space + textHeight / 2 + 1));
      painter.setTransform(QTransform());
      painter.setPen(blackPen);
      painter.setBrush(QBrush(QColor(0xff, 0xff, 0xff, 0x99)));
      painter.drawRect(legendRect);
      for(const RobotConsole::Layer& layer : plotList)
      {
        painter.setPen(blackPen);
        painter.drawText(rect, Qt::AlignLeft, tr(layer.description.c_str()));
        const ColorRGBA& color = layer.color;
        painter.setPen(QColor(color.r, color.g, color.b));
        painter.drawLine(line);
        rect.moveTop(static_cast<int>(rect.top() + textHeight + space));
        line.translate(0, static_cast<int>(textHeight + space));
      }
    }
  }
}

void PlotWidget::determineMinMaxValue()
{
  bool started = false;
  {
    SYNC_WITH(view.console);
    const std::list<RobotConsole::Layer>& plotList = view.console.plotViews[view.name];
    for(const auto& layer : plotList)
      for(const RobotConsole::Plot* plot : getPlots(layer.layer))
      {
        int numOfPoints = std::min(static_cast<int>(plot->points.size()), static_cast<int>(view.plotSize));
        if(numOfPoints > 1)
        {
          std::list<float>::const_iterator k = plot->points.begin();
          for(int j = view.plotSize - numOfPoints; j < static_cast<int>(view.plotSize); ++j)
          {
            const float& value(*(k++));
            if(started)
            {
              if(value < view.minValue)
                view.minValue = value;
              else if(value > view.maxValue)
                view.maxValue = value;
            }
            else
            {
              view.minValue = value;
              view.maxValue = view.minValue + 0.00001f;
              started = true;
            }
          }
        }
      }
  }

  if(started)
  {
    int precision = static_cast<int>(std::ceil(std::log10(view.maxValue - view.minValue))) - 1;
    float rounder = std::pow(10.f, static_cast<float>(precision));
    view.minValue = std::floor(view.minValue / rounder) * rounder;
    view.maxValue = std::ceil(view.maxValue / rounder) * rounder;
    view.valueLength = view.maxValue - view.minValue;
  }

  QWidget::update();
}

void PlotWidget::toggleDrawUnits()
{
  drawUnits = !drawUnits;
  QWidget::update();
}

void PlotWidget::toggleDrawLegend()
{
  drawLegend = !drawLegend;
  QWidget::update();
}

void PlotWidget::toggleAntialiasing()
{
  antialiasing = !antialiasing;
  QWidget::update();
}

void PlotWidget::exportAsGnuplot()
{
  // ask for destination files
  QSettings& settings = RoboCupCtrl::application->getSettings();
  QString fileName = QFileDialog::getSaveFileName(this,
                                                  tr("Export as Gnuplot"), settings.value("ExportDirectory", "").toString(), tr("Gnuplot (*.plt)")
#ifdef LINUX
                                                  , nullptr, QFileDialog::DontUseNativeDialog
#endif
                                                  );
  if(fileName.isEmpty())
    return;
  settings.setValue("ExportDirectory", QFileInfo(fileName).dir().path());

  // prepare plot data
  SYNC_WITH(view.console);
  QVector<QVector<float>> data;
  int numOfPoints = view.plotSize;
  int numOfPlots = 0;
  {
    const std::list<RobotConsole::Layer>& plotList(view.console.plotViews[view.name]);
    numOfPlots = static_cast<int>(plotList.size());
    for(const RobotConsole::Layer& layer : plotList)
      for(const RobotConsole::Plot* plot : getPlots(layer.layer))
      {
        int curNumOfPoints = std::min(static_cast<int>(plot->points.size()), static_cast<int>(view.plotSize));
        if(curNumOfPoints < numOfPoints)
          numOfPoints = curNumOfPoints;
      }
  }
  if(!numOfPoints || !numOfPlots)
    return;
  data.resize(numOfPoints);
  for(int i = 0; i < numOfPoints; ++i)
    data[i].resize(numOfPlots);

  {
    const std::list<RobotConsole::Layer>& plotList = view.console.plotViews[view.name];
    int currentPlot = 0;
    for(const RobotConsole::Layer& layer : plotList)
      for(const RobotConsole::Plot* plot : getPlots(layer.layer))
      {
        std::list<float>::const_reverse_iterator k = plot->points.rbegin();
        for(int j = numOfPoints - 1; j >= 0; --j)
          data[j][currentPlot] = *(k++);
        ++currentPlot;
      }
  }

  // open output steams
  QFileInfo fileInfo(fileName);
  QFile file(fileName);
  if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
    return;
  QTextStream out(&file);
  QFile dataFile(fileInfo.dir().path() + "/" + fileInfo.baseName() + ".dat");
  if(!dataFile.open(QIODevice::WriteOnly | QIODevice::Text))
    return;
  QTextStream dataOut(&dataFile);

  // create .plt file
  out << "reset\n";
  out << "set title \"" << QString(view.name.c_str()) << "\"\n";
  if(!view.xUnit.empty())
    out << "set xlabel \"[" << QString(view.xUnit.c_str()) << "]\"\n";
  else
    out << "#set xlabel \"x\"\n";
  if(!view.yUnit.empty())
    out << "set ylabel \"[" << QString(view.yUnit.c_str()) << "]\"\n";
  else
    out << "#set ylabel \"y\"\n";
  out << "set xrange [" << static_cast<float>(view.plotSize) * view.xScale << ":" << 0 << "]\n";
  out << "set yrange [" << view.minValue << ":" << view.maxValue << "]\n";
  out << "set terminal postscript eps enhanced color\n";
  out << "set output \"" << fileInfo.baseName() << ".eps\"\n";
  const std::list<RobotConsole::Layer>& plotList(view.console.plotViews[view.name]);
  int currentPlot = 0;
  for(const RobotConsole::Layer& layer : plotList)
  {
    if(currentPlot == 0)
      out << "plot ";
    else
      out << ", ";
    out << "\"" << fileInfo.baseName() << ".dat\" using 1:" << currentPlot + 2 << " title \"" << layer.description.c_str() << "\"";
    char rgbcolor[10];
    sprintf(rgbcolor, "%06X", layer.color.r << 16 | layer.color.g << 8 | layer.color.b);
    out << " with lines linetype 1 linecolor rgbcolor \"#" << rgbcolor << "\"";
    ++currentPlot;
  }
  out << "\n";

  // create .dat file
  for(int i = 0, j = view.plotSize - numOfPoints; j < int(view.plotSize); ++j, ++i)
  {
    dataOut << static_cast<float>(view.plotSize - 1 - j) * view.xScale;
    for(int k = 0; k < numOfPlots; ++k)
      dataOut << " " << data[i][k];
    dataOut << "\n";
  }
}

QMenu* PlotWidget::createUserMenu() const
{
  QMenu* menu = new QMenu(tr("&Plot"));

  QAction* action = menu->addAction(tr("Export as Gnuplot..."));
  connect(action, &QAction::triggered, this, &PlotWidget::exportAsGnuplot);

  menu->addSeparator();
  action = menu->addAction(tr("Show Units"));
  action->setCheckable(true);
  action->setChecked(drawUnits);
  connect(action, &QAction::triggered, this, &PlotWidget::toggleDrawUnits);
  action = menu->addAction(tr("Show Key"));
  action->setCheckable(true);
  action->setChecked(drawLegend);
  connect(action, &QAction::triggered, this, &PlotWidget::toggleDrawLegend);
  action = menu->addAction(tr("Anti-aliased"));
  action->setCheckable(true);
  action->setChecked(antialiasing);
  connect(action, &QAction::triggered, this, &PlotWidget::toggleAntialiasing);

  menu->addSeparator();
  action = menu->addAction(tr("Auto-min-max"));
  connect(action, &QAction::triggered, this, &PlotWidget::determineMinMaxValue);

  return menu;
}

void PlotWidget::update()
{
  if(needsRepaint())
    QWidget::update();
}

std::vector<const RobotConsole::Plot*> PlotWidget::getPlots(const std::string& name) const
{
  std::vector<const RobotConsole::Plot*> plots;
  if(!view.threadName.empty())
  {
    auto& data = view.console.threadData[view.threadName];

    // First search the thread belonging to this view
    auto plot = data.plots.find(name);
    if(plot != data.plots.end())
    {
      plots.push_back(&plot->second);
      return plots;
    }
  }

  // Search other threads if
  const auto threadsFound = view.console.ctrl->getThreadsFor(view.console.threadData, "plot:" + name);
  if(view.threadName.empty()
     || std::find(threadsFound.begin(), threadsFound.end(), view.threadName) == threadsFound.end())
    for(const std::string& threadName : threadsFound)
    {
      const RobotConsole::ThreadData& data = view.console.threadData[threadName];
      auto plot = data.plots.find(name);
      if(plot != data.plots.end())
        plots.push_back(&plot->second);
    }

  return plots;
}

PlotView::PlotView(const QString& fullName, RobotConsole& console, const std::string& name, const std::string& threadName) :
  fullName(fullName), icon(":/Icons/icons8-line-chart-50.png"), console(console), name(name), threadName(threadName)
{
  icon.setIsMask(true);
}

PlotView::~PlotView()
{
  delete[] points;
}

void PlotView::setParameters(unsigned int plotSize, float minValue, float maxValue, const std::string& yUnit, const std::string& xUnit, float xScale)
{
  delete[] points;
  points = new QPointF[plotSize];
  for(unsigned int i = 0; i < plotSize; ++i)
    points[i].rx() = i;

  this->plotSize = plotSize;
  this->minValue = minValue;
  this->maxValue = maxValue;
  this->valueLength = maxValue - minValue;
  this->yUnit = yUnit;
  this->xUnit = xUnit;
  this->xScale = xScale;
  if(widget)
    widget->update();
}

SimRobot::Widget* PlotView::createWidget()
{
  ASSERT(!widget);
  widget = new PlotWidget(*this);
  return widget;
}
