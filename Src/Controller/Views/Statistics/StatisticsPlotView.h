/**
 * @file Controller/Views/Evaluation/StatisticsPlotView.h
 *
 * Declaration of class StatisticsPlotView.
 * Based on implementation of @file Controller/Views/PlotView.h
 *
 * @author <a href="mailto:jan_fie@uni-bremen.de">Jan Fiedler</a>
 */

#pragma once

#include <QWidget>
#include <QPainter>
#include <QIcon>
#include <string>
#include <SimRobot.h>

class Statistics;
class StatisticsPlotWidget;
class QPainter;
class QPointF;

/**
 * @class StatisticsPlotView
 * A class to represent a map with drawings from Statistics.
 */
class StatisticsPlotView : public SimRobot::Object
{
public:
  /**
   * @param fullName The path to this view in the scene graph
   * @param statistics The statistics object.
   * @param name The name of the view.
   * @param plotSize The number of entries in a plot.
   * @param minValue The minimum value in a plot.
   * @param maxValue The maximum value in a plot.
   * @param yUnit The name of the y-axis.
   * @param xUnit The name of the x-axis.
   * @param xScale A scale factor for the x-axis.
   */
  StatisticsPlotView(const QString& fullName, Statistics& statistics, const std::string& name,
           unsigned int plotSize, float minValue, float maxValue,
           const std::string& yUnit = "", const std::string& xUnit = "", float xScale = 1);

  ~StatisticsPlotView();

  /**
   * Changes the parameters of the plot.
   * @param plotSize The number of entries in a plot.
   * @param minValue The minimum value in a plot.
   * @param maxValue The maximum value in a plot.
   * @param yUnit The name of the y-axis.
   * @param xUnit The name of the x-axis.
   * @param xScale A scale factor for the x-axis.
   */
  void setParameters(unsigned int plotSize, float minValue, float maxValue,
                     const std::string& yUnit = "", const std::string& xUnit = "", float xScale = 1);

private:
  const QString fullName; /**< The path to this view in the scene graph */
  const QIcon icon; /**< The icon used for listing this view in the scene graph */
  Statistics& statistics; /**< A reference to the statistics object. */
  StatisticsPlotWidget* plotWidget = nullptr;
  const std::string name; /**< The name of the view. */
  unsigned int plotSize; /**< The number of entries in a plot. */
  float minValue; /**< The minimum value in a plot. */
  float maxValue; /**< The maximum value in a plot. */
  float valueLength;
  std::string yUnit; /**< The name of the y-axis. */
  std::string xUnit; /**< The name of the x-axis. */
  float xScale; /**< A scale factor for the x-axis. */
  QPointF* points; /**< A buffer for drawing points. */

  /**
   * The method returns a new instance of a widget for this direct view.
   * The caller has to delete this instance. (Qt handles this)
   * @return The widget.
   */
  SimRobot::Widget* createWidget() override;

  const QString& getFullName() const override { return fullName; }
  const QIcon* getIcon() const override { return &icon; }

  friend class StatisticsPlotWidget;
};

class StatisticsPlotWidget : public QWidget, public SimRobot::Widget
{
  Q_OBJECT

public:
  StatisticsPlotWidget(StatisticsPlotView& plotView, StatisticsPlotWidget*& plotWidget);

  ~StatisticsPlotWidget();

public slots:
  void determineMinMaxValue();
  void toggleDrawUnits();
  void toggleDrawLegend();
  void toggleAntialiasing();
  void exportAsGnuplot();

private:
  StatisticsPlotView& plotView;
  StatisticsPlotWidget*& plotWidget;
  unsigned int lastTimeStamp = 0; /**< Timestamp of the last plot drawing. */
  QPainter painter; /**< The painter used for painting the plot. */
  QPen blackPen;
  QPen grayPen;

  bool drawUnits = true;
  bool drawLegend = true;
  bool antialiasing = false;

  bool needsRepaint() const;

  QSize sizeHint() const override { return QSize(320, 240); }

  void paintEvent(QPaintEvent* event) override;

  QWidget* getWidget() override { return this; }
  void update() override;
  QMenu* createUserMenu() const override;
  void paint(QPainter& painter) override;
};
