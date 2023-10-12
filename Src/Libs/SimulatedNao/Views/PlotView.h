/**
 * @file SimulatedNao/Views/PlotView.h
 *
 * Declaration of class PlotView
 *
 * @author Thomas Röfer
 * @author Colin Graf
 */

#pragma once

#include <QWidget>
#include <QPainter>
#include <QIcon>
#include <string>
#include <SimRobot.h>

class RobotConsole;
class QPointF;
class QPainter;
class PlotWidget;

/**
 * @class PlotView
 * A class to represent a map with drawings sent by the robot code.
 */
class PlotView : public SimRobot::Object
{
public:
  /**
   * Creates a new plot view.  \c setParameters must be called for a valid initialization.
   * @param fullName The path to this view in the scene graph
   * @param console The console object.
   * @param name The name of the view.
   * @param threadName The name of the associated thread. If empty, no thread is associated.
   */
  PlotView(const QString& fullName, RobotConsole& console, const std::string& name, const std::string& threadName);

  ~PlotView();

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
                     const std::string& yUnit, const std::string& xUnit, float xScale);

private:
  const QString fullName; /**< The path to this view in the scene graph */
  QIcon icon; /**< The icon used for listing this view in the scene graph */
  RobotConsole& console; /**< A reference to the console object. */
  const std::string name; /**< The name of the view. */
  std::string threadName; /**< The name of the associated thread. If empty, no thread is associated. */
  PlotWidget* widget = nullptr;
  unsigned int plotSize = 0; /**< The number of entries in a plot. */
  float minValue; /**< The minimum value in a plot. */
  float maxValue; /**< The maximum value in a plot. */
  float valueLength;
  std::string yUnit; /**< The name of the y-axis. */
  std::string xUnit; /**< The name of the x-axis. */
  float xScale; /**< A scale factor for the x-axis. */
  QPointF* points = nullptr; /**< A buffer for drawing points. */

  /**
   * The method returns a new instance of a widget for this direct view.
   * The caller has to delete this instance. (Qt handles this)
   * @return The widget.
   */
  SimRobot::Widget* createWidget() override;

  const QString& getFullName() const override { return fullName; }
  const QIcon* getIcon() const override { return &icon; }

  friend class PlotWidget;
};

class PlotWidget : public QWidget, public SimRobot::Widget
{
  Q_OBJECT

public:
  PlotWidget(PlotView& view);
  ~PlotWidget();
  void update() override;

protected:

  QWidget* getWidget() override { return this; }
  void paint(QPainter& painter) override;
  void paintEvent(QPaintEvent* event) override;
  QMenu* createUserMenu() const override;
  QSize sizeHint() const override { return QSize(320, 240); }

public slots:
  void determineMinMaxValue();
  void toggleDrawUnits();
  void toggleDrawLegend();
  void toggleAntialiasing();
  void exportAsGnuplot();

private:
  bool needsRepaint() const;
  std::vector<const RobotConsole::Plot*> getPlots(const std::string& name) const;

  PlotView& view;
  QPen grayPen;
  bool drawUnits = true;
  bool drawLegend = true;
  bool antialiasing = false;
  unsigned int lastTimestamp = 0; /**< Timestamp of the last plot drawing. */
  QPainter painter; /**< The painter used for painting the plot. */
};
