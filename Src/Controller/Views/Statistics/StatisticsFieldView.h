/**
 * @file Controller/Views/Statistics/StatisticsFieldView.h
 *
 * Declaration of class StatisticsFieldView.
 * Based on implementation of @file Controller/Views/FieldView.h
 *
 * @author <a href="mailto:jan_fie@uni-bremen.de">Jan Fiedler</a>
 */

#pragma once

#include <QString>
#include <QIcon>
#include <SimRobot.h>

class Statistics;

/**
 * @class StatisticsFieldView
 *
 * A class to represent a view displaying debug drawings in field coordinates.
 */
class StatisticsFieldView : public SimRobot::Object
{
public:
  /**
   * @param fullName The path to this view in the scene graph
   * @param statistics The statistics object.
   * @param name The name of the view.
   */
  StatisticsFieldView(const QString& fullName, Statistics& statistics, const std::string& name);

private:
  const QString fullName; /**< The path to this view in the scene graph */
  const QIcon icon; /**< The icon used for listing this view in the scene graph */
  Statistics& statistics; /**< A reference to the statistics object. */
  const std::string name; /**< The name of the view. */

  /**
   * The method returns a new instance of a widget for this direct view.
   * The caller has to delete this instance. (Qt handles this)
   * @return The widget.
   */
  SimRobot::Widget* createWidget() override;

  const QString& getFullName() const override { return fullName; }
  const QIcon* getIcon() const override { return &icon; }

  friend class StatisticsFieldWidget;
};
