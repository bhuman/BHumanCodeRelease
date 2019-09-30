/**
 * @file AnnotationView.h
 * @author Andreas Stolpmann
 */

#pragma once

#include "SimRobot.h"
#include "Platform/SystemCall.h"
#include "Controller/Representations/AnnotationInfo.h"

#include <QIcon>
#include <QWidget>
#include <unordered_set>

class AnnotationInfo;
class AnnotationWidget;
class RobotConsole;
class LogPlayer;

class QTableWidget;
class QCheckBox;

class AnnotationView : public SimRobot::Object
{
public:
  AnnotationView(const QString& fullName, AnnotationInfo& info, LogPlayer& logPlayer, SystemCall::Mode mode, SimRobot::Application* application);

private:
  const QString fullName; /**< The path to this view in the scene graph */
  const QIcon icon; /**< The icon used for listing this view in the scene graph */
  AnnotationInfo& info;
  LogPlayer& logPlayer; /**< Used to jump to frames */
  SystemCall::Mode mode; /**< Only jump if replaying logs */
  SimRobot::Application* application;

  bool stopOnFilter = false;
  bool filterIsRegEx = false;
  QString filter = "";

  /**
   * The method returns a new instance of a widget for this direct view.
   * The caller has to delete this instance. (Qt handles this)
   * @return The widget.
   */
  SimRobot::Widget* createWidget() override;

  const QString& getFullName() const override { return fullName; }
  const QIcon* getIcon() const override { return &icon; }

  friend class AnnotationWidget; //AnnotationWidget needs access to console
  friend class AnnotationInfo;
};

class AnnotationWidget : public QWidget, public SimRobot::Widget
{
  Q_OBJECT;

public:
  AnnotationWidget(AnnotationView& view, SystemCall::Mode mode);
  ~AnnotationWidget();

  QWidget* getWidget() override;
  void update() override;

private:
  AnnotationView& view;
  unsigned timeOfLastUpdate;

  QTableWidget* table;
  std::unordered_set<unsigned> annotationNumbers;

  QCheckBox* stopCheckBox;
  QCheckBox* useRegExCheckBox;

  QString filter;
  void applyFilter();

private slots:
  void useRegExCheckBoxStateChanged(int state);
  void stopCheckBoxStateChanged(int state);
  void filterChanged(const QString& newFilter);
  void jumpFrame(int row, int column);
};
