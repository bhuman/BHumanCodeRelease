/**
 * @file AnnotationWidget.h
 * @author Andreas Stolpmann
 */

#pragma once

#include <SimRobot.h>
#include <QWidget>
#include <unordered_map>
#include "Platform/SystemCall.h"

struct Row;
class AnnotationView;
class QTableWidget;
class QCheckBox;

class AnnotationWidget : public QWidget, public SimRobot::Widget
{
  Q_OBJECT;

public:
  AnnotationWidget(AnnotationView& view, SystemCall::Mode mode);
  virtual ~AnnotationWidget();

  virtual QWidget* getWidget();
  virtual void update();

private:
  AnnotationView& view;
  unsigned timeOfLastUpdate;

  QTableWidget* table;
  std::unordered_map<unsigned, Row*> rows;

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
