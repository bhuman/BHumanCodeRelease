/**
* @file AnnotationWidget.h
* @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
*/

#pragma once

#include <SimRobot.h>
#include <QWidget>
#include <unordered_map>

struct Row;
class AnnotationView;
class QTableWidget;
class QCheckBox;

class AnnotationWidget : public QWidget, public SimRobot::Widget
{
  Q_OBJECT;

public:
  AnnotationWidget(AnnotationView& view);
  virtual ~AnnotationWidget();

  virtual QWidget* getWidget();
  virtual void update();

private:
  AnnotationView& view;
  unsigned timeOfLastUpdate;

  QTableWidget* table;
  std::unordered_map<unsigned, Row*> rows;

  QCheckBox* stopCheckBox;

  QString filter;
  void applyFilter();

private slots:
  void filterChanged(const QString& newFilter);
  void jumpFrame(int row, int column);
};
