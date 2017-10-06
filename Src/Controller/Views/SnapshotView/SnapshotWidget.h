#pragma once

#include <QCheckBox>
#include <QElapsedTimer>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTimer>
#include <QWidget>
#include <SimRobot.h>
#include <string>

#include "Controller/RobotConsole.h"
#include "SnapshotView.h"

class SnapshotWidget : public QWidget, public SimRobot::Widget
{
  Q_OBJECT;

public:
  SnapshotWidget(SnapshotView& view);
  virtual ~SnapshotWidget();

  virtual QWidget* getWidget() { return this; }
  virtual void update();

public slots:
  void onSnapshot();
  void onLoop();
  void run();

private:
  SnapshotView& view;

  bool alreadyLogging;
  int count;
  qint64 interval;
  std::string prefix;

  QGroupBox* gridBox;
  QLineEdit* prefixEdit;
  QLineEdit* countEdit;
  QLineEdit* intervalEdit;
  QCheckBox* upper;
  QCheckBox* lower;
  QPushButton* loopButton;
  QPushButton* snapshotButton;
  QLabel* stateReportText;

  QElapsedTimer elapsedtimer;
  QTimer* loopTimer;

  void saveImages(const std::string& path, int number);

  void saveImage(DebugImage* image, const std::string& path, int number);
};
