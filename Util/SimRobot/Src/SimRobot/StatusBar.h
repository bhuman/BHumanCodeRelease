
#pragma once

#include <QStatusBar>

#include "SimRobot.h"

class QAction;

class StatusBar : public QStatusBar
{
  Q_OBJECT

public:
  StatusBar(QWidget* parent);

  void addLabel(const SimRobot::Module* module, SimRobot::StatusLabel* statusLabel);
  void removeAllLabels();
  void removeLabelsFromModule(const SimRobot::Module* module);

  void setUserMessage(const QString& userMessage);

  void update();

  QAction* toggleViewAction();

private:
  class RegisteredLabel
  {
  public:
    const SimRobot::Module* module;
    SimRobot::StatusLabel* label;

    RegisteredLabel(const SimRobot::Module* module, SimRobot::StatusLabel* label) :
      module(module), label(label) {}
  };

  QAction* toggleViewAct;
  QList<RegisteredLabel> registeredLables;
  QString userMessage;
  QString latestMessage;

  void hideEvent(QHideEvent* event) override;
  void showEvent(QShowEvent* event) override;

private slots:
  void messageChanged(const QString& message);
};
