
#pragma once

#include <QDockWidget>

#include "SimRobot.h"

class RegisteredDockWidget : public QDockWidget
{
  Q_OBJECT

public:
  RegisteredDockWidget(const QString& fullName, QWidget* parent);

  void setWidget(SimRobot::Widget* widget, const SimRobot::Module* module, SimRobot::Object* object, int flags);
  bool hasWidget() {return widget != 0;}

  bool canClose();
  QMenu* createFileMenu() const;
  QMenu* createEditMenu();
  QMenu* createUserMenu() const;

  void update();

  bool isReallyVisible() const {return reallyVisible;}
  const SimRobot::Module* getModule() const {return module;}
  const SimRobot::Object* getObject() const {return object;}
  const QString& getFullName() const {return fullName;}

  QAction* toggleViewAction() const;

signals:
  void closedObject(const QString& object);
  void closedContextMenu();

private:
  QString fullName;
  const SimRobot::Module* module;
  SimRobot::Object* object;
  SimRobot::Widget* widget;
  int flags;
  bool reallyVisible;

  void closeEvent(QCloseEvent* event) override;
  void contextMenuEvent(QContextMenuEvent* event) override;
  void keyPressEvent(QKeyEvent* event) override;
  void keyReleaseEvent(QKeyEvent* event) override;

private slots:
  void visibilityChanged(bool visible);
  void copy();
  void exportAsSvg();
  void exportAsPng();
};
