/**
* @file SimRobot/MainWindow.h
* Declaration of the main window of SimRobot
* @author Colin Graf
*/

#pragma once

#include <QMainWindow>
#include <QSignalMapper>
#include <QActionGroup>
#include <QSettings>
#include <QSet>
#include <QHash>
#include <QLibrary>

#include "SimRobot.h"

#ifdef LINUX
#if QT_VERSION >= QT_VERSION_CHECK(4, 6, 0) && QT_VERSION < QT_VERSION_CHECK(4, 9, 0)
#define FIX_LINUX_DOCK_WIDGET_SIZE_RESTORING_BUG
#endif
#endif

#ifdef OSX
#define FIX_MACOSX_TOOLBAR_VISIBILITY_RESTORING_BUG
#define FIX_MACOSX_UNDOCKED_WIDGETS_DISAPPEAR_WHEN_DOCKED_BUG
#endif

#ifdef WINDOWS
#define FIX_WIN32_WINDOWS7_BLOCKING_BUG
#define FIX_WIN32_CRASH_WITHOUT_QGLWIDGET_BUG
#endif

class SceneGraphDockWidget;
class RegisteredDockWidget;
class StatusBar;

class MainWindow : public QMainWindow, public SimRobot::Application
{
  Q_OBJECT

public:
  static SimRobot::Application* application;

  MainWindow(int argc, char *argv[]);

  QMenu* createSimMenu();

  // public only for FIX_WIN32_WINDOWS7_BLOCKING_BUG
  int timerId; /**< The id of the timer used to get something like an OnIdle callback function to update the simulation. */
  virtual void timerEvent(QTimerEvent* event);

private:

  static QString getAppPath(const char* argv0);
  static unsigned int getAppLocationSum(const QString& appPath);
  static unsigned int getSystemTime();

  class LoadedModule : public QLibrary
  {
  public:
    SimRobot::Module* module;
    int flags;
    bool compiled;
    typedef SimRobot::Module* (*CreateModuleProc)(SimRobot::Application&);
    CreateModuleProc createModule;

    LoadedModule(const QString& name, int flags) : QLibrary(name), module(0), flags(flags), compiled(false) {}
  };

  QAction* fileOpenAct;
  QAction* fileCloseAct;
  QAction* fileExitAct;
  QAction* toolbarOpenAct;
  QAction* simResetAct;
  QAction* simStartAct;
  QAction* simStepAct;

  QMenu* fileMenu;
  QSignalMapper recentFileMapper;
  QMenu* recentFileMenu;
  QMenu* viewMenu;
  QMenu* viewUpdateRateMenu;
  QSignalMapper viewUpdateRateMapper;
  QActionGroup* viewUpdateRateActionGroup;
  QMenu* addonMenu;
  QSignalMapper addonMapper;
  QMenu* helpMenu;

  QMenuBar* menuBar;
  QToolBar* toolBar;
  StatusBar* statusBar;

  QString appPath;
  QString appString;

  QSettings settings;
  QSettings layoutSettings;
  QStringList recentFiles;

  bool opened, compiled, running, performStep;
  bool layoutRestored;
  int guiUpdateRate;
  unsigned int lastGuiUpdate;
  QString filePath; /**< the path to the currently opened file */

  class RegisteredModule
  {
  public:
    QString name;
    QString displayName;
    int flags;

    RegisteredModule(const QString& name, const QString& displayName, int flags) : name(name), displayName(displayName), flags(flags) {}
  };

  QMap<QString, RegisteredModule> registeredModules; /**< suggested modules (a.k.a. addons) */
  QStringList manuallyLoadedModules; /**< modules (a.k.a. addons) that were loaded manually */
  QList<LoadedModule*> loadedModules;
  QHash<QString, LoadedModule*> loadedModulesByName; /**< all loaded modules associated to the currently opened file */

  QDockWidget* activeDockWidget;
  QMenu* dockWidgetFileMenu;
  QMenu* dockWidgetEditMenu;
  QMenu* dockWidgetUserMenu;
  QMenu* moduleUserMenu;

  SceneGraphDockWidget* sceneGraphDockWidget;
  QStringList openedObjects;
  QMap<QString, RegisteredDockWidget*> openedObjectsByName;

  virtual bool registerObject(const SimRobot::Module& module, SimRobot::Object& object, const SimRobot::Object* parent, int flags);
  virtual bool unregisterObject(const SimRobot::Object& object);
  virtual SimRobot::Object* resolveObject(const QString& fullName, int kind);
  virtual SimRobot::Object* resolveObject(const QVector<QString>& parts, const SimRobot::Object* parent, int kind);
  virtual int getObjectChildCount(const SimRobot::Object& object);
  virtual SimRobot::Object* getObjectChild(const SimRobot::Object& object, int index);
  virtual bool addStatusLabel(const SimRobot::Module& module, SimRobot::StatusLabel* statusLabel);
  virtual bool registerModule(const SimRobot::Module& module, const QString& displayName, const QString& name, int flags);
  virtual bool loadModule(const QString& name);
  virtual bool openObject(const SimRobot::Object& object);
  virtual bool closeObject(const SimRobot::Object& object);
  virtual bool selectObject(const SimRobot::Object& object);
  virtual void showWarning(const QString& title, const QString& message);
  virtual void setStatusMessage(const QString& message);
  virtual const QString& getFilePath() const {return filePath;}
  virtual const QString& getAppPath() const {return appPath;}
  virtual QSettings& getSettings() {return settings;}
  virtual QSettings& getLayoutSettings() {return layoutSettings;}

  virtual void closeEvent(QCloseEvent* event);
  virtual void dragEnterEvent(QDragEnterEvent* event);
  virtual void dropEvent(QDropEvent* event);
  virtual void keyPressEvent(QKeyEvent* event);
  virtual void keyReleaseEvent(QKeyEvent* event);
  virtual QMenu* createPopupMenu();

  bool loadModule(const QString& name, bool manually);
  void unloadModule(const QString& name);
  bool compileModules();
  void updateViewMenu(QMenu* menu);
  void addToolBarButtonsFromMenu(QMenu* menu, QToolBar* toolBar, bool addSeparator);

public slots:
  void openFile(const QString& fileName);

private slots:
  void unlockLayout();

  void updateFileMenu();
  void updateRecentFileMenu();
  void updateViewMenu();
  void updateAddonMenu();
  void updateMenuAndToolBar();

  void setGuiUpdateRate(int rate);

  void open();
  bool closeFile();
  void help();
  void about();
  void loadAddon(const QString& name);

  void openObject(const QString& fullName, const SimRobot::Module* module, SimRobot::Object* object, int flags);
  void closeObject(const QString& fullName);
  void closedObject(const QString& fullName);
  void visibilityChanged(bool visible);

  void focusChanged(QWidget *old, QWidget* now);

public slots:
  void simReset();
  void simStart();
  void simStep();
  void simStop();
};
