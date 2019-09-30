/**
* @file SimRobot/MainWindow.cpp
* Implementation of the main window of SimRobot
* @author Colin Graf
*/

#include "MainWindow.h"
#include "SceneGraphDockWidget.h"
#include "RegisteredDockWidget.h"
#include "StatusBar.h"

#include <QApplication>
#include <QAction>
#include <QMenu>
#include <QMenuBar>
#include <QToolBar>
#include <QFileInfo>
#include <QFileDialog>
#include <QMessageBox>
#include <QMimeData>
#include <QLibrary>
#include <QToolButton>
#include <QCloseEvent>
#include <QUrl>
#include <QTimer>
#ifdef WINDOWS
#include <Windows.h>
#elif defined MACOS
#include <mach/mach_time.h>
#include "Helper.h"
#else
#include <ctime>
#endif
#include <iostream>

#define QDOCKWIDGET_STYLE ""
#define QDOCKWIDGET_STYLE_FOCUS "QDockWidget {font-weight: bold;}"

SimRobot::Application* MainWindow::application;

#ifdef WINDOWS
#define PATH_SEPARATOR "\\"
#else
#define PATH_SEPARATOR "/"
#endif

MainWindow::MainWindow(int argc, char *argv[]) :
  appPath(getAppPath(argv[0])),
  appString(QString("SimRobot" PATH_SEPARATOR "%1").arg(getAppLocationSum(appPath))),
  settings("B-Human", appString),
  layoutSettings("B-Human", appString + PATH_SEPARATOR "Layouts"),
  recentFiles(settings.value("RecentFiles").toStringList())
{
  application = this;

  // initialize main window attributes
  setWindowTitle(tr("SimRobot"));
  setWindowIcon(QIcon(":/Icons/SimRobot.png"));
  setAcceptDrops(true);
  setDockNestingEnabled(true);
  setAttribute(Qt::WA_AlwaysShowToolTips);
  setCorner(Qt::TopLeftCorner, Qt::LeftDockWidgetArea);
  resize(600, 400);
  connect(qApp, SIGNAL(focusChanged(QWidget*,QWidget*)), this, SLOT(focusChanged(QWidget*,QWidget*)));

  // create actions
  fileOpenAct = new QAction(QIcon(":/Icons/folder_page.png"), tr("&Open..."), this);
  fileOpenAct->setShortcut(QKeySequence(QKeySequence::Open));
  fileOpenAct->setStatusTip(tr("Open an existing scene file"));
  connect(fileOpenAct, SIGNAL(triggered()), this, SLOT(open()));

  fileCloseAct = new QAction(tr("&Close"), this);
  fileCloseAct->setStatusTip(tr("Close the scene"));
  fileCloseAct->setEnabled(false);
  connect(fileCloseAct, SIGNAL(triggered()), this, SLOT(closeFile()));

  fileExitAct = new QAction(/*QIcon(":/Icons/Exit.png"), */tr("E&xit"), this);
#ifdef WINDOWS
  fileExitAct->setShortcut(QKeySequence(Qt::ALT + Qt::Key_F4));
#elif defined MACOS
  fileExitAct->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_Q));
#else
  fileExitAct->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_Q, Qt::ALT + Qt::Key_F4));
#endif
  fileExitAct->setStatusTip(tr("Exit the application"));
  connect(fileExitAct, SIGNAL(triggered()), this, SLOT(close()));

  toolbarOpenAct = new QAction(QIcon(":/Icons/folder_page.png"), tr("&Open..."), this);
  toolbarOpenAct->setStatusTip(tr("Open an existing file"));
  connect(toolbarOpenAct, SIGNAL(triggered()), this, SLOT(open()));

  simResetAct = new QAction(QIcon(":/Icons/control_start_blue.png"), tr("&Reset"), this);
  simResetAct->setStatusTip(tr("Reset the simulation to the beginning"));
  simResetAct->setShortcut(QKeySequence(Qt::SHIFT + Qt::Key_F5));
  simResetAct->setEnabled(false);
  connect(simResetAct, SIGNAL(triggered()), this, SLOT(simReset()));

  simStartAct = new QAction(QIcon(":/Icons/control_play_blue.png"), tr("&Start"), this);
  simStartAct->setStatusTip(tr("Start or stop the simulation"));
  simStartAct->setShortcut(QKeySequence(Qt::Key_F5));
  simStartAct->setCheckable(true);
  simStartAct->setEnabled(false);
  connect(simStartAct, SIGNAL(triggered()), this, SLOT(simStart()));

  simStepAct = new QAction(QIcon(":/Icons/control_step_blue.png"), tr("&Step"), this);
  simStepAct->setStatusTip(tr("Execute a single simulation step"));
  simStepAct->setShortcut(QKeySequence(Qt::Key_F8));
  simStepAct->setEnabled(false);
  connect(simStepAct, SIGNAL(triggered()), this, SLOT(simStep()));

  // add props
  toolBar = addToolBar(tr("&Toolbar"));
  toolBar->setObjectName("Toolbar");
  toolBar->setIconSize(QSize(16, 16));
#ifdef MACOS
  setWindowTitleTransparent(this);
  setUnifiedTitleAndToolBarOnMac(true);
  toolBar->setFloatable(false);
  toolBar->setMovable(false);
  toolBar->setFixedHeight(toolBar->height() * 6 / 5);
#endif

  menuBar = new QMenuBar(this);
  setMenuBar(menuBar);

  statusBar = new StatusBar(this);
  setStatusBar(statusBar);

  // create menus
  fileMenu = new QMenu(tr("&File"), this);
  connect(&recentFileMapper, SIGNAL(mapped(const QString&)), this, SLOT(openFile(const QString&)));
  connect(fileMenu, SIGNAL(aboutToShow()), this, SLOT(updateFileMenu()));
  updateFileMenu();

  recentFileMenu = new QMenu(tr("&File"), this);
  connect(recentFileMenu, SIGNAL(aboutToShow()), this, SLOT(updateRecentFileMenu()));
  toolbarOpenAct->setMenu(recentFileMenu);

  connect(&viewUpdateRateMapper, SIGNAL(mapped(int)), this, SLOT(setGuiUpdateRate(int)));
  viewMenu = new QMenu(tr("&View"), this);
  connect(viewMenu, SIGNAL(aboutToShow()), this, SLOT(updateViewMenu()));
  updateViewMenu();

  addonMenu = new QMenu(tr("&Add-ons"), this);
  connect(&addonMapper, SIGNAL(mapped(const QString&)), this, SLOT(loadAddon(const QString&)));
  connect(addonMenu, SIGNAL(aboutToShow()), this, SLOT(updateAddonMenu()));
  updateAddonMenu();

  helpMenu = new QMenu(tr("&Help"), this);
  QAction* action;
  connect(action = helpMenu->addAction(tr("&About...")), SIGNAL(triggered()), this, SLOT(about()));
  action->setMenuRole(QAction::AboutRole);
  action->setStatusTip(tr("Show the application's About box"));
  connect(action = helpMenu->addAction(tr("About &Qt...")), SIGNAL(triggered()), qApp, SLOT(aboutQt()));
  action->setMenuRole(QAction::AboutQtRole);
  action->setStatusTip(tr("Show the Qt library's About box"));

  updateMenuAndToolBar();
}

QString MainWindow::getAppPath(const char* argv0)
{
#ifdef WINDOWS
  char fileName[_MAX_PATH];
  char longFileName[_MAX_PATH];
  GetModuleFileNameA(GetModuleHandleA(0), fileName, _MAX_PATH);
  GetLongPathNameA(fileName, longFileName, _MAX_PATH);
  return QString(longFileName);
#else
  return QDir::cleanPath(*argv0 == '/' ? QObject::tr(argv0) : QDir::root().current().path() + "/" + argv0);
#endif
}

unsigned int MainWindow::getAppLocationSum(const QString& appPath)
{
  unsigned int sum = 0;
#ifdef MACOS
  QString path = appPath;
  for(int i = 0; i < 5; ++i)
    path = QFileInfo(path).dir().path();

  // HACK: use old layout files
  path.replace("macOS", "OSX");
  path.replace("SimRobot-caaaeucszhgdzjbjfxcuvrrhfobi", "SimRobot-coxkvyjsooixypajomfxlpxuegqc");
#else
  const QString& path(QFileInfo(QFileInfo(appPath).dir().path()).dir().path());
#endif
  const QChar* data = path.data();
  const QChar* dataEnd = data + path.count();
  for(; data < dataEnd; ++data)
  {
    sum ^= sum >> 16;
    sum <<= 1;
    sum += data->toLower().unicode();
  }
  return sum;
}

unsigned int MainWindow::getSystemTime()
{
#ifdef WINDOWS
  return GetTickCount();
#elif defined MACOS
  static mach_timebase_info_data_t info = {0, 0};
  if(info.denom == 0)
    mach_timebase_info(&info);
  return static_cast<unsigned>(mach_absolute_time() * (info.numer / info.denom) / 1000000);
#else
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<unsigned int>(ts.tv_sec * 1000 + ts.tv_nsec / 1000000l);
#endif
}

bool MainWindow::registerObject(const SimRobot::Module& module, SimRobot::Object& object, const SimRobot::Object* parent, int flags)
{
  if(sceneGraphDockWidget)
    sceneGraphDockWidget->registerObject(&module, &object, parent, flags);

  RegisteredDockWidget* dockWidget = openedObjectsByName.value(object.getFullName());
  if(dockWidget && !dockWidget->hasWidget())
  {
    SimRobot::Widget* widget = object.createWidget();
    if(widget)
    {
      if(flags & SimRobot::Flag::verticalTitleBar)
        dockWidget->setFeatures(dockWidget->features() | QDockWidget::DockWidgetVerticalTitleBar);
      dockWidget->setStyleSheet(dockWidget == activeDockWidget ? QDOCKWIDGET_STYLE_FOCUS : QDOCKWIDGET_STYLE);
      dockWidget->setWidget(widget, &module, &object, flags);
      QWidget* qwidget = widget->getWidget();
      Q_ASSERT(qwidget->parent() == dockWidget);
      dockWidget->setFocusProxy(qwidget);
      if(sceneGraphDockWidget)
        sceneGraphDockWidget->setOpened(&object, true);

      if(dockWidget == activeDockWidget)
        updateMenuAndToolBar();
    }
  }

  return true;
}

bool MainWindow::unregisterObject(const SimRobot::Object& object)
{
  return sceneGraphDockWidget ? sceneGraphDockWidget->unregisterObject(&object) : false;
}

SimRobot::Object* MainWindow::resolveObject(const QString& fullName, int kind)
{
  return sceneGraphDockWidget ? sceneGraphDockWidget->resolveObject(fullName, kind) : 0;
}

SimRobot::Object* MainWindow::resolveObject(const QVector<QString>& parts, const SimRobot::Object* parent, int kind)
{
  return sceneGraphDockWidget ? sceneGraphDockWidget->resolveObject(parent, parts, kind) : 0;
}

int MainWindow::getObjectChildCount(const SimRobot::Object& object)
{
  return sceneGraphDockWidget ? sceneGraphDockWidget->getObjectChildCount(&object) : 0;
}

SimRobot::Object* MainWindow::getObjectChild(const SimRobot::Object& object, int index)
{
  return sceneGraphDockWidget ? sceneGraphDockWidget->getObjectChild(&object, index) : 0;
}

bool MainWindow::addStatusLabel(const SimRobot::Module& module, SimRobot::StatusLabel* statusLabel)
{
  if(!statusLabel)
    return false;
  statusBar->addLabel(&module, statusLabel);
  return true;
}

bool MainWindow::registerModule(const SimRobot::Module& module, const QString& displayName, const QString& name, int flags)
{
  registeredModules.insert(name, RegisteredModule(name, displayName, flags));
  LoadedModule* loadedModule = loadedModulesByName.value(name);
  if(loadedModule)
    loadedModule->flags = flags;
  updateAddonMenu();
  return true;
}

bool MainWindow::loadModule(const QString& name)
{
  return loadModule(name, false);
}

bool MainWindow::openObject(const SimRobot::Object& object)
{
  return sceneGraphDockWidget ? sceneGraphDockWidget->activateObject(&object) : false;
}

bool MainWindow::closeObject(const SimRobot::Object& object)
{
  QMap<QString, RegisteredDockWidget*>::iterator it = openedObjectsByName.find(object.getFullName());
  if(it == openedObjectsByName.end())
    return false;
  it.value()->close();
  return true;
}

bool MainWindow::selectObject(const SimRobot::Object& object)
{
  for(LoadedModule* module : loadedModules)
    module->module->selectedObject(object);
  return true;
}

void MainWindow::showWarning(const QString& title, const QString& message)
{
  QMessageBox::warning(this, title, message);
}

void MainWindow::setStatusMessage(const QString& message)
{
  statusBar->setUserMessage(message);
}

void MainWindow::closeEvent(QCloseEvent* event)
{
  if(!closeFile())
  {
    event->ignore();
    return;
  }

  QMainWindow::closeEvent(event);
}

void MainWindow::timerEvent(QTimerEvent* event)
{
  performStep = false;

  for(LoadedModule* loadedModule : loadedModules)
    loadedModule->module->update();

  // update gui
  const unsigned int now = getSystemTime();
  if(!running || now - lastGuiUpdate > static_cast<unsigned int>(guiUpdateRate))
  {
    lastGuiUpdate = now;
    for(RegisteredDockWidget* dockWidget : openedObjectsByName)
      if(dockWidget->isReallyVisible())
        dockWidget->update();
    if(statusBar->isVisible())
      statusBar->update();
  }
  if(!running)
  {
    Q_ASSERT(event->timerId() == timerId);
    killTimer(timerId);
    timerId = 0;
  }
}

void MainWindow::dragEnterEvent(QDragEnterEvent *event)
{
  if(event->mimeData()->hasUrls())
    event->acceptProposedAction();
}

void MainWindow::dropEvent(QDropEvent *event)
{
  for(const QUrl& url : event->mimeData()->urls())
  {
    QString file(url.toLocalFile());
    if(!file.isEmpty())
    {
      openFile(file);
      break;
    }
  }
  event->acceptProposedAction();
}

void MainWindow::keyPressEvent(QKeyEvent* event)
{
  if((event->modifiers() & (Qt::ControlModifier | Qt::ShiftModifier)) == (Qt::ControlModifier | Qt::ShiftModifier))
  {
    int key = event->key();
    if((key >= Qt::Key_0 && key <= Qt::Key_9) || (key >= Qt::Key_A && key <= Qt::Key_Z))
    {
      key -= (key >= Qt::Key_0 && key <= Qt::Key_9) ? Qt::Key_0 : (Qt::Key_A - 11);
      event->accept();
      for(LoadedModule* module : loadedModules)
        module->module->pressedKey(key, true);
      return;
    }
  }
  else if(event->key() == Qt::Key_F11)
  {
    if(isFullScreen())
      showNormal();
    else
      showFullScreen();
  }

  QMainWindow::keyPressEvent(event);
}

void MainWindow::keyReleaseEvent(QKeyEvent* event)
{
  if((event->modifiers() & (Qt::ControlModifier | Qt::ShiftModifier)) == (Qt::ControlModifier | Qt::ShiftModifier))
  {
    int key = event->key();
    if((key >= Qt::Key_0 && key <= Qt::Key_9) || (key >= Qt::Key_A && key <= Qt::Key_Z))
    {
      key -= (key >= Qt::Key_0 && key <= Qt::Key_9) ? Qt::Key_0 : (Qt::Key_A - 11);
      event->accept();
      for(LoadedModule* module : loadedModules)
        module->module->pressedKey(key, false);
      return;
    }
  }

  QMainWindow::keyReleaseEvent(event);
}

QMenu* MainWindow::createPopupMenu()
{
  QMenu* menu = new QMenu();
  updateViewMenu(menu);
  return menu;
}

bool MainWindow::loadModule(const QString& name, bool manually)
{
  if(loadedModulesByName.contains(name))
    return true; // already loaded

  // look for module flags
  int flags = 0;
  {
    QMap<QString, RegisteredModule>::const_iterator i = registeredModules.find(name);
    if(i != registeredModules.end())
      flags = i->flags;
  }

#ifdef WINDOWS
  const QString& moduleName = name;
#elif defined MACOS
  QString moduleName = QFileInfo(application->getAppPath()).dir().path() + "/../Resources/" + name;
#else
  QString moduleName = QFileInfo(appPath).path() + "/lib" + name + ".so";
#endif
  {
    LoadedModule* loadedModule = new LoadedModule(moduleName, flags);
    loadedModule->createModule = reinterpret_cast<LoadedModule::CreateModuleProc>(loadedModule->resolve("createModule"));
    if(!loadedModule->createModule)
    {
      QMessageBox::warning(this, tr("SimRobot"), loadedModule->errorString());
      loadedModule->unload();
      delete loadedModule;
      return false;
    }
    loadedModule->module = loadedModule->createModule(*this);
    Q_ASSERT(loadedModule->module);
    QHash<QString, LoadedModule*>::iterator it = loadedModulesByName.insert(name, loadedModule);
    if(manually)
    {
      loadedModule->compiled = loadedModule->module->compile(); // compile it right now
      if(!loadedModule->compiled)
      {
        loadedModulesByName.erase(it);
        delete loadedModule->module;
        loadedModule->unload();
        delete loadedModule;
        return false;
      }
      manuallyLoadedModules.append(name);
    }
    loadedModules.append(loadedModule);
  }

  // relink modules
  if(manually)
  {
    for(LoadedModule* loadedModule : loadedModules)
      loadedModule->module->link();
  }

  return true;
}

void MainWindow::unloadModule(const QString& name)
{
  LoadedModule* loadedModule = loadedModulesByName.value(name);
  Q_ASSERT(loadedModule);
  Q_ASSERT(loadedModule->compiled);
  SimRobot::Module* module = loadedModule->module;

  // widget "can close" check
  QList<RegisteredDockWidget*> objectsToClose;
  for(RegisteredDockWidget* dockWidget : openedObjectsByName)
    if(dockWidget->getModule() == module)
    {
      if(!dockWidget->canClose())
        return;
      objectsToClose.append(dockWidget);
    }

  // closed opened widgets (from the module)
  for(RegisteredDockWidget* objectToClose : objectsToClose)
  {
    objectToClose->setAttribute(Qt::WA_DeleteOnClose, false);
    objectToClose->close();
    delete objectToClose;
  }

  // remove registered stuff
  if(sceneGraphDockWidget)
    sceneGraphDockWidget->unregisterObjectsFromModule(module);
  statusBar->removeLabelsFromModule(module);

  // unload the module
  delete loadedModule->module;
  loadedModule->unload();
  delete loadedModule;
  loadedModules.removeOne(loadedModule);
  loadedModulesByName.remove(name);
  manuallyLoadedModules.removeOne(name);

  // relink modules
  for(LoadedModule* loadedModule : loadedModules)
    loadedModule->module->link();
}

bool MainWindow::compileModules()
{
  if(compiled)
    return true;

  bool success = true;
  for(int i = 0; i < loadedModules.count(); ++i) // note: list of modules may grow while compiling modules
  {
    LoadedModule* loadedModule = loadedModules[i];
    if(!loadedModule->compiled)
    {
      loadedModule->compiled = loadedModule->module->compile();
      if(!loadedModule->compiled)
        success = false;
    }
  }
  if(!success)
    return false;

  compiled = true;

  // link modules
  for(LoadedModule* loadedModule : loadedModules)
    loadedModule->module->link();
  return true;
}

void MainWindow::updateViewMenu(QMenu* menu)
{
  menu->clear();

  if(viewUpdateRateMenu)
  {
    delete viewUpdateRateMenu;
    delete viewUpdateRateActionGroup;
  }

  viewUpdateRateActionGroup = new QActionGroup(this);
  viewUpdateRateMenu = new QMenu(tr("Update Rate"), this);

  QAction* action = viewUpdateRateMenu->addAction(tr("10 fps"));
  action->setCheckable(true);
  action->setChecked(guiUpdateRate == 100);
  viewUpdateRateActionGroup->addAction(action);
  viewUpdateRateMapper.setMapping(action, 100);
  connect(action, SIGNAL(triggered()), &viewUpdateRateMapper, SLOT(map()));

  action = viewUpdateRateMenu->addAction(tr("20 fps"));
  action->setCheckable(true);
  action->setChecked(guiUpdateRate == 50);
  viewUpdateRateActionGroup->addAction(action);
  viewUpdateRateMapper.setMapping(action, 50);
  connect(action, SIGNAL(triggered()), &viewUpdateRateMapper, SLOT(map()));

  action = viewUpdateRateMenu->addAction(tr("30 fps"));
  action->setCheckable(true);
  action->setChecked(guiUpdateRate == 33);
  viewUpdateRateActionGroup->addAction(action);
  viewUpdateRateMapper.setMapping(action, 33);
  connect(action, SIGNAL(triggered()), &viewUpdateRateMapper, SLOT(map()));

  action = viewUpdateRateMenu->addAction(tr("50 fps"));
  action->setCheckable(true);
  action->setChecked(guiUpdateRate == 20);
  viewUpdateRateActionGroup->addAction(action);
  viewUpdateRateMapper.setMapping(action, 20);
  connect(action, SIGNAL(triggered()), &viewUpdateRateMapper, SLOT(map()));

  action = viewUpdateRateMenu->addAction(tr("Every Frame"));
  action->setCheckable(true);
  action->setChecked(guiUpdateRate == 0);
  viewUpdateRateActionGroup->addAction(action);
  viewUpdateRateMapper.setMapping(action, 0);
  connect(action, SIGNAL(triggered()), &viewUpdateRateMapper, SLOT(map()));

  menu->addMenu(viewUpdateRateMenu);
  menu->addSeparator();
  //menu->addAction(menuBar->toggleViewAction());
  menu->addAction(toolBar->toggleViewAction());
  menu->addAction(statusBar->toggleViewAction());

  viewUpdateRateMenu->setEnabled(opened);
  if((opened && sceneGraphDockWidget) || openedObjectsByName.count() > 0)
  {
    menu->addSeparator();
    if(opened && sceneGraphDockWidget)
      menu->addAction(sceneGraphDockWidget->toggleViewAction());
    for(const RegisteredDockWidget* dockWidget : openedObjectsByName)
      menu->addAction(dockWidget->toggleViewAction());
  }
}

void MainWindow::updateMenuAndToolBar()
{
  menuBar->clear();
  toolBar->clear();

  if(dockWidgetFileMenu)
  {
    delete dockWidgetFileMenu;
    dockWidgetFileMenu = nullptr;
  }
  if(dockWidgetEditMenu)
  {
    delete dockWidgetEditMenu;
    dockWidgetEditMenu = nullptr;
  }
  if(moduleUserMenu)
  {
    delete moduleUserMenu;
    moduleUserMenu = nullptr;
  }
  if(dockWidgetUserMenu)
  {
    delete dockWidgetUserMenu;
    dockWidgetUserMenu = nullptr;
  }

  RegisteredDockWidget* registeredDockWidget = opened && activeDockWidget ? qobject_cast<RegisteredDockWidget*>(activeDockWidget) : nullptr;

  if(registeredDockWidget)
  {
    dockWidgetFileMenu = registeredDockWidget->createFileMenu();
    dockWidgetEditMenu = registeredDockWidget->createEditMenu();
    dockWidgetUserMenu = registeredDockWidget->createUserMenu();

    // Default is the menu of the module belonging to the current view
    if(registeredDockWidget->getModule())
      moduleUserMenu = registeredDockWidget->getModule()->createUserMenu();

    // Otherwise use the first menu of a module found
    for(const LoadedModule* loadedModule : loadedModules)
      if(!moduleUserMenu)
        moduleUserMenu = loadedModule->module->createUserMenu();
  }

  menuBar->addMenu(fileMenu);

  toolBar->addAction(toolbarOpenAct);
  if(dockWidgetFileMenu)
    addToolBarButtonsFromMenu(dockWidgetFileMenu, toolBar, false);

  toolBar->addSeparator();
  toolBar->addAction(simStartAct);
  toolBar->addAction(simResetAct);
  toolBar->addAction(simStepAct);
  if(opened && sceneGraphDockWidget)
  {
    toolBar->addSeparator();
    toolBar->addAction(sceneGraphDockWidget->toggleViewAction());
  }

  if(dockWidgetEditMenu)
  {
    menuBar->addMenu(dockWidgetEditMenu);
    addToolBarButtonsFromMenu(dockWidgetEditMenu, toolBar, true);
  }
  menuBar->addMenu(viewMenu);
  menuBar->addMenu(createSimMenu());

  if(moduleUserMenu)
  {
    menuBar->addMenu(moduleUserMenu);
    addToolBarButtonsFromMenu(moduleUserMenu, toolBar, true);
  }

  if(dockWidgetUserMenu)
  {
    menuBar->addMenu(dockWidgetUserMenu);
    addToolBarButtonsFromMenu(dockWidgetUserMenu, toolBar, true);
  }

  if(opened)
    menuBar->addMenu(addonMenu);

  menuBar->addMenu(helpMenu);

#ifndef LINUX
  QTimer::singleShot(0, toolBar, SLOT(update()));
#endif
}

QMenu* MainWindow::createSimMenu()
{
  QMenu* simMenu = new QMenu(tr("&Simulation"), this);
  simMenu->addAction(simStartAct);
  simMenu->addAction(simResetAct);
  simMenu->addAction(simStepAct);
  return simMenu;
}

void MainWindow::addToolBarButtonsFromMenu(QMenu* menu, QToolBar* toolBar, bool addSeparator)
{
  for(QAction* action : menu->actions())
  {
    if(!action->icon().isNull())
    {
      if(addSeparator)
        toolBar->addSeparator();
      toolBar->addAction(action);
      if(action->menu())
        qobject_cast<QToolButton*>(toolBar->widgetForAction(action))->setPopupMode(QToolButton::InstantPopup);
    }
    addSeparator = action->isSeparator();
  }
}

void MainWindow::updateFileMenu()
{
  fileMenu->clear();
  fileMenu->addAction(fileOpenAct);
  fileMenu->addAction(fileCloseAct);
  if(dockWidgetFileMenu)
  {
    fileMenu->addSeparator();
    for(QAction* action : dockWidgetFileMenu->actions())
      fileMenu->addAction(action);
  }

  if(recentFiles.size() > 0)
  {
    fileMenu->addSeparator();
    char shortcut = '1';
    for(QStringList::const_iterator i = recentFiles.begin(), end = recentFiles.end(); i != end; ++i)
    {
      const QString& file(*i);
      QAction* action = fileMenu->addAction("&" + QString(shortcut++) + " " + QFileInfo(file).fileName());
      connect(action, SIGNAL(triggered()), &recentFileMapper, SLOT(map()));
      recentFileMapper.setMapping(action, file);
    }
  }
#ifndef MACOS
  fileMenu->addSeparator();
  fileMenu->addAction(fileExitAct);
#endif
}

void MainWindow::updateRecentFileMenu()
{
  recentFileMenu->clear();
  char shortcut = '1';
  for(QStringList::const_iterator i = recentFiles.begin(), end = recentFiles.end(); i != end; ++i)
  {
    const QString& file(*i);
    QAction* action = recentFileMenu->addAction("&" + QString(shortcut++) + " " + QFileInfo(file).fileName());
    connect(action, SIGNAL(triggered()), &recentFileMapper, SLOT(map()));
    recentFileMapper.setMapping(action, file);
  }
}

void MainWindow::updateViewMenu()
{
  updateViewMenu(viewMenu);
}

void MainWindow::updateAddonMenu()
{
  addonMenu->clear();
  for(const RegisteredModule& info : registeredModules)
  {
    QAction* action = addonMenu->addAction(info.displayName);
    action->setCheckable(true);
    if(loadedModulesByName.contains(info.name))
      action->setChecked(true);
    connect(action, SIGNAL(triggered()), &addonMapper, SLOT(map()));
    addonMapper.setMapping(action, info.name);
  }
}

void MainWindow::setGuiUpdateRate(int rate)
{
  guiUpdateRate = rate;
}

void MainWindow::open()
{
  QString fileName = QFileDialog::getOpenFileName(this,
    tr("Open File"), settings.value("OpenDirectory", "").toString(), tr("Robot Simulation Files (*.ros2)"));

  if(fileName.isEmpty())
    return;
  settings.setValue("OpenDirectory", QFileInfo(fileName).dir().path());

  openFile(fileName);
}

void MainWindow::openFile(const QString& fileName)
{
  closeFile();

  // get full file path
  QFileInfo fileInfo(fileName);
  filePath = fileInfo.absoluteDir().canonicalPath() + '/' + fileInfo.fileName();

  // remove file path from recent file list
  recentFiles.removeAll(filePath);

  // check if file exists
  if(!fileInfo.exists())
  {
    settings.setValue("RecentFiles", recentFiles);
    QMessageBox::warning(this, tr("SimRobot"), tr("Cannot open file %1.").arg(fileName));
    return;
  }
  opened = true;

  // add file path to recent file list
  const QString& baseName = fileInfo.baseName();
  recentFiles.prepend(filePath);
  while(recentFiles.count() > 8)
    recentFiles.removeLast();
  settings.setValue("RecentFiles", recentFiles);
  setWindowTitle(baseName + " - " + tr("SimRobot"));

  // open layout settings
  layoutSettings.beginGroup(baseName);

  // create scene graph window
  sceneGraphDockWidget = new SceneGraphDockWidget(createSimMenu(), this);
  sceneGraphDockWidget->setStyleSheet(QDOCKWIDGET_STYLE);
  connect(sceneGraphDockWidget, SIGNAL(visibilityChanged(bool)), this, SLOT(visibilityChanged(bool)));
  addDockWidget(Qt::TopDockWidgetArea, sceneGraphDockWidget);
  connect(sceneGraphDockWidget, SIGNAL(activatedObject(const QString&, const SimRobot::Module*, SimRobot::Object*, int)), this, SLOT(openObject(const QString&, const SimRobot::Module*, SimRobot::Object*, int)));
  connect(sceneGraphDockWidget, SIGNAL(deactivatedObject(const QString&)), this, SLOT(closeObject(const QString&)));

  // load all other windows
  const QVariant& openedObjectsVar = layoutSettings.value("OpenedObjects");
  if(openedObjectsVar.isValid())
  {
    for(const QString& object : openedObjectsVar.toStringList())
      openObject(object, 0, 0, 0);
  }
  restoreGeometry(layoutSettings.value("Geometry").toByteArray());
  restoreState(layoutSettings.value("WindowState").toByteArray());
  statusBar->setVisible(layoutSettings.value("ShowStatus", true).toBool());
  manuallyLoadedModules = layoutSettings.value("LoadedModules").toStringList();
  guiUpdateRate = layoutSettings.value("GuiUpdateRate", -1).toInt();
  if(guiUpdateRate < 0)
    guiUpdateRate = 100;
  QAction* action = qobject_cast<QAction*>(viewUpdateRateMapper.mapping(guiUpdateRate));
  if(action)
    action->setChecked(true);

  // load core module
  Q_ASSERT(!compiled);
  loadModule("SimRobotCore2");

  for(int i = 0; i < manuallyLoadedModules.size();)
    if(loadModule(manuallyLoadedModules[i]))
      ++i;
    else
      manuallyLoadedModules.removeAt(i);

  compileModules();

  // restore focus
  layoutRestored = true;
  QVariant activeObject = layoutSettings.value("ActiveObject");
  if(activeObject.isValid())
  {
    QDockWidget* activeDockWidget = findChild<QDockWidget*>(activeObject.toString());
    if(activeDockWidget)
    {
      activeDockWidget->raise();
      activeDockWidget->activateWindow();
      activeDockWidget->setFocus();
    }
  }
  if(!activeDockWidget)
    updateMenuAndToolBar();

  // gui update
  fileCloseAct->setEnabled(true);
  simResetAct->setEnabled(true);
  simStartAct->setEnabled(true);
  simStepAct->setEnabled(true);

  // start simulation
  if(compiled && layoutSettings.value("Run", true).toBool())
    simStart();
}

void MainWindow::unlockLayout()
{
  for(QMap<QString, RegisteredDockWidget*>::iterator it = openedObjectsByName.begin(), end = openedObjectsByName.end(); it != end; ++it)
  {
    RegisteredDockWidget* widget = it.value();
    widget->setMinimumSize(QSize(0, 0));
  }
  sceneGraphDockWidget->setMinimumSize(QSize(0, 0));
}

bool MainWindow::closeFile()
{
  // "can close" check
  for(RegisteredDockWidget* dockWidget : openedObjectsByName)
    if(!dockWidget->canClose())
      return false;

  // start closing...
  const bool wasOpened = opened;
  opened = false;
  filePath.clear();
  layoutRestored = false;

  // save layout
  if(wasOpened)
  {
    layoutSettings.setValue("Geometry", saveGeometry());
    layoutSettings.setValue("WindowState", saveState());
    layoutSettings.setValue("ShowStatus", statusBar->isVisible());
    layoutSettings.setValue("OpenedObjects", openedObjects);
    layoutSettings.setValue("ActiveObject", activeDockWidget ? QVariant(activeDockWidget->objectName()) : QVariant());
    layoutSettings.setValue("LoadedModules", manuallyLoadedModules);
    layoutSettings.setValue("Run", running);
    layoutSettings.setValue("GuiUpdateRate", guiUpdateRate == 100 ? -1 : guiUpdateRate);
  }

  // delete menus from active window
  if(activeDockWidget)
    activeDockWidget = nullptr;

  updateMenuAndToolBar();
  setFocus();

  // close opened windows
  if(sceneGraphDockWidget)
  {
    delete sceneGraphDockWidget;
    sceneGraphDockWidget = nullptr;
  }
  for(RegisteredDockWidget* dockWidget : openedObjectsByName)
    delete dockWidget;
  openedObjects.clear();
  openedObjectsByName.clear();

  // remove registered status lables and modules
  statusBar->removeAllLabels();
  registeredModules.clear();

  // unload all modules
  for(LoadedModule* loadedModule : loadedModules)
  {
    delete loadedModule->module;
    loadedModule->unload();
    delete loadedModule;
  }
  loadedModules.clear();
  loadedModulesByName.clear();
  manuallyLoadedModules.clear();
  registeredModules.clear();

  //
  if(wasOpened)
    layoutSettings.endGroup();

  // reset gui
  if(wasOpened)
  {
    fileCloseAct->setEnabled(false);
    simResetAct->setEnabled(false);
    simStartAct->setEnabled(false);
    simStepAct->setEnabled(false);
    simStartAct->setChecked(false);
    viewUpdateRateMenu->setEnabled(false);
    setWindowTitle(tr("SimRobot"));
    statusBar->setUserMessage(QString());
    compiled = false;
    running = false;
    performStep = false;
  }

  return true;
}

void MainWindow::simReset()
{
  // "can close" check
  for(RegisteredDockWidget* dockWidget : openedObjectsByName)
    if(!dockWidget->canClose())
      return;

  // start resetting
  QString openedFilePath = filePath;
  const bool wasRunning = running || !compiled;
  QString activeObject;
  if(activeDockWidget)
    activeObject = activeDockWidget->objectName();
  if(running)
    simStart(); // stop

  filePath.clear(); // modules may read the filePath to distinguish between user request window closing and document closing / resetting
  layoutRestored = false;

  // delete menus from active window
  if(activeDockWidget)
    activeDockWidget = nullptr;
  updateMenuAndToolBar();
  setFocus();

  // remove all registered status labes and modules and most registered objects
  QSet<const SimRobot::Module*> ignoredModules;
  if(sceneGraphDockWidget)
    for(LoadedModule* loadedModule : loadedModules)
    {
      if(loadedModule->flags & SimRobot::Flag::ignoreReset)
        ignoredModules.insert(loadedModule->module);
      else
        sceneGraphDockWidget->unregisterObjectsFromModule(loadedModule->module);
    }
  statusBar->removeAllLabels();
  registeredModules.clear();

  // destroy most widgets (just the widgets while keeping the docking frame intact)
  for(RegisteredDockWidget* dockWidget : openedObjectsByName)
  {
    const SimRobot::Module* module = dockWidget->getModule();
    if(module && ignoredModules.contains(module))
      continue;
    dockWidget->setWidget(0, 0, 0, 0);
  }

  // unload all modules
  for(LoadedModule* loadedModule : loadedModules)
  {
    if(loadedModule->flags & SimRobot::Flag::ignoreReset)
      continue;
    delete loadedModule->module;
    loadedModule->module = nullptr;
    loadedModule->compiled = false;
  }
  compiled = false;
  filePath = openedFilePath;

  // reload all modules
  for(LoadedModule* loadedModule : loadedModules)
  {
    if(loadedModule->module)
      continue;
    loadedModule->module = loadedModule->createModule(*this);
    Q_ASSERT(loadedModule->module);
  }

  // recompile modules
  compileModules();

  // restore focus
  layoutRestored = true;
  if(!activeObject.isEmpty())
  {
    QDockWidget* activeDockWidget = findChild<QDockWidget*>(activeObject);
    if(activeDockWidget)
    {
      activeDockWidget->raise();
      activeDockWidget->activateWindow();
      activeDockWidget->setFocus();
    }
  }
  if(!activeDockWidget)
    updateMenuAndToolBar();

  // start!?
  if(compiled && wasRunning)
    simStart();
}

void MainWindow::simStart()
{
  simStartAct->setChecked(false);
  if(running)
    running = false;
  else
  {
    if(!compileModules())
      return;
    running = true;
    simStartAct->setChecked(true);
    if(!timerId)
      timerId = startTimer(0);
  }
}

void MainWindow::simStep()
{
  if(running)
    simStart(); // stop
  performStep = true;
  if(!timerId)
    timerId = startTimer(0);
}

void MainWindow::simStop()
{
  simStartAct->setChecked(false);
  running = false;
}

void MainWindow::about()
{
  QMessageBox::about(this, tr("About SimRobot"),
    tr("<b>SimRobot</b> 2011 (B-Human Edition)<br><br>\
Authors:\
<blockquote>Tim Laue<br>\
Thomas Röfer<br>\
Kai Spiess<br>\
Dennis Pachur<br>\
Colin Graf<br>\
Thijs Jeffry de Haas<br>\
</blockquote>\
German Research Center for Artificial Intelligence (DFKI)<br>University of Bremen<br><br>\
Icons by Mark James <a href=\"http://www.famfamfam.com/lab/icons/silk/\">http://www.famfamfam.com/lab/icons/silk/</a>"));
}

void MainWindow::loadAddon(const QString& name)
{
  if(loadedModulesByName.contains(name))
    unloadModule(name);
  else
    loadModule(name, true);
}

void MainWindow::openObject(const QString& fullName, const SimRobot::Module* module, SimRobot::Object* object, int flags)
{
  RegisteredDockWidget* dockWidget = openedObjectsByName.value(fullName);

  if(dockWidget && object && (!dockWidget->getObject() || dockWidget->getObject()->getKind() != object->getKind()))
    dockWidget = nullptr;
  if(dockWidget)
  {
    dockWidget->setVisible(true);
    dockWidget->raise();
    dockWidget->activateWindow();
    dockWidget->setFocus();
    return;
  }

  SimRobot::Widget* widget = object ? object->createWidget() : 0;
  if(object && !widget)
  {
    // the object does not have a widget
    return;
  }

  dockWidget = new RegisteredDockWidget(fullName, this);
  connect(dockWidget, SIGNAL(closedContextMenu()), this, SLOT(updateMenuAndToolBar()));
  if(flags & SimRobot::Flag::verticalTitleBar)
    dockWidget->setFeatures(dockWidget->features() | QDockWidget::DockWidgetVerticalTitleBar);
  dockWidget->setStyleSheet(dockWidget == activeDockWidget ? QDOCKWIDGET_STYLE_FOCUS : QDOCKWIDGET_STYLE);
  connect(dockWidget, SIGNAL(visibilityChanged(bool)), this, SLOT(visibilityChanged(bool)));
  dockWidget->setAttribute(Qt::WA_DeleteOnClose);
  dockWidget->setWindowTitle(fullName);
  dockWidget->setObjectName(fullName);
  addDockWidget(Qt::TopDockWidgetArea, dockWidget);
  dockWidget->setFloating(true);
  if(widget)
  {
    dockWidget->setWidget(widget, module, object, flags);
    QWidget* qwidget = widget->getWidget();
    Q_ASSERT(qwidget->parent() == dockWidget);
    dockWidget->setFocusProxy(qwidget);
  }

  Q_ASSERT(openedObjectsByName.value(fullName) == 0);
  openedObjectsByName.insert(fullName, dockWidget);
  openedObjects.append(fullName);
  connect(dockWidget, SIGNAL(closedObject(const QString&)), this, SLOT(closedObject(const QString&)));
  if(sceneGraphDockWidget && object)
    sceneGraphDockWidget->setOpened(object, true);

  if(layoutRestored)
  {
    dockWidget->setVisible(true);
    dockWidget->raise();
    dockWidget->activateWindow();
    dockWidget->setFocus();
  }
}

void MainWindow::closeObject(const QString& fullName)
{
  RegisteredDockWidget* dockWidget = openedObjectsByName.value(fullName);
  if(dockWidget)
    dockWidget->close();
}

void MainWindow::closedObject(const QString& fullName)
{
  const RegisteredDockWidget* dockWidget = openedObjectsByName.value(fullName);
  if(dockWidget)
  {
    if(dockWidget == activeDockWidget)
    {
      activeDockWidget = nullptr;
      updateMenuAndToolBar(); // delete menus from active window
    }
    openedObjectsByName.remove(fullName);
    openedObjects.removeOne(fullName);
    if(sceneGraphDockWidget)
      sceneGraphDockWidget->setOpened(dockWidget->getObject(), false);
  }
}

void MainWindow::visibilityChanged(bool visible)
{
  if(visible && layoutRestored)
  {
    QDockWidget* dockWidget = qobject_cast<QDockWidget*>(sender());
    if(dockWidget)
    {
      if(dockWidget->isFloating())
      {
        dockWidget->raise();
        dockWidget->activateWindow();
      }
      dockWidget->setFocus();
    }
  }
}

void MainWindow::focusChanged(QWidget *old, QWidget* now)
{
  if(!layoutRestored)
    return;

  QWidget* newActive = now;
  while(newActive)
  {
    QWidget* parent = newActive->parentWidget();
    if(parent == this)
      break;
    newActive = parent;
  }

  QDockWidget* newDockWidget = newActive ? qobject_cast<QDockWidget*>(newActive) : 0;
  if(newDockWidget == activeDockWidget)
    return;

  if(!newDockWidget && activeDockWidget)
    if(activeDockWidget->isVisible())
      return;

  if(activeDockWidget)
  {
    activeDockWidget->setStyleSheet(QDOCKWIDGET_STYLE);

    const RegisteredDockWidget* regDockWidget = qobject_cast<RegisteredDockWidget*>(activeDockWidget);
    if(sceneGraphDockWidget && regDockWidget)
    {
      sceneGraphDockWidget->setActive(regDockWidget->getObject(), false);
    }
  }

  activeDockWidget = newDockWidget;
  if(activeDockWidget)
  {
    activeDockWidget->setStyleSheet(QDOCKWIDGET_STYLE_FOCUS);

    const RegisteredDockWidget* regDockWidget = qobject_cast<RegisteredDockWidget*>(activeDockWidget);
    if(sceneGraphDockWidget && regDockWidget)
    {
      sceneGraphDockWidget->setActive(regDockWidget->getObject(), true);
    }
    if(activeDockWidget->isFloating())
    {
      // Set focus to the main window, so that the active window does not change when the focus returns to the main window.
      // Otherwise it would not be possible to use the customized menu or toolbar.
      setFocus();
    }
  }
  updateMenuAndToolBar();
}
