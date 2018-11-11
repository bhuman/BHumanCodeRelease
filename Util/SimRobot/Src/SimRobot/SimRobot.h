/**
* @file SimRobot/SimRobot.h
* Declaration of an interface to the SimRobot GUI
* @author Colin Graf
*/

#pragma once

class QString;
template<typename T> class QVector;
class QIcon;
class QMenu;
class QSettings;
class QPainter;
class QWidget;

namespace SimRobot
{
  class Widget
  {
  public:
    virtual ~Widget() = default;
    virtual QWidget* getWidget() = 0;
    virtual void update() {}
    virtual bool canClose() {return true;}
    virtual QMenu* createFileMenu() const {return nullptr;}
    virtual QMenu* createEditMenu() const {return nullptr;}
    virtual QMenu* createUserMenu() const {return nullptr;}
    virtual void paint(QPainter& painter) {}
  };

  /**
  * An object that can be added to the scene graph
  */
  class Object
  {
  public:
    virtual ~Object() = default;
    virtual Widget* createWidget() {return nullptr;}

    /** Accesses pathname to the object in the scene graph
    * @return The pathname
    */
    virtual const QString& getFullName() const = 0;

    virtual const QIcon* getIcon() const {return nullptr;}
    virtual int getKind() const {return 0;}

    /**
     * For objects which do not have a widget (i.e. do not override createWidget() and are
     * registered with the SimRobot::Flag::windowless flag), this function provides a callback
     * that is called when the item is clicked in the Scene Graph.
     * This is done in SceneGraphDockWidget::itemActivated(), just before MainWindow::openObject()
     * would have been called if we were dealing with a non-windowless widget item.
     */
    virtual void widgetlessActivationCallback() {}
  };

  /**
  * An object that will be displayed in the status bar
  */
  class StatusLabel
  {
  public:
    virtual ~StatusLabel() = default;
    virtual QWidget* getWidget() = 0;
    virtual void update() {}
  };

 /**
  * Flags that can be used for registering modules and objects
  */
  class Flag
  {
  public:

    // flags for registerObject
    static const int hidden = 0x0001; /**< The object will not be listed in the scene graph */
    static const int verticalTitleBar = 0x0002; /**< The object's dock widget has a vertical title bar */
    static const int windowless = 0x0004; /**< The object does not have a widget but will be listed in the scene graph */
    static const int copy = 0x0008; /**< The object's widget has a "copy" entry in its edit menu that can be used to copy a screenshot of the widget to the clipboard */
    static const int exportAsImage = 0x0010; /**< The object's widget  has an "Export Image" entry in its edit menu that can be used to create a svg using the \c paint method of the widget */
    static const int showParent = 0x0020; /**< When added, the parent will be made visible if hidden */

    // flags for registerModule
    static const int ignoreReset = 0x1000; /**< The module keeps beeing loaded on scene resets */
  };

  /**
  * An interface to the SimRobot module
  */
  class Module
  {
  public:

    /** Virtual destructor */
    virtual ~Module() = default;

    /**
    * Called to initialize the module. In this phase the module can do the following tasks
    *   - registering its own objects to the scene graph (using \c Application::registerObject)
    *   - adding status labels to the GUI (using \c Application::addStatusLabel)
    *   - suggest or load further modules (using \c Application::registerModule, \c Application::loadModule)
    * @return Whether an error occurred while initializing the module or not
    */
    virtual bool compile() {return true;}

    /**
    * Called after all modules have been compiled. In this phase the module can update references to scene graph objects of other modules. (using \c resolveObject)
    */
    virtual void link() {}

    /**
    * Called to perform another simulation step
    */
    virtual void update() {}

    /**
    * A handler that will be called when any modules uses \c Application::selectObject
    */
    virtual void selectedObject(const Object& object) {}

    /**
    * A handler that can be used to implement CTRL + SHIFT shortcuts
    */
    virtual void pressedKey(int key, bool pressed) {}

    /**
    * Create a menu for this module. If 0 is returned, there is no menu.
    */
    virtual QMenu* createUserMenu() const {return nullptr;}
  };

  /**
  * An interface to the SimRobot GUI
  */
  class Application
  {
  public:
    virtual ~Application() = default;
    virtual bool registerObject(const Module& module, Object& object, const Object* parent, int flags = 0) = 0;
    virtual bool unregisterObject(const Object& object) = 0;
    virtual Object* resolveObject(const QString& fullName, int kind = 0) = 0;
    virtual Object* resolveObject(const QVector<QString>& parts, const Object* parent = 0, int kind = 0) = 0;
    virtual int getObjectChildCount(const Object& object) = 0;
    virtual Object* getObjectChild(const Object& object, int index) = 0;
    virtual bool addStatusLabel(const Module& module, StatusLabel* statusLabel) = 0;
    virtual bool registerModule(const Module& module, const QString& displayName, const QString& name, int flags = 0) = 0;
    virtual bool loadModule(const QString& name) = 0;
    virtual bool openObject(const Object& object) = 0;
    virtual bool closeObject(const Object& object) = 0;
    virtual bool selectObject(const Object& object) = 0;
    virtual void showWarning(const QString& title, const QString& message) = 0;
    virtual void setStatusMessage(const QString& message) = 0;
    virtual const QString& getFilePath() const = 0;
    virtual const QString& getAppPath() const = 0;
    virtual QSettings& getSettings() = 0;
    virtual QSettings& getLayoutSettings() = 0;
    virtual bool isSimRunning() = 0;
    virtual void simReset() = 0;
    virtual void simStart() = 0;
    virtual void simStep() = 0;
    virtual void simStop() = 0;
  };
}

#ifdef WINDOWS
#define DLL_EXPORT __declspec(dllexport)
#else
#define DLL_EXPORT
#endif
