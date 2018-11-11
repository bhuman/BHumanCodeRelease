
#pragma once

#include <QDockWidget>
#include <QSet>
#include <QHash>
#include <QTreeWidgetItem>

#include "SimRobot.h"

class SceneGraphDockWidget : public QDockWidget
{
  Q_OBJECT

public:
  SceneGraphDockWidget(QMenu* contextMenu, QWidget* parent);
  ~SceneGraphDockWidget();

  void registerObject(const SimRobot::Module* module, SimRobot::Object* object, const SimRobot::Object* parent, int flags);
  void unregisterAllObjects();
  void unregisterObjectsFromModule(const SimRobot::Module* module);
  bool unregisterObject(const SimRobot::Object* object);
  SimRobot::Object* resolveObject(const QString& fullName, int kind);
  SimRobot::Object* resolveObject(const SimRobot::Object* parent, const QVector<QString>& parts, int kind);
  int getObjectChildCount(const SimRobot::Object* object);
  SimRobot::Object* getObjectChild(const SimRobot::Object* object, int index);

  bool activateFirstObject();
  bool activateObject(const SimRobot::Object* object);
  bool setOpened(const SimRobot::Object* object, bool opened);
  bool setActive(const SimRobot::Object* object, bool active);

  QAction* toggleViewAction() const;

signals:
  void activatedObject(const QString& fullName, const SimRobot::Module* module, SimRobot::Object* object, int flags);
  void deactivatedObject(const QString& fullName);

private:
  class RegisteredObject : public QTreeWidgetItem
  {
  public:
    RegisteredObject(const SimRobot::Module* module, SimRobot::Object* object, QTreeWidgetItem* parentItem, int flags) :
        QTreeWidgetItem(parentItem), module(module), object(object), fullName(object->getFullName()), flags(flags), opened(false) {}

    const SimRobot::Module* module;
    SimRobot::Object* object;
    const QString fullName;
    int flags;
    bool opened;
  };

  QMenu* contextMenu;
  QTreeWidget* treeWidget;
  QFont italicFont;
  QFont boldFont;
  QSet<QString> expandedItems;
  QHash<const void*, RegisteredObject*> registeredObjectsByObject;
  QHash<int, QHash<QString, RegisteredObject*>*> registeredObjectsByKindAndName;

  RegisteredObject* clickedItem = nullptr;

  void deleteRegisteredObjectsFromModule(RegisteredObject* registeredObject, const SimRobot::Module* module);
  void deleteRegisteredObject(RegisteredObject* registeredObject);

  void contextMenuEvent(QContextMenuEvent* event) override;

private slots:
  void itemActivated(const QModelIndex& index);
  void itemCollapsed(const QModelIndex& index);
  void itemExpanded(const QModelIndex& index);

  void openOrCloseObject();
  void expandOrCollabseObject();
};
