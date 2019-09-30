/**
 * DataView.h
 *
 *  Created on: Mar 22, 2012
 *      Author: Arne Böckmann (arneboe@tzi.de)
 */

#pragma once

#include <SimRobot.h>

#include <QIcon>
#include <QObject>
#include <QString>
#include <map>
#include <string>
#include <qtvariantproperty.h>
#include "Platform/Thread.h" // for SYNC
#include "PropertyManager.h"

class RobotConsole;
class InMessage;
class DataWidget;
class QtProperty;
class QtVariantProperty;
class DataWidget;
class QEvent;
class QWidget;
class OutBinaryMessage;
struct TypeInfo;

/**
 * A class which can be used to display streamable data.
 *
 * The view will display any data that is given to it via handleMessage.
 */
class DataView : public SimRobot::Object
{
public:
  /** Creates a new DataView.
   * @param fullName The path to this view in the scene graph.
   * @param console The console object. Used to display error messages.
   * @param repName Name of the streamable data. This is used to generate get requests.
   * @param typeInfo Is used to get the type information while parsing.
   */
  DataView(const QString& fullName, const std::string& repName, RobotConsole& console, const TypeInfo& typeInfo);

  SimRobot::Widget* createWidget() override;
  const QString& getFullName() const override { return theFullName; }
  const QIcon* getIcon() const override { return &theIcon; }

  const QString& getFullName() { return theFullName; }

  /**
   * Disconnects the current widget from the view.
   */
  void removeWidget();

  /**
   * Parses the specified message and displays its contents.
   * @param msg The message containing the data.
   * @param type The type of the data contained within the msg.
   * @param repName Name of the streamable data.
   * @return true if the message has been handled. False otherwise.
   */
  bool handleMessage(InMessage& msg, const std::string& type, const std::string& repName);

  /**
   * If set to true, the view will ignore further updates from the RobotConsole.
   */
  void setIgnoreUpdates(bool value);

  /**
   * Sends the current values to the robot.
   */
  void set();

  /**
   * Unset changes
   */
  void setUnchanged();

  /**
   * Enabled/disable auto-set mode.
   * In auto-set mode the view will send data changes to the robot as soon as
   * the user has finished editing a field (the field has lost focus).
   */
  void setAutoSet(bool value);

  /**
   * Is called if a value has changed in the tree.
   */
  void valueChanged();

private:
  DECLARE_SYNC;
  const QString theFullName; /**< The path to this view in the scene graph */
  const QIcon theIcon; /**< The icon used for listing this view in the scene graph */
  RobotConsole& theConsole; /**< A reference to the console object. */
  const std::string theName; /**< The name of the view and the data displayed. */
  std::string type; /**< The type of the data shown. */
  DataWidget* pTheWidget = nullptr; /**< The widget which displays the properties */
  const TypeInfo& typeInfo;
  PropertyManager theVariantManager; /**< responsible for the creation and destruction of QtProperties */
  bool theIgnoreUpdatesFlag; /**< If true handleMessage returns without doing anything */
  QtProperty* pTheCurrentRootNode; /** Pointer to the current root property */
  using PropertiesMapType = std::map<std::string, QtVariantProperty*>;

  /**
   * This map is used to store all properties that have been created so far.
   * Recreating the properties on on every parser run causes memory leaks and gui bugs.
   * key: the fully qualified name of a property ( e.g. "this.is.a.fully.qualified.name")
   * value: pointer to the property.
   */
  PropertiesMapType theProperties;

  unsigned lastUpdated = 0; /**< Time when this view was updated last (in ms). */

  /**
   * True if auto-set is enabled.
   * In auto-set mode the view will send a set command directly after the user finished editing one value.
   */
  bool theAutoSetModeIsEnabled = true;

  int theUpdateTime = 100; /**< time between updates in ms */

  //The widget needs to access theConsole to synchronize while drawing.
  friend class DataWidget;

  //The EventFilter forwards the propertyEditor events to the view.
  //To do that it needs access to handlePropertyEditorEvent;
  friend class EditorEventFilter;

  friend class PropertyTreeCreator; // Private helper class of this class

  /**
   * Handles all events coming from the property editors.
   * @param pEditor the editor that is the source of this event.
   * @param pProperty the property which belongs to the editor
   * @return true if the event has been handled, otherwise false.
   *         If false is returned the event will be forwarded to other eventHandlers.
   */
  bool handlePropertyEditorEvent(QWidget* pEditor, QtProperty* pProperty, QEvent* pEvent);

  /**
   * Returns the property with the specified fully qualified name.
   * If a property with the fqn already exists it is returned, else a new property is created.
   * The Property is added to the parent if it is not a child of pParent already.
   * @param propertyType the property type. See documentation of QtVariantPropertyManager for a list of allowed types.
   * @param fqn the fully qualified name of the property.
   */
  QtVariantProperty* getProperty(const std::string& fqn, int propertyType, const QString& name, QtProperty* pParent);
};
