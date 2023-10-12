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
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <qtvariantproperty.h>
#include "Platform/Thread.h" // for SYNC
#include "PropertyManager.h"

class RobotConsole;
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
   * @param repName Name of the streamable data. This is used to generate get requests.
   * @param threadName The thread that provides this data.
   * @param console The console object. Used to display error messages.
   * @param typeInfo Is used to get the type information while parsing.
   */
  DataView(const QString& fullName, const std::string& repName, const std::string& threadName, RobotConsole& console, const TypeInfo& typeInfo);

  SimRobot::Widget* createWidget() override;
  const QString& getFullName() const override { return theFullName; }
  const QIcon* getIcon() const override { return &theIcon; }

  const QString& getFullName() { return theFullName; }

  /**
   * Disconnects the current widget from the view.
   */
  void removeWidget();

  /**
   * Set the data message received from the robot.
   * @param message The data message received from the robot
   */
  void setMessageReceived(MessageQueue* message);

  /**
   * Control further updates from the RobotConsole.
   * @param change If 1, ignore updates. If -1, release a previous ignorance.
   *               If 0, reevaluate the ignorance state.
   */
  void updateIgnoreUpdates(int change);

  /** Request the data again if necessary. */
  void repoll();

  /**
   * Sends the current values to the robot.
   */
  void set();

  /**
   * Unset changes
   */
  void setUnchanged();

  /**
   * The console calls the method to notify if a "set" or "set unchanged" command was
   * executed.
   * @param set Was "set" executed (vs. "set unchanged")?
   */
  void notifyAboutSetStatus(bool set);

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
  const QString theFullName; /**< The path to this view in the scene graph */
  QIcon theIcon; /**< The icon used for listing this view in the scene graph */
  RobotConsole& theConsole; /**< A reference to the console object. */
  const std::string theName; /**< The name of the view and the data displayed. */
  const std::string theThread; /**< The name of the thread that provides the data. */
  std::string type; /**< The type of the data shown. */
  DataWidget* pTheWidget = nullptr; /**< The widget which displays the properties */
  const TypeInfo& typeInfo;
  MessageQueue* data = nullptr; /**< The data to read from. nullptr if no (new) data available. */
  PropertyManager theVariantManager; /**< responsible for the creation and destruction of QtProperties */
  int ignoreUpdates = 0; /**< If greater than 0, handleMessage returns without doing anything */
  QtProperty* pTheCurrentRootNode; /**< Pointer to the current root property */
  bool setWasCalled = false; /**< Data was sent to the robot. */

  /**
   * This map is used to store all properties that have been created so far.
   * Recreating the properties on on every parser run causes memory leaks and gui bugs.
   * key: the fully qualified name of a property ( e.g. "this.is.a.fully.qualified.name")
   * value: pointer to the property.
   */
  std::unordered_map<std::string, QtVariantProperty*> pathsToProperties;

  /** This map stores the reverse direction. */
  std::unordered_map<QtProperty*, std::string> propertiesToPaths;

  /**
   * Which properties are differently expanded than the default? They are stored
   * in form of their paths. Since the root-level properties are expanded by default,
   * they are added to this set if they are not expanded. All other properties are
   * included in this map if they are expanded.
   */
  std::unordered_set<std::string> expandedPaths;

  /**
   * This flag allows to suppress updating the expanded paths, because the
   * QtTreePropertyBrowser initially expands them all, which would destroy
   * the set.
   */
  bool updateExpandedPaths = true;

  /** The paths of property displayed in radians instead of degrees. */
  std::unordered_set<std::string> radiansPaths;

  bool forceUpdate = true; /**< At least one update is required. */

  /**
   * True if auto-set is enabled.
   * In auto-set mode the view will send a set command directly after the user finished editing one value.
   */
  bool theAutoSetModeIsEnabled = true;

  /**
   * Updates the property tree with data received from the robot if necessary.
   */
  void updateTree();

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

  /**
   * Determines whether the property should be expanded.
   * @param property The property in question.
   * @return Should it be expanded?
   */
  bool shouldExpand(QtProperty* property) const;

  /**
   * Sets whether a property was just expanded or collapsed.
   * @param property The property the expansion status of which just changed.
   * @param expanded Was it expanded (or collapsed)?
   */
  void updateExpansion(QtProperty* property, bool expanded);

  /**
   * Should this property be displayed using radians instead of degrees?
   * @param property The property in question.
   * @return Should it be displayed in radians?
   */
  bool shouldUseRadians(QtProperty* property) const;

  /**
   * Sets whether a property visualization was just switched to radians or degrees.
   * @param property The property the visualization of which just changed.
   * @param radians Was it switched to radians (or degrees)?
   */
  void updateUseRadians(QtProperty* property, bool radians);

  /** Checks whether data updates from the robot should be ignored. */
  bool shouldIgnoreUpdates() const;

  /** Enables/disables the "Unchanged" menu item. */
  void setUnchangedEnabled(bool value);

  /** Updates the dock window title based on the changed and set state. */
  void updateWindowTitle();

  friend class DataWidget; // Accesses several private data and methods
  friend class PropertyTreeCreator; // Private helper class of this class
  friend class PropertyEditorFactory; // Calls updateUseRadians
};
