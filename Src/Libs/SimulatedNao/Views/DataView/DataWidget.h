/**
 * DataWidget.h
 *
 *  Created on: Apr 17, 2012
 *      Author: arne
 */

#pragma once

#include "SimulatedNao/RobotConsole.h"
#include "PropertyEditorFactory.h"
#include <QMenu>
#include <qttreepropertybrowser.h>

class DataWidget : public QtTreePropertyBrowser, public SimRobot::Widget
{
  Q_OBJECT

public:
  /**
   * @param manager reference to the property manager which has been used to create the properties.
   */
  DataWidget(DataView& view, QtVariantPropertyManager& manager);

  ~DataWidget();

  QWidget* getWidget() override { return this; }

  void update() override;

  bool canClose() override { return true; }

  QMenu* createFileMenu() const override { return nullptr; }
  QMenu* createUserMenu() const override;
  QMenu* createEditMenu() const override { return nullptr; }

  /**
   * Sets the root property which is displayed by this widget.
   * @note The property must have been created using the manager specified in the constructor.
   */
  void setRootProperty(QtProperty* pRootProperty);

  /** Enables or disables the "Set" menu item. */
  void setSetEnabled(bool value);

  /** Enables of disables the "Unchanged/Reset" menu item. */
  void setUnchangedEnabled(bool value);

  /** Returns, whether the "Unchanged/Reset" menu item is enabled. */
  bool isUnchangedEnabled() const;

protected:
  void itemInserted(QtBrowserItem* insertedItem, QtBrowserItem* precedingItem) override;

private:
  DataView& theView;
  QtProperty* pTheCurrentProperty = nullptr; /**< Pointer to the property that should be displayed */
  PropertyEditorFactory theEditorFactory; /**< provides qt widgets for properties */
  QtVariantPropertyManager& theManager; /**< reference to the manager that has been used to create the properties */
  bool theRootPropertyHasChanged = false; /**< True when the root property has changed*/
  QAction* pSetAction;
  QAction* pUnchangedAction;
  QAction* pAutoSetAction;

private slots:
  /** Is called when the set menu entry was selected. */
  void setPressed();

  void autoSetToggled(bool checked);

  void unchangedPressed();

  void valueChanged(QtProperty*, const QVariant&);

  void collapsed(QtBrowserItem *item);
  void expanded(QtBrowserItem *item);
};
