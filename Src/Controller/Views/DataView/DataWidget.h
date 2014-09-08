/*
 * DataWidget.h
 *
 *  Created on: Apr 17, 2012
 *      Author: arne
 */

#pragma once

#include "Controller/RobotConsole.h"
#include "PropertyEditorFactory.h"
#include <QMenu>
#include <QtTreePropertyBrowser>

class DataWidget : public QtTreePropertyBrowser, public SimRobot::Widget
{
  Q_OBJECT

public:
  /**
   * @param manager reference to the property manager which has been used to create the properties.
   */
  DataWidget(DataView& view, QtVariantPropertyManager& manager);

  ~DataWidget();

  virtual QWidget* getWidget() {return this;}

  virtual void update();

  virtual bool canClose() {return true;}

  virtual QMenu* createFileMenu() const {return 0;}
  virtual QMenu* createUserMenu() const;
  virtual QMenu* createEditMenu() const {return 0;}

  /**
  * Sets the root property which is displayed by this widget.
  * @note The property must have been created using the manager specified in the constructor.
  */
  void setRootProperty(QtProperty* pRootProperty);

  void setUnchangedButtonEnabled(bool value);

  void setSetButtonEnabled(bool value);

protected:
  virtual void itemInserted(QtBrowserItem* insertedItem, QtBrowserItem* preceedingItem);

private slots:
  /** Is called when the set menu entry was selected. */
  void setPressed();

  void autoSetToggled(bool checked);

  void unchangedPressed();

private:
  DataView& theView;
  QtProperty* pTheCurrentProperty; /**< Pointer to the property that should be displayed */
  PropertyEditorFactory theEditorFactory; /**< provides qt widgets for properties */
  QtVariantPropertyManager& theManager; /**< reference to the manager that has been used to create the properties */
  bool theRootPropertyHasChanged; /**< True when the root property has changed*/
  QAction* pSetAction;
  QAction* pUnchangedAction;
  QAction* pAutoSetAction;
};
