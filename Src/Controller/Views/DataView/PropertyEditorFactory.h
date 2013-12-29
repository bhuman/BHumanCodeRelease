/*
 * PropertyEditorFactory.h
 *
 *  Created on: Apr 27, 2012
 *      Author: arne
 */

#pragma once

#include <QtVariantEditorFactory>
#include <QMap>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include "DataView.h"

// HACK to make Debug version compile on Mac OS.
class MakeDebugCompileOnMac : public QtVariantEditorFactory {};

/**
 * Extended QtVariantFactory to provide custom controls for certain types.
 * The factory is responsible for creating editor widgets for certain types.
 * It manages the connection between properties and those editor widgets.
 */
class PropertyEditorFactory : public MakeDebugCompileOnMac
{
  Q_OBJECT

public:
  /**
   * @param pView All editor events will be send to this view.
   */
  PropertyEditorFactory(DataView* pView) : pTheView(pView), pTheManager(NULL)
  {}

  QWidget* createEditor(QtVariantPropertyManager* pManager, QtProperty* pProperty, QWidget* pParent);

protected:
  void connectPropertyManager(QtVariantPropertyManager* manager);

  void disconnectPropertyManager(QtVariantPropertyManager* manager);

protected slots:
  //FIXME override slotEditorDestroyed to remove the editor from the maps

  /**
   * This slot is invoked whenever a properties value changes.
   * It updates the editors value.
   */
  void slotManagerValueChanged(QtProperty* property, const QVariant& val);

  /**
   * This slot is invoked whenever one of the managed spinboxes changes its value.
   * It updates the value in the property belonging to the box.
   */
  void slotSpinBoxValueChanged(int newValue);

  /**
   * This slot is invoked whenever one of the managed DoubleSpinboxes (float really) changes its value.
   * It updates the value of the property belonging to the box.
   */
  void slotFloatSpinBoxValueChanged(double newValue);

private slots:
  void slotEditorDestroyed(QObject* pObject);

private:
  DataView* pTheView;
  //FIXME comments
  QMap<QtProperty*, QSpinBox*> propertyToSpinBox;
  QMap<QSpinBox*, QtProperty*> spinBoxToProperty;
  QMap<QtProperty*, QDoubleSpinBox*> propertyToDoubleSpinBox;
  QMap<QDoubleSpinBox*, QtProperty*> doubleSpinBoxToProperty;
  QtVariantPropertyManager* pTheManager;
};
