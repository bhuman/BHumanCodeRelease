/**
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

class AngleEditor;

/**
 * Extended QtVariantFactory to provide custom controls for certain types.
 * The factory is responsible for creating editor widgets for certain types.
 * It manages the connection between properties and those editor widgets.
 */
class PropertyEditorFactory : public QtVariantEditorFactory
{
  Q_OBJECT

private:
  DataView* pTheView;
  //FIXME comments
  QMap<QtProperty*, QSpinBox*> propertyToSpinBox;
  QMap<QSpinBox*, QtProperty*> spinBoxToProperty;
  QMap<QtProperty*, QDoubleSpinBox*> propertyToDoubleSpinBox;
  QMap<QDoubleSpinBox*, QtProperty*> doubleSpinBoxToProperty;
  QMap<QtProperty*, AngleEditor*> propertyToAngleEditor;
  QMap<AngleEditor*, QtProperty*> angleEditorToProperty;
  QtVariantPropertyManager* pTheManager = nullptr;

public:
  /**
   * @param pView All editor events will be send to this view.
   */
  PropertyEditorFactory(DataView* pView) : pTheView(pView) {}

  QWidget* createEditor(QtVariantPropertyManager* pManager, QtProperty* pProperty, QWidget* pParent);

protected:
  void connectPropertyManager(QtVariantPropertyManager* manager);

  void disconnectPropertyManager(QtVariantPropertyManager* manager);

protected slots:
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

  void slotAngleEditorValueChanged(float newValue);
  void slotAngleEditorUnityChanged(int index);

private slots:
  void slotEditorDestroyed(QObject* pObject);
};

class FloatSpinBox;
class QComboBox;

class AngleEditor : public QWidget
{
private:
  Q_OBJECT;

public:
  FloatSpinBox* fBox;
  QComboBox* unityBox;

  AngleEditor(QWidget* parent = nullptr);

  void setValue(const AngleWithUnity& value);

signals:
  void valueChanged(float value);
  void unityChanged(int index);

private slots:
  void updateValue(double value)
  {
    emit valueChanged(static_cast<float>(value));
  }

  void updateUnity(int index)
  {
    emit unityChanged(index);
  }
};
