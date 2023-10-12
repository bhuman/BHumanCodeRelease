/**
 * PropertyEditorFactory.h
 *
 *  Created on: Apr 27, 2012
 *      Author: arne
 */

#pragma once

#include <qtvariantproperty.h>
#include <QMap>
#include <QDoubleSpinBox>
#include "DataView.h"

class QComboBox;

/**
 * This class creates spin box that handles both integer and floating point
 * values. It rejects group separators (',') as well as '.', 'e', and 'E' if it edits
 * a integer value, i.e. decimals() == 0.. It assumes that the current locale
 * is 'C', i.e. '.' represents the decimal point and ',' would represent the
 * group separator.
 */
class SpinBox : public QDoubleSpinBox
{
  Q_OBJECT

protected:
  /**
   * Convert a value to a string and  remove trailing zeros and a trailing decimal point.
   * @value The value to be converted.
   * @return The string representation of the value.
   */
  QString textFromValue(double value) const;

  /**
   * Validate the input string. Group separators are removed (',') as well as '.', 'e', and
   * 'E'  if this represents an integer value.
   * @param input The string to be validated. It might be modified.
   * @param pos The current cursor position in the string. It might be modified.
   * @return Is the string valid?
   */
  QValidator::State validate(QString& input, int& pos) const;

public:
  /** Use the same constructors as the super class. */
  using QDoubleSpinBox::QDoubleSpinBox;
};

class AngleEditor : public QWidget
{
  Q_OBJECT;
  QComboBox* unitBox;

public:
  SpinBox* spinBox;

  AngleEditor(QWidget* parent = nullptr);

  void setValue(const AngleWithUnit& value);

signals:
  void valueChanged(float value);
  void unitChanged(int index);

private slots:
  void updateValue(double value)
  {
    emit valueChanged(static_cast<float>(value));
  }

  void updateUnit(int index)
  {
    emit unitChanged(index);
  }
};

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
  QMap<SpinBox*, QtProperty*> spinBoxToProperty;
  QMap<AngleEditor*, QtProperty*> angleEditorToProperty;
  QtVariantPropertyManager* pTheManager = nullptr;

public:
  /**
   * @param pView All editor events will be send to this view.
   */
  PropertyEditorFactory(DataView* pView) : pTheView(pView) {}

  /**
   * Whenever the user wants to edit a property a new editor is created.
   * The editor is destroyed as soon as the user finished editing the property.
   */
  QWidget* createEditor(QtVariantPropertyManager* pManager, QtProperty* pProperty, QWidget* pParent);

protected:
  void connectPropertyManager(QtVariantPropertyManager* manager);

  void disconnectPropertyManager(QtVariantPropertyManager* manager);

private:
  template <typename T> QWidget* createEditor(QtVariantPropertyManager* pManager, QtProperty* pProperty, QWidget* pParent, int precision);

protected slots:
  /**
   * This slot is invoked whenever one of the managed spinboxes changes its value.
   * It updates the value in the property belonging to the box.
   */
  void slotSpinBoxValueChanged(double newValue);

  void slotAngleEditorValueChanged(float newValue);
  void slotAngleEditorUnitChanged(int index);

private slots:
  void slotEditorDestroyed(QObject* pObject);
};
