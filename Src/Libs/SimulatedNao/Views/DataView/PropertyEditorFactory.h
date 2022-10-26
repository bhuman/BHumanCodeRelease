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
 * The class fixes the strange behavior that QSpinBox accepts the decimal
 * point, but ignoring it when converting the text to int ("1.0" -> 10).
 * The implementation in this class truncates everything after the decimal
 * point instead.
 */
class IntSpinBox : public QSpinBox
{
  Q_OBJECT

protected:
  QValidator::State validate(QString& input, int& pos) const
  {
    while(true)
    {
      auto index = input.indexOf('.');
      if(index == -1)
        break;
      input = input.left(index) + input.mid(index + 1);
      if(pos > index)
        --pos;
    }
    return QSpinBox::validate(input, pos);
  }

public:
  IntSpinBox(QWidget* parent) : QSpinBox(parent) {}
};

/**
 * The class fixes the strange behavior that QDoubleSpinBox sometimes
 * expects the decimal point symbol according to the locale and ignores
 * the dot, and sometimes expects the dot. In addition, trailing zeros
 * are removed.
 * The class also fixes that values starting with a decimal points are
 * not allowed, which is annoying while editing a number.
 */
class FloatSpinBox : public QDoubleSpinBox
{
  Q_OBJECT

protected:
  QString textFromValue(double value) const
  {
    QString text = QString("%1").arg(value, 0, 'f', 5); // avoid exponents
    while(text.size() > 1 && text.contains('.') && (text.endsWith('0') || text.endsWith('.'))) // remove trailing '0's
      text = text.left(text.size() - 1);
    return text;
  }

  double valueFromText(const QString& text) const
  {
    return text.toFloat();
  }

  /** Accept numbers such as ".17" */
  QValidator::State validate(QString& input, int& pos) const
  {
    QValidator::State state = QDoubleSpinBox::validate(input, pos);
    if(state != QValidator::Invalid || input.isEmpty() || input[0] != '.')
      return state;
    QString temp = "0" + input;
    return QDoubleSpinBox::validate(temp, pos);
  }

public:
  FloatSpinBox(QWidget* parent = nullptr) : QDoubleSpinBox(parent) {}
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
