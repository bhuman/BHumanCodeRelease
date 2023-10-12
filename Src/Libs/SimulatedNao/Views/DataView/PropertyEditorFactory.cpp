/**
 * PropertyEditorFactory.cpp
 *
 *  Created on: Apr 27, 2012
 *      Author: arne
 */

#include "PropertyEditorFactory.h"
#include <qtpropertybrowser.h>
#include <limits>
#include <cmath>
#include "DataWidget.h"
#include "Platform/BHAssert.h"
#include "Math/Angle.h"
#include <QLabel>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QComboBox>
#include <QAbstractItemView>
#include <QLocale>
#include <QRegularExpression>

template <typename T> QWidget* PropertyEditorFactory::createEditor(QtVariantPropertyManager* pManager, QtProperty* pProperty,
                                                                   QWidget* pParent, int precision)
{
  SpinBox* pBox = new SpinBox(pParent);
  pBox->setKeyboardTracking(false);
  pBox->setDecimals(precision);
  spinBoxToProperty[pBox] = pProperty;
  pBox->setMinimum(std::numeric_limits<T>::lowest());
  pBox->setMaximum(std::numeric_limits<T>::max());
  pBox->setValue(pManager->value(pProperty).value<T>());
  connect(pBox, &SpinBox::valueChanged, this, &PropertyEditorFactory::slotSpinBoxValueChanged);
  connect(pBox, &SpinBox::destroyed, this, &PropertyEditorFactory::slotEditorDestroyed);
  pTheView->updateIgnoreUpdates(1);
  return pBox;
}

QWidget* PropertyEditorFactory::createEditor(QtVariantPropertyManager* pManager, QtProperty* pProperty, QWidget* pParent)
{

  if(qMetaTypeId<unsigned char>() == pManager->propertyType(pProperty))
    return createEditor<unsigned char>(pManager, pProperty, pParent, 0);
  else if(qMetaTypeId<short>() == pManager->propertyType(pProperty))
    return createEditor<short>(pManager, pProperty, pParent, 0);
  else if(qMetaTypeId<unsigned short>() == pManager->propertyType(pProperty))
    return createEditor<unsigned short>(pManager, pProperty, pParent, 0);
  else if(qMetaTypeId<int>() == pManager->propertyType(pProperty))
    return createEditor<int>(pManager, pProperty, pParent, 0);
  else if(qMetaTypeId<unsigned int>() == pManager->propertyType(pProperty))
    return createEditor<unsigned int>(pManager, pProperty, pParent, 0);
  else if(qMetaTypeId<float>() == pManager->propertyType(pProperty))
    return createEditor<float>(pManager, pProperty, pParent, std::numeric_limits<float>::digits10);
  else if(qMetaTypeId<double>() == pManager->propertyType(pProperty))
    return createEditor<double>(pManager, pProperty, pParent, std::numeric_limits<double>::digits10);
  else if(qMetaTypeId<AngleWithUnit>() == pManager->propertyType(pProperty))
  {
    AngleEditor* editor = new AngleEditor(pParent);
    angleEditorToProperty[editor] = pProperty;
    AngleWithUnit angle = pManager->value(pProperty).value<AngleWithUnit>();
    editor->setValue(angle);
    connect(editor, &AngleEditor::valueChanged, this, &PropertyEditorFactory::slotAngleEditorValueChanged);
    connect(editor, &AngleEditor::unitChanged, this, &PropertyEditorFactory::slotAngleEditorUnitChanged);
    connect(editor, &AngleEditor::destroyed, this, &PropertyEditorFactory::slotEditorDestroyed);
    pTheView->updateIgnoreUpdates(1);
    return editor;
  }
  else
  {
    QWidget* editor = QtVariantEditorFactory::createEditor(pManager, pProperty, pParent);
    if(editor)
    {
      connect(editor, &QWidget::destroyed, this, &PropertyEditorFactory::slotEditorDestroyed);
      pTheView->updateIgnoreUpdates(1);
    }
    return editor;
  }
}

void PropertyEditorFactory::connectPropertyManager(QtVariantPropertyManager* manager)
{
  pTheManager = manager;
  QtVariantEditorFactory::connectPropertyManager(manager);
}

void PropertyEditorFactory::disconnectPropertyManager(QtVariantPropertyManager* manager)
{
  QtVariantEditorFactory::disconnectPropertyManager(manager);
  pTheManager = nullptr;
}

/**
 * This slot is invoked whenever one of the managed DoubleSpinboxes (float really) changes its value.
 * It updates the value of the property belonging to the box.
 */
void PropertyEditorFactory::slotSpinBoxValueChanged(double newValue)
{
  SpinBox* pCaller = qobject_cast<SpinBox*>(QObject::sender());

  ASSERT(nullptr != pCaller); //qobject_cast returns nullptr in case of error
  ASSERT(spinBoxToProperty.contains(pCaller));
  ASSERT(nullptr != pTheManager);

  pTheManager->setValue(spinBoxToProperty[pCaller], QVariant::fromValue(newValue));
}

void PropertyEditorFactory::slotAngleEditorValueChanged(float newValue)
{
  AngleEditor* pCaller = qobject_cast<AngleEditor*>(QObject::sender());

  ASSERT(nullptr != pCaller); //qobject_cast returns nullptr in case of error
  ASSERT(angleEditorToProperty.contains(pCaller));
  ASSERT(nullptr != pTheManager);

  AngleWithUnit oldAngle = pTheManager->value(angleEditorToProperty[pCaller]).value<AngleWithUnit>();
  AngleWithUnit newAngle = oldAngle.deg ? Angle::fromDegrees(newValue) : Angle(newValue);
  newAngle.deg = oldAngle.deg;
  if(std::abs(newAngle - oldAngle) > std::numeric_limits<float>::epsilon() * 10.f)
    pTheManager->setValue(angleEditorToProperty[pCaller], QVariant::fromValue(newAngle));
}

void PropertyEditorFactory::slotAngleEditorUnitChanged(int index)
{
  AngleEditor* pCaller = qobject_cast<AngleEditor*>(QObject::sender());

  ASSERT(nullptr != pCaller); //qobject_cast returns nullptr in case of error
  ASSERT(angleEditorToProperty.contains(pCaller));
  ASSERT(nullptr != pTheManager);

  QtProperty* property = angleEditorToProperty[pCaller];
  AngleWithUnit angle = pTheManager->value(property).value<AngleWithUnit>();
  angle.deg = index == 0;

  // Avoid auto-setting by not emitting the signal "valueChanged", because the angle did not change
  {
    QSignalBlocker blocker(pTheManager);
    pTheManager->setValue(property, QVariant::fromValue(angle));
  }

  // However, the window needs the signal "propertyChanged" to show the correct value later
  emit pTheManager->propertyChanged(angleEditorToProperty[pCaller]);

  pCaller->setValue(angle);

  pTheView->updateUseRadians(property, !angle.deg);
}

void PropertyEditorFactory::slotEditorDestroyed(QObject* pObject)
{
  spinBoxToProperty.remove(static_cast<SpinBox*>(pObject));
  angleEditorToProperty.remove(static_cast<AngleEditor*>(pObject));
  pTheView->updateIgnoreUpdates(-1);
}

QString SpinBox::textFromValue(double value) const
{
  QString text = QString("%1").arg(value, 0, decimals() == 0 ? 'f' : 'g', decimals()); // avoid exponents for integers
  while(text.size() > 1 && text.contains('.') && (text.endsWith('0') || text.endsWith('.'))) // remove trailing '0's
    text = text.left(text.size() - 1);
  return text;
}

QValidator::State SpinBox::validate(QString& input, int& pos) const
{
  qsizetype index;
  while((index = input.indexOf(',')) != -1 || (decimals() == 0 && (index = input.indexOf(QRegularExpression("[.eE]"))) != -1))
  {
    input = input.left(index) + input.mid(index + 1);
    if(pos > index)
      --pos;
  }
  return QDoubleSpinBox::validate(input, pos);
}

AngleEditor::AngleEditor(QWidget* parent) :
  QWidget(parent),
  unitBox(new QComboBox()),
  spinBox(new SpinBox())
{
  QGridLayout* layout = new QGridLayout();
  setLayout(layout);
  layout->addWidget(spinBox, 0, 0);

#ifdef MACOS
  setAttribute(Qt::WA_MacShowFocusRect);
  QWidget* w = new QWidget();
  w->setLayout(new QHBoxLayout);
  w->layout()->addWidget(unitBox);
  w->layout()->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(w, 0, 1);
#else
  layout->addWidget(unitBox, 0, 1);
#endif

  layout->setColumnStretch(0, 100);
  layout->setContentsMargins(0, 0, 0, 0);

  spinBox->setKeyboardTracking(false);
  spinBox->setDecimals(std::numeric_limits<float>::digits10);
  spinBox->setMinimum(std::numeric_limits<float>::lowest());
  spinBox->setMaximum(std::numeric_limits<float>::max());
  setFocusProxy(spinBox);

  unitBox->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
  unitBox->setMinimumContentsLength(1);
  unitBox->view()->setTextElideMode(Qt::ElideRight);
  QStringList enumNames;
  enumNames.push_back("deg");
  enumNames.push_back("rad");
  unitBox->addItems(enumNames);

  connect(spinBox, &SpinBox::valueChanged, this, &AngleEditor::updateValue);
  connect(unitBox, &QComboBox::currentIndexChanged, this, &AngleEditor::updateUnit);
}

void AngleEditor::setValue(const AngleWithUnit& value)
{
  unitBox->setCurrentIndex(value.deg ? 0 : 1);
  if(value.deg)
  {
    spinBox->setValue(value.toDegrees());
    spinBox->setSingleStep(1.f);
  }
  else
  {
    spinBox->setValue(value);
    spinBox->setSingleStep(1_deg);
  }
}
