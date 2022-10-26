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
#include "EditorEventFilter.h"
#include "Platform/BHAssert.h"
#include "Math/Angle.h"
#include <QLabel>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QComboBox>
#include <QAbstractItemView>

QWidget* PropertyEditorFactory::createEditor(QtVariantPropertyManager* pManager, QtProperty* pProperty, QWidget* pParent)
{
  /**
   * Whenever the user wants to edit a property a new editor is created.
   * The editor is destroyed as soon as the user finished editing the property.
   */
  QWidget* pReturnWidget = nullptr;
  QWidget* pFilterWidget = nullptr;
  //FIXME copy&paste code
  if(qMetaTypeId<unsigned int>() == pManager->propertyType(pProperty))
  {
    QSpinBox* pBox = new IntSpinBox(pParent);
    pBox->setKeyboardTracking(false); //This is done to avoid changed updates on every keypress
    propertyToSpinBox[pProperty] = pBox;
    spinBoxToProperty[pBox] = pProperty;
    pBox->setMinimum(0);
    //Maximum can only be int max because the spinbox uses int internally
    pBox->setMaximum(std::numeric_limits<int>::max());
    pBox->setValue(static_cast<int>(pManager->value(pProperty).value<unsigned int>())); //the cast to int is not very nice :(
    pReturnWidget = pBox;
    connect(pBox, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, static_cast<void (PropertyEditorFactory::*)(int)>(&PropertyEditorFactory::slotSpinBoxValueChanged));
    connect(pBox, &QSpinBox::destroyed, this, &PropertyEditorFactory::slotEditorDestroyed);
  }
  else if(qMetaTypeId<float>() == pManager->propertyType(pProperty))
  {
    FloatSpinBox* pBox = new FloatSpinBox(pParent);
    pBox->setKeyboardTracking(false);
    pBox->setDecimals(5);
    propertyToDoubleSpinBox[pProperty] = pBox;
    doubleSpinBoxToProperty[pBox] = pProperty;
    pBox->setMinimum(-std::numeric_limits<float>::max());
    pBox->setMaximum(std::numeric_limits<float>::max());
    pBox->setValue(pManager->value(pProperty).value<float>());
    pReturnWidget = pBox;
    connect(pBox, static_cast<void (FloatSpinBox::*)(double)>(&FloatSpinBox::valueChanged), this, &PropertyEditorFactory::slotFloatSpinBoxValueChanged);
    connect(pBox, &FloatSpinBox::destroyed, this, &PropertyEditorFactory::slotEditorDestroyed);
  }
  else if(qMetaTypeId<short>() == pManager->propertyType(pProperty))
  {
    QSpinBox* pBox = new IntSpinBox(pParent);
    pBox->setKeyboardTracking(false);
    propertyToSpinBox[pProperty] = pBox;
    spinBoxToProperty[pBox] = pProperty;
    pBox->setMinimum(std::numeric_limits<short>::min());
    pBox->setMaximum(std::numeric_limits<short>::max());
    pBox->setValue(static_cast<int>(pManager->value(pProperty).value<short>()));
    pReturnWidget = pBox;
    connect(pBox, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &PropertyEditorFactory::slotSpinBoxValueChanged);
    connect(pBox, &QSpinBox::destroyed, this, &PropertyEditorFactory::slotEditorDestroyed);
  }
  else if(qMetaTypeId<unsigned short>() == pManager->propertyType(pProperty))
  {
    QSpinBox* pBox = new IntSpinBox(pParent);
    pBox->setKeyboardTracking(false);
    propertyToSpinBox[pProperty] = pBox;
    spinBoxToProperty[pBox] = pProperty;
    pBox->setMinimum(std::numeric_limits<unsigned short>::min());
    pBox->setMaximum(std::numeric_limits<unsigned short>::max());
    pBox->setValue(static_cast<int>(pManager->value(pProperty).value<unsigned short>()));
    pReturnWidget = pBox;
    connect(pBox, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &PropertyEditorFactory::slotSpinBoxValueChanged);
    connect(pBox, &QSpinBox::destroyed, this, &PropertyEditorFactory::slotEditorDestroyed);
  }
  else if(qMetaTypeId<unsigned char>() == pManager->propertyType(pProperty))
  {
    QSpinBox* pBox = new IntSpinBox(pParent);
    pBox->setKeyboardTracking(false);
    propertyToSpinBox[pProperty] = pBox;
    spinBoxToProperty[pBox] = pProperty;
    pBox->setMinimum(std::numeric_limits<unsigned char>::min());
    pBox->setMaximum(std::numeric_limits<unsigned char>::max());
    pBox->setValue(static_cast<int>(pManager->value(pProperty).value<unsigned char>()));
    pReturnWidget = pBox;
    connect(pBox, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &PropertyEditorFactory::slotSpinBoxValueChanged);
    connect(pBox, &QSpinBox::destroyed, this, &PropertyEditorFactory::slotEditorDestroyed);
  }
  else if(qMetaTypeId<AngleWithUnity>() == pManager->propertyType(pProperty))
  {
    AngleEditor* aEdit = new AngleEditor(pParent);
    pFilterWidget = aEdit->fBox;
    propertyToAngleEditor[pProperty] = aEdit;
    angleEditorToProperty[aEdit] = pProperty;
    AngleWithUnity angle = pManager->value(pProperty).value<AngleWithUnity>();
    aEdit->setValue(angle);

    connect(aEdit, &AngleEditor::valueChanged, this, &PropertyEditorFactory::slotAngleEditorValueChanged);
    connect(aEdit, &AngleEditor::unityChanged, this, &PropertyEditorFactory::slotAngleEditorUnityChanged);
    connect(aEdit, &AngleEditor::destroyed, this, &PropertyEditorFactory::slotEditorDestroyed);

    pReturnWidget = aEdit;
  }
  else
    pReturnWidget = QtVariantEditorFactory::createEditor(pManager, pProperty, pParent);

  if(nullptr != pReturnWidget)
  {
    if(!pFilterWidget)
      pFilterWidget = pReturnWidget;

    //The event filter is used to forward all events to the DataView
    pFilterWidget->installEventFilter(new EditorEventFilter(this, pTheView, pFilterWidget, pProperty));
  }
  else
  {
    //In case of error return a label containing an error message
    QLabel* l = new QLabel(pParent);
    if(TypeDescriptor::getGroupType() != pManager->propertyType(pProperty))
      l->setText("Failed to create an editor for this type");
    pReturnWidget = l;
  }

  return pReturnWidget;
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
 * This slot is invoked whenever one of the managed spinboxes changes its value.
 * It updates the value in the property belonging to the box.
 */
void PropertyEditorFactory::slotSpinBoxValueChanged(int newValue)
{
  QSpinBox* pCaller = qobject_cast<QSpinBox*>(QObject::sender());

  ASSERT(pCaller != nullptr); //qobject_cast returns nullptr in case of error
  ASSERT(spinBoxToProperty.contains(pCaller));
  ASSERT(nullptr != pTheManager);

  pTheManager->setValue(spinBoxToProperty[pCaller], QVariant::fromValue(newValue));
}

/**
 * This slot is invoked whenever one of the managed DoubleSpinboxes (float really) changes its value.
 * It updates the value of the property belonging to the box.
 */
void PropertyEditorFactory::slotFloatSpinBoxValueChanged(double newValue)
{
  QDoubleSpinBox* pCaller = qobject_cast<QDoubleSpinBox*>(QObject::sender());

  ASSERT(nullptr != pCaller); //qobject_cast returns nullptr in case of error
  ASSERT(doubleSpinBoxToProperty.contains(pCaller));
  ASSERT(nullptr != pTheManager);

  float val = static_cast<float>(newValue);//this is ok because the spinBox was limited to float ranges
  pTheManager->setValue(doubleSpinBoxToProperty[pCaller], QVariant::fromValue(val));
}

void PropertyEditorFactory::slotAngleEditorValueChanged(float newValue)
{
  AngleEditor* pCaller = qobject_cast<AngleEditor*>(QObject::sender());

  ASSERT(nullptr != pCaller); //qobject_cast returns nullptr in case of error
  ASSERT(angleEditorToProperty.contains(pCaller));
  ASSERT(nullptr != pTheManager);

  AngleWithUnity oldAngle = pTheManager->value(angleEditorToProperty[pCaller]).value<AngleWithUnity>();
  AngleWithUnity newAngle = oldAngle.deg ? Angle::fromDegrees(newValue) : Angle(newValue);
  newAngle.deg = oldAngle.deg;
  pTheManager->setValue(angleEditorToProperty[pCaller], QVariant::fromValue(newAngle));
}

void PropertyEditorFactory::slotAngleEditorUnityChanged(int index)
{
  AngleEditor* pCaller = qobject_cast<AngleEditor*>(QObject::sender());

  ASSERT(nullptr != pCaller); //qobject_cast returns nullptr in case of error
  ASSERT(angleEditorToProperty.contains(pCaller));
  ASSERT(nullptr != pTheManager);

  AngleWithUnity angle = pTheManager->value(angleEditorToProperty[pCaller]).value<AngleWithUnity>();
  angle.deg = index == 0;
  pTheManager->setValue(angleEditorToProperty[pCaller], QVariant::fromValue(angle));
  pCaller->setValue(angle);
}

void PropertyEditorFactory::slotEditorDestroyed(QObject* pObject)
{
  //since we only get a pointer to QObject and not to QSpinBox or QDoubleSpinBox
  //we have to search all maps to find the right one :(
  //note: qobject_cast does not work. Casting pObject to QSpinBox always returns nullptr, even if it is a QSpinBox ...

  QMap<QSpinBox*, QtProperty*>::ConstIterator itEditor = spinBoxToProperty.constBegin();
  while(itEditor != spinBoxToProperty.constEnd())
  {
    if(itEditor.key() == pObject)
    {
      QSpinBox* editor = itEditor.key();
      QtProperty* property = itEditor.value();
      propertyToSpinBox.remove(property);
      spinBoxToProperty.remove(editor);
      return; //there can only be one editor in any of the lists.
    }
    itEditor++;
  }

  QMap<QDoubleSpinBox*, QtProperty*>::ConstIterator itEditorDouble = doubleSpinBoxToProperty.constBegin();
  while(itEditorDouble != doubleSpinBoxToProperty.constEnd())
  {
    if(itEditorDouble.key() == pObject)
    {
      QDoubleSpinBox* editor = itEditorDouble.key();
      QtProperty* property = itEditorDouble.value();
      propertyToDoubleSpinBox.remove(property);
      doubleSpinBoxToProperty.remove(editor);
      return; //there can only be one editor in any of the lists.
    }
    itEditorDouble++;
  }

  QMap<AngleEditor*, QtProperty*>::ConstIterator itEditorAngle = angleEditorToProperty.constBegin();
  while(itEditorAngle != angleEditorToProperty.constEnd())
  {
    if(itEditorAngle.key() == pObject)
    {
      AngleEditor* editor = itEditorAngle.key();
      QtProperty* property = itEditorAngle.value();
      propertyToAngleEditor.remove(property);
      angleEditorToProperty.remove(editor);
      return; //there can only be one editor in any of the lists.
    }
    itEditorAngle++;
  }
}

AngleEditor::AngleEditor(QWidget* parent) :
  QWidget(parent)
{
  QGridLayout* layout = new QGridLayout();
  setLayout(layout);
  fBox = new FloatSpinBox();
  unityBox = new QComboBox();
  layout->addWidget(fBox, 0, 0);

#ifdef MACOS
  setAttribute(Qt::WA_MacShowFocusRect);
  QWidget* w = new QWidget();
  w->setLayout(new QHBoxLayout);
  w->layout()->addWidget(unityBox);
  w->layout()->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(w, 0, 1);
#else
  layout->addWidget(unityBox, 0, 1);
#endif

  layout->setColumnStretch(0, 100);
  layout->setContentsMargins(0, 0, 0, 0);

  fBox->setKeyboardTracking(false);
  fBox->setDecimals(5);
  fBox->setMinimum(-std::numeric_limits<float>::max());
  fBox->setMaximum(std::numeric_limits<float>::max());

  unityBox->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
  unityBox->setMinimumContentsLength(1);
  unityBox->view()->setTextElideMode(Qt::ElideRight);
  QStringList enumNames;
  enumNames.push_back("deg");
  enumNames.push_back("rad");
  unityBox->addItems(enumNames);

  connect(fBox, static_cast<void (FloatSpinBox::*)(double)>(&FloatSpinBox::valueChanged), this, &AngleEditor::updateValue);
  connect(unityBox, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &AngleEditor::updateUnity);
}

void AngleEditor::setValue(const AngleWithUnity& value)
{
  unityBox->setCurrentIndex(value.deg ? 0 : 1);
  if(value.deg)
  {
    // round to two decimals. Using the setDecimals() of the Spinbox
    // triggers another valueChanged witch leads to some problems..
    fBox->setValue(round(value.toDegrees() * 100) / 100);
    fBox->setSingleStep(1.f);
  }
  else
  {
    fBox->setValue(value);
    fBox->setSingleStep(1_deg);
  }
}
