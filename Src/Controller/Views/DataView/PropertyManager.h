/**
 * PropertyManager.h
 *
 *  Created on: Apr 27, 2012
 *      Author: arne
 */

#pragma once

#include <qtvariantproperty.h>
#include <QMetaType>
#include "TypeDeclarations.h"
#include <QList>
#include <QMap>
#include <QVariant>
#include <QObject>
#include "TypeDescriptor.h"

using namespace Type;

/**
 * Extends the VariantPropertyManager to add support for additional datatypes.
 *
 * The QtPropertyBrowser does support most basic types out of the box.
 * However it only supports one kind of int/float. That is perfectly ok for
 * gui applications but we need to differentiate between the different kinds of
 * signed/unsigned/short/long ints etc.
 *
 * To understand what is happening here you should read:
 * http://doc.qt.nokia.com/qq/qq18-propertybrowser.html#extendingwithvariants
 *
 *
 * How to add new types:
 * (1) Create a TypeDescriptor for your type in TypeDescriptor.h
 * (2) Add it to initTypes().
 * (3) Register the datatype with qt in TypeDeclarations.h
 * (4) Modify PropertyEditorFactor::CreateEditor to provide an editor for your
 *     new type.
 * (5) Maybe you have to modify or add slots to the PropertyEditorFactory as well
 *
 */
class PropertyManager : public QtVariantPropertyManager
{
  Q_OBJECT

public:
  PropertyManager();
  ~PropertyManager();

  /**
   * @return The value of the specified property or an invalid QVariant of the
   *         property is not managed by this manager.
   */
  QVariant value(const QtProperty* pProperty) const override;

  /**
   * @return the type-id of the specified property.
   */
  int valueType(int propertyType) const override;

  /**
   * @return the value of the specified property as string or a default String of the property
   *         is not managed by this manager.
   */
  QString valueText(const QtProperty* property) const override;

  /**
   * @return true if this VariantManager supports the type. False otherwise.
   */
  bool isPropertyTypeSupported(int type) const override;

public slots:
  /**
   * Changes the value of the specified property.
   * Emits valueChanged and propertyChanged signals
   */
  void setValue(QtProperty* property, const QVariant& val) override;

protected:
  /**
   * This method is called by the base class whenever a new property should
   * be created.
   */
  void initializeProperty(QtProperty* property) override;

  /**
   * This method is called by the base class whenever a property should be
   * destroyed.
   */
  void uninitializeProperty(QtProperty* property) override;

private slots:
  void slotValueChanged(QtProperty* property, const QVariant& value);

private:

  /**
   * QtVariantProperties don't know anything about the type of the data they contain.
   * Therefore they cannot store the value internally. Instead the Manager stores
   * the values for all properties.
   */
  QMap<const QtProperty*, QVariant> theValues;

  /**
   * Mapping that assigns type descriptors to type ids.
   * key: type id
   * value: descriptor of that type
   */
  QMap<int, TypeDescriptor*> theDescriptors;

  /**
   * Initialize information about the allowed types
   */
  void initTypes();
};
