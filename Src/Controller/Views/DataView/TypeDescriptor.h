#pragma once

#include <QString>
#include <QVariant>
#include "TypeDeclarations.h"
#include "Tools/Motion/SensorData.h"

namespace Type //The namespace is here to fix a VC-compiler bug
{
  /**
   * A TypeDescriptor provides basic construction and conversion information
   * for a certain type.
   */
  class TypeDescriptor
  {
  public:
    virtual ~TypeDescriptor() = default;

    /**
     * Returns the type id which is supported by this descriptor.
     */
    virtual int getSupportedTypeId() = 0;

    /**
     * Converts a variant to a readable string.
     * This method should only be called for variants that match this descriptor's type id.
     * If the type id does not match the behavior is undefined.
     */
    virtual QString toString(const QVariant& var) const = 0;

    /**
     * Returns a default instance.
     */
    virtual QVariant createDefault() = 0;

    /**
     * Provides a mapping from c++ types to type ids.
     */
    template<typename T>
    static int getTypeId();

    /**
     * Returns a special type for groups.
     */
    static int getGroupType();

    /**
     * Returns a special type id for enums.
     */
    static int getEnumTypeId();
  };

  template<> int TypeDescriptor::getTypeId<int>();
  template<> int TypeDescriptor::getTypeId<double>();
  template<> int TypeDescriptor::getTypeId<char>();
  template<> int TypeDescriptor::getTypeId<bool>();
  template<> int TypeDescriptor::getTypeId<std::string>();

  /**
   * This is the default implementation of getTypeId.
   * It provides an automatic mapping for all types registered in TypeDeclarations.h
   */
  template<typename T>
  int TypeDescriptor::getTypeId()
  {
    //This will fail if T has not been defined using the Q_DECLARE_METATYPE macro.
    return qMetaTypeId<T>();
  }

  /**
   * A TypeDescriptor template to create numeric types with a numeric default value.
   */
  template<typename datatype, int defaultValue>
  class NumericValueTypeDescriptor : public TypeDescriptor
  {
  public:
    int getSupportedTypeId() override
    {
      return getTypeId<datatype>();
    }

    QString toString(const QVariant& var) const override
    {
      datatype value = var.value<datatype>();
      return QString::number(value);
    }

    QVariant createDefault() override
    {
      return QVariant::fromValue<datatype>(defaultValue);
    }
  };

  /**
   * Type descriptor for unsigned int
   */
  class UIntDescriptor : public NumericValueTypeDescriptor<unsigned int, 0> {};

  /**
   * Type descriptor for unsigned char
   */
  class UCharDescriptor : public NumericValueTypeDescriptor<unsigned char, 0> {};

  /**
   * Type descriptor for short
   */
  class ShortDescriptor : public NumericValueTypeDescriptor<short, 0> {};

  /**
   * Type descriptor for unsigned short
   */
  class UShortDescriptor : public NumericValueTypeDescriptor<unsigned short, 0> {};

  /**
   * Type descriptor for float
   */
  class FloatDescriptor : public NumericValueTypeDescriptor<float, 0> {};

  /**
   * Type descriptor to display angles correctly, i.e. as 12deg or 0.3rad
   */
  class AngleTypeDescriptor : public TypeDescriptor
  {
  public:
    int getSupportedTypeId() override
    {
      return getTypeId<AngleWithUnity>();
    }

    QString toString(const QVariant& var) const override
    {
      AngleWithUnity value = var.value<AngleWithUnity>();
      return value == SensorData::off ? "off" : QString::number(value.deg ? value.toDegrees() : value) + " " + (value.deg ? "deg" : "rad");
    }

    QVariant createDefault() override
    {
      return QVariant::fromValue<AngleWithUnity>(AngleWithUnity());
    }
  };
}
