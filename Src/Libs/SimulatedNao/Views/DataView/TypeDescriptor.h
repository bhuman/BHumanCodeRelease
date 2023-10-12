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
     * Returns a special type for groups.
     */
    static int getGroupType();

    /**
     * Returns a special type id for enums.
     */
    static int getEnumTypeId();
  };

  /**
   * A TypeDescriptor template to create numeric types with a numeric default value.
   */
  template<typename datatype, int defaultValue>
  class NumericValueTypeDescriptor : public TypeDescriptor
  {
  public:
    int getSupportedTypeId() override
    {
      return qMetaTypeId<datatype>();
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
      return qMetaTypeId<AngleWithUnit>();
    }

    QString toString(const QVariant& var) const override
    {
      AngleWithUnit value = var.value<AngleWithUnit>();
      return value == SensorData::off ? "off" : QString::number(value.deg ? value.toDegrees() : value) + (value.deg ? "°" : "");
    }

    QVariant createDefault() override
    {
      return QVariant::fromValue<AngleWithUnit>(AngleWithUnit());
    }
  };
}
