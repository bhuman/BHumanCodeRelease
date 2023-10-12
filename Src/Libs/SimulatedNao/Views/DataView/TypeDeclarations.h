/**
 * TypeDeclarations.h
 *
 * This file registers additional types to qt.
 * see http://doc.qt.nokia.com/qq/qq14-metatypes.html for details
 *
 * Note:
 * If you forget to add a type SimRobot will crash upon displaying
 * data that contains the missing type.
 * It will segfault in QtVariantProperty::setValue.
 *
 *  Created on: Apr 27, 2012
 *      Author: arne
 */

#pragma once

#include <QMetaType>
#include "Math/Angle.h"

struct AngleWithUnit : public Angle
{
  bool deg = true;

  AngleWithUnit() = default;
  AngleWithUnit(const Angle& angle) : Angle(angle) {}

  bool operator==(const AngleWithUnit& other) const
  {
    return static_cast<float>(*this) == static_cast<float>(other) && deg == other.deg;
  }

  bool operator!=(const AngleWithUnit& other) const {return !(*this == other);}
};

//Q_DECLARE_METATYPE(AngleWithUnit);

// Hardcode meta type id, because dynamic method does not support
// unloading and reloading shared libraries on Windows.
template<> constexpr int qMetaTypeId<AngleWithUnit>() {return QMetaType::User;}
