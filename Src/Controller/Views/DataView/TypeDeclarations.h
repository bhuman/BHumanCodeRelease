/**
 * TypeDeclarations.h
 *
 * This file registers additional types to qt.
 * see http://doc.qt.nokia.com/qq/qq14-metatypes.html for details
 *
 * Note:
 * If you forget to add a type SimRobot will crash uppon displaying
 * data that contains the missing type.
 * It will segfault in QtVariantProperty::setValue.
 *
 *  Created on: Apr 27, 2012
 *      Author: arne
 */

#pragma once

#include <QMetaType>
#include <string>
#include "Tools/Math/Angle.h"

struct AngleWithUnity : public Angle
{
  bool deg = true;

  AngleWithUnity() = default;
  AngleWithUnity(const Angle& angle) : Angle(angle) {}
};

Q_DECLARE_METATYPE(unsigned int);
Q_DECLARE_METATYPE(short);
Q_DECLARE_METATYPE(unsigned short);
Q_DECLARE_METATYPE(unsigned char);
Q_DECLARE_METATYPE(float);
Q_DECLARE_METATYPE(std::string);
Q_DECLARE_METATYPE(AngleWithUnity);
