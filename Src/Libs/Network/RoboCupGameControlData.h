/**
 * @file RoboCupGameControlData.h
 * The file encapsulates definitions in the file RoboCupGameControlData.h
 * that is provided with the GameController in a namespace.
 * @author Thomas Röfer
 */

#pragma once

#include <cstdint>
#include <cstring>

namespace RoboCup
{
#define teamColour teamColor
#include <RoboCupGameControlData.h>
#include <SPLStandardMessage.h>
#undef teamColour
}
