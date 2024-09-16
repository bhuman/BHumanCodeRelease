/**
 * @file TeamMessageContainer.h
 *
 * This file exists only to avoid Windows.h/WinSock2.h conflicts.
 * (see Libs/SimulatedNao/GameController.h)
 *
 * @author Arne Hasselbring
 */

#pragma once

#include <cstdint>

struct TeamMessageContainer
{
  char data[128]; // The length of this is a rule book constant.
  uint8_t length;
};
