/**
 * @file Robot.h
 *
 * This file defines a class that represents the network information of a robot.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Streaming/AutoStreamable.h"

STREAMABLE(Robot,
{
  unsigned lastStatusUpdate = 0, /**< The last time the status information was retrieved. */

  (std::string) lan, /**< The ethernet ip address of this robot. */
  (std::string) wlan, /**< The wifi ip address of this robot. */
  (std::string) name, /**< The name of this robot. */
});
