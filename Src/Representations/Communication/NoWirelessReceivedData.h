/**
 * Data received from the No Wi-fi CommTester.
 * Location data is already packed into the data array. In that case, the
 * first byte contains the x coordinate -4500..4500 scaled to 0..255. The
 * second byte contains the y coordinate -3000..3000 scaled to 0..255.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "SPLNoWifiChallenge.h"
#include "Tools/Streams/Enum.h"
#include "Tools/Math/Eigen.h"

STREAMABLE(NoWirelessReceivedData,
{
  ENUM(Type,
  {,
    noType,
    locationType,
    dataType,
  }),

  (Type)(noType) type, /**< The type of the message. */
  (std::vector<unsigned char>) data, /**< The data to transmit via other means. */
  (bool)(false) connected, /**< Was a connection established (just for setting LED). */
  (unsigned)(0) lastReceived, /**< The last time a message was received. */
});
