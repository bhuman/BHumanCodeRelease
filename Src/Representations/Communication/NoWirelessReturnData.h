/**
* @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
 */

#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(NoWirelessReturnData,
{,
  (Vector2f)(Vector2f::Zero()) positionToPointAt,
  (unsigned)(0) timeOfLastLocationData,

  (std::vector<unsigned char>) data, /**< The data to transmit to the comtester */
});
