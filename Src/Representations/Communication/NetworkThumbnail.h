/**
 * @author Felix Thielke
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include <vector>

STREAMABLE(NetworkThumbnail,
{
  inline NetworkThumbnail()
  {
    data.reserve(256);
  },

  (char)(-1) sequence,               /**< Sequence number of the current part of the image (starts at 0). */
  (std::vector<unsigned char>) data, /**< Part of the RLE-encoded image data. */
});
