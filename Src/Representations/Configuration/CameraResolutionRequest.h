/**
 * @file CameraResolutionRequest.h
 *
 * This file implements a representation to request a resolution change.
 *
 * @author Dana Jenett, Alexis Tsogias, Felix Thielke
 */

#pragma once

#include "Representations/Infrastructure/CameraInfo.h"
#include "Tools/Streams/EnumIndexedArray.h"

STREAMABLE(CameraResolutionRequest,
{
  ENUM(Resolutions,
  {,
    defaultRes, /**< Use resolutions as specified in the config file. */
    w320h240,   /**< 160x120 in YCbCr222 */
    w640h480,   /**< 320x240 in YCbCr222 */
    w1280h960,  /**< 640x480 in YCbCr222 */
    noRequest,  /**< Keep the resolution as it is. */
  }),

  (ENUM_INDEXED_ARRAY(Resolutions, CameraInfo::Camera))({defaultRes, defaultRes}) resolutions, /**< The desired resolutions */
});
