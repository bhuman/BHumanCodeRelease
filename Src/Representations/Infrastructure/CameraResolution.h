/**
 * @file CameraResolution.h
 *
 * This file implements a representation for the camera resolutions and an interface to request a resolution change.
 *
 * @author Dana Jenett, Alexis Tsogias, Felix Thielke
 */

#pragma once

#include "Tools/Streams/Enum.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(CameraResolution,
{
  ENUM(Resolutions,
  {,
    defaultRes, /**< Use resolutions as specified in the config file. */
    w320h240,   /**< 160x120 in YCbCr222 */
    w640h480,   /**< 320x240 in YCbCr222 */
    w1280h960,  /**< 640x480 in YCbCr222 */
    noRequest,  /**< Default Value for CameraResolutionRequest. This should never be used otherwise! */
  }),

  (Resolutions)(defaultRes) resolutionUpper, /**< the currently used resolution for the upper camera */
  (Resolutions)(defaultRes) resolutionLower, /**< the currently used resolution for the lower camera */
  (unsigned)(0) timestamp,                   /**< timestamp of the last processed CameraResolutionRequest */
});

STREAMABLE_WITH_BASE(CameraResolutionRequest, CameraResolution,
{
  /**
   * Request a resolution change.
   * A resolution change is only possible on a physical robot. Thus, all other modes will return false;
   *
   * @return true if the request is viable.
   */
  bool setRequest(CameraResolution::Resolutions requestedResolutionUpper, CameraResolution::Resolutions requestedResolutionLower),
});
