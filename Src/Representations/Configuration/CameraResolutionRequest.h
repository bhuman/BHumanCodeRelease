/**
 * @file CameraResolutionRequest.h
 *
 * This file implements a representation to request a resolution change.
 *
 * @author Dana Jenett
 * @author Alexis Tsogias
 * @author Felix Thielke
 */

#pragma once

#include "Platform/BHAssert.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Streaming/EnumIndexedArray.h"

STREAMABLE(CameraResolutionRequest,
{
  ENUM(Resolution,
  {,
    defaultRes,    /**< Use resolutions as specified in the config file. */

    // RealSense
    w424h240f5,    /**< 212x240 in YUV422 at  5 Hz */
    w424h240f15,   /**< 212x240 in YUV422 at 15 Hz */
    w424h240f30,   /**< 212x240 in YUV422 at 30 Hz */
    w424h240f60,   /**< 212x240 in YUV422 at 60 Hz */
    w424h240f90,   /**< 212x240 in YUV422 at 90 Hz */

    w480h270f5,    /**< 240x270 in YUV422 at  5 Hz */
    w480h270f15,   /**< 240x270 in YUV422 at 15 Hz */
    w480h270f30,   /**< 240x270 in YUV422 at 30 Hz */
    w480h270f60,   /**< 240x270 in YUV422 at 60 Hz */
    w480h270f90,   /**< 240x270 in YUV422 at 90 Hz */

    // MIPI
    w544h448f30,   /**< 272x224 in YUV422 at 30 Hz */

    // X5
    w544h448f60,   /**< 272x224 in YUV422 at 60 Hz */

    // RealSense
    w640h360f5,    /**< 320x360 in YUV422 at  5 Hz */
    w640h360f15,   /**< 320x360 in YUV422 at 15 Hz */
    w640h360f30,   /**< 320x360 in YUV422 at 30 Hz */
    w640h360f60,   /**< 320x360 in YUV422 at 60 Hz */
    w640h360f90,   /**< 320x360 in YUV422 at 90 Hz */

    // RealSense
    w640h480f5,    /**< 320x480 in YUV422 at  5 Hz */
    w640h480f15,   /**< 320x480 in YUV422 at 15 Hz */
    w640h480f30,   /**< 320x480 in YUV422 at 30 Hz */

    // RealSense
    w640h480f60,   /**< 320x480 in YUV422 at 60 Hz */

    w848h480f5,    /**< 424x480 in YUV422 at  5 Hz */
    w848h480f15,   /**< 424x480 in YUV422 at 15 Hz */
    w848h480f30,   /**< 424x480 in YUV422 at 30 Hz */
    w848h480f60,   /**< 424x480 in YUV422 at 60 Hz */

    w1280h720f5,   /**< 640x720 in YUV422 at  5 Hz */
    w1280h720f15,  /**< 640x720 in YUV422 at 15 Hz */
    w1280h720f30,  /**< 640x720 in YUV422 at 30 Hz */

    // RealSense
    w1280h800f5,   /**< 640x800 in YUV422 at 5 Hz  */
    w1280h800f15,  /**< 640x800 in YUV422 at 15 Hz */
    w1280h800f30,  /**< 640x800 in YUV422 at 30 Hz */

    noRequest,     /**< Keep the resolution as it is. */
  });

  /**
   * Set resolution based on setting for the camera.
   * @param cameraInfo The width, height, and frameRate are written here.
   */
  void apply(CameraInfo& cameraInfo) const
  {
    VERIFY(sscanf(TypeRegistry::getEnumName(resolution), "w%dh%df%d",
                  &cameraInfo.width, &cameraInfo.height, &cameraInfo.frameRate) == 3);
  },

  (Resolution)(defaultRes) resolution, /**< The desired resolutions */
});
