/**
 * @file CameraResolutionRequest.h
 *
 * This file implements a representation to request a resolution change.
 *
 * @author Dana Jenett, Alexis Tsogias, Felix Thielke
 */

#pragma once

#include "Platform/BHAssert.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Streaming/EnumIndexedArray.h"

STREAMABLE(CameraResolutionRequest,
{
  ENUM(Resolutions,
  {,
    defaultRes, /**< Use resolutions as specified in the config file. */
    w320h240,   /**< 160x240 in YUV422 */
    w640h480,   /**< 320x480 in YUV422 */
    w1280h960,  /**< 640x960 in YUV422 */
    w424h240,   /**< 212x240 in YUV422 */
    w480x270,   /**< 240x270 in YUV422 */
    w640h360,   /**< 320x360 in YUV422 */
    w848h480,   /**< 424x480 in YUV422 */
    w1280h720,  /**< 640x720 in YUV422 */
    w1280h800,  /**< 640x800 in YUV422 */
    noRequest,  /**< Keep the resolution as it is. */
  });

  /**
   * Set resolution based on setting for a certain camera.
   * @param camera The resolution for which camera?
   * @param cameraInfo The width and height are written here.
   */
  void apply(const CameraInfo::Camera camera, CameraInfo& cameraInfo) const
  {
    switch(resolutions[camera])
    {
      case CameraResolutionRequest::w320h240:
        cameraInfo.width = 320;
        cameraInfo.height = 240;
        break;
      case CameraResolutionRequest::w424h240:
        cameraInfo.width = 424;
        cameraInfo.height = 240;
        break;
      case CameraResolutionRequest::w480x270:
        cameraInfo.width = 480;
        cameraInfo.height = 270;
        break;
      case CameraResolutionRequest::w640h360:
        cameraInfo.width = 640;
        cameraInfo.height = 360;
        break;
      case CameraResolutionRequest::w640h480:
        cameraInfo.width = 640;
        cameraInfo.height = 480;
        break;
      case CameraResolutionRequest::w848h480:
        cameraInfo.width = 848;
        cameraInfo.height = 480;
        break;
      case CameraResolutionRequest::w1280h720:
        cameraInfo.width = 1280;
        cameraInfo.height = 720;
        break;
      case CameraResolutionRequest::w1280h800:
        cameraInfo.width = 1280;
        cameraInfo.height = 800;
        break;
      case CameraResolutionRequest::w1280h960:
        cameraInfo.width = 1280;
        cameraInfo.height = 960;
        break;
      default:
        FAIL("Unknown resolution.");
        break;
    }
  },

  (ENUM_INDEXED_ARRAY(Resolutions, CameraInfo::Camera))({defaultRes, defaultRes}) resolutions, /**< The desired resolutions */
});
