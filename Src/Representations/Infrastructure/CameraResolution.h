/**
* @file CameraResolution.h
*
* This file implements a representation for the camera resolutions and an interface to request a resolution change.
*
* @author Dana Jenett, Alexis Tsogias
*/

#pragma once

#include "Tools/Enum.h"
#include "Tools/Streams/Streamable.h"

class CameraInfo;

class CameraResolution : public Streamable
{
public:
  ENUM(Resolutions,
    defaultRes, /**< Use resolutions as specified in the config file. */
    upper640,   /**< Upper resolution is 640 and lower is 320. */
    lower640,   /**< Upper resolution is 320 and lower is 640. */
    both320,    /**< Both resolutions are set to 320. This is the default case when running on SimRobot. */
    both640,    /**< Both resolutions are set to 640. This option is only valide in the simulator. */
    noRequest   /**< Default Value for CameraResolutionRequest. This should never be used otherwise! */
  );

  Resolutions resolution = defaultRes; /**< the currently used resolutions */
  unsigned timestamp = 0; /** A timestamp for the last procesed CameraResolutionRequest */

private:
  virtual void serialize(In* in, Out* out);
};

class CameraResolutionRequest : public Streamable
{
private:
  CameraResolution::Resolutions resolution = CameraResolution::noRequest; /**< The requested resolution. */
  unsigned timestamp = 0; /**< Thimestamp of the request. */

public:
  /**
   * Returns the requested resolution.
   */
  CameraResolution::Resolutions getRequest() const {
    return resolution;
  }

  /**
   * Returns the timestamp of the resolution request.
   */
  unsigned getTimestamp() const {
    return timestamp;
  }

  /**
   * Request a resolution change.
   * A resolution change is only possible on a phisical robot. Thus, all other modes will return false;
   * 
   * @return true if the request is viable.
   */
  bool setRequest(CameraResolution::Resolutions requestedResolution);


private:
  virtual void serialize(In* in, Out* out);
};