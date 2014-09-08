/**
* @file CameraInfo.h
*
* Declaration of class CameraInfo
*
* @author <a href="mailto:juengel@informatik.hu-berlin.de">Matthias Jüngel</a>
* @author <a href="mailto:walter.nistico@uni-dortmund.de">Walter Nistico</a>
*/

#pragma once

#include "Tools/Math/Vector2.h"
#include "Tools/Enum.h"

/**
* Information about the camera which provides the images for the robot
*/
class CameraInfo : public Streamable
{
public:
  /**
   * @enum Camera
   * Enum representing the possible sources of an image.
   */
  ENUM(Camera,
    upper,
    lower
  );

  Camera camera;
  int width;
  int height;
  float openingAngleWidth;
  float openingAngleHeight;
  Vector2<> opticalCenter;

  /** Intrinsic camera parameters: axis skew is modelled as 0 (90¬∞ perfectly orthogonal XY)
  * and the same has been modeled for focal axis aspect ratio; distortion is considering
  * only 2nd and 4th order coefficients of radial model, which account for about 95% of total.
  */
  float focalLength;
  float focalLengthInv; // (1/focalLength) used to speed up certain calculations
  float focalLenPow2;

  CameraInfo() = default;

  CameraInfo(Camera camera) :
    camera(camera)
  {}

  void updateFocalLength();

private:
  virtual void serialize(In* in, Out* out);
};

class CameraInfoFullRes : public CameraInfo
{
public:
  CameraInfoFullRes() = default;

  explicit CameraInfoFullRes(const CameraInfo& other) :
    CameraInfo(other)
  {
    width *= 2;
    height *= 2;
    opticalCenter *= 2.f;
    updateFocalLength();
  };

  CameraInfoFullRes& operator=(const CameraInfo& other)
  {
    camera = other.camera;
    width = other.width * 2;
    height = other.height * 2;
    openingAngleWidth = other.openingAngleWidth;
    openingAngleHeight = other.openingAngleHeight;
    opticalCenter = other.opticalCenter * 2.f;
    updateFocalLength();
    return *this;
  };
};
