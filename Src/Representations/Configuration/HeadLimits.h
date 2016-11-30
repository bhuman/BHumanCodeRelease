/**
 * @file HeadLimits.h
 * Declaration of a struct for representing the limits of the head joints.
 * @author Felix Wenk
 */

#pragma once

#include "Tools/Math/Angle.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Range.h"

struct RobotCameraMatrix;

STREAMABLE(HeadLimits,
{
private:
  bool intersectionWithShoulderPlane(const RobotCameraMatrix& robotCameraMatrix,
                                     const Vector3f& shoulderInOrigin,
                                     Angle imageTilt, Vector3f& intersection) const;

public:
  Rangea getTiltBound(Angle pan) const;

  /**
   * Method to determine whether the image would show mostly parts of the shoulder.
   * @param robotCameraMatrix Position and orientation of the camera in origin coordinates.
   * @param shoulderInOrigin Vector to the shouler in origin coordinates.
   * @param imageTilt 0 for center of image, <0 to move the intersection point upwards in the image, >0 to move it downwards.
   * @return true if the target point specified by imageTilt is hidden by the shoulder.
   */
  bool imageCenterHiddenByShoulder(const RobotCameraMatrix& robotCameraMatrix,
                                   const Vector3f& shoulderInOrigin, Angle imageTilt,
                                   float hysteresis = 0.f) const;

  /**
   * Calculates the upper intersection point of the vertical line through the center of the image
   * and the edge of the circle around the shoulder.
   * @param robotCameraMatrix Position and orientation of the camera in origin coordinates.
   * @param shoulderInOrigin Vector to the shouler in origin coordinates.
   * @param intersection The intersection point in origin coordinates (output parameter).
   * @return true if such an intersecion point exists.
   */
  bool intersectionWithShoulderEdge(const RobotCameraMatrix& robotCameraMatrix,
                                    const Vector3f& shoulderInOrigin, Vector3f& intersection) const;

  Angle maxPan() const { return intervals.back(); }
  Angle minPan() const { return intervals.front(); }

  /**< Draws this representation. */
  void draw() const,

  (float) shoulderRadius,
  (std::vector<Angle>) intervals,
  (std::vector<Angle>) lowerBounds,
  (std::vector<Angle>) upperBounds,
});
