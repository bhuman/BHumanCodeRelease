/**
 * @file CameraCalibrationTester.h
 * @author René Schröder
 */

#pragma once

#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/CameraCalibrationRating.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/FieldPercepts/LinesPercept.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/CameraResolution.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Module/Module.h"

#include <algorithm>

MODULE(CameraCalibrationTester,
{,
  REQUIRES(LinesPercept),
  REQUIRES(FieldDimensions),
  REQUIRES(CameraCalibration),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraResolution),
  REQUIRES(KeyStates),
  REQUIRES(FrameInfo),
  REQUIRES(RobotPose),
  PROVIDES(CameraResolutionRequest),
  PROVIDES(CameraCalibrationRating),
  DEFINES_PARAMETERS(
  {,
    (float)(8.f) maxErrorUpper,
    (float)(6.f) maxErrorLower,
    (int)(30) minDis,
  }),
});

class CameraCalibrationTester : public CameraCalibrationTesterBase
{
private:
  void update(CameraCalibrationRating& theCameraCalibrationRating);
  void update(CameraResolutionRequest& theCameraResolutionRequest);

  /**
   * This method projects a line given in robot relative field coordinates into
   * the image using an arbitrary camera matrix.
   * @param lineOnField The field line in robot relative coordinates.
   * @param cameraMatrix The camera matrix used for the projection.
   * @param cameraInfo The camera parameters used for the projection.
   * @param lineInImage The field line projected into the image, if this is possible.
   * @return Whether a valid result was computed, which is not the case if the field line lies completely behind the camera plane.
   */
  bool projectLineOnFieldIntoImage(const Geometry::Line& lineOnField, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, Geometry::Line& lineInImage) const;

  /**
   * This method extracts sample-points from fieldlines and puts them into the given std::vector.
   * @param samples The std::vector to be filled with sample-points
   */
  void calcSamples(std::vector<Vector2i>& samples) const;

  /**
   * This method assembles an output for the console and plays a sound wether the camera calibration is good or bad.
   * @param isGood Is the camera calibration good or bad?
   * @param errorUpper The error for the upper camera
   * @param errorLower The error for the lower camera
   */
  void evalOutput(const bool& isGood, const float* errorUpper, const float* errorLower) const;

  /**
   * This method evaluates the camera calibration based on the calculated errors for the cameras.
   * @param theCameraCalibrationRating Representation that is filled with the results of the evaluation
   */
  void evaluate(CameraCalibrationRating& theCameraCalibrationRating) const;

  /**
   * This method calculates the errors which is basically the reprojection error
   * @param curCamIsUpper Is the current camera the upper camera?
   */
  void testPoints(const bool& curCamIsUpper);

  /**
   * This method changes the resolutions of the cameras to get better results later on.
   * @param curCamIsUpper Is the current camera the upper camera?
   */
  void setResolution(const bool& curCamIsUpper);

  /**
   * This method draws the fieldLines from the fieldDimensions on the image.
   */
  void draw() const;

  /**
   * This method resets all containers.
   */
  void abort();

  std::vector<float> valuesUpper;
  std::vector<float> valuesLower;
  bool aborted = false;

  enum class NextResolution
  {
    none,
    hiResUpper,
    hiResLower
  }
  nextResolution = NextResolution::none;
};
