/**
 * @file CameraCalibrator.h
 *
 * This file implements a module that can provide a semiautomatic camera calibration.
 *
 * @author Alexander Härtl
 */

#pragma once

#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Module/Module.h"
#include "Tools/Optimization/GaussNewtonOptimizer.h"

#include <algorithm>

MODULE(CameraCalibrator,
{,
  REQUIRES(CameraInfo),
  REQUIRES(FieldDimensions),
  REQUIRES(JointAngles),
  REQUIRES(RobotDimensions),
  REQUIRES(TorsoMatrix),
  PROVIDES(CameraCalibration),
  DEFINES_PARAMETERS(
  {,
    (float)(0.001f) terminationCriterion, /**< The difference of two succesive parameter sets that are taken as a convergation */
    (float)(1000000.f) aboveHorizonError, /**< The error for a sample the error of which cannot be computed regularly */
    (unsigned)(15) numOfFramesToWait, /**< The number of frames to wait between two iterations (necessary to keep the debug connection alive) */
    (unsigned)(5) minSuccessiveConvergations, /**< The number of consecutive iterations that fulfil the termination criterion to converge */
  }),
});

class CameraCalibrator : public CameraCalibratorBase
{
private:
  /**
   * The current state of the calibrator.
   */
  ENUM(State,
  {,
    Idle,
    Accumulate,
    Optimize,
  });

  /**
   * This enum is used to translate between the indices of the parameter vector used in the
   * optimizer and their actual meaning.
   */
  ENUM(ParameterTranslation,
  {,
    lowerCameraRollCorrection,
    lowerCameraTiltCorrection,
    //lowerCameraPanCorrection,

    upperCameraRollCorrection,
    upperCameraTiltCorrection,
    //upperCameraPanCorrection,

    bodyRollCorrection,
    bodyTiltCorrection,

    robotPoseXCorrection,
    robotPoseYCorrection,
    robotPoseRotationCorrection,
  });

  /**
   * A class representing a single reference point within the calibration procedure.
   * It contains all information necessary to construct a camera matrix for the point
   * in time the sample was taken using an arbitrary camera calibration.
   */
  class Sample
  {
  public:
    Vector2i pointInImage = Vector2i::Zero();
    Vector2f pointOnField = Vector2f::Zero();   /**< For drawing */
    TorsoMatrix torsoMatrix;
    float headYaw, headPitch;
    CameraInfo cameraInfo;
  };

  using Parameters = GaussNewtonOptimizer<numOfParameterTranslations>::Vector;

  struct Functor2 : public GaussNewtonOptimizer<numOfParameterTranslations>::Functor
  {
    CameraCalibrator& calibrator;

    Functor2(CameraCalibrator& calibrator) : calibrator(calibrator) {};

    /**
     * This method computes the error value for a sample and a parameter vector.
     * @param params The parameter vector for which the error should be evaluated.
     * @param measurement The i-th measurement for which the error should be computed.
     * @return The error.
     */
    float operator()(const Parameters& params, size_t measurement) const;

    size_t getNumOfMeasurements() const { return calibrator.samples.size(); };
  };
  Functor2 functor;
  friend struct Functor2;

  State state; /**< The state of the calibrator. */
  std::function<void()> states[numOfStates];

  // overall variables
  Pose2f currentRobotPose;                     /**< the pose used to set the calibration points */
  CameraMatrix theCameraMatrix;                   /**< The camera matrix that fits the current image (not the current camera) */
  std::vector<Sample> samples; /**< The set of samples used to calibrate the camera. */

  // accumulation variables
  Vector2i lastFetchedPoint = Vector2i::Zero(); /**< The coordinates of the last fetched point in the image. */
  CameraInfo::Camera currentCamera;           /**< The camera that is currently used for selecting points. */
  Vector2i currentPoint = Vector2i::Zero();

  // optimization variables
  GaussNewtonOptimizer<numOfParameterTranslations>* optimizer = nullptr; /**< A pointer to the currently used optimizer, or nullptr if there is none. */
  Parameters optimizationParameters;
  CameraCalibration nextCameraCalibration;
  unsigned successiveConvergations; /**< The number of consecutive iterations that fulfil the termination criterion. */
  int framesToWait; /**< The remaining number of frames to wait for the next iteration. */

public:
  CameraCalibrator();
  ~CameraCalibrator();

private:
  void idle();
  void accumulate();
  void optimize();

  /**
   * The method to calculate the new camera calibration, depending on the state of the calibrator.
   * @param cameraCalibration The current calibration of the robot's camera.
   */
  void update(CameraCalibration& cameraCalibration);

  /**
   * Processes the debug responces to manual set the state of the
   * configurator
   */
  void processManualControls();

  /**
   * Declares the drawings of the configurator
   */
  void draw() const;

  /**
   * This method creates a debug drawing in which all field lines are projected into the image.
   */
  void drawFieldLines() const;

  /**
   * This method creates a debug drawing in which all selected points are projected into the image.
   */
  void drawSamples() const;

  /**
   * This method computes the distance of a sampled point to the next field line, either in image
   * image coordinates using the back projection of the field lines into the image, or in field
   * coordinates using the projection of the point onto the field plane. The error is computed
   * using the given camera calibration from which a modified camera matrix is built.
   * @param sample The sample point for which the distance / error should be computed.
   * @param cameraCalibration The camera calibration used to compute the error.
   * @param robotPose The assumed robot pose.
   * @param inImage Whether the distance in image or in field coordinates should be computed.
   * @return The distance.
   */
  float computeError(const Sample& sample, const CameraCalibration& cameraCalibration,
                     const Pose2f& robotPose, bool inImage = true) const;

  /**
   * This method packs a camera calibration and a robot pose into a parameter vector.
   * @param cameraCalibration The camera calibration to be translated.
   * @param robotPose The robot pose to be translated.
   */
  Parameters pack(const CameraCalibration& cameraCalibration, const Pose2f& robotPose) const;

  /**
   * This method unpacks a parameter vector into a camera calibration and a robot pose.
   * @param params The parameter vector to be unpacked.
   * @param cameraCalibration The camera calibration the values of which are set to the corresponding values in the parameter vector.
   * @param robotPose The robot pose the values of which are set to the corresponding values in the parameter vector.
   */
  void unpack(const Parameters& params, CameraCalibration& cameraCalibration, Pose2f& robotPose) const;

  /**
   * This method projects a line given in robot relative field coordinates into
   * the image using an arbitrary camera matrix.
   * @param lineOnField The field line in robot relative coordinates.
   * @param cameraMatrix The camera matrix used for the projection.
   * @param cameraInfo The camera parameters used for the projection.
   * @param lineInImage The field line projected into the image, if this is possible.
   * @return Whether a valid result was computed, which is not the case if the field line lies completely behind the camera plane.
   */
  bool projectLineOnFieldIntoImage(const Geometry::Line& lineOnField, const CameraMatrix& cameraMatrix,
                                   const CameraInfo& cameraInfo, Geometry::Line& lineInImage) const;
};
