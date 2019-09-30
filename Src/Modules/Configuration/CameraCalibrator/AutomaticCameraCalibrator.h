/**
 * @file AutomaticCameraCalibrator.h
 *
 * This file implements a module that can provide an automatic camera calibration.
 *
 * @author Dana Jenett, Alexis Tsogias
 */

#pragma once

#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/CameraResolutionRequest.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/HeadAngleRequest.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/FieldPercepts/LinesPercept.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Tools/RingBufferWithSum.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Module/Module.h"
#include "Tools/Optimization/GaussNewtonOptimizer.h"
#include <algorithm>

MODULE(AutomaticCameraCalibrator,
{,
  REQUIRES(CameraCalibration),
  REQUIRES(CameraInfo),
  REQUIRES(FieldDimensions),
  REQUIRES(JointAngles),
  REQUIRES(LinesPercept),
  REQUIRES(RobotDimensions),
  REQUIRES(TorsoMatrix),
  REQUIRES(RobotPose),
  PROVIDES(CameraCalibrationNext),
  REQUIRES(CameraCalibrationNext),
  PROVIDES(CameraResolutionRequest),
  PROVIDES(HeadAngleRequest),
  DEFINES_PARAMETERS(
  {,
    (float)(0.001f) terminationCriterion, /**< The difference of two succesive parameter sets that are taken as a convergation */
    (float)(1000000.f) aboveHorizonError, /**< The error for a sample the error of which cannot be computed regularly */
    (unsigned)(10) numOfFramesToWait, /**< The number of frames to wait between two iterations (necessary to keep the debug connection alive) */
    (unsigned)(5) minSuccessiveConvergations, /**< The number of consecutive iterations that fulfil the termination criterion to converge */
    (float)(1.f) waitForUserTime, /** (in s) */
    (std::vector<float>)({2.1f, 1.0f, 0.1f, -1.0f, -2.1f}) headPans,
    (float)(0.38f) upperCameraHeadTilt,
    (float)(-0.25f) lowerCameraHeadTilt,
    (float)(0.5f) headSpeed, /** speed of headmovement*/
    (float)(0.5f) headMotionWaitTime,  /** time the robot has to wait to change the headposition (in s) */
    (unsigned)(300) minimumSampleDistance,  /** the minimum field distance of samples to each other */
    (float)(5.f) deletionThreshold,
  }),
});

class AutomaticCameraCalibrator : public AutomaticCameraCalibratorBase
{
private:
  /**
   * The current state of the calibrator.
   */
  ENUM(State,
  {,
    Idle,
    Init,
    MoveHead,
    WaitForCamera,
    WaitForHeadToReachPosition,
    WaitForAccumulate,
    Accumulate,
    WaitForUser,
    WaitForOptimize,
    Optimize,
    ManualManipulation,
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
  struct Sample
  {
    Vector2i pointInImage;
    Vector2f pointOnField;   /**< For drawing */
    TorsoMatrix torsoMatrix;
    float headYaw, headPitch;
    CameraInfo cameraInfo;
  };

  using Parameters = GaussNewtonOptimizer<numOfParameterTranslations>::Vector;

  struct Functor2 : public GaussNewtonOptimizer<numOfParameterTranslations>::Functor
  {
    AutomaticCameraCalibrator& calibrator;

    Functor2(AutomaticCameraCalibrator& calibrator) : calibrator(calibrator) {};

    /**
     * This method computes the error value for a sample and a parameter vector.
     * @param params The parameter vector for which the error should be evaluated.
     * @param measurement The i-th measurement for which the error should be computed.
     * @return The error.
     */
    float operator()(const Parameters& params, size_t measurement) const override;

    size_t getNumOfMeasurements() const override { return calibrator.samples.size(); };
  };
  Functor2 functor;
  friend struct Functor2;

  State state; /**< The state of the calibrator. */
  std::function<void()> states[numOfStates];

  // overall variables
  const CameraInfo::Camera startCamera = CameraInfo::lower;
  CameraInfo::Camera currentCamera = startCamera; /**< The camera that is currently used for selecting points. */
  Pose2f currentRobotPose; /**< the pose used to set the calibration points */
  CameraMatrix theCameraMatrix; /**< The camera matrix that fits the current image (not the current camera) */
  std::vector<Sample> samples; /**< The set of samples used to calibrate the camera. */
  CameraCalibration startingCameraCalibration;

  // head movement variables
  std::vector<float> firstHeadPans;
  std::vector<float> secondHeadPans;
  float currentHeadPan;
  float currentHeadTilt;
  HeadAngleRequest nextHeadAngleRequest;
  RingBufferWithSum<float, 5> headPositionPanBuffer;
  RingBufferWithSum<float, 5> headPositionTiltBuffer;
  unsigned int waitTill = 0; //For timing

  // accumulation variables
  unsigned accumulationTimestamp;
  bool lastActionWasInsertion = false; //For the undo mechanism
  //for deleting unwanted samples
  Vector2i unwantedPoint = Vector2i::Zero(); /**< The sample to be deleted. */
  CameraInfo::Camera deletionOnCamera = CameraInfo::lower; /**< The camera which detected the sample to delete. */
  Sample lastDeletedSample; /**< The last sample the calibrator has deleted.  */
  bool alreadyRevertedDeletion = true; /**< Have you already done undo?*/
  //for inserting wanted samples
  Vector2i wantedPoint = Vector2i::Zero(); /**< The sample to be inserted. */
  CameraInfo::Camera insertionOnCamera = CameraInfo::lower; /**< The camera which detected the sample to insert. */
  Sample lastInsertedSample; /**< The last sample the calibrator has inserted.  */
  bool alreadyRevertedInsertion = true; /**< Have you already done undo?*/
  bool insertionValueExistant = false, deletionValueExistant = false;

  // optimization variables
  std::unique_ptr<GaussNewtonOptimizer<numOfParameterTranslations>> optimizer;
  Parameters optimizationParameters;
  CameraCalibration nextCameraCalibration;
  unsigned successiveConvergations; /**< The number of consecutive iterations that fulfil the termination criterion. */
  int framesToWait; /**< The remaining number of frames to wait for the next iteration. */
  bool setJointOffsets = false;

public:
  AutomaticCameraCalibrator();

private:
  void idle();
  void init();
  void moveHead();
  void waitForCamera();
  void waitForHeadToReachPosition();
  void accumulate();
  void waitForUser();
  void optimize();
  void listen();

  /**
   * Automatically inverts the BodyRotationCorrection so the user does not have to do it.
   * @param the current cameracalibration
   */
  void invert(const CameraCalibration& cameraCalibration);

  void abort();

  void update(CameraCalibrationNext& cameraCalibrationNext) override;
  void update(CameraResolutionRequest& cameraResolutionRequest) override;
  void update(HeadAngleRequest& headAngleRequest) override;

  /**
   * Delete the sample which has the same relative to robot positions as the point transformed and has
   * been collected on the correct camera.
   * @param point, the sample position on the current image
   * @paramm camera, the camera on which sample has been clicked
   */
  void deleteSample(Vector2i point, CameraInfo::Camera camera);

  /** Revert the last deletion or insertion. */
  void undo();

  /**
   * Insert a sample with the same data as the clicked point
   * @param point, the sample position on the current image
   * @paramm camera, the camera on which sample has been clicked
   */
  void insertSample(Vector2i point, CameraInfo::Camera camera);

  /**
   * Processes the debug responces to manual set the state of the
   * configurator
   */
  void processManualControls();

  /** Declares the drawings of the configurator */
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
