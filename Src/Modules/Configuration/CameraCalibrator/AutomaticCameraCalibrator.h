/**
 * @file AutomaticCameraCalibrator.h
 *
 * This file declares a module that provides an automatic camera calibration
 * based on the goal area.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/CalibrationRequest.h"
#include "Representations/Configuration/CameraCalibrationStatus.h"
#include "Representations/Configuration/CameraResolutionRequest.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/LineCorrector.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/LEDRequest.h"
#include "Representations/Perception/FieldPercepts/LinesPercept.h"
#include "Representations/Perception/FieldPercepts/PenaltyMarkPercept.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Modeling/CustomCalibrationLines.h"
#include "Framework/Module.h"
#include "Math/GaussNewtonOptimizer.h"

MODULE(AutomaticCameraCalibrator,
{,
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  USES(CustomCalibrationLines),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(LineCorrector),
  REQUIRES(LinesPercept),
  REQUIRES(OptionalECImage),
  REQUIRES(PenaltyMarkPercept),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  REQUIRES(TorsoMatrix),

  PROVIDES(CameraCalibration),
  USES(CameraCalibration),

  REQUIRES(CalibrationRequest),
  PROVIDES(CameraCalibrationStatus),
  PROVIDES(CameraResolutionRequest),

  DEFINES_PARAMETERS(
  {,
    (ENUM_INDEXED_ARRAY(CameraResolutionRequest::Resolutions, CameraInfo::Camera))({CameraResolutionRequest::CameraResolutionRequest::Resolutions::w1280h960, CameraResolutionRequest::CameraResolutionRequest::Resolutions::w1280h960}) resRequest, /**< Last camera resolution requested. */
    (float)(0.00005f) terminationCriterion, /**< If the norm of the parameter vector update is less than this in an optimization step, it is counted as termination step. */
    (unsigned int)(3) minSuccessiveConvergences, /**< The number of successive steps the termination criterion must be fulfilled. */
    (float)(10000.f) notValidError, /**< The error that results from parameters that result in a sample that cannot be projected. */
    (float)(100.f) distanceErrorDivisor, /**< By how much the computed errors regarding distances should be divided. */
    (Angle)(6_deg) angleErrorDivisor, /**< By how much the computed errors regarding angles should be divided. */
    (int)(10) discardsUntilIncrease, /**< How many potential samples have to be discarded until the acceptance limit is raised. */
    (float)(200.f) increase, /**< By how much the acceptance limit should be increased. */
    (Pose2f)(0, -750, 0) validationRobotPose, /**< The position on the field used to validate the calibration. */
    (int)(1000) maxTimeBetweenExecutionCycle, /**< If two images took longer than this time in between, we assume the camera froze. */
    (int)(100) minTimeSinceNoCameraTimeout, /** < In case the camera froze, we want to wait this time (in s). */
  }),
});

class AutomaticCameraCalibrator : public AutomaticCameraCalibratorBase
{
public:
  /** Constructor. */
  AutomaticCameraCalibrator();

private:
  /** This enumeration maps entries in the parameter vector of the optimizer to members of the CameraCalibration representation. */
  ENUM(ParameterTranslation,
  {,
    lowerCameraRollCorrection,
    lowerCameraTiltCorrection,

    upperCameraRollCorrection,
    upperCameraTiltCorrection,

    bodyRollCorrection,
    bodyTiltCorrection,
  });

  /** This struct is the base class for a sample (measurement). */
  struct Sample
  {
    /** Constructor. */
    Sample(const AutomaticCameraCalibrator& calibrator, const TorsoMatrix& torsoMatrix, const RobotModel& robotModel, const CameraInfo& cameraInfo, const ImageCoordinateSystem& coordSys) :
      calibrator(calibrator),
      torsoMatrix(torsoMatrix),
      robotModel(robotModel),
      cameraInfo(cameraInfo),
      coordSys(coordSys)
    {}

    /** Virtual destructor for polymorphism. */
    virtual ~Sample() = default;

    /**
     * Computes the error of this sample with a given camera calibration.
     * @param cameraCalibration The camera calibration from which the camera matrix is to be computed.
     * @return The error (usually between 0 and 1).
     */
    float computeError(const CameraCalibration& cameraCalibration) const;

    /**
     * Computes the error of this sample with a given camera matrix (is different for every sample type).
     * @param cameraMatrix The camera matrix with which the error is to be computed.
     * @return The error (usually between 0 and 1).
     */
    virtual float computeError(const CameraMatrix& cameraMatrix) const = 0;

    const AutomaticCameraCalibrator& calibrator; /**< The owning module. */
    TorsoMatrix torsoMatrix; /**< The torso matrix at the time this sample has been recorded. */
    RobotModel robotModel; /**< The robot model at the time this sample has been recorded. */
    CameraInfo cameraInfo; /**< The camera info at the time this sample has been recorded (from the camera which recorded this sample). */
    ImageCoordinateSystem coordSys; /**< The image coordinate system at the time this sample has been recorded. */
  };

  /** This struct defines a configuration (combination of camera and head angles) in which to record samples. */
  struct SampleConfiguration
  {
    CameraInfo::Camera camera; /**< The camera with which the sample(s) should be taken. */
    Angle headPan; /**< The head pan that should be set while the sample is taken. */
    Angle headTilt; /**< The head tilt that should be set while the sample is taken. */
    unsigned sampleTypes; /**< A bitfield which describes what types of samples should be taken in this configuration. */
    size_t sampleIndexBase; /**< The first index in the samples array that belongs to this configuration. */

    /**
     * Checks whether in this configuration a sample of a given type still needs to be recorded.
     * @param samples The sample array.
     * @param sampleType The sample type to check.
     * @return Whether a sample still needs to be recorded.
     */
    bool needToRecord(const std::vector<std::unique_ptr<Sample>>& samples, SampleType sampleType) const;

    /**
     * Records a sample of a given type.
     * @param samples The sample array.
     * @param sampleType The type of which the recorded sample is.
     * @param sample The sample that has been recorded.
     */
    void record(std::vector<std::unique_ptr<Sample>>& samples, SampleType sampleType, std::unique_ptr<Sample> sample) const;

    /**
     * Checks whether all samples for this configuration have been recorded.
     * @param samples The sample array.
     * @return Whether all samples have been recorded.
     */
    bool samplesExist(const std::vector<std::unique_ptr<Sample>>& samples) const;
  };

  struct CornerAngleSample : Sample
  {
    CornerAngleSample(const AutomaticCameraCalibrator& calibrator, const TorsoMatrix& torsoMatrix, const RobotModel& robotModel,
                      const CameraInfo& cameraInfo, const ImageCoordinateSystem& coordSys, const LineCorrector::Line& cLine1, const LineCorrector::Line& cLine2) :
      Sample(calibrator, torsoMatrix, robotModel, cameraInfo, coordSys), cLine1(cLine1), cLine2(cLine2) {}
    float computeError(const CameraMatrix& cameraMatrix) const override;
    mutable LineCorrector::Line cLine1, cLine2; /**< The corrected goal line or front goal area line & corrected short connecting line. */
  };

  struct ParallelAngleSample : Sample
  {
    ParallelAngleSample(const AutomaticCameraCalibrator& calibrator, const TorsoMatrix& torsoMatrix, const RobotModel& robotModel,
                        const CameraInfo& cameraInfo, const ImageCoordinateSystem& coordSys, const LineCorrector::Line& cLine1, const LineCorrector::Line& cLine2) :
      Sample(calibrator, torsoMatrix, robotModel, cameraInfo, coordSys), cLine1(cLine1), cLine2(cLine2) {}
    float computeError(const CameraMatrix& cameraMatrix) const override;
    mutable LineCorrector::Line cLine1, cLine2; /**< The close goal area connecting line and the far away goal area connecting line. */
  };

  struct ParallelLinesDistanceSample : Sample
  {
    ParallelLinesDistanceSample(const AutomaticCameraCalibrator& calibrator, const TorsoMatrix& torsoMatrix, const RobotModel& robotModel,
                                const CameraInfo& cameraInfo, const ImageCoordinateSystem& coordSys, const LineCorrector::Line& cLine1, const LineCorrector::Line& cLine2) :
      Sample(calibrator, torsoMatrix, robotModel, cameraInfo, coordSys), cLine1(cLine1), cLine2(cLine2) {}
    float computeError(const CameraMatrix& cameraMatrix) const override;
    mutable LineCorrector::Line cLine1, cLine2; /**< The corrected goal line and front goal area line. */
  };

  struct ParallelLinesDistanceShortSample : Sample
  {
    ParallelLinesDistanceShortSample(const AutomaticCameraCalibrator& calibrator, const TorsoMatrix& torsoMatrix, const RobotModel& robotModel,
                                     const CameraInfo& cameraInfo, const ImageCoordinateSystem& coordSys, const LineCorrector::Line& cLine1, const LineCorrector::Line& cLine2) :
      Sample(calibrator, torsoMatrix, robotModel, cameraInfo, coordSys), cLine1(cLine1), cLine2(cLine2) {}
    float computeError(const CameraMatrix& cameraMatrix) const override;
    mutable LineCorrector::Line cLine1, cLine2; /**< The corrected goal line and front goal area line. */
  };

  struct GoalAreaDistanceSample : Sample
  {
    GoalAreaDistanceSample(const AutomaticCameraCalibrator& calibrator, const TorsoMatrix& torsoMatrix, const RobotModel& robotModel,
                           const CameraInfo& cameraInfo, const ImageCoordinateSystem& coordSys, const Vector2i& penaltyMarkInImage, const LineCorrector::Line& cLine) :
      Sample(calibrator, torsoMatrix, robotModel, cameraInfo, coordSys), penaltyMarkInImage(penaltyMarkInImage), cLine(cLine) {}
    float computeError(const CameraMatrix& cameraMatrix) const override;
    Vector2i penaltyMarkInImage; /**< The penalty mark in the image. */
    mutable LineCorrector::Line cLine; /**< The corrected front goal area line. */
  };

  struct GoalLineDistanceSample : Sample
  {
    GoalLineDistanceSample(const AutomaticCameraCalibrator& calibrator, const TorsoMatrix& torsoMatrix, const RobotModel& robotModel,
                           const CameraInfo& cameraInfo, const ImageCoordinateSystem& coordSys, const Vector2i& penaltyMarkInImage, const LineCorrector::Line& cLine) :
      Sample(calibrator, torsoMatrix, robotModel, cameraInfo, coordSys), penaltyMarkInImage(penaltyMarkInImage), cLine(cLine) {}
    float computeError(const CameraMatrix& cameraMatrix) const override;
    Vector2i penaltyMarkInImage; /**< The penalty mark in the image. */
    mutable LineCorrector::Line cLine; /**< The corrected goal line. */
  };

  using Parameters = GaussNewtonOptimizer<numOfParameterTranslations>::Vector;

  /** This struct is used to evaluate the error by the optimizer. */
  struct Functor : public GaussNewtonOptimizer<numOfParameterTranslations>::Functor
  {
    /** Constructor. */
    Functor(AutomaticCameraCalibrator& calibrator) : calibrator(calibrator) {}

    /**
     * Calculates a specific row of the residual vector.
     * @param params The function parameters for which to calculate the residual.
     * @param measurement The component index for which to calculate the residual.
     * @return The residual.
     */
    float operator()(const Parameters& params, size_t measurement) const override;

    /**
     * Returns the number of measurements that are used for optimization.
     * @return The number of measurements.
     */
    size_t getNumOfMeasurements() const override { return calibrator.samples.size(); }

    AutomaticCameraCalibrator& calibrator; /**< The owning module. */
  };
  friend struct Functor;

  /**
   * Calculates the angle between two lines defined by their start and end points.
   */
  static float calculateAngle(const Vector2f& lineAFirst, const Vector2f& lineASecond, const Vector2f& lineBFirst, const Vector2f& lineBSecond);

  /**
   * Updates the CameraCalibrationStatus
   * @param cameraCalibrationStatus The representation updated.
   */
  void update(CameraCalibrationStatus& cameraCalibrationStatus) override;

  /**
   * Updates the CameraCalibrationNext.
   * @param cameraCalibrationNext The representation updated.
   */
  void update(CameraCalibration& cameraCalibration) override;

  /**
   * Updates the CameraResolutionRequest.
   * @param cameraResolutionRequest The representation updated.
   */
  void update(CameraResolutionRequest& cameraResolutionRequest) override;

  /** Records samples from the current image. */
  void recordSamples();

  /** Executes one optimization step and checks for termination. */
  void optimize();

  /**
   * Resets the optimization state.
   * @param finished Whether the optimization is finished or just has to be restarted..
   */
  void resetOptimization(const bool finished);

  /**
   * Packs a camera calibration to a parameter vector for the optimizer.
   * @param cameraCalibration The camera calibration.
   * @return The parameter vector.
   */
  Parameters pack(const CameraCalibration& cameraCalibration) const;

  /**
   * Unpacks a parameter vector from the optimizer to a camera calibration.
   * @param params The parameter vector.
   * @param cameraCalibration The camera calibration.
   */
  void unpack(const Parameters& params, CameraCalibration& cameraCalibration) const;

  void updateSampleConfiguration();

  /**
   * Creates a debug drawing in which all field lines are projected into the image.
   */
  void drawFieldLines() const;

  /**
   * Projects a line given in robot relative field coordinates into the image using an arbitrary camera matrix.
   * @param lineOnField The field line in robot relative coordinates.
   * @param cameraMatrix The camera matrix used for the projection.
   * @param cameraInfo The camera parameters used for the projection.
   * @param lineInImage The field line projected into the image, if this is possible.
   * @return Whether a valid result was computed, which is not the case if the field line lies completely behind the camera plane.
   */
  bool projectLineOnFieldIntoImage(const Geometry::Line& lineOnField, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, Geometry::Line& lineInImage) const;

  CameraCalibrationStatus::State state = CameraCalibrationStatus::idle; /**< The state in which the calibrator currently is. */
  unsigned inStateSince = 0; /**< The frameInfo.time when the current state was set. */

  std::unique_ptr<GaussNewtonOptimizer<numOfParameterTranslations>> optimizer; /**< The optimizer (can be null if the optimization is not running). */
  Functor functor; /**< The functor that calculates the error for the optimizer. */
  Parameters optimizationParameters; /**< The parameters on which the optimizer operates. */
  unsigned successiveConvergences = 0, optimizationSteps = 0; /**<  The successive number of times the termination criterion has been fulfilled (only valid during optimization). */

  std::unique_ptr<SampleConfiguration> currentSampleConfiguration; /**< The sample configuration that is currently recording. */
  int lastSampleConfigurationIndex = -1;

  size_t numOfSamples = 0; /**< The overall number of samples that is needed for the calibration. */
  std::vector<std::unique_ptr<Sample>> samples; /**< The sample array (some may be null). */
  CameraCalibration nextCameraCalibration; /**< The camera calibration which is set as next camera calibration by this module. */
  bool allRequiredFeaturesVisible = true; /**< Whether all currently required features are seen. */

  int numOfDiscardedParallelLines = 0, numOfDiscardedGoalAreaLines = 0, numOfDiscardedGoalLines = 0; /**< How many potential samples have been discarded yet. */
  Rangef parallelDisRangeLower, parallelDisRangeUpper, goalAreaDisRangeLower, goalLineDisRangeLower, goalAreaDisRangeUpper, goalLineDisRangeUpper; /**< The currently allowed min/max distances for the different features. */

  float lowestDelta = std::numeric_limits<float>::max(); /**< The lowest delta value so far achieved in optimization step. */
  Parameters lowestDeltaParameters; /**< The parameters which should be set if the optimization is stopped manually through the converge debug response. */
  float lowestError = std::numeric_limits<float>::max(); /**< The lowest error that was calculated, after the first delta was below the threshold. */
  Parameters lowestErrorParameters; /**< The parameters which should be set after convergence. */

  unsigned noCameraTimeoutSinceTimestamp = 0; /**< The current used camera did not freeze since this timestamp. */
  unsigned lastCameraInfoTimestamp = 0; /**< The current used camera did not freeze since this timestamp. */
};
