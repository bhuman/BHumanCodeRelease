/**
 * @file AutomaticCameraCalibratorPA.h
 *
 * This file declares a module that provides an automatic camera calibration
 * based on the penalty area.
 *
 * @author Arne Hasselbring (based on the old AutomaticCameraCalibrator, written by its authors)
 * @author Andreas Baude
 */

#pragma once

#include "Representations/Communication/RobotInfo.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/CameraResolutionRequest.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/LEDRequest.h"
#include "Representations/MotionControl/HeadAngleRequest.h"
#include "Representations/MotionControl/HeadMotionEngineOutput.h"
#include "Representations/Perception/FieldPercepts/LinesPercept.h"
#include "Representations/Perception/FieldPercepts/PenaltyMarkPercept.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Tools/ImageProcessing/Sobel.h"
#include "Tools/Math/Approx.h"
#include "Tools/Module/Module.h"
#include "Tools/Optimization/GaussNewtonOptimizer.h"

STREAMABLE_WITH_BASE(LowerECImage, ECImage, {, });
STREAMABLE_WITH_BASE(UpperECImage, ECImage, {, });

MODULE(AutomaticCameraCalibratorPA,
{,
  REQUIRES(CameraCalibration),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GroundContactState),
  REQUIRES(HeadMotionEngineOutput),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(JointAngles),
  REQUIRES(LowerECImage),
  REQUIRES(LinesPercept),
  REQUIRES(PenaltyMarkPercept),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(TorsoMatrix),
  REQUIRES(UpperECImage),
  PROVIDES(CameraCalibrationNext),
  REQUIRES(CameraCalibrationNext),
  PROVIDES(CameraResolutionRequest),
  PROVIDES(HeadAngleRequest),
  PROVIDES(LEDRequest),
  DEFINES_PARAMETERS(
  {,
    (Angle)(120_deg) headSpeed, /**< The maximum speed with which the head moves between targets (deg/s). */
    (Angle)(0.38f) headTiltUpper, /**< The head tilt angle for recording samples with the upper camera. */
    (Angle)(-.25f) headTiltLower, /**< The head tilt angle for recording samples with the lower camera. */
    (std::vector<Angle>)({0.f, -0.75f, 0.75f}) headPansUpper, /**< The head pan angles for recording samples with the upper camera. */
    (std::vector<Angle>)({0.f, -0.75f, 0.75f}) headPansLower, /**< The head pan angles for recording sample with the lower camera. */
    (int)(4000) minGroundContactDurationForSampling, /**< The minimum duration for which the robot must have had ground contact before sampling. */
    (int)(2000) minRestingHeadDurationForSampling, /**< The minimum duration for which the head must not have been moved before sampling. */
    (float)(0.001f) terminationCriterion, /** If the norm of the parameter vector update is less than this in an optimization step, it is counted as termination step. */
    (unsigned)(3) minSuccessiveConvergations, /**< The number of successive steps the termination criterion must be fulfilled. */
    (float)(10000.f) notValidError, /**< The error that results from parameters that result in a sample that cannot be projected. */
    (int)(1800) numOfAngles, /**< The number of angles that should be considered in the hough lines transformation. */
    (int)(2) minDisImage, /**< Minimum distance that lines in hough space should have in the image */
    (float)(0.25f) sobelThreshValue, /**< Discard pixel in sobelimage smaller than this value multiplied by the maximum value in the sobel image */
    (float)(100.f) distanceErrorDivisor, /**< By how much the computed errors regarding distances should be divided. */
    (Angle)(6_deg) angleErrorDivisor, /**< By how much the computed errors regarding angles should be divided. */
    (int)(25) discardsUntilIncreasement, /**< How many potential samples have to be discarded until the acceptance limit is raised. */
    (float)(200.f) increasement, /**< By how much the acceptance limit should be increased. */
    (Pose2f)(0, -750, 0) validationRobotPose,
  }),
});

class AutomaticCameraCalibratorPA : public AutomaticCameraCalibratorPABase
{
public:
  /** Constructor. */
  AutomaticCameraCalibratorPA();

private:
  /** This enumeration describes the states that this module can be in. */
  ENUM(State,
  {,
    Idle, /**< Nothing special is done. */
    RecordSamples, /**< Samples are constructed from observations. */
    Optimize, /**< The optimization is running (one iteration per frame). */
  });

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

  /** This enumeration lists the possible types of samples. */
  ENUM(SampleType,
  {,
    cornerAngle, /**< An angle in the penalty area that must be 90 degrees. */
    parallelAngle, /**< An angle in the penalty area that must be 180 degrees. */
    parallelLinesDistance, /**< The (known) distance between the groundline and the front penalty area line. */
    penaltyAreaDistance, /**< The (known) distance between the penalty mark and the front penalty area line. */
    groundlineDistance, /**< The (known) distance between the penalty mark and the groundline. */
  });

  /** This struct represents a local maxima in the hough space. */
  struct Maxima
  {
    int maxAcc;         /**< The accumulator value of the local maxima. */
    int angleIndex;     /**< The angle index of the local maxima in the hough space. */
    int distanceIndex;  /**< The distance index of the local maxima in the hough space. */
  };

  /** This struct represents a corrected line with its offset. */
  struct CorrectedLine
  {
    Vector2f aInImage, bInImage; /**< The first and last point of the line in the image. */
    Vector2f aOnField, bOnField; /**< The first and last point of the line on the field. */
    float offset = 0.f;
  };

  /** This struct is the base class for a sample (measurement). */
  struct Sample
  {
    /** Constructor. */
    Sample(const AutomaticCameraCalibratorPA& calibrator, const TorsoMatrix& torsoMatrix, float headYaw, float headPitch, const CameraInfo& cameraInfo) :
      calibrator(calibrator),
      torsoMatrix(torsoMatrix),
      headYaw(headYaw),
      headPitch(headPitch),
      cameraInfo(cameraInfo)
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
    const AutomaticCameraCalibratorPA& calibrator; /**< The owning module. */
    TorsoMatrix torsoMatrix; /**< The torso matrix at the time this sample has been recorded. */
    float headYaw; /**< The head yaw angle at the time this sample has been recorded. */
    float headPitch; /**< The head pitch angle at the time this sample has been recorded. */
    CameraInfo cameraInfo; /**< The camera info at the time this sample has been recorded (from the camera which recorded this sample). */
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
    CornerAngleSample(const AutomaticCameraCalibratorPA& calibrator, const TorsoMatrix& torsoMatrix, float headYaw, float headPitch,
                      const CameraInfo& cameraInfo, const CorrectedLine& cLine1, const CorrectedLine& cLine2) :
      Sample(calibrator, torsoMatrix, headYaw, headPitch, cameraInfo), cLine1(cLine1), cLine2(cLine2) {}
    float computeError(const CameraMatrix& cameraMatrix) const override;
    mutable CorrectedLine cLine1, cLine2; /**< The two corrected lines. */
  };

  struct ParallelAngleSample : Sample
  {
    ParallelAngleSample(const AutomaticCameraCalibratorPA& calibrator, const TorsoMatrix& torsoMatrix, float headYaw, float headPitch,
                        const CameraInfo& cameraInfo, const CorrectedLine& cLine1, const CorrectedLine& cLine2) :
      Sample(calibrator, torsoMatrix, headYaw, headPitch, cameraInfo), cLine1(cLine1), cLine2(cLine2) {}
    float computeError(const CameraMatrix& cameraMatrix) const override;
    mutable CorrectedLine cLine1, cLine2; /**< The two corrected lines. */
  };

  struct ParallelLinesDistanceSample : Sample
  {
    ParallelLinesDistanceSample(const AutomaticCameraCalibratorPA& calibrator, const TorsoMatrix& torsoMatrix, float headYaw, float headPitch,
                                const CameraInfo& cameraInfo, const CorrectedLine& cLine1, const CorrectedLine& cLine2) :
      Sample(calibrator, torsoMatrix, headYaw, headPitch, cameraInfo), cLine1(cLine1), cLine2(cLine2) {}
    float computeError(const CameraMatrix& cameraMatrix) const override;
    mutable CorrectedLine cLine1, cLine2; /**< The two corrected lines. */
  };

  struct PenaltyAreaDistanceSample : Sample
  {
    PenaltyAreaDistanceSample(const AutomaticCameraCalibratorPA& calibrator, const TorsoMatrix& torsoMatrix, float headYaw, float headPitch,
                              const CameraInfo& cameraInfo, const Vector2i& penaltyMarkInImage, const CorrectedLine cLine) :
      Sample(calibrator, torsoMatrix, headYaw, headPitch, cameraInfo), penaltyMarkInImage(penaltyMarkInImage), cLine(cLine) {}
    float computeError(const CameraMatrix& cameraMatrix) const override;
    Vector2i penaltyMarkInImage; /**< The penalty mark in the image. */
    mutable CorrectedLine cLine; /**< The corrected line. */
  };

  struct GroundlineDistanceSample : Sample
  {
    GroundlineDistanceSample(const AutomaticCameraCalibratorPA& calibrator, const TorsoMatrix& torsoMatrix, float headYaw, float headPitch,
                             const CameraInfo& cameraInfo, const Vector2i& penaltyMarkInImage, const CorrectedLine cLine) :
      Sample(calibrator, torsoMatrix, headYaw, headPitch, cameraInfo), penaltyMarkInImage(penaltyMarkInImage), cLine(cLine) {}
    float computeError(const CameraMatrix& cameraMatrix) const override;
    Vector2i penaltyMarkInImage; /**< The penalty mark in the image. */
    mutable CorrectedLine cLine; /**< The corrected line. */
  };

  using Parameters = GaussNewtonOptimizer<numOfParameterTranslations>::Vector;

  /** This struct is used to evaluate the error by the optimizer. */
  struct Functor : public GaussNewtonOptimizer<numOfParameterTranslations>::Functor
  {
    /** Constructor. */
    Functor(AutomaticCameraCalibratorPA& calibrator) : calibrator(calibrator) {}

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

    AutomaticCameraCalibratorPA& calibrator; /**< The owning module. */
  };
  friend struct Functor;

  /**
   * Updates the CameraCalibrationNext.
   * @param cameraCalibrationNext The representation updated.
   */
  void update(CameraCalibrationNext& cameraCalibrationNext) override;

  /**
   * Updates the HeadAngleRequest.
   * @param headAngleRequest The representation updated.
   */
  void update(HeadAngleRequest& headAngleRequest) override;

  /**
   * Updates the CameraResolutionRequest.
   * @param cameraResolutionRequest The representation updated.
   */
  void update(CameraResolutionRequest& cameraResolutionRequest) override;

  /**
   * Updates the LEDRequest.
   * @param ledRequest The representation updated.
   */
  void update(LEDRequest& ledRequest) override;

  /** Fills the sinus/cosinus lookup tables. */
  void createLookUpTables();

  /**
   * Corrects the given start and end point of a line using the hough lines transformation.
   * To avoid additional error potential, only the top/bottom edge of a line is searched for and an offset
   * is set accordingly, instead of using the mean of the top and bottom edges.
   * @param cline The line to be corrected.
   * @return Whether the line could be corrected or not.
   */
  bool fitLine(CorrectedLine& cline);

  /**
   * Extracts a grayscaled image patch with given start pixel and size.
   * @param start The start pixel of the desired image patch.
   * @param size The desired size of the image patch.
   * @param grayImage the image to be filled
   */
  void extractImagePatch(const Vector2i& start, const Vector2i& size, Sobel::Image1D& grayImage);

  /**
   * Determines a suitable threshold at which a pixel in the hough transformation is assumed to be an edge given the sobel image.
   * @param sobelImage The sobel image used in the hough transformation.
   * @return The selected threshold value.
   */
  float determineSobelThresh(const Sobel::SobelImage& sobelImage);

  /**
   * Performs the hough lines transformation to find lines in the given sobel image.
   * @param sobelImage The sobel image in which lines should be searched.
   * @param minIndex The minimum angle index to be considered.
   * @param maxIndex The maximum angle index to be considered.
   * @param dMax The maximum distance a line can possibly have in the given sobel image.
   * @param thresh The value at which a pixel in the sobel image is assumed to be a edge.
   * @param houghSpace The hough space to be calculated.
   */
  void calcHoughSpace(const Sobel::SobelImage& sobelImage, const int minIndex, const int maxIndex,
                      const int dMax, std::vector<std::vector<int> >& houghSpace);

  /**
   * Searches the hough space for local maxima
   * @param houghSpace The hough space to be searched.
   * @param minIndex The minimum angle index from which the hough space is to be considered.
   * @param maxIndex The maximum angle index up to which the hough space is to be considered.
   * @param localMaximas The Container in which the local maximas should be stored.
   */
  void determineLocalMaxima(const std::vector<std::vector<int> >& houghSpace, const int minIndex,
                            const int maxIndex, std::vector<Maxima>& localMaximas);

  /** Records samples from the current image. */
  void recordSamples();

  /** Executes one optimization step and checks for termination. */
  void optimize();

  /**
   * Resets the optimization state.
   * @param finished Wheter the optimization variables should be resetted cause the optimization finished.
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

  /**
   * Adds a sample configuration and increases the number of samples accordingly.
   * @param camera The camera with which samples should be recorded.
   * @param headPan The head pan angle that should be set in this configuration.
   * @param headTilt The head tilt angle that should be set in this configuration.
   * @param sampleTypes The sample types that should be recorded in this configuration.
   */
  void addSampleConfiguration(CameraInfo::Camera camera, float headPan, float headTilt, unsigned sampleTypes);

  void drawFieldLines() const;
  bool projectLineOnFieldIntoImage(const Geometry::Line& lineOnField, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, Geometry::Line& lineInImage) const;

  State state; /**< The state in which the calibrator currently is. */
  std::unique_ptr<GaussNewtonOptimizer<numOfParameterTranslations>> optimizer; /**< The optimizer (can be null if the optimization is not running). */
  Functor functor; /**< The functor that calculates the error for the optimizer. */
  Parameters optimizationParameters; /**< The parameters on which the optimizer operates. */
  unsigned successiveConvergations = 0, optimizationSteps = 0; /**<  The successive number of times the termination criterion has been fulfilled (only valid during optimization). */

  std::vector<SampleConfiguration> sampleConfigurations; /**< The sample configurations in the order they are used. */
  std::vector<SampleConfiguration>::const_iterator currentSampleConfiguration; /**< The sample configuration that is currently recording. */
  size_t numOfSamples = 0; /**< The overall number of samples that is needed for the calibration. */
  std::vector<std::unique_ptr<Sample>> samples; /**< The sample array (some may be null). */
  CameraCalibration nextCameraCalibration; /**< The camera calibration which is set as next camera calibration by this module. */
  unsigned lastTimeWithoutGroundContact = 0; /**< The last time when the robot had no ground contact. */
  unsigned lastTimeWhenHeadMoved = 0; /**< The last time when the head was moving. */
  bool allowSampling = true; /**< Whether it is allowed to take samples (only used in the simulator). */
  bool allRequiredFeaturesVisible = true;

  int numOfDiscardedPossibleParallelLines = 0, numOfDiscardedPossiblePenaltyLines = 0, numOfDiscardedPossibleGroundLines = 0;
  Rangef parallelDisRangeLower, parallelDisRangeUpper, penaltyDisRangeLower, groundlineDisRangeLower, penaltyDisRangeUpper, groundlineDisRangeUpper;

  std::vector<float> cosAngles, sinAngles; /**< The sinus/cosinus lookup tables used in the hough lines transformation. */
  float lowestDelta = std::numeric_limits<float>::max(); /**< The lowest delta value so far achieved in optimization step. */
  Parameters lowestDeltaParameters; /**< The parameters which should be set if the optimization is stopped manually through the converg debug response. */
};
