/**
 */

#pragma once

#include "Framework/Module.h"

#include "Representations/Infrastructure/CameraImage.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"

#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Perception/ImagePreprocessing/BodyContour.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ObstaclePercepts/RobotPercept.h"

#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Perception/ImagePreprocessing/FieldBoundary.h"
#include "Representations/Perception/MeasurementCovariance.h"
#include "Representations/Perception/ObstaclePercepts/JerseyClassifier.h"

#include <CompiledNN2ONNX/CompiledNN.h>
#include <CompiledNN2ONNX/Model.h>

#include <cstdint>
#include <memory>
#include <vector>
#include <algorithm>

MODULE(RobotBallPerceptor,
{,
  REQUIRES(BallSpecification),
  REQUIRES(BodyContour),
  REQUIRES(CameraImage),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(ECImage),
  REQUIRES(FieldBoundary),
  REQUIRES(JerseyClassifier),
  REQUIRES(MeasurementCovariance),
  REQUIRES(RobotDimensions),

  PROVIDES(BallPercept),
  PROVIDES(RobotPercept),
  DEFINES_PARAMETERS(
  {,
    (float)(3.f) minObstacleToBallRatio, /**< The obstacle area of an obstacle should be three times bigger than the ball */
    (int)(10) numOfBodyContourSamples, /**< Number of samples taken when checking if a detected robot is this robot itself */
    (float)(0.5f) minRatioAboveBodyContour,
    (float)(0.45f) boxConfidenceThreshold, /**< Minimum confidence of a bounding box to be used */
    (float)(0.5f) iouThreshold, /**< If the IOU of two boxes is lower, the two boxes probably point to separate objects */
    (float)(200.f) distanceForBallCenterEstimation, /**< If the ball center is this distance away from the center, estimate the actual center of the ball */
    (float)(0.5f) ballPixelRadiusFactor, /**< Ball radius times this factor is used to check, if the detected ball is too big or small in the image. */
    (float)(2.f) ballPixelRadiusBaseOffset, /**< Base offset added to check, whether the ball is too big or small in the image. */
  }),
});

class RobotBallPerceptor : public RobotBallPerceptorBase
{
private:
  /**
   * Different object types that the Object detection model can see.
   * The order should be the same as in the output of the Model.
   */
  ENUM(ObjectType,
  {,
    ball,
    robot,
  });

  /**
   * Represents a prediction that the Object detection model makes
   */
  struct Prediction;

  /**
   * Used to load and compile the model
   */
  std::unique_ptr<NeuralNetworkONNX::Model> model;
  NeuralNetworkONNX::CompiledNN compiled;

  // TODO: Remove this
  uint32_t networkInputSize = 0;

  /** Predictions that are good, i.e have not been filtered out */
  std::vector<Prediction> usefulPredicts;

  /** The last time image processing ran */
  unsigned int lastProcessedTimestamp = 0;

  /** Cached percepts, which are populated at once */
  BallPercept cachedBallPercept;
  RobotPercept cachedRobotPercept;

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theBallPercept The representation updated.
   */
  void update(BallPercept& theBallPercept);

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theRobotPercept The representation updated.
   */
  void update(RobotPercept& theRobotPercept);

  /**
   * Converts the image from YUYV to RGB and
   * puts it into the top left corner of the network's input.
   */
  void preProcess();

  /**
   * Perform post processing on the network output and extract relevant bounding
   * boxes.
   *
   * @param confidenceThresh ignore predictions that have a confidence that is lower than confidenceThresh
   * @param IOUthresh if the IOU of two predictions is greater than IOUthresh, take the box that
   *        has a higher confidence
   */
  void extractPredictions(const float confidenceThresh, const float IOUthresh);

  /**
   * Filter out unrealistic obstacles (too small and in own body).
   */
  void filterPredictions();

  /**
   * Processes the entire frame at most once per time stamp
   */
  void processFrameOnce();

  /**
   * Populates the given Representation. It does what update() would usually do.
   *
   * @param theBallPercept the ball representation to populate
   */
  void writeBallPercept(BallPercept& theBallPercept);

  /**
   * Populates the given Representation. It does what update() would usually do.
   *
   * @param theRobotPercept the ball representation to populate
   */
  void writeRobotPercept(RobotPercept& theRobotPercept);

  /**
   * Attempts to get the extrema closer to the actual contour of the ball
   *
   * @param image Reference to the image in which the ball was seen
   * @param closerToMiddle Reference to the extremum that is closer to the optical center of the image
   * @param furtherFromMiddle Reference to the extremum that is further away from the optical center of the image
   * @param ballPrediction Reference to the prediction for the ball
   *
   * @return whether the extrema were modified
   */
  bool improveBallExtrema(Vector2f& closerToMiddle, Vector2f& furtherFromMiddle,
                          const Geometry::Line& ballCenterToOpticalCenter, const Prediction& ballPrediction);

  /**
   * Checks whether a point is inside a rectangle.
   *
   * @param point Point to be checked
   * @param topLeft Top left boundary
   * @param botRight Bottom right boundary
   *
   * @return True if point is inside rectangle
   */
  bool pointIsInBoundingBox(const Vector2f& point, const Vector2f& topLeft, const Vector2f& botRight) const;

public:
  RobotBallPerceptor();
};

/**
 * Encapsulates a Bounding box
 */
struct RobotBallPerceptor::Prediction
{
  Vector2f center; /**< Center of the Bounding box */

  float height; /**< Height of the bounding box */
  float width; /**< Width of the bounding box */

  Vector2f topLeft; /**< Top left corner of the Bounding box */
  Vector2f bottomRight; /**< Bottom right corner of the bounding box */

  ObjectType type; /**< Kind of object this prediction points to */

  float probability; /**< Probability that this Object an object of the label */

  /**
   * Constructor to construct a Prediction from the output of the yolox-model
   *
   * @param center Center of the bounding box
   * @param height Height of the bounding box
   * @param width Width of the bounding box
   * @param type Object type this Prediction points to
   * @param prob probability this Prediction actually points to an object of the given type
   */
  Prediction(const Vector2f& center, const float height, const float width, const ObjectType type, const float prob)
    : center(center), height(height), width(width),
      topLeft(Vector2f(center.x() - (width / 2), center.y() - (height / 2))),
      bottomRight(Vector2f(center.x() + (width / 2), center.y() + (height / 2))),
      type(type), probability(prob) {}

  /**
   * Computes the area of the prediction (this bounding box)
   *
   * @return area of this prediction
   */
  constexpr float area() const
  {
    return width * height;
  }

  /**
   * Computes the intersection area of this and another Prediction
   *
   * @param other The other prediction
   */
  float intersection(const Prediction& other) const
  {
    if(topLeft.x() <= other.bottomRight.x() &&
       bottomRight.x() >= other.topLeft.x() &&
       topLeft.y() <= other.bottomRight.y() &&
       bottomRight.y() >= other.topLeft.y())
    {
      const float x0 = std::max(topLeft.x(), other.topLeft.x());
      const float y0 = std::max(topLeft.y(), other.topLeft.y());
      const float x1 = std::min(bottomRight.x(), other.bottomRight.x());
      const float y1 = std::min(bottomRight.y(), other.bottomRight.y());

      if(x1 > x0 && y1 > y0)
      {
        return std::abs((x1 - x0) * (y1 - y0));
      }
    }
    return 0;
  }

  /**
   * Computes the IOU between this and another prediction
   *
   * @param other The other prediction
   */
  float IOU(const Prediction& other) const
  {
    const float inter = intersection(other);
    const float uni = area() + other.area() - inter;
    return uni > 0 ? static_cast<float>(inter) / static_cast<float>(uni) : 0.f;
  }
};
