/**
 * @file RobotDetector.h
 *
 * This file defines a module that detects SPL robots in an image using a neural network.
 * Most Code is taken from PlayersDeeptector and changes simply adapt the code of Bernd Poppinga to a newly created model
 *
 * @author Kelke van Lessen
 * @author Lukas Malte Monnerjahn
 * @author Fynn Boese
 * @author Bernd Poppinga
 */

#pragma once

#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Perception/MeasurementCovariance.h"
#include "Representations/Perception/ImagePreprocessing/BodyContour.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ImagePreprocessing/FieldBoundary.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Perception/ObstaclesPercepts/JerseyClassifier.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesFieldPercept.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesImagePercept.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesPerceptorData.h"
#include "Representations/Perception/RefereePercept/OptionalImageRequest.h"
#include "Framework/Module.h"
#include "ImageProcessing/LabelImage.h"
#include "Math/Eigen.h"
#include "CompiledNN/CompiledNN.h"
#include "CompiledNN2ONNX/CompiledNN.h"

STREAMABLE(NetworkParameters,
{,
  // network input shape
  (unsigned int) inputHeight,
  (unsigned int) inputWidth,
  (unsigned int) inputChannels,

  // network output shape
  (unsigned int) outputHeight,
  (unsigned int) outputWidth,
  (unsigned int) outputAnchors,
  (unsigned int) paramsPerAnchor,

  // class prediction features
   (bool) predictFallen,

  // indices of network outputs inside an anchor
  (int) confidenceIndex,
  (int) yMidIndex,
  (int) xMidIndex,
  (int) heightIndex,
  (int) widthIndex,
  (int) fallenClassIndex,

  // Allow up to 6 anchors (predictors per output pixel), not all need to be used
  (std::vector<Vector2f>) anchors,

  // network model configuration
  (float) sizeConversionFactor,
});

MODULE(RobotDetector,
{,
  REQUIRES(BallSpecification),
  REQUIRES(BodyContour),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(ECImage),
  REQUIRES(FieldBoundary),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(JerseyClassifier),
  REQUIRES(MeasurementCovariance),
  REQUIRES(MotionInfo),
  REQUIRES(ObstaclesFieldPercept),
  REQUIRES(OdometryData),
  REQUIRES(OptionalImageRequest),
  REQUIRES(OtherObstaclesPerceptorData),
  REQUIRES(OtherOdometryData),
  PROVIDES(ObstaclesFieldPercept),
  PROVIDES(ObstaclesImagePercept),
  USES(ObstaclesPerceptorData),
  PROVIDES(ObstaclesPerceptorData),
  DEFINES_PARAMETERS(
  {,
    (float)(0.6f) objectThres, /**< Limit from which a robot is accepted. */
    (float)(0.55f) fallenThres, /**< Confidence threshold for a robot to be predicted as lying on the ground */
    (float)(0.3f) nonMaximumSuppressionIoUThreshold, /**< Suppress non-maximal robot predictions with intersection over union above this threshold */
    (unsigned int)(2) xyStep, /**< Step size in x/y direction for scanning the image. */
    (unsigned int)(16) xyRegions, /**< Number of regions in x/y direction. */
    (short)(40) minContrastDiff, /**< Minimal contrast difference to consider a spot. */
    (short)(100) brightnessThreshold, /**< Minimal brightness to count a spot as bright. */
    (unsigned char)(40) satThreshold, /**< Maximum saturation to count a spot as non saturated. */
    (float)(0.2f) mixedThresh, /**< Ratio of the horizontal & vertical contrast changes from which a region is considered a mixed region. */
    (int)(2) minHetSpots, /**< Minimal number of differently classified neighbor regions to not discard a region. */
    (unsigned int)(5) minNeighborPoints, /**< Minimal number of neighbors to count a region as a core region (dbScan). */
    (int)(20) minPixel, /**< Minimal width of an obstacle in the image (in pixel). */
    (float)(200.f) minWidthOnFieldNoMatch, /**< Minimal width of an obstacle on the field (in mm) for which there is NO matching obstacle in the upper image. */
    (bool)(true) trimObstacles, /**< Whether the width of obstacles should be corrected. */
    (float)(0.32f) minBeforeAfterTrimRatio,
    (bool)(true) mergeLowerObstacles, /**< Whether overlapping obstacles should be merged (only for the lower camera). */
    (Vector2f)(0.02f, 0.04f) pRobotRotationDeviationInStand, /**< Deviation of the rotation of the robot's torso while standing. */
    (Vector2f)(0.04f, 0.04f) pRobotRotationDeviation,        /**< Deviation of the rotation of the robot's torso. */
  }),
});

class RobotDetector : public RobotDetectorBase
{
public:
  RobotDetector();

private:
  Vector2i inputImageSize;
  // Double structure to allow switching between CompiledNN- and ONNX models. Only one of each set will be used at a time.
  std::unique_ptr<NeuralNetwork::Model> cnnModel;
  NeuralNetwork::CompiledNN cnnConvModel;
  std::unique_ptr<NeuralNetworkONNX::Model> onnxModel;
  NeuralNetworkONNX::CompiledNN onnxConvModel;
  bool useOnnx;
  Image<PixelTypes::GrayscaledPixel> grayscaleThumbnail;
  // Splitting into three images and recombining them into one is probably a performance issue
  // We need to optimize this later
  Image<PixelTypes::GrayscaledPixel> redChromaThumbnail;
  Image<PixelTypes::GrayscaledPixel> blueChromaThumbnail;
  std::vector<ObstaclesImagePercept::Obstacle> obstaclesUpper, obstaclesLower;

  // todo: move model_path into the config or extract config_path from model_path
  const std::string model_path = "/Config/NeuralNets/RobotDetector/4_anchor_boxes_model_no_activation_20230629-220730.hdf5";
  const std::string model_config_path = "NeuralNets/RobotDetector/4_anchor_boxes_20230629-220730.cfg";
  NetworkParameters networkParameters;

  [[maybe_unused]] const int HEIGHT_SHAPE_INDEX = 0;
  [[maybe_unused]] const int WIDTH_SHAPE_INDEX = 1;
  [[maybe_unused]] const int IMAGE_CHANNELS_INDEX = 2; /**< On network input */
  [[maybe_unused]] const int ANCHOR_SHAPE_INDEX = 2;   /**< On network output */
  [[maybe_unused]] const int BOX_SHAPE_INDEX = 3;

  /** This enumeration lists the possible classes of a image region. */
  ENUM(Classification,
  {,
    Horizontal,
    Vertical,
    Mixed,
    Nothing,
    Unknown,
    Default,
  });

  /** This struct represents an image region. */
  struct Region
  {
    Vector2i regionIndices; /**< The x- and y-indices of this region in the image. */
    std::vector<int> contrastChanges = {0, 0}; /**< Number of registered contrast changes within the region (horizontal/vertical) */
    Classification classification = Nothing; /**< The classification of this region. */
    bool clustered = false; /**< The current state of the region in terms of belonging to a cluster. */
    bool bright = false; /**< Whether the region is marked as bright. */
    int brightSpots = 0; /**< Number of bright spots in the region. */
    int maxY = -1; /**< The maximum y coordinate of all spots within the region at which a contrast change was registered. */
  };

  /**
   * Initialize/Compile the model.
   * @tparam Model Either NeuralNetwork::Model or NeuralNetworkONNX::Model
   * @tparam ConvModel Either NeuralNetwork::CompiledNN or NeuralNetworkONNX::CompiledNN
   * @tparam CompilationSettings Either NeuralNetwork::CompilationSettings or NeuralNetworkONNX::CompilationSettings
   * @param model the neural network model definition
   * @param convModel the model inference object
   * @param settings settings for inference
   */
  template<typename Model, typename ConvModel, typename CompilationSettings>
  void initializeModel(const std::unique_ptr<Model>& model, ConvModel& convModel, const CompilationSettings& settings);

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theObstaclesImagePercept The representation updated.
   */
  void update(ObstaclesImagePercept& theObstaclesImagePercept) override;

  /**
   * This method is called when the representation provided needs to be updated.
   * @param ObstaclesFieldPercept the representation updated.
   */
  void update(ObstaclesFieldPercept& theObstaclesFieldPercept) override;

  /**
   * This method is called when the representation provided needs to be updated.
   * @param ObstaclesPerceptorData the representation updated.
   */
  void update(ObstaclesPerceptorData& theObstaclesPerceptorData) override;

  /**
   * Apply a network to extract obstacles from the image.
   * @param obstacles list of obstacle percepts to be updated
   */
  void extractImageObstaclesFromNetwork(std::vector<ObstaclesImagePercept::Obstacle>& obstacles);

  /**
   * Fill the Y-Thumbnail image with a downscaled grayscale image.
   */
   void fillGrayscaleThumbnail();

   /**
    * Fill the redChroma- and V-Thumbnail images with downscale images of the same size as the grayscale thumbnail.
    * First subsamples from the CameraImage to obtain the same resolution vertically and horizontally.
    * Then downscales to the expected input size.
    */
   void fillChromaThumbnails();

  /**
   * Applies the cnnConvModel on the downscaled grayscale image.
   */
  void applyGrayscaleNetwork();

  /**
   * Applies the cnnConvModel on a downscaled YUV image.
   */
  void applyColorNetwork();

  /**
   * This method gets the bounding boxes from the network output.
   * @tparam ConvModel Type of the network. Either NeuralNetwork::CompiledNN or NeuralNetworkONNX::CompiledNN
   * @param the bounding boxes
   * @param convModel network model
   */
  template<typename ConvModel>
  void boundingBoxes(LabelImage& labelImage, ConvModel& convModel);

  /**
   * Computes the bounding box position and size, given a prediction form the network.
   * Computes intermediate results in place, i.e. network output becomes invalidated.
   * @param pred network prediction, also serves as output vector
   * @param y vertical position of the output cell this prediction belongs to
   * @param x horizontal position of the output cell this prediction belongs to
   * @param b index of the anchor this prediction belongs to
   */
  LabelImage::Annotation predictionToBoundingBox(Eigen::Map<Eigen::Vector<float, 5>>& pred, unsigned int y, unsigned int x, unsigned int b) const;

  /**
   *
   * @param theObstaclesFieldPercept representation to be updated
   * @param obstacles
   */
  void mergeObstacles(ObstaclesFieldPercept& theObstaclesFieldPercept, std::vector<ObstaclesImagePercept::Obstacle>& obstacles);

  /**
   * Corrects the left and right and optionally the bottom boundary of an obstacle in the image.
   * @param trimHeight Whether the bottom boundary should be corrected.
   * @param obstacleInImage The obstacle whose boundaries should be corrected.
   */
  bool trimObstacle(bool trimHeight, ObstaclesImagePercept::Obstacle& obstacleInImage);

  /**
   * Divides the grayscale image into regions and searches them for changes in contrast.
   * @param regions The regions in which the registered contrast changes should be recorded.
   */
  void scanImage(std::vector<std::vector<Region>>& regions);

  /**
   * Classifies image regions based on the number and type of spots contained.
   * @param regions The regions to be classified.
   */
  void classifyRegions(std::vector<std::vector<Region>>& regions);

  /**
   * Discards contiguous, homogeneous regions.
   * @param regions The regions to be processed.
   */
  void discardHomogeneousAreas(std::vector<std::vector<Region>>& regions);

  /**
   * Creates a previously unknown number of clusters from all image regions and stores them as obstacles.
   * @param regions The regions to be clustered.
   * @param obstacles The container to store the obstacles.
   */
  void dbScan(std::vector<std::vector<Region>>& regions, std::vector<ObstaclesImagePercept::Obstacle>& obstacles);

  /**
   * Expands iteratively a cluster given a region within the cluster and its neighboring regions.
   * @param regions The regions to be clustered.
   * @param region The first region to be part of the cluster.
   * @param neighbors The neighbors of the first cluster region.
   * @param cluster The cluster to be expanded.
   * @param topLeft The indices of the top left region within a bounding box around the formed cluster.
   * @param bottomRight The indices of the bottom right region within a bounding box around the formed cluster.
   * @return Whether the cluster formed contains regions that are classified as mixed.
   */
  bool expandCluster(std::vector<std::vector<Region>>& regions, Region& region, std::vector<Vector2i>& neighbors, std::vector<Vector2i>& cluster, Vector2i& topLeft, Vector2i& bottomRight);

  /**
   * Collects the indices of all regions with a specified classification in a specified radius around a given region.
   * @param regions The image regions to be considered.
   * @param region The region surrounding which further regions are to be looked for.
   * @param dis The maximum distance up to which further regions should be looked for.
   * @param neighbor The container to store the found region indices.
   * @param classificationToSearch The classification which regions must have in order to be collected.
   */
  void regionQuery(std::vector<std::vector<Region>>& regions, Vector2i& region, int dis, std::vector<Vector2i>& neighbors, Classification classificationToSearch = Default);
};
