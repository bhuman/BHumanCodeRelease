/**
 * @file PlayersDeeptectorFeatBOPLower.h
 *
 * This file declares a module that transplants the SegmentedObstacleImage onto the lower camera handling of the PlayersDeeptector.
 *
 * @author Bernd Poppinga
 * @author Andreas Baude
 * @author Arne Hasselbring
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
#include "Representations/Perception/ImagePreprocessing/SegmentedObstacleImage.h"
#include "Representations/Perception/ObstaclesPercepts/JerseyClassifier.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesFieldPercept.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesImagePercept.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesPerceptorData.h"
#include "Framework/Module.h"
#include "Math/Eigen.h"
#include <CompiledNN/CompiledNN.h>

MODULE(PlayersDeeptectorFeatBOPLower,
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
  REQUIRES(OtherObstaclesPerceptorData),
  REQUIRES(OtherOdometryData),
  REQUIRES(SegmentedObstacleImage),
  PROVIDES(ObstaclesFieldPercept),
  PROVIDES(ObstaclesImagePercept),
  USES(ObstaclesPerceptorData),
  PROVIDES(ObstaclesPerceptorData),
  DEFINES_PARAMETERS(
  {,
    (unsigned int)(2) xyStep, /**< Step size in x/y direction for scanning the image. */
    (short)(40) minContrastDiff, /**< Minimal contrast difference to consider a spot. */
    (unsigned char)(40) satThreshold, /**< Maximum saturation to count a spot as non saturated. */
    (unsigned int)(5) minNeighborPoints, /**< Minimal number of neighbors to count a region as a core region (dbScan). */
    (int)(20) minPixel, /**< Minimal width of an obstacle in the image (in pixel). */
    (float)(200.f) minWidthOnFieldNoMatch, /**< Minimal width of an obstacle on the field (in mm) for which there is NO matching obstacle in the upper image. */
    (bool)(false) trimObstacles, /**< Whether the width of obstacles should be corrected. */
    (float)(0.32f) minBeforeAfterTrimRatio,
    (bool)(true) mergeLowerObstacles, /**< Whether overlapping obstacles should be merged (only for the lower camera). */
    (Vector2f)(0.02f, 0.04f) pRobotRotationDeviationInStand, /**< Deviation of the rotation of the robot's torso while standing. */
    (Vector2f)(0.04f, 0.04f) pRobotRotationDeviation,        /**< Deviation of the rotation of the robot's torso. */
  }),
});

class PlayersDeeptectorFeatBOPLower : public PlayersDeeptectorFeatBOPLowerBase
{
public:
  PlayersDeeptectorFeatBOPLower();

private:
  std::vector<ObstaclesImagePercept::Obstacle> obstaclesUpper, obstaclesLower;

  /** This struct represents an image region. */
  struct Region
  {
    Vector2i regionIndices; // The x- and y-indices of this region in the image.
    bool isObstacle = false;
    bool clustered = false; // The current state of the region in terms of belonging to a cluster.
    int maxY = -1; // The maximum y coordinate of all spots within the region at which a contrast change was registered.
  };

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theObstaclesImagePercept The representation updated.
   */
  void update(ObstaclesImagePercept& theObstaclesImagePercept) override;

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theObstaclesImagePercept The representation updated.
   */
  void update(ObstaclesFieldPercept& theObstaclesFieldPercept) override;

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theObstacleScanScores The representation updated.
   */
  void update(ObstaclesPerceptorData& theObstaclesPerceptorData) override;

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
  void regionQuery(std::vector<std::vector<Region>>& regions, Vector2i& region, int dis, std::vector<Vector2i>& neighbors);
};
