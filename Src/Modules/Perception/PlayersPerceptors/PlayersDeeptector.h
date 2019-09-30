/**
 * @file PlayersDeeptector.h
 *
 * This file declares a module that detects players in images with a neural network.
 *
 * @author Bernd Poppinga
 */

#pragma once

#include "Representations/Communication/TeamInfo.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/DummyRepresentation.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/Thumbnail.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/LabelImage.h"
#include "Representations/Perception/ImagePreprocessing/BodyContour.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ImagePreprocessing/FieldBoundary.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesFieldPercept.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesImagePercept.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesPerceptorData.h"
#include "Tools/ImageProcessing/PatchUtilities.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Module/Module.h"
#include "Tools/NeuralNetwork/CompiledNN.h"
#include <fstream>

MODULE(PlayersDeeptector,
{,
  REQUIRES(BodyContour),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(ECImage),
  REQUIRES(FieldBoundary),
  REQUIRES(FrameInfo),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(Odometer),
  REQUIRES(OpponentTeamInfo),
  REQUIRES(OtherObstaclesPerceptorData),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RobotDimensions),
  REQUIRES(Thumbnail),
  REQUIRES(ObstaclesFieldPercept),
  PROVIDES(ObstaclesFieldPercept),
  PROVIDES(ObstaclesImagePercept),
  USES(ObstaclesPerceptorData),
  PROVIDES(ObstaclesPerceptorData),
  DEFINES_PARAMETERS(
  {,
    (float)(0.5) nonMaximumSuppressionThres,
    (float)(0.6f) objectThres,
    (bool)(false) logImages,
    (PatchUtilities::ExtractionMode)(PatchUtilities::fast) extractionMode,
    (bool)(false) useContrastNormalization,
    (unsigned int)(2) xyStep, /** Step size in x/y direction for scanning the image. */
    (unsigned int)(16) xyRegions, /** Number of regions in x/y direction. */
    (short)(60) minContrastDif, /** Minimal contrast difference between an interesting spot and its neighbour. */
    (short)(150) brightnessThreshold, /** Minimum brightness to count a spot as bright. */
    (float)(0.2f) mixedThresh, /** */
    (int)(2) minNonHomogenSpots, /** */
    (unsigned int)(5) minNeighborPoints, /** Mininmal number of neighbors to count a region as a core region (dbscan) */
    (int)(20) minWidthInPixel, /** Minimum width of an obstacle in the image (in pixel) */
    (float)(100.f) minWidthOnFieldMatch, /** Minimum width of an obstacle on the field (in mm) for which there is a matching obstacle in the upper picture. */
    (float)(200.f) minWidthOnFieldNoMatch, /** Minimum width of an obstacle on the field (in mm) for which there is NO matching obstacle in the upper picture. */
    (bool)(true) trimObstacles,
    (Rangef)(300.f, 400.f) jerseyYRange, /**< The expected height range of the jersey in the image in mm. */
    (float)(0.8f) whiteScanHeightRatio, /**< How high to scan for the maximum brightness between the foot point and the lower jersey edge (0..1). */
    (Rangef)(0.33f, 0.67f) grayRange, /**< Which ratio range of the maximum brightness is considered to be gray? */
    (int)(20) jerseyXSamples, /**< How many horizontal samples for the jersey scan? */
    (int)(10) jerseyYSamples, /**< How many vertical samples for the jersey scan? */
    (int)(32) hueSimilarityThreshold, /**< Maximum deviation from team color hue value still accepted (0 - 128). */
    (int)(10) minJerseyPixels, /**< The minumum number of supporters of a jersey color required. */
    (float)(0.6f) minJerseyRatio, /**< The majority required of one jersey color over the other. */
  }),
});

class PlayersDeeptector : public PlayersDeeptectorBase
{
public:
  PlayersDeeptector();

private:
  Vector2i patchSize;
  std::unique_ptr<NeuralNetwork::Model> model;
  NeuralNetwork::CompiledNN convModel;
  Matrix4x2f anchors;
  std::vector<ObstaclesImagePercept::Obstacle> obstaclesUpper, obstaclesLower;

  /** This enumeration lists the possible classes of a region. */
  ENUM(Classification,
  {,
    Horizontal,
    Vertical,
    Mixed,
    Nothing,
    Unknown,
  });

  /** This enumeration lists the possible states of a region in terms of belonging to a cluster. */
  ENUM(ClusterLabel,
  {,
    Undefined,
    Noise,
    Clustered,
  });

  /** This struct represents an image region. */
  struct Region
  {
    std::vector<std::vector<Vector2i> > spots = std::vector<std::vector<Vector2i> >(3, std::vector<Vector2i>()); // The spots with contrast changes found in this region
    Vector2i regionIndices; // The x- and y-indices of this region in the image.
    Classification classification = Nothing; // The classification of this region.
    ClusterLabel label = Undefined; // The current state of the region in terms of belonging to a cluster.
    bool wonky = false, bright = false; //Whether the region is marked as wonky/bright.
    int brightSpots = 0; // Number of bright spots in the region
  };

  /** This struct represents a cluster of interesting regions in the image. */
  struct Cluster
  {
    std::vector<Vector2i> regions; // The indices of the regions belonging to this cluster.
    Vector2f massCenterOfRegions; // The mass center of the non wonky regions in this cluster.
    Vector2i topLeft = Vector2i(0, 0), bottomRight = Vector2i(0, 0);
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

  void trimObstacle(ObstaclesImagePercept::Obstacle& obstacleInImage);

  /**
   * Divides the grayscale image into regions, searches them for changes
   * in contrast and saves the corresponding spots.
   * @param regions The regions in which the spots should be stored
   */
  void scanImage(std::vector<std::vector<Region> >& regions);

  /**
   * Classifies image regions based on the number and type of spots contained.
   * @param regions The regions to be classified.
   */
  void classifyRegions(std::vector<std::vector<Region> >& regions);

  /**
   * Discards contiguous, homogeneous regions.
   * @param regions The regions to be processed.
   */
  void discardHomogenAreas(std::vector<std::vector<Region> >& regions);

  /**
   * Creates a previously unknown number of clusters from all image regions.
   * @param regions The regions to be clustered.
   * @param clusters The clusters to be formed.
   */
  void dbscan(std::vector<std::vector<Region> >& regions, std::vector<Cluster>& clusters);

  /**
   * Returns the indices of all regions with a specified classification in a specified radius around a given region.
   * @param regions
   * @param region The region around which further regions are to be searched.
   * @param dis The maximum distance up to which further regions should be searched.
   * @param queryBright Whether regions marked as bright should also be returned.
   * @param classificationToSearch The classification which regions must have in order to be returned.
   * @return The indices of matching regions.
   */
  bool regionQuery(std::vector<std::vector<Region> >& regions, Vector2i& region, int dis, bool queryBright, std::vector<Vector2i>& neighbors, Classification classificationToSearch = Unknown);

  /**
   * Expands iteratively a cluster given a region within the cluster and its neighboring regions.
   * @param regions
   * @param region The first region to be part of the cluster.
   * @param neighbors The neighbours of the first cluster region.
   * @param cluster The cluster to be expanded.
   */
  bool expandCluster(std::vector<std::vector<Region> >& regions, Region& region, std::vector<Vector2i>& neighbors, Cluster& cluster);

  /**
   * Calculates the convex hulls and bounding rectangles for a set of clusters using the spots in the image regions belonging to the clusters.
   * @param clusters The clusters for which convex hulls and bonding rectangles should be calculated.
   * @param obstacles
   * @param regions Die Menge der klassifizierten Bildregionen.
   */
  void calcConvexHulls(std::vector<Cluster>& clusters, std::vector<ObstaclesImagePercept::Obstacle>& obstacles, std::vector<std::vector<Region> >& regions);


  /**
   * Detect the jersey color. The method estimates the expected position of the jersey
   * in the image and samples a grid inside a paralellogram. It is checked whether more
   * samples support the own or the opponent jersey color. The features used are
   * brightness (to distinguish gray from black or white), field color, and hue.
   * If one team has the jersey color white, the results will probably misleading,
   * because arms and goalposts are also white. The team color green is not supported.
   * @param obstacleInImage The obstacle as it was detected in the image.
   * @param obstacleOnField The fields detectedJersey and ownTeam will be updated if a
   *                        jersey color was detected.
   */
  void detectJersey(const ObstaclesImagePercept::Obstacle& obstacleInImage, ObstaclesFieldPercept::Obstacle& obstacleOnField) const;

  /**
   * The method determines the best way to detect whether a pixel belongs to the jersey
   * color of a specific team. It also considers the jersey color the other team uses.
   * @param teamColor The team color index of the jersey color that should be detected.
   * @param otherTeam The team color index of the other team playing.
   * @param maxBrighness The intensity of the brightest pixel below the jersey. This
   *                     functions as a reference if one of the two teams uses the
   *                     jersey color gray.
   * @return The classifier that detects whether the pixel at (x, y) (the two parameters)
   *         belongs to the jersey color of teamColor.
   */
  std::function<bool(const int, const int)> getPixelClassifier(const int teamColor, const int otherColor, const int maxBrightness) const;
};
