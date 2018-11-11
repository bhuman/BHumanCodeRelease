/**
 * @file ObstaclesPerceptor.h
 *
 * This file declares a module that detects obstacles in the color-segmented image.
 * This is a more compact rewrite of the RobotsPerceptor by Michel Bartsch.
 * Points below obstacles are found by vertically scanning down from the field boundary.
 * Such a candidate point is the lowest edge between non-field color and field color
 * where enough non-field color came above it. These candidates are processed from
 * bottom to top, merging neighboring candidates on a similar height.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Perception/ImagePreprocessing/BodyContour.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ImagePreprocessing/FieldBoundary.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Perception/ImagePreprocessing/ScanGrid.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesImagePercept.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesFieldPercept.h"
#include "Tools/Module/Module.h"

MODULE(ObstaclesPerceptor,
{,
  REQUIRES(BodyContour),
  REQUIRES(CameraMatrix),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(CameraInfo),
  REQUIRES(ECImage),
  REQUIRES(FieldBoundary),
  REQUIRES(Odometer),
  REQUIRES(OpponentTeamInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RobotDimensions),
  REQUIRES(ScanGrid),
  REQUIRES(ObstaclesImagePercept),
  PROVIDES(ObstaclesImagePercept),
  PROVIDES(ObstaclesFieldPercept),
  DEFINES_PARAMETERS(
  {,
    (int)(2) xStep, /**< Horizontal distance between vertical scanlines in pixels. */
    (int)(5) yStartBelow, /**< Scanning starts this number of pixels below the field boundary. */
    (float)(35.f) yStep, /**< Vertical distance between scan points in mm. */
    (int)(5) minHeight, /**< The minimum number of scan points from the to accept an obstacle. */
    (int)(3) shortRangeThreshold, /**< The minumum short range score to accept an obstacle. */
    (int)(5) longRangeThreshold, /**< The minimum long range score to accet an obstacle. */
    (int)(3) longRangePenalty, /**< The penalty for field-colored scan points in the long range score. */
    (int)(5) maxObstaclesToDetect, /**< The maximum number of obstacles that will be detected per image. */
    (float)(120.f) maxXGap, /**< The maximum horizontal gap between obstacle spots to group them together (in mm). */
    (float)(65.f) maxYGap, /**< The maximum vertical gap between obstacle spots to group them together (in mm). */
    (int)(1) maxYMicroGap, /**< The maximum number of field pixels skipped when searching the true bottom of an obstacle. */
    (float)(75.f) minWidth, /**< The minimum width of grouped spots to accept an obstacle (in mm). */
    (int)(20) minWidthInPixel, /**< The minimum width of grouped spots to accept an obstacle (in pixel). */
    (float)(0.6f) minNeigboredRatio, /**< The minimum ratio of spots that are directly neighbored to accept an obstacle. */
    (float)(8.f) minScoreRatio, /**< A threshold for the minumum accumulted score for an obstacle. */
    (float)(150.f) ignoreWidth, /**< An additional width to ignore around a detected obstacle (in mm). */
    (float)(0.8f) minGreenAboveRatio, /**< The minumum ratio of spots with green above the obstacle to detect an obstacle as fallen. */
    (float)(650.f) uprightRobotHeight, /**< Assumed height of an upright robot in mm. */
    (float)(250.f) fallenRobotHeight, /**< Assumed height of a fallen robot in mm. */
    (float)(250.f) assumedWidth, /**< Assumed width of the obstacle at the lower image border if the lower end is outside (in mm). */
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

class ObstaclesPerceptor : public ObstaclesPerceptorBase
{
  /** Result of a vertical scan. */
  struct ScanResult
  {
    int y; /**< y coordinate at the bottom of the potential obstacle or 0 if nothing significant was found. */
    int score; /**< The long range score of this point (how much non-green above). */
    int shortRangeScore; /**< The short range score at the lower end of the scan. */
    int longRangeScore; /**< The long range score at lower end of the scan. */
    bool greenAbove; /**< Is enough green above the obstacle to be lying on the ground? */
  };

  constexpr static int fieldColor = 1 << FieldColors::field | 1 << FieldColors::black; /**< Color classes considered as field. */
  CameraMatrix upperCameraMatrix; /**< The camera matrix of the last upper image. */
  ImageCoordinateSystem upperImageCoordinateSystem; /**< The image coordinate system of the last upper image. */
  ScanResult scanResults[CameraInfo::numOfCameras][Image::maxResolutionWidth]; /**< All results of vertical scans. Only multiples of xStep are used. */
  std::vector<ObstaclesFieldPercept::Obstacle> incompleteObstacles; /**< Obstacles in the upper image that ended in the lower image border. */

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theObstaclesImagePercept The representation updated.
   */
  void update(ObstaclesImagePercept& theObstaclesImagePercept) override;

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theObstaclesFieldPercept The representation updated.
   */
  void update(ObstaclesFieldPercept& theObstaclesFieldPercept) override;

  /**
   * Project the scan information of the upper image to the lower image.
   * The projection includes the long range and short range scores, as well as the
   * y coordinate at which the scan should start in the lower image.
   * All scanlines that cannot be projected are marked with y = -1.
   */
  void project();

  /**
   * Perform a single vertical scan.
   * @param x The x coordinate of the vertical scan.
   * @param top The topmost (i.e. the smallest number) y coordinate that can be used.
   * @param topStep The vertical step size to be used at top.
   * @param bottomStep The vertical step size to be used at the lower end of the image.
   * @param scanResult The results written to this structure. Member y will be 0 if no candidate is found.
   * @return Was a candidate point found?
   */
  bool scan(const int x, const int top, const int topStep, const int bottomStep, ScanResult& scanResult) const;

  /**
   * Check a single candidate point for indicating at an obstacle.
   * @param scanResults The results of vertical scans.
   * @param x The x coordinate of the candidate point.
   * @param top The topmost (i.e. the smallest number) y coordinate that can be used.
   * @param topUnit 1 mm in pixels at top.
   * @param bottomUnit 1 mm in pixels at the lower end of the image.
   * @param obstacles Detected obstacles are appended to this vector.
   */
  void check(ScanResult* scanResults, const int x, const int top, const float topUnit, const float bottomUnit,
             std::vector<ObstaclesImagePercept::Obstacle>& obstacles) const;

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

  /**
   * Draw the scan grid. This basically replicates the scan loop. Drawing separately
   * avoids slowing down the actual scanning.
   * @param scanResults The preparations for the vertical scans.
   * @param top The topmost (i.e. the smallest number) y coordinate that can be used.
   * @param topStep The vertical step size to be used at top.
   * @param bottomStep The vertical step size to be used at the lower end of the image.
   */
  void draw(const ScanResult* scanResults, const int top, const int topStep, const int bottomStep) const;
};
