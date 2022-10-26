/**
 * @file JerseyClassifierProvider2020.h
 *
 * This file declares a module that provides functionality to classify jerseys
 * by sampling pixels in an estimated area and classifying each of them as
 * belonging to the own team or to the opponent team.
 *
 * @author Thomas Röfer (the algorithm)
 * @author Lukas Malte Monnerjahn (the algorithm)
 * @author Arne Hasselbring (the module)
 */

#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Perception/ObstaclesPercepts/JerseyClassifier.h"
#include "Framework/Module.h"
#include "Math/Range.h"
#include <functional>

MODULE(JerseyClassifierProvider2020,
{,
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(ECImage),
  REQUIRES(GameState),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(RobotDimensions),
  PROVIDES(JerseyClassifier),
  PROVIDES(JerseyClassifierOld),
  DEFINES_PARAMETERS(
  {,
    (Rangef)(300.f, 410.f) jerseyYRange, /** The expected height range of the jersey in the image in mm. */
    (float)(0.8f) whiteScanHeightRatio, /** How high to scan for the maximum brightness between the foot point and the lower jersey edge (0..1). */
    (Rangef)(0.37f, 0.67f) grayRange, /** Which ratio range of the maximum brightness is considered to be gray? */
    (int)(12) jerseyMinYSamples, /** How many vertical samples for the jersey scan at minimum? */
    (float)(20) jerseyMaxYSamples, /** How many vertical samples for the jersey scan at maximum? */
    (float)(30) jerseyMaxXSamples, /** How many horizontal samples for the jersey scan at maximum? */
    (float)(0.5f) relativeJerseyWidth, /** The typical relative width of the jersey in the scanned region. */
    (int)(32) hueSimilarityThreshold, /** Maximum deviation from team color hue value still accepted (0 - 128). */
    (unsigned char)(110) colorDelimiter, /** Delimiter for grey, black and white against colors by saturation, only used if no color is involved. */
    (bool)(true) weighPixels, /** Weigh each pixel individually or give all weight one. */
    (float)(0.1f) minJerseyWeightRatio, /** The minimum number of weighted points of a jersey color required in ratio to examined pixels . */
    (float)(0.6f) minJerseyRatio, /** The majority required of one jersey color over the other. */
    (int)(4) whiteScanOffSet, /** y-Pixel offset for additional whitescan rows*/
    (unsigned char)(87) satThreshold, /** Maximum saturation to count a spot as non saturated. */
  }),
});

class JerseyClassifierProvider2020 : public JerseyClassifierProvider2020Base
{
  /**
   * Updates the jersey classifier.
   * @param jerseyClassifier The updated representation.
   */
  void update(JerseyClassifier& jerseyClassifier) override;
  void update(JerseyClassifierOld& jerseyClassifier) override {update(static_cast<JerseyClassifier&>(jerseyClassifier));}

  /**
   * Detect the jersey color. The method estimates the expected position of the jersey
   * in the image and samples a grid inside a parallelogram. It is checked whether more
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
   * @param maxBrightness The intensity of the brightest pixel below the jersey. This
   *                     functions as a reference if one of the two teams uses the
   *                     jersey color gray.
   * @return The classifier that detects whether the pixel at (x, y) (the two parameters)
   *         belongs to the jersey color of teamColor.
   */
  std::function<bool(const int, const int)> getPixelClassifier(const GameState::Team::Color teamColor, const GameState::Team::Color otherColor, const int maxBrightness) const;
};
