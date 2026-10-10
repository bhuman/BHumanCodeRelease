/**
 * @file JerseyClassifierProvider.h
 *
 * This file declares a module that provides functionality to classify jerseys
 * by sampling pixels in an estimated area and classifying each of them as
 * belonging to the own team or to the opponent team.
 *
 * @author Thomas Röfer (the algorithm)
 * @author Lukas Malte Monnerjahn (the algorithm)
 * @author Arne Hasselbring (the module)
 * @author Tim Laue (the updates for 2023 and 2026)
 */

#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ObstaclePercepts/JerseyClassifier.h"
#include "Framework/Module.h"
#include "Math/Range.h"
#include <functional>
#include <vector>

MODULE(JerseyClassifierProvider,
{,
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(ECImage),
  REQUIRES(GameState),
  PROVIDES(JerseyClassifier),
  LOADS_PARAMETERS(
  {,
    (unsigned int)(42) numberOfSamples, /**< Number of samples used for determining the jersey color */
    (float)(50.f) maxSampleOffset, /**< Maximum distance (along each 3D axis) of a random sample from the assumed jersey center */
    (float)(620.f) jerseyCenterZ, /**< Quite hardcoded value for the jersey center's z position in world coordinates*/
    (int)(24) hueSimilarityThreshold, /**< Maximum deviation from team color hue value still accepted (0 - 128). */
    (float)(0.2f) minJerseyWeightRatio, /**< The minimum percentage of points of a jersey color required in ratio to examined pixels . */
    (float)(0.6f) minJerseyRatio, /**< The majority required of one jersey color over all others. */
    (unsigned char)(87) satThreshold, /**< Maximum saturation to count a spot as non saturated. */
    (unsigned char)(100) whiteThreshold, /**< Grayscale value must be larger than this to be considered as white . */
  }),
});

class JerseyClassifierProvider : public JerseyClassifierProviderBase
{
  /**
   * Updates the jersey classifier.
   * @param jerseyClassifier The updated representation.
   */
  void update(JerseyClassifier& jerseyClassifier) override;

  /**
   * Detect the jersey color. The method uses the expected position of the jersey
   * center in the image and samples random points around it. It is checked whether more
   * samples support the own or the opponent jersey color(s). The features used are
   * brightness (to distinguish gray from black or white), field color, and hue.
   * The team color green is not supported.
   * @param robotOnField The field type is updated.
   */
  void detectJersey(RobotPercept::Robot& robotOnField);

  /**
   * The method determines a way to detect whether a pixel belongs to a specific jersey
   * color by comparing it to all other jersey colors used for this game.
   * @param checkColor The team color index of the jersey color that should be detected.
   * @param o1 A team color index of another color on the pitch.
   * @param o2 A team color index of another color on the pitch.
   * @param o3 A team color index of another color on the pitch.
   * @param maxBrightness The intensity of the brightest pixel below the jersey. This
   *                     functions as a reference if one of the two teams uses the
   *                     jersey color gray.
   * @return The classifier that detects whether the pixel at (x, y) (the two parameters)
   *         belongs to the jersey color of teamColor.
   */
  std::function<bool(const int, const int)> getPixelClassifier(const GameState::Team::Color checkColor,
                                                               const GameState::Team::Color o1,
                                                               const GameState::Team::Color o2,
                                                               const GameState::Team::Color o3) const;

  std::vector<Vector2i> jSamples; /**< List for storing sample positions */
};
