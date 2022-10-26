/**
 * @file JerseyClassifierProvider2022.h
 *
 * This file declares a module to classify jerseys by using a convolutional neural network to detect a teammate or an opponent.
 *
 * @author Florian Scholz
 */

#pragma once
#include "Framework/Module.h"
#include "Representations/Infrastructure/CameraImage.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Perception/ObstaclesPercepts/JerseyClassifier.h"

#include <CompiledNN/CompiledNN.h>
#include <CompiledNN/Model.h>

MODULE(JerseyClassifierProvider2022,
{,
  REQUIRES(CameraImage),
  REQUIRES(CameraInfo),
  REQUIRES(GameState),
  REQUIRES(JerseyClassifierOld),
  PROVIDES(JerseyClassifier),
  DEFINES_PARAMETERS(
  {,
    (int)(10) minWidth, /**< Minimum width of an obstacle to try to detect a jersey. */
    (int)(10) minHeight, /**< Minimum height of an obstacle to try to detect a jersey. */
  }),
});

class JerseyClassifierProvider2022 : public JerseyClassifierProvider2022Base
{
private:
  NeuralNetwork::CompiledNN networkArray[Settings::numOfTeamColors]; /**< Array of networks for each team color */
  Vector2i patchSizeArray[Settings::numOfTeamColors];

public:

  /**
   * Constructor
   */
  JerseyClassifierProvider2022();

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theJerseyClassifier The representation updated.
   */
  void update(JerseyClassifier& theJerseyClassifier) override;

  /**
   * Loads the neural networks for each teamcolor
   */
  void loadNeuralNetworks();

  /**
   * Image formatting for the used neural net.
   * Resizes, normalizes and converts a YUYV image to YUV.
   * @param orig The input image.
   * @param output The resized and normalized image.
   * @param outSize The size of the output image.
   * @param inSize The size of the patch.
   * @param offset The position of the patch.
   */
  void resizeBilinear(const CameraImage& orig, float* output, const Vector2i& outSize, const Vector2i& inSize, const Vector2i& offset);

  /**
   * Detect the jersey color based on a provided neural net.
   * The current net requires black jerseys as the own team color.
   * @param obstacleInImage The obstacle as it was detected in the image.
   * @param obstacleOnField The fields detectedJersey and ownTeam will be updated if a
   *                        jersey color was detected.
   */
  void detectJersey(const ObstaclesImagePercept::Obstacle& obstacleInImage, ObstaclesFieldPercept::Obstacle& obstacleOnField);
};
