/**
 * @file RefereeGestureDetectionNN.h
 *
 * This file declares a module that <#...#>
 *
 * @author Ayleen Lührsen
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Perception/ImagePreprocessing/OptionalCameraImage.h"
#include "Representations/Perception/RefereePercept/Keypoints.h"
#include "Representations/Perception/RefereePercept/RefereePercept.h"
#include <CompiledNN2ONNX/CompiledNN.h>


MODULE(RefereeGestureDetectionNN,
       {,
  REQUIRES(FieldDimensions),
  REQUIRES(GameState),
  REQUIRES(Keypoints),
  REQUIRES(RobotPose),
  REQUIRES(OptionalCameraImage),
  PROVIDES(RefereePercept),
  DEFINES_PARAMETERS(
  {,
    (unsigned int)(2) bufferPortion,
    (Rangef)(0.4f,0.6f) validXAvrg,
    (Rangef)(0.4f,0.7f) validYAvrg,
  }),
});

class RefereeGestureDetectionNN : public RefereeGestureDetectionNNBase
{

  NeuralNetworkONNX::CompiledNN detector; /**< The network that detects keypoints. */
  bool isCompiled = false;
  ENUM_INDEXED_ARRAY(float, RefereePercept::Gesture) detectorOutput;
  // TODO: change to NAO friendly value
  RingBuffer<RefereePercept::Gesture, 15> gestureMemory;
  ENUM_INDEXED_ARRAY(int, RefereePercept::Gesture) gestureAppearances;
  unsigned lastBufferUpdate = 0; /**< For updating the gesture history in SimRobot as slow as on the real robot. */

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theRefereePercept The representation updated.
   */
  void update(RefereePercept& theRefereePercept) override;

  /** Compiles the neural network. */
  void compileNetwork();

public:
  /** The constructor loads the network. */
  RefereeGestureDetectionNN();

};
