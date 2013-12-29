/**
* @file FrictionLearner.h
*
* This file declares a module which can learn the ball friction.
*
* @author Martin Böschen
*
* This module is intended to learn the ball friction. The model for the ball is the following: The ball movement
* follows the law: acceleration = ball_friction*velocity. This ball_friction can be configured in
* FieldDimensions.cfg. Its unit is 1/s.
*
* You can use this module as follows.
* - dr module:FrictionLearner:startCollectData
* - Roll the ball several times through the sight field of the robot. Make sure that the robot only see
*   ball movements, where the ball is without exterior influences.
* - After collecting a sample, an estimate is written on the output
* - After having collected enough samples, you can use the value as your ball friction.
*/

#pragma once

#include <vector>
#include <utility>

#include "Tools/Module/Module.h"
#include "Tools/Streams/InOut.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Infrastructure/CameraInfo.h"

MODULE(FrictionLearner)
  USES(CameraInfo)
  USES(FrameInfo)
  USES(CognitionFrameInfo)
  USES(BallPercept)
  PROVIDES_WITH_MODIFY(FieldDimensions)
END_MODULE

/* A ball percept, used for saving them for offline debugging. */
struct ballPercept{
  int time;
  int camera;
  float x;
  float y;
};

/* The coefficients for Savitzky–Golay filter of order 4 and windowsize of nine as given here:
http://en.wikipedia.org/wiki/Savitzky%E2%80%93Golay_filter_for_smoothing_and_differentiation */
static const int firstDirCoeffs[9] = {86,-142,-193,-126,0,126,193,142,-86};
static const int secondDirCoeffs[9] = {-4158,12243,4983,-6963,-12210,-6963,4983,12243,-4158};

class FrictionLearner : public FrictionLearnerBase
{
private:
  bool collectData; /* If data is currently collected. */

  float estimate_denom; /* The denominator of the estimator. */
  float estimate_nom;  /* The nominator of the estimator. */
  int numSamples; /* Number of samples in estimator. */

  std::vector< Vector2<> > lowerCameraPercepts; /* Collector for ball percepts of the lower camera. */
  std::vector<Vector2<> > upperCameraPercepts;  /* Collector for ball percepts of the upper camera. */

  std::vector<ballPercept> allBallPercepts; /* A list of ball percepts which can be saved into an csv file for offline analysis. */

  void update(FieldDimensions& fieldDimensions);
  void writeToFile();    /* Writes all recorded ballPercepts into a file, which is saved in  ~/Config . */
  void handleBallPercept(CameraInfo::Camera c, std::vector<Vector2<>>& ballPecepts); /* Checks for ball percepts for the corresponding camera und updates the ball percepts list and the estimator. */
  void updateAndOutput(float vec, float acc); /* Updates the estimator with a new tuple of acceleartion and velocity and outputs the result. */
};
