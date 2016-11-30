/**
 * @file BallPerceptorEvaluator.h
 *
 * The evaluator for the BallPerceptors that fills the representation BallPerceptorEvaluation.
 * @author Alexander Stöwing <stoewing@uni-bremen.de>
 */
#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Modeling/BallPerceptorEvaluation.h"
#include "Tools/RingBuffer.h"

MODULE(BallPerceptorEvaluator,
{
  ,
  REQUIRES(CameraInfo),
  REQUIRES(BallPercept),
  PROVIDES(BallPerceptorEvaluation),
  DEFINES_PARAMETERS(
  {
    ,
    (float)(5.f) jumpAngleThreshold,  /**< At which angle difference on the xy-plane should you consider the percept as false-positive */
    (float)(1000.f) jumpOnFieldThreshold, /**< At which distance difference on the xy-plane should you consider the percept as false-positive */
  }),
});

class BallPerceptorEvaluator : public BallPerceptorEvaluatorBase
{
public:
  BallPerceptorEvaluator(); //Initializes the ring buffers
  ~BallPerceptorEvaluator();
private:
  /**
   * The function of highest level that calls all functions below.
   * @param evaluation, the representation to update
   */
  void update(BallPerceptorEvaluation& evaluation);

  /**
  * Calculates how often the ball was seen in percent.
  * False-positives are seen as a ball too.
  * @param evaluation, the representation to update
  */
  void updateSeenPercentage(BallPerceptorEvaluation& evaluation);

  /**
  * Calculates the amount of guessed false-positives in percent to the percepts.
  * @param evaluation, the representation to update
  */
  void updateGuessedFalsePositives(BallPerceptorEvaluation& evaluation);

  /**
  * Calculates the average difference for the current camera.
  * @param evaluation, the representation to update
  */
  void updateAverageDifferenceForCamera(BallPerceptorEvaluation& evaluation);

  /**
  * A helper function to calculate the average from a ring buffer.
  * @param buffer, the buffer to calculate the average from
  */
  float getAverageFromBuffer(const RingBuffer<float, 100>& buffer);

  /**
  * A helper function to keep the variables inside numeric limits.
  * @param value, the variable to check
  * @param substitute, the replacement if the variable would get out of the limits
  */
  void keepInLimits(unsigned& value, unsigned substitute);

  /**
  * The function to update all the distance related statistics.
  * @param evaluation, the representation to update
  * @param distance, the currently measured distance
  */
  void updateDistanceStatistics(BallPerceptorEvaluation& evaluation, const float& distance);

  //Here are some variables to make the calculations possible
  unsigned frameCounter = 0; //How many frames have passed
  unsigned perceptCounter = 0; //How many percepts did we have
  unsigned falsePositiveCounter = 0; //How many guessed false positives do we have
  float maximumDistance = 0.f, minimumDistance = std::numeric_limits<float>::max(); //Extrema of the distance calculation
  Vector2f lastPosition = Vector2f::Zero(), notValid = Vector2f::Zero(), currentPosition = Vector2f::Zero(); //Buffer for position comparison
  Vector2f lastPositionUpper = Vector2f::Zero(), lastPositionLower = Vector2f::Zero(); //Buffer for the positions regarding the camera
  //Ring buffer for further statistics
  RingBuffer<float, 100> distanceContainer;
  RingBuffer<float, 100> lowerDifferenceContainer;
  RingBuffer<float, 100> upperDifferenceContainer;
};
