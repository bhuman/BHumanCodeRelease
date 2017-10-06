/**
 * @file MovingBallPerceptRedeemer.h
 * Takes BallPercepts and checks for plausible, straight movements.
 * @author Markus Prinzler (changes)
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Tools/Debugging/DebugDrawings.h"
#include <deque>
#include "Tools/Math/BHMath.h"

#include "BallHypothesis.h"

struct PerceptData
{
  Vector2f pos;
  unsigned int timestamp_ms;
  BallPercept ballPercept;
};

class MovingBallPerceptRedeemer
{
public:
  MovingBallPerceptRedeemer();
  void update(const BallPercept& ballPercept, unsigned int timestamp_ms, const OdometryData& odometryData);

  bool plausiblePercept;
  std::deque<PerceptData> sightings; //robotCoordinates
private:
  float v0_min = 510; //mm/s, feel free to adjust;  510mm/s with -0.13mm/s^2 friction-decceleration -> ball should move about 1m in theory
  float v0_max = 3610; //mm/s, feel free to adjust; 3610mm/s with -0.13mm/s^2 friction-decceleration -> ball should move about 50m in theory
  float timeLimit = 500; //ms, feel free to adjust
  float plausibleDeviationThreshold_xDirection = 100; //mm, feel free to adjust
  float plausibleDeviationThreshold_yDirection = 25; //mm, feel free to adjust

  unsigned int maxSightings = 3; //3 minimum!
  float minMovement = v0_min / 2 / (1000 / timeLimit); //DO NOT TOUCH
  float minMovement_sq = sqr(minMovement); //squared4performance, DO NOT TOUCH
  float minMovement_doubled_sq = sqr(minMovement * 2); //squared4performance, DO NOT TOUCH
  float velocity_max_sq = sqr(v0_max); //squared4performance, DO NOT TOUCH

  OdometryData lastOdometryData;

  void updateOdometry(Pose2f odometryOffset);
  void rectangle(Vector2f pos);
  void solidCircle(Vector2f pos, float radius, ColorRGBA color);
};
