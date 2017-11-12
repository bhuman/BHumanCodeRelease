/**
 * @file MovingBallPerceptRedeemer.cpp
 * Implementation of the MovingBallPerceptRedeemer-Class.
 * @author Markus Prinzler
 */
#include "MovingBallPerceptRedeemer.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Transformation.h"
#include <sstream>

MovingBallPerceptRedeemer::MovingBallPerceptRedeemer()
{
  unsigned int timestamp_ms = 0; //to prevent possible division by 0
  while(sightings.size() < maxSightings)
  {
    sightings.push_front(PerceptData{ Vector2f(0, 0), timestamp_ms, BallPercept() });
    timestamp_ms++;
  }
}

void MovingBallPerceptRedeemer::update(const BallPercept& ballPercept, unsigned int timestamp_ms, const OdometryData& odometryData)
{
  //update odometry
  Pose2f odometryOffset = odometryData - lastOdometryData;
  lastOdometryData = odometryData;
  updateOdometry(odometryOffset);

  //exclude strange things and non-moving-percepts
  if(!(ballPercept.status == BallPercept::seen || ballPercept.status == BallPercept::guessed) ||
     ballPercept.positionOnField.hasNaN() ||
     ((ballPercept.positionOnField - sightings.at(sightings.size() - 1).pos).squaredNorm() < minMovement_sq) ||
     ((ballPercept.positionOnField - sightings.at(sightings.size() - 2).pos).squaredNorm() < minMovement_doubled_sq))
  {
    plausiblePercept = false;
    return;
  }

  //add sighting
  sightings.pop_front();
  sightings.push_back(PerceptData{ ballPercept.positionOnField, timestamp_ms, ballPercept });

  PerceptData sighting0 = sightings.at(0);
  PerceptData sighting1 = sightings.at(1);
  PerceptData sighting2 = sightings.at(2);

  Vector2f movement0to2 = sighting2.pos - sighting0.pos;
  unsigned int movement0to2Time = sighting2.timestamp_ms - sighting0.timestamp_ms;
  //check timeLimit
  if(movement0to2Time > timeLimit)
  {
    COMPLEX_DRAWING("module:BallLocatorPerceptRedeemer:redemption")
    {
      OUTPUT_TEXT("Time-limit breach!");
    }
    plausiblePercept = false;
    return;
  }
  float timeScale = 1.f / (movement0to2Time * 0.001f);
  Vector2f velocity = movement0to2 * timeScale;
  //check max velocity, dont check min velocity, because there is minMovement + timeLimit
  if(velocity.squaredNorm() > velocity_max_sq)
  {
    COMPLEX_DRAWING("module:BallLocatorPerceptRedeemer:redemption")
    {
      std::stringstream ss;
      ss << "Velocity too high! (" << velocity.norm() << "mm/s > " << std::sqrt(velocity_max_sq) << "mm/s)";
      OUTPUT_TEXT(ss.str());
    }
    plausiblePercept = false;
    return;
  }
  //check position of middle-percept
  unsigned int movement0to1Time = sighting1.timestamp_ms - sighting0.timestamp_ms;
  float movementRatio = static_cast<float>(movement0to1Time) / static_cast<float>(movement0to2Time);
  Vector2f expectedMovement0to1 = Vector2f(movement0to2 * movementRatio);
  Vector2f expectedPos1 = sighting0.pos + expectedMovement0to1;
  float deviation_x = expectedPos1.x() - sighting1.pos.x();
  float deviation_y = expectedPos1.y() - sighting1.pos.y();
  plausiblePercept = std::abs(deviation_x) < plausibleDeviationThreshold_xDirection &&
                     std::abs(deviation_y) < plausibleDeviationThreshold_yDirection;

  COMPLEX_DRAWING("module:BallLocatorPerceptRedeemer:redemption")
  {
    if(plausiblePercept)
    {
      std::stringstream ss;
      ss << "Aligned! (dist0to2=" << movement0to2.norm() << "mm, v=" << velocity.norm() << "mm/s, x-deviation=" << deviation_x << " y-deviation=" << deviation_y << ")";
      OUTPUT_TEXT(ss.str());
    }
    else
    {
      std::stringstream ss;
      ss << "Not aligned! (x: " << deviation_x << " of +-" << plausibleDeviationThreshold_xDirection <<
         " y: " << deviation_y << " of +-" << plausibleDeviationThreshold_yDirection << ")";
      OUTPUT_TEXT(ss.str());
    }

    solidCircle(sighting2.pos, 75, ColorRGBA::green);
    solidCircle(sighting0.pos, 75, ColorRGBA::red);
    solidCircle(sighting1.pos, 75, ColorRGBA::white);
    rectangle(expectedPos1);
  }
}

void MovingBallPerceptRedeemer::updateOdometry(Pose2f odometryOffset)
{
  Matrix2f fixedOdometryRotation;
  float odometryCos = std::cos(odometryOffset.rotation);
  float odometrySin = std::sin(odometryOffset.rotation);
  fixedOdometryRotation << odometryCos, odometrySin, -odometrySin, odometryCos;
  Vector2f fixedOdometryTranslation = -odometryOffset.translation;

  for(unsigned int i = 0; i < sightings.size(); i++)
  {
    sightings.at(i).pos = fixedOdometryRotation * sightings.at(i).pos + fixedOdometryTranslation;
  }
}

void MovingBallPerceptRedeemer::rectangle(Vector2f pos)
{
  float halfDeviation_x = plausibleDeviationThreshold_xDirection / 2;
  float halfDeviation_y = plausibleDeviationThreshold_yDirection / 2;
  Vector2f cornerTL = pos + Vector2f(halfDeviation_x, -halfDeviation_y);
  Vector2f cornerTR = pos + Vector2f(halfDeviation_x, halfDeviation_y);
  Vector2f cornerBL = pos + Vector2f(-halfDeviation_x, -halfDeviation_y);
  Vector2f cornerBR = pos + Vector2f(-halfDeviation_x, halfDeviation_y);
  LINE("module:BallLocatorPerceptRedeemer:redemption", cornerTL.x(), cornerTL.y(), cornerTR.x(), cornerTR.y(), 5, Drawings::solidPen, ColorRGBA::black);
  LINE("module:BallLocatorPerceptRedeemer:redemption", cornerTR.x(), cornerTR.y(), cornerBR.x(), cornerBR.y(), 5, Drawings::solidPen, ColorRGBA::black);
  LINE("module:BallLocatorPerceptRedeemer:redemption", cornerBR.x(), cornerBR.y(), cornerBL.x(), cornerBL.y(), 5, Drawings::solidPen, ColorRGBA::black);
  LINE("module:BallLocatorPerceptRedeemer:redemption", cornerBL.x(), cornerBL.y(), cornerTL.x(), cornerTL.y(), 5, Drawings::solidPen, ColorRGBA::black);
}

void MovingBallPerceptRedeemer::solidCircle(Vector2f pos, float radius, ColorRGBA color)
{
  CIRCLE("module:BallLocatorPerceptRedeemer:redemption", pos.x(), pos.y(), radius, 0, Drawings::solidPen, color, Drawings::solidBrush, color);
}