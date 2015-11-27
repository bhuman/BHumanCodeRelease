/**
 * @file Representations/MotionControl/MotionRequest.cpp
 * Implementation of a struct that represents the motions that can be requested from the robot.
 */

#include <cstdio>
#include <cstring>

#include "MotionRequest.h"
#include "Tools/Debugging/DebugDrawings.h"

void MotionRequest::printOut(char* destination) const
{
  strcpy(destination, getName(motion));
  destination += strlen(destination);
  switch(motion)
  {
    case walk:
      if(walkRequest.mode == WalkRequest::targetMode)
        sprintf(destination, ": %.0lfmm %.0lfmm %.0lf°",
                walkRequest.target.translation.x(), walkRequest.target.translation.y(),
                walkRequest.target.rotation.toDegrees());
      else if(walkRequest.mode == WalkRequest::percentageSpeedMode)
        sprintf(destination, ": %.0lf%% %.0lf%% %.0lf%%",
                walkRequest.speed.translation.x() * 100, walkRequest.speed.translation.y() * 100,
                walkRequest.speed.rotation * 100);
      else
        sprintf(destination, ": %.0lfmm/s %.0lfmm/s %.0lf°/s",
                walkRequest.speed.translation.x(), walkRequest.speed.translation.y(),
                walkRequest.speed.rotation.toDegrees());
      break;
    case stand:
      sprintf(destination, ": stand");
      break;
    case specialAction:
      sprintf(destination, ": %s", SpecialActionRequest::getName(specialActionRequest.specialAction));
      break;
    case kick:
      sprintf(destination, ": %s", KickRequest::getName(kickRequest.kickMotionType));
      break;
  }
}

void MotionRequest::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:MotionRequest", "drawingOnField"); // drawing of a request walk vector
  if(motion == walk)
  {
    switch(walkRequest.mode)
    {
      case WalkRequest::targetMode:
      {
        LINE("representation:MotionRequest", 0, 0, walkRequest.target.translation.x(), walkRequest.target.translation.y(), 0, Drawings::solidPen, ColorRGBA(0xcd, 0, 0));
        CROSS("representation:MotionRequest", walkRequest.target.translation.x(), walkRequest.target.translation.y(), 50, 0, Drawings::solidPen, ColorRGBA(0xcd, 0, 0));
        Vector2f rotation(500.f, 0.f);
        rotation.rotate(walkRequest.target.rotation);
        ARROW("representation:MotionRequest", walkRequest.target.translation.x(), walkRequest.target.translation.y(), walkRequest.target.translation.x() + rotation.x(), walkRequest.target.translation.y() + rotation.y(), 0, Drawings::solidPen, ColorRGBA(0xcd, 0, 0, 127));
        break;
      }
      case WalkRequest::speedMode:
      case WalkRequest::percentageSpeedMode:
      {
        Vector2f translation = walkRequest.mode == WalkRequest::speedMode ? walkRequest.speed.translation * 10.f : walkRequest.speed.translation * 1000.f;
        ARROW("representation:MotionRequest", 0, 0, translation.x(), translation.y(), 0, Drawings::solidPen, ColorRGBA(0xcd, 0, 0));
        if(walkRequest.target.rotation != 0.0f)
        {
          translation.x() = translation.norm();
          translation.y() = 0;
          translation.rotate(walkRequest.speed.rotation);
          ARROW("representation:MotionRequest", 0, 0, translation.x(), translation.y(), 0, Drawings::solidPen, ColorRGBA(0xcd, 0, 0, 127));
        }
        break;
      }
    }
  }
}
