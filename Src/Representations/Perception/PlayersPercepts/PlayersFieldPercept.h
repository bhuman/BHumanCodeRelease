/**
 * @file PlayersFieldPercept.h
 * @author Andre Mühlenbrock
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"

STREAMABLE(PlayersFieldPercept,
{
  STREAMABLE(PlayerOnField,
  {,
    (bool) lowerCamera, // true, if the robot was spotted in the lower camera
    (bool) detectedJersey, // true, if a jersey was found
    (bool) detectedFeet, // true, if the bottom of the obstacle was visible; never calculate position on field without detectedFeet!
    (unsigned char) detectedOrientation, // 0: no orientation detected, 1: [-pi/2, pi/2) information about front/back is missing, 2: [-pi, pi) complete information.
    (bool) ownTeam, // true, if a detected jersey was red
    (bool) fallen, // true, if the obstacle seems to lay on the field
    (Vector2f) centerOnField, // player center in relative to the robot in field coordinates
    (Vector2f) leftOnField, // player's left edge in relative to the robot in field coordinates
    (Vector2f) rightOnField, // player's right edge in relative to the robot in field coordinates
    (float) orientation, // player's orientation on field (radian, in robots coordinate system).
    (int[4]) borderInImage, // border in image, just for debug drawings.
  });

  /** Draws this percept. */
  void draw() const,

  (std::vector<PlayerOnField>) players,
});
