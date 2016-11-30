/**
 * @file ObstacleRose.h
 *
 * TODO: Description
 *
 * @author <a href="mailto:aluebken@informatik.uni-bremen.de">Andre Luebken</a>
 */

#pragma once

#include "Tools/Math/Angle.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Modeling/Obstacle.h"
#include "Tools/Streams/AutoStreamable.h"

#define NUM_SEGMENTS 6
#define FRAME_COUNT 10
#define DRAW_SECTOR_LINE_LENGTH 1000


STREAMABLE(ObstacleSector,
{
  ObstacleSector() = default;
  ObstacleSector(const float start, const float stop, const unsigned frameCount);
  ,
  (Angle)(0.0) startBearing,
  (Angle)(0.0) stopBearing,
  (bool[FRAME_COUNT]) blocked, /** ring buffer, consecutive frames a sector has been blocked */
});

inline ObstacleSector::ObstacleSector(const float start, const float stop,
                                      const unsigned frameCount) : startBearing(start), stopBearing(stop)
{
  for(unsigned i = 0; i < frameCount; ++i)
  {
    blocked[i] = false;
  }
};

STREAMABLE(ObstacleRose,
{
  ObstacleRose();
  void draw() const;
  void updateSectorSettings(const unsigned frameCount, const unsigned sectorCount), /** change frame_count, if parameter was changed by module */

  (Pose2f) pose, /** Pose of the robot on the field */
  (unsigned)(0) currentFrame, /** frame count for blocked ring buffer */
  (unsigned)(FRAME_COUNT) frameCount,  /** frame count limit */
  (unsigned)(NUM_SEGMENTS) sectorCount, /** frame count limit */
  (std::vector<ObstacleSector>) obstacleSectors,
});
