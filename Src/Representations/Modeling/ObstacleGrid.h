/**
 * @file ObstacleGrid.h
 *
 * Declaration of struct ObstacleGrid.
 *
 * @author Nele Matschull
 */

#pragma once

#include "Streaming/FunctionList.h"
#include "Math/Eigen.h"
#include "Math/Pose2f.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/InOut.h"
#include "Streaming/Streamable.h"

/**
 * A cell of the grid. It is streamed externally to avoid a virtual method
 * table pointer in each cell (which also the compiler does not like in
 * combination with memmove).
 */
struct Cell
{
  unsigned lastUpdate = 0;     /** The point of time of the last change of the state*/
  unsigned char state = 0;     /** The state of the cell, i.e. the number of positive measurements within the last few frames*/

private:
  static void reg()
  {
    PUBLISH(reg);
    REG_CLASS(Cell);
    REG(lastUpdate);
    REG(state);
  }
};

inline Out& operator<<(Out& stream, const Cell& cell)
{
  return stream << cell.lastUpdate << cell.state;
}

inline In& operator>>(In& stream, Cell& cell)
{
  return stream >> cell.lastUpdate >> cell.state;
}

STREAMABLE(ObstacleGrid,
  {
    void draw();

  /**
   * Returns the index of the cell the position is assigned to
   * @param inWorld A position relative to the robot
   */
  int worldToGrid(const Vector2f& inWorld) const,

  (Cell[2025]) cells,      /**< The grid (For the moment, try a 30x30 grid of 4cm cells - TODO) */
  (Pose2f) gridOffset,    /**< The relative position of the grid's origin */
});
