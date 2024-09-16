/**
 * @file BallSearchAreas.h
 *
 * This file declares a representation that describes the field areas in which the ball should be searched for
 *
 * @author Sina Schreiber
 */

#pragma once

#include "Math/Eigen.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/Function.h"
struct Agent;

STREAMABLE(BallSearchAreas,
{
  STREAMABLE(Cell,
  {
    Cell() = default;
    Cell(const Vector2f& positionOnField, unsigned timestamp),

    (Vector2f) positionOnField, /**< position of the cell in field coordinates*/
    (unsigned) timestamp,/**< the timestamp of the cell, time since last checked*/
    (unsigned)(1) priority, /**< the priority of the cell, calculated out of the timestamp anf the current game state*/
  });
  FUNCTION(Vector2f(const Agent& agent)) cellToSearchNext;  /**< function that returns the cell with the highest score calculated by multiplying the timestamp and the priority.*/
  FUNCTION(std::vector<BallSearchAreas::Cell>(const Agent& agent)) filterCellsToSearch; /**< function that filters the cells of the ballsearch grid that are inside the Voronoi region of the robot*/
  std::vector<BallSearchAreas::Cell> grid, /**< the complete grid */
});

inline BallSearchAreas::Cell::Cell(const Vector2f& positionOnField, unsigned timestamp)
  : positionOnField(positionOnField), timestamp(timestamp)
{}
