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
#include "Streaming/Enum.h"
#include "Streaming/Function.h"
struct Agent;

STREAMABLE(BallSearchAreas,
{
  STREAMABLE(Cell,
  {
    Cell() = default;
    Cell(unsigned id, unsigned priority, unsigned timestamp, const Vector2f& positionOnField),

    (unsigned) id, /**< the number of the cell*/
    (unsigned)(1) priority, /**< the priority of the cell, calculated out of the timestamp anf the current game state*/
    (unsigned)(0) timestamp,/**< the timestamp of the cell, time since last checked*/
    (Vector2f) positionOnField, /**< position of the cell in field coordinates*/
  });
  FUNCTION(Vector2f(const Agent& agent)) cellToSearchNext;  /**< function that returns the cell with the highest score calculated by multiply the timestamp and the priority.*/
  FUNCTION(std::vector<BallSearchAreas::Cell>(const Agent& agent)) filterCellsToSearch, /**< function that filteres the cells of the ballsearch grid that are inside the voronoi region of the robot*/

  (std::vector<Cell>) grid, /**< vector contains the grid*/
  (float) cellLength, /**< length of the cell, height and width*/
  (int) cellCountX, /**< the number of the cells on field in x direction*/
  (int) cellCountY, /**< the number of the cells on field in y direction*/
});
