/**
* @file EvidenceGrid.h
* The file declares a class that represents a grid of evidences for the presence of obstacles.
* Each cell contains a measurement history representing the previous up to 16 measurements.
* It also contains a counter that represents how many of these measurements considered the
* cell as occupied.
* The grid is centered as good as possible around the robot, i.e. it shifts so that its center
* is only up to half of a cell width away from the position of the robot. The grid is never
* rotated.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include "Grid.h"

class EvidenceGrid : public Grid
{
public:
  struct Cell
  {
    unsigned short history; /**< Each bit represents one of the up to 16 previous measurements. */
    unsigned char count; /**< The number bits set in the part of 'history' that is used. */
    unsigned char clustered; /**< Is this cell already part of a cell cluster? */
  };

private:
  /**
  * The method makes the object streamable.
  * @param in The stream from which the object is read (if in != 0).
  * @param out The stream to which the object is written (if out != 0).
  */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM_BASE(Grid);
    std::vector<unsigned>& cells = (std::vector<unsigned>&) this->cells;
    unsigned& empty = (unsigned&) this->empty;
    STREAM(cells);
    STREAM(empty);
    STREAM_REGISTER_FINISH;
  }

  Cell empty; /**< Template for an empty cell. Used for initializing cells. */
  std::vector<Cell> cells; /**< The cells of the grid. */

public:
  /**
  * Constructor defining the size of the grid.
  * @param width The width of the grid in mm.
  * @param widthInCells The width of the grid in number of cells.
  * @param heightInCells The height of the grid in number of cells.
  * @param empty Template for an empty cell. Used for initializing cells.
  */
  EvidenceGrid(float width, int widthInCells, int heightInCells, const Cell& empty);

  /**
  * The function clears the evidence grid.
  */
  void clear();

  /**
  * The operator provides access to a row of the grid.
  * @param y The row that is accessed.
  * @return The address of the first entry in the row.
  */
  Cell* operator[](int y) {return &cells[y * widthInCells];}

  /**
  * The operator provides read-only access to a row of the grid.
  * @param y The row that is accessed.
  * @return The address of the first entry in the row.
  */
  const Cell* operator[](int y) const {return &cells[y * widthInCells];}

  /**
  * The function moves the the grid to a new center.
  * @param newCenter The new center of the grid. The resulting grid center will
  *                  have a maximum deviation of cellSize/2 in both directions.
  */
  void move(const Vector2<>& newCenter);

private:
  /**
  * The function copies a rectangle in the grid to another location.
  * Both source and target areas may overlap, but both must be inside the grid.
  * @param fromX The minimum x coordinate of the rectangle to be copied.
  * @param fromY The minimum y coordinate of the rectangle to be copied.
  * @param toX The minimum x coordinate of the target rectangle.
  * @param toY The minimum y coordinate of the target rectangle.
  * @param sizeX The width of the rectangle.
  * @param sizeY The height of the rectangle.
  */
  void copy(int fromX, int fromY, int toX, int toY, int sizeX, int sizeY);

  /**
  * The function clears a rectangle of the grid with the value of 'empty'.
  * The rectangle must be inside the grid.
  * @param startX The minimum x coordinate of the rectangle to be filled.
  * @param startY The minimum y coordinate of the rectangle to be filled.
  * @param sizeX The width of the rectangle.
  * @param sizeY The height of the rectangle.
  */
  void clear(int startX, int startY, int sizeX, int sizeY);
};
