#pragma once

#include "Representations/Perception/ColorReference.h"
#include "Representations/Infrastructure/Image.h"
#include "Tools/Debugging/DebugDrawings.h"

/**
 * @class PointExplorer
 *
 * The PointExplorer is a class which runs through the image. It has several
 * function which run on scanlines throught the image. They start from a given
 * point and run until the color changes or until they find a certain color.
 * Additionally there's a function (explorePoint) which not simply runs along
 * a scanline but "explores" a scanline(segment). It runs throught the image
 * and additionally to the normal run functions it also takes the space between
 * the actual scanline and the last into account.
 */
class PointExplorer
{
public:
  /**
   * Initialize the PointExplorer for a frame. Pass the ColorTable, the Image and some parameters.
   * @param image theImage Representation of the frame
   * @param colRef theColorReference Representation
   * @param exploreStepSize the distance in pixels between the explore scanlines
   * @param gridStepSize the distance in pixels between (normal) scanlines
   * @param skipOffset the amount of pixels allowed to skip in a run
   * @param minSegLength a array giving holding the minimum segment size for each color
   */
  void initFrame(const Image* image, const ColorReference* colRef, int exploreStepSize, int gridStepSize, int skipOffset, int* minSegLength);

  /**
   * Run down from (x,y) unless there is a run of skipOffset pixels with color != col.
   * @param pixel pixel to start from
   * @param x x-coordinate to start from
   * @param yStart y-coordinate to start from
   * @param col the color of the run
   * @param yEnd the maximal y-coordinate, a run will end when it reaches yEnd
   * @param draw the PenStyle used for the DebugDrawings
   * @return the first pixel (y-coordinate) after the run (col(x,returny) != col || returny == yEnd)
   */
  int runDown(const Image::Pixel* pixel, int x, int yStart, ColorClasses::Color col, int yEnd, Drawings::PenStyle draw = Drawings::ps_null);

  /**
   * Run down from (x,y) unless there is a run of skipOffset pixels with the color col. This method is only for searching balls.
   * @param pixel pixel to start from
   * @param x x-coordinate to start from
   * @param yStart y-coordinate to start from
   * @param col the color to find
   * @param yEnd the maximal y-coordinate, a run will end when it reaches yEnd
   * @param draw the PenStyle used for the DebugDrawings
   * @return the first pixel (y-coordinate) followed by skipOffset pixels with the color col or yEnd
   */
  int findDown(const Image::Pixel* pixel, int x, int yStart, ColorClasses::Color col, int yEnd, Drawings::PenStyle draw = Drawings::ps_null);

  /**
   * Run up from (x,y) unless there is a run of skipOffset pixels with color != col.
   * @param pixel pixel start from
   * @param x x-coordinate to start from
   * @param y y-coordinate to start from
   * @param col the color of the run
   * @param yEnd the minimal y-coordinate, a run will end when it reaches yEnd
   * @param draw the PenStyle used for the DebugDrawings
   * @return the first pixel (y-coordinate) before the run (col(x,returny) != col || returny == yEnd)
   */
  int runUp(const Image::Pixel* pixel, int x, int y, ColorClasses::Color col, int yEnd, Drawings::PenStyle draw = Drawings::ps_null);

  /**
   * Explore a pixel: if col == green and force_detailed == false this is a normal runDown call
   *                  and additionally the size and explored_min_y (==y) and explored_max_y (==runEnd)
   *                  are returned.
   *                  Otherwise it will do a runDown and afterwards look from x to xMin(last scanline)
   *                  whether there are runs connected to the "runDown-run" which start before or end
   *                  after the "runDown-run". These y-coordinates are stored in explored_min_y and
   *                  explored_max_y.
   *                  The size returned is the length of the run multiplied by gridStepSize.
   * @param x x-coordiante to start from
   * @param y y-coordinate to start from
   * @param col the color of the run
   * @param xMin the x-coordinate of the last scanline (exploration is done from x to xMin)
   * @param yEnd the maximal y-coordinate, a run will end when it reaches yEnd
   * @param yMin the minimal y-coordinate (exploration will stop there)
   * @param runEnd returns the y-coordinate of the end of the run
   * @param explored_min_y returns the minimal y-coordinate of all the runs (including the explored ones)
   * @param explored_max_y returns the maximal y-coordinate of all the runs (including the explored ones)
   * @param force_detailed explore green pixels/runs ?
   * @return the size of the run (length * gridStepSize)
   */
  int explorePoint(int x, int y, const ColorClasses::Color col, int xMin, int yEnd, int yMin, int& runEnd, int& explored_min_y, int& explored_max_y);

  /**
   * Returns the ColorClass of the pixel in the image.
   * @param pixel pixel to get the color from
   * @return the color of the pixel
   */
  ColorClasses::Color getColor(const Image::Pixel* pixel);

  const Image* theImage; /**< a pointer to the image Representation */
  const ColorReference* theColRef;

private:
  /**
   * @class Parameters
   * Internal parameters for the PointExplorer.
   */
  class Parameters
  {
  public:
    int exploreStepSize; /**< distance between explore scanlines */
    int gridStepSize; /**< distance between two scanlines */
    int skipOffset; /**< the amount of pixels allowed to skip in a run */
    int* minSegSize; /**< the minimum length of a run for each color */
  };
  Parameters parameters;

};
