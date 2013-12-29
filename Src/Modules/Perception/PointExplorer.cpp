#include "Representations/Perception/RegionPercept.h"
#include "PointExplorer.h"
#include "Tools/Debugging/Modify.h"
#include "Tools/Debugging/Asserts.h"

void PointExplorer::initFrame(const Image* image, const ColorReference* colRef, int exploreStepSize, int gridStepSize, int skipOffset, int* minSegLength)
{
  theImage = image;
  theColRef = colRef;
  parameters.exploreStepSize = exploreStepSize;
  parameters.gridStepSize = gridStepSize;
  parameters.skipOffset = skipOffset;
  parameters.minSegSize = minSegLength;
  DECLARE_DEBUG_DRAWING("module:PointExplorer:runs", "drawingOnImage");
}

ColorClasses::Color PointExplorer::getColor(const Image::Pixel* pixel)
{
  ColorReference::MultiColor colors = theColRef->getColorClasses(pixel);
  if(colors.isOrange())//this order is highly recommended
    return ColorClasses::orange;
  else if(colors.isWhite())
    return ColorClasses::white;
  else if(colors.isGreen())
    return ColorClasses::green;
  else
    return ColorClasses::none;
}

int PointExplorer::explorePoint(int x, int y, const ColorClasses::Color col, int xMin, int yEnd, int yMin, int& run_end, int& explored_min_y, int& explored_max_y)
{
  int size = 0;
  const Image::Pixel* pixel3 = (*theImage)[y] + x;
  run_end = runDown(pixel3, x, y, col, yEnd, Drawings::ps_solid);
  explored_min_y = y;
  size = (run_end - y) * parameters.gridStepSize;
  if(col != ColorClasses::green)
  {
    explored_max_y = run_end - 1;
    if(!(run_end - y >= parameters.minSegSize[col]))
    {
      return -1;
    }
    const Image::Pixel* pixel1 = (*theImage)[explored_min_y] + x;
    const Image::Pixel* pixel2 = (*theImage)[explored_max_y] + x;
    for(x -= parameters.exploreStepSize, pixel1 -= parameters.exploreStepSize, pixel2 -= parameters.exploreStepSize;
      x > xMin;
      x -= parameters.exploreStepSize, pixel1 -= parameters.exploreStepSize, pixel2 -= parameters.exploreStepSize)
    {
      if(getColor(pixel1) == col)
      {
        const int expl_run_end = runUp(pixel1, x, explored_min_y, col, yMin, Drawings::ps_dot);
        if(expl_run_end < explored_min_y)
        {
          explored_min_y = expl_run_end + 1;
          pixel1 = (*theImage)[explored_min_y] + x;
        }
      }
      if(getColor(pixel2) == col)
      {
        const int expl_run_end = runDown(pixel2, x, explored_max_y, col, yEnd, Drawings::ps_dot);
        if(expl_run_end > explored_max_y)
        {
          explored_max_y = expl_run_end - 1;
          pixel2 = (*theImage)[explored_max_y] + x;
        }
      }
    }
  }
  else
  {
    explored_max_y = run_end;
  }
  return size;
}

int PointExplorer::runDown(const Image::Pixel* pixel, int x, int yStart, ColorClasses::Color col, int yEnd, Drawings::PenStyle draw)
{
  int y = yStart;
  int tmp;
  for(y += parameters.skipOffset, pixel += parameters.skipOffset * (theImage->widthStep);
    y < yEnd;
    y += parameters.skipOffset, pixel += parameters.skipOffset * (theImage->widthStep))
  {
    if(getColor(pixel) != col)
    {
      tmp = y - parameters.skipOffset;
      for(--y, pixel -= (theImage->widthStep); y > tmp; y--, pixel -= (theImage->widthStep))
      {
        if(getColor(pixel) == col)
        {
          break;
        }
      }
      if(y == tmp)
      {
        y++;
        break;
      }
    }
  }
  if(y > yEnd)
  {
    y = yEnd;
  }
  LINE("module:PointExplorer:runs", x, yStart, x, y, 0, draw, getOnFieldDrawColor(col));
  return y;
}

int PointExplorer::findDown(const Image::Pixel* pixel, int x, int yStart, ColorClasses::Color col, int yEnd, Drawings::PenStyle draw)
{//this method is only for searching balls
  int y = yStart;
  int tmp;
  for(y++, pixel += (theImage->widthStep);
    y < yEnd;
    y++, pixel += (theImage->widthStep))
  {
    if(getColor(pixel) == col)
    {
      tmp = y - 1;
      for(--y, pixel -= (theImage->widthStep); y > tmp; y--, pixel -= (theImage->widthStep))
      {
        if(getColor(pixel) != col)
        {
          break;
        }
      }
      if(y == tmp)
      {
        y++;
        break;
      }
    }
  }
  if(y > yEnd)
  {
    y = yEnd;
  }
  LINE("module:PointExplorer:runs", x, yStart, x, y, 0, draw, getOnFieldDrawColor(col));
  return y;
}

int PointExplorer::runUp(const Image::Pixel* pixel,int x, int yStart, ColorClasses::Color col, int yEnd, Drawings::PenStyle draw)
{
  int y = yStart;
  int tmp;
  for(y -= parameters.skipOffset, pixel -= theImage->widthStep * parameters.skipOffset;
    y > yEnd;
    y -= parameters.skipOffset, pixel -= theImage->widthStep * parameters.skipOffset)
  {
    if(getColor(pixel) != col)
    {
      tmp = y + parameters.skipOffset;
      for(++y, pixel += theImage->widthStep; y < tmp; y++, pixel += theImage->widthStep)
      {
        if(getColor(pixel) == col)
        {
          break;
        }
      }
      if(y == tmp)
      {
        y--;
        break;
      }
    }
  }
  if(y < yEnd)
  {
    y = yEnd;
  }
  LINE("module:PointExplorer:runs", x, yStart, x, y, 0, draw, getOnFieldDrawColor(col));
  return y;
}
