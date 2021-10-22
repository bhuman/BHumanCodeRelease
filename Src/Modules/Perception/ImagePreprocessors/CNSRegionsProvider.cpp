/**
 * This file implements a module that calculates the regions around interesting spots that must
 * be searched by CNS-based image processing modules.
 * @author Thomas Röfer
 */

#include "CNSRegionsProvider.h"
#include "Tools/Math/BHMath.h"

MAKE_MODULE(CNSRegionsProvider, perception);

void CNSRegionsProvider::update(CNSRegions& cnsRegions)
{
  ASSERT(blockSizeX % 16 == 0 && blockSizeY % 16 == 0);

  cnsRegions.regions.clear();

  const int xGridSize = theCameraInfo.width / blockSizeX;
  const int yGridSize = theCameraInfo.height / blockSizeY;
  grid.resize((xGridSize + 2) * (yGridSize + 2));
  stack.reserve((xGridSize + 2) * (yGridSize + 2));
  std::fill(grid.begin(), grid.end(), 0);
  auto searchGrid = [this, xGridSize](int y, int x) -> char& {return grid[y * (xGridSize + 2) + x];};

  // Add penalty mark regions
  for(const Boundaryi& region : theCNSPenaltyMarkRegions.regions)
    for(int y = region.y.min / blockSizeY; y < region.y.max / blockSizeY; ++y)
      for(int x = region.x.min / blockSizeX; x < region.x.max / blockSizeX; ++x)
        searchGrid(y + 1, x + 1) = 1;

  for(int y = 0; y < yGridSize; ++y)
    for(int x = 0; x < xGridSize; ++x)
      if(searchGrid(y + 1, x + 1))
      {
        Rangei xRange(x, x);
        Rangei yRange(y, y);
        floodFill(searchGrid, x + 1, y + 1, xRange, yRange);
        for(int yy = yRange.min; yy <= yRange.max; ++yy)
          for(int xx = xRange.min; xx <= xRange.max; ++xx)
            searchGrid(yy + 1, xx + 1) = 0;
        cnsRegions.regions.emplace_back(Rangei(xRange.min * blockSizeX, (xRange.max + 1) * blockSizeX),
                                        Rangei(yRange.min * blockSizeY, (yRange.max + 1) * blockSizeY));
      }
}

void CNSRegionsProvider::floodFill(const std::function<char& (int y, int x)>& searchGrid, int x, int y,
                                   Rangei& xRange, Rangei& yRange)
{
  stack.clear();
  searchGrid(y, x) = 0;
  stack.push_back(Vector2i(x, y));

  while(!stack.empty())
  {
    Vector2i point(stack.back());
    stack.pop_back();
    xRange.add(point.x() - 1);
    yRange.add(point.y() - 1);

    if(searchGrid(point.y() - 1, point.x()))
    {
      searchGrid(point.y() - 1, point.x()) = 0;
      stack.push_back(Vector2i(point.x(), point.y() - 1));
    }
    if(searchGrid(point.y() + 1, point.x()))
    {
      searchGrid(point.y() + 1, point.x()) = 0;
      stack.push_back(Vector2i(point.x(), point.y() + 1));
    }
    if(searchGrid(point.y(), point.x() - 1))
    {
      searchGrid(point.y(), point.x() - 1) = 0;
      stack.push_back(Vector2i(point.x() - 1, point.y()));
    }
    if(searchGrid(point.y(), point.x() + 1))
    {
      searchGrid(point.y(), point.x() + 1) = 0;
      stack.push_back(Vector2i(point.x() + 1, point.y()));
    }
  }
}
