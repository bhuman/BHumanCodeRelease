/**
 * @file PenaltyMarkRegionsProvider.cpp
 *
 * This file implements a module that determines candidate regions for
 * penalty marks in the image. It provides regions in which should be
 * searched for the center as well as the regions that must be provided
 * in the CNS image for the search to work.
 *
 * @author Thomas Röfer
 */

#include "PenaltyMarkRegionsProvider.h"
#include "Tools/ImageProcessing/InImageSizeCalculations.h"

MAKE_MODULE(PenaltyMarkRegionsProvider, perception)

void PenaltyMarkRegionsProvider::update(PenaltyMarkRegions& thePenaltyMarkRegions)
{
  DECLARE_DEBUG_DRAWING("module:PenaltyMarkRegionsProvider:regions", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:PenaltyMarkRegionsProvider:mergedRegions", "drawingOnImage");

  thePenaltyMarkRegions.regions.clear();
  cnsRegions.clear();
  spots.clear();

  if(theScanGrid.y.empty())
    return;

  Vector2f pointInImage;
  if(Transformation::robotWithCameraRotationToImage(Vector2f(maxDistanceOnField, 0), theCameraMatrix, theCameraInfo, pointInImage)
     && theColorScanLineRegionsVerticalClipped.scanLines.size() > theColorScanLineRegionsVerticalClipped.lowResStart
     + theColorScanLineRegionsVerticalClipped.lowResStep)
  {
    int xStep = theColorScanLineRegionsVerticalClipped.scanLines[theColorScanLineRegionsVerticalClipped.lowResStart + theColorScanLineRegionsVerticalClipped.lowResStep].x
                - theColorScanLineRegionsVerticalClipped.scanLines[theColorScanLineRegionsVerticalClipped.lowResStart].x;

    unsigned short upperBound = static_cast<unsigned short>(std::max(0.f, pointInImage.y()));
    if(upperBound < theScanGrid.y[0])
    {
      initTables(upperBound);
      if(initRegions(upperBound))
      {
        unionFind(xStep);
        analyseRegions(upperBound, xStep, thePenaltyMarkRegions.regions);
      }
    }
  }
}

void PenaltyMarkRegionsProvider::initTables(unsigned short upperBound)
{
  extendedLower.resize(theCameraInfo.height + 1);

  int y = theScanGrid.y[0] + 1;
  int yGrid;
  for(size_t i = 1; i < theScanGrid.y.size() && (yGrid = theScanGrid.y[i]) >= upperBound; ++i)
  {
    int extension = (theScanGrid.y[i - 1] - yGrid) * regionExtensionFactor;
    while(y >= yGrid)
    {
      extendedLower[y] = static_cast<unsigned short>(y + extension);
      --y;
    }
  }
}

bool PenaltyMarkRegionsProvider::initRegions(unsigned short upperBound)
{
  regions.clear();
  for(size_t i = theColorScanLineRegionsVerticalClipped.lowResStart;
      i < theColorScanLineRegionsVerticalClipped.scanLines.size();
      i += theColorScanLineRegionsVerticalClipped.lowResStep)
  {
    auto& scanLine = theColorScanLineRegionsVerticalClipped.scanLines[i];
    bool first = true;
    for(auto& region : scanLine.regions)
    {
      if(!region.is(FieldColors::field) && region.range.lower > upperBound)
      {
        if(regions.empty() || regions.back().left != scanLine.x || regions.back().upper != region.range.lower)
        {
          if(regions.size() == regions.capacity())
            return false;
          regions.emplace_back(std::max(region.range.upper, upperBound), region.range.lower, scanLine.x, region.is(FieldColors::white));
        }
        else
        {
          Region& r = regions.back();
          unsigned short upper = std::max(region.range.upper, upperBound);
          r.upper = upper;
          r.pixels += region.range.lower - upper;
          if(region.is(FieldColors::white))
            r.whitePixels += region.range.lower - upper;
        }
        if(first || region.range.upper <= upperBound)
          regions.back().pixels = 0x8000; // force ignoring any region containing this one.
      }
      first = false;
    }
  }
  return true;
}

void PenaltyMarkRegionsProvider::unionFind(int xStep)
{
  auto i = regions.begin();
  auto j = regions.begin();
  while(i != regions.end())
  {
    if(j->left + xStep == i->left && extendedLower[j->lower] > i->upper && j->upper < extendedLower[i->lower])
      i->getRoot()->parent = j->getRoot();

    if(j->left + xStep < i->left || (j->left + xStep == i->left && j->upper > i->upper))
      ++j;
    else
      ++i;
  }

  COMPLEX_DRAWING("module:PenaltyMarkRegionsProvider:regions")
    for(auto& region : regions)
    {
      const Region* root = region.getRoot();
      LINE("module:PenaltyMarkRegionsProvider:regions", region.left, region.upper,
           region.left, region.lower - 1, 1, Drawings::solidPen, ColorRGBA(root->left & 0xff, root->upper & 0xff, root->lower & 0xff));
    }
}

void PenaltyMarkRegionsProvider::analyseRegions(unsigned short upperBound, int xStep, std::vector<Boundaryi>& searchRegions)
{
  std::vector<Region*> mergedRegions;
  mergedRegions.reserve(100);
  for(Region& region : regions)
    if(region.parent == &region)
      mergedRegions.emplace_back(&region);
    else
    {
      Region* root = region.getRoot();
      root->upper = std::min(root->upper, region.upper);
      root->lower = std::max(root->lower, region.lower);
      root->left = std::min(root->left, region.left);
      root->right = std::max(root->right, region.right);
      root->pixels += region.pixels;
      root->whitePixels += region.whitePixels;
    }

  upperBound = extendedLower[upperBound];
  unsigned short lowerBound = static_cast<unsigned short>(theScanGrid.y[0] + 1);

  struct Candidate
  {
    Boundaryi region;
    Vector2i center;
    int xExtent;
    int yExtent;
  } candidate;
  std::vector<Candidate> candidates;
  for(Region* region : mergedRegions)
    if(region->upper >= upperBound && region->lower < lowerBound)
    {
      Vector2f center(static_cast<float>(region->right + region->left) * .5f, static_cast<float>(region->lower + region->upper) * .5f);
      float expectedWidth;
      float expectedHeight;
      if(IISC::calculateImagePenaltyMeasurementsByCenter(center, expectedWidth, expectedHeight, theCameraInfo, theCameraMatrix, theFieldDimensions))
      {
        float measuredWidth = static_cast<float>(region->right - region->left);
        float measuredHeight = static_cast<float>(region->lower - region->upper);
        if(measuredWidth <= expectedWidth * (1.f + sizeToleranceRatio)
           && measuredWidth + 2 * (xStep - 1) >= expectedWidth * (1.f - sizeToleranceRatio)
           && measuredHeight <= expectedHeight * (1.f + sizeToleranceRatio)
           && measuredHeight >= expectedHeight * (1.f - sizeToleranceRatio)
           && region->left > theScanGrid.lines[theScanGrid.lowResStart].x
           && region->right < theScanGrid.lines[theScanGrid.lines.size() - 1 - theScanGrid.lowResStart].x
           && region->pixels * minWhiteRatio < region->whitePixels)
        {
          candidate.center = center.cast<int>();
          candidate.region = Boundaryi(Rangei((candidate.center.x() - xStep + 1) / blockSizeX * blockSizeX,
                                              (candidate.center.x() + xStep) / blockSizeX * blockSizeX),
                                       Rangei(candidate.center.y() / blockSizeY * blockSizeY,
                                              (candidate.center.y() + blockSizeY) / blockSizeY * blockSizeY));
          candidate.xExtent = (static_cast<int>(expectedWidth * (1.f + sizeToleranceRatio) / 2.f) + blockSizeX - 1) / blockSizeX * blockSizeX;
          candidate.yExtent = (static_cast<int>(expectedHeight * (1.f + sizeToleranceRatio) / 2.f) + blockSizeY - 1) / blockSizeY * blockSizeY;
          Boundaryi cnsRegion(Rangei(std::max(0, candidate.region.x.min - candidate.xExtent),
                                     std::min(theCameraInfo.width, candidate.region.x.max + candidate.xExtent)),
                              Rangei(std::max(0, candidate.region.y.min - candidate.yExtent),
                                     std::min(theCameraInfo.height, candidate.region.y.max + candidate.yExtent)));
          candidate.region = Boundaryi(Rangei(cnsRegion.x.min + candidate.xExtent, cnsRegion.x.max - candidate.xExtent),
                                       Rangei(cnsRegion.y.min + candidate.yExtent, cnsRegion.y.max - candidate.yExtent));
          if(candidate.region.x.min < candidate.region.x.max && candidate.region.y.min < candidate.region.y.max)
            candidates.emplace_back(candidate);
        }
      }
    }

  std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {return a.center.y() > b.center.y();});
  for(size_t i = 0; i < candidates.size() && i < maxNumberOfRegions; ++i)
  {
    const Candidate& candidate = candidates[i];
    searchRegions.emplace_back(candidate.region);
    cnsRegions.emplace_back(Rangei(candidate.region.x.min - candidate.xExtent,
                                   candidate.region.x.max + candidate.xExtent),
                            Rangei(candidate.region.y.min - candidate.yExtent,
                                   candidate.region.y.max + candidate.yExtent));
    spots.emplace_back(candidate.center.cast<int>());
  }

  COMPLEX_DRAWING("module:PenaltyMarkRegionsProvider:mergedRegions")
    for(Region* region : mergedRegions)
      RECTANGLE("module:PenaltyMarkRegionsProvider:mergedRegions", region->left, region->upper, region->right - 1, region->lower - 1,
                2, Drawings::solidPen, ColorRGBA::black);
}
