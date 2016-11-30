//
//  TImageBinarizer.cpp
//  B-Human
//
//  Created by Peter Schulz on 12/06/16.
//
//

#include "TImageBinarizer.h"
#include <map>

TImageBinarizer::TImageBinarizer() : threshold(128), minLength(4)
{
  
}

TImageBinarizer::IntervalSet TImageBinarizer::binarize(TImage<PixelTypes::GrayscaledPixel> image, int xOffset, int xLength, int yOffset, int yLength)
{
  IntervalSet intervals;
  for(int y = yOffset; y < (yOffset + yLength); y++)
  {
    PixelTypes::GrayscaledPixel* line = image[y];
    for(int x = xOffset; x < (xOffset + xLength); x++)
    {
      PixelTypes::GrayscaledPixel pixel = line[x];
      if(line[x] > threshold)
      {
        int xLow = x;
        while(x < (xOffset + xLength) && line[x] > threshold)
        {
          pixel = line[x];
          x++;
        }
        if((x - xLow) >= minLength)
        {
          Interval interVal = Interval(xLow, x, y);
          intervals.rle.push_back(interVal);
        }
      }
    }
  }
  
  return intervals;
};

void TImageBinarizer::IntervalSet::groupIntervals()
{
  initialize();
  std::vector<TImageBinarizer::Interval>::iterator flw = rle.begin();
  std::vector<TImageBinarizer::Interval>::iterator run = rle.begin();
  while(run != rle.end())
  {
    if(touch(&(*run), &(*flw)))
    {
      unite(&(*run), &(*flw));
    }
    if(ahead(&(*run), &(*flw)))
    {
      flw++;
    }
    else
    {
      run++;
    }
  }
  setRegionIndex();
}

void TImageBinarizer::IntervalSet::pathCompress (Interval* iv)
{
  Interval* root = iv;
  while(root->parent != root)
  {
    root = root->parent;
  }
  while(iv != root)
  {
    Interval* buffer = iv->parent;
    iv->parent = root;
    iv = buffer;
  }
}

void TImageBinarizer::IntervalSet::unite (Interval* iv1, Interval* iv2)
{
  pathCompress(iv1);
  pathCompress(iv2);
  if(iv1->parent < iv2->parent)
  {
    iv2->parent->parent = iv1->parent;
  }
  else
  {
    iv1->parent->parent = iv2->parent;
  }
}


void TImageBinarizer::IntervalSet::initialize ()
{
  for(std::vector<Interval>::iterator it = rle.begin(); it != rle.end(); it++)
  {
    (*it).parent = &(*it);
  }
}


void TImageBinarizer::IntervalSet::setRegionIndex ()
{
  std::vector<Interval>::iterator iv = rle.begin();
  int regionCounter = 0;
  while(iv != rle.end())
  {
    if(iv->parent == &(*iv))
    {
      iv->region = regionCounter;
      regionCounter++;
    }
    else
    {
      iv->region = iv->parent->region;
    }
    iv++;
  }
}


bool TImageBinarizer::IntervalSet::touch (Interval* run, Interval* flw)
{
  return (run->y == flw->y+1) && (run->xLo <= flw->xHi) && (flw->xLo <= run->xHi);
}

bool TImageBinarizer::IntervalSet::ahead (Interval* run, Interval* flw)
{
  return (run->y > (flw->y + 1)) || ((run->y == (flw-> y + 1)) && (run->xHi > flw->xHi));
}

void TImageBinarizer::Region::extractRegions(TImageBinarizer::IntervalSet& intervalSet, std::vector<Region>& regions)
{
  std::map<int, Region> indexToRegion;
  for(std::size_t i = 0; i < intervalSet.rle.size(); i++)
  {
    Interval iv = intervalSet.rle[i];
    std::map<int, Region>::iterator regionIt = indexToRegion.find(iv.region);
    if(regionIt == indexToRegion.end())
    {
      regionIt = indexToRegion.insert(std::make_pair(iv.region, Region())).first;
    }
    
    Region& r = regionIt->second;
    r.integralX += ((iv.xHi * (iv.xHi + 1)) - (iv.xLo * (iv.xLo - 1))) / 2;
    r.integralY += (iv.xHi - iv.xLo + 1) * iv.y;
  }
  
  for(std::map<int, Region>::iterator it = indexToRegion.begin(); it != indexToRegion.end(); ++it)
  {
    regions.push_back(it->second);
  }
}