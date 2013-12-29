/**
* @file GaussianTable.h
*
* Definition of class GaussianTable
*
* @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
*/

#pragma once

#include "Probabilistics.h"

/**
* @class GaussianTable
* Structure for precomputing values of the gaussian function
*/
class GaussianTable
{
public:
  /** Default constructor. */
  GaussianTable() : table(0) {}

  /** Constructor*/
  GaussianTable(int maxX, int xSteps,
                int minStdDev, int maxStdDev, int stdDevSteps) : table(0)
  {
    computeTable(maxX, xSteps, minStdDev, maxStdDev, stdDevSteps);
  }

  /** Destructor*/
  ~GaussianTable()
  {
    if(table)
      delete[] table;
  }

  bool computeTable(int maxX, int xSteps, int minStdDev, int maxStdDev, int stdDevSteps)
  {
    if(!table ||
       this->maxX != maxX ||
       this->xSteps != xSteps + 1 ||
       this->minStdDev != minStdDev ||
       this->maxStdDev != maxStdDev ||
       this->stdDevSteps != stdDevSteps + 1)
    {
      this->maxX = maxX;
      this->xSteps = xSteps + 1;
      this->minStdDev = minStdDev;
      this->maxStdDev = maxStdDev;
      this->stdDevSteps = stdDevSteps + 1;
      if(table)
        delete[] table;
      init();
      return true;
    }
    else
      return false;
  }

  float value(int x, int s)
  {
    s /= stdDevSegmentWidth;
    x /= xSegmentWidth;
    if(s >= stdDevSteps)
      s = stdDevSteps - 1;
    if(x >= xSteps)
      x = xSteps - 1;
    return table[x * stdDevSteps + s];
  }

private:
  void init()
  {
    table = new float[xSteps * stdDevSteps];
    xSegmentWidth = maxX / xSteps;
    stdDevSegmentWidth = maxStdDev / stdDevSteps;
    for(int s = 1; s < stdDevSteps; s++)
      for(int x = 0; x < xSteps; x++)
      {
        float value = float((maxStdDev + minStdDev) / 2) * gaussianProbability((float)(x * xSegmentWidth), (float)(s * stdDevSegmentWidth));
        table[x * stdDevSteps + s] = value;
      }
  }

  int maxX;
  int xSegmentWidth;
  int xSteps;
  int minStdDev;
  int maxStdDev;
  int stdDevSteps;
  int stdDevSegmentWidth;
  float* table;
};
