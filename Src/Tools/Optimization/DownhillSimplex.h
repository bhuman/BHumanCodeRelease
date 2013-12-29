/**
* @file Tools/Optimization/DownhillSimplex.h
* Declaration of a downhill simplex optimization algorithm
* @author Colin Graf
*/

#pragma once

#include <vector>
#include <list>

/**
* @class DownhillSimplex
* An optimizer that uses a downhill simplex algorithm to minimize a rating of a parameter set
*/
class DownhillSimplex
{
public:

  /** Constructor */
  DownhillSimplex();

  /**
  * Adds a parameter to the optimizer
  * @param variable A variable that stores a start value of the parameter and receives parameter values for evaluation
  * @param min (not used)
  * @param max (not used)
  * @param minDelta The smallest expedient offset between to parameter values
  */
  void addDimension(float& variable, float min, float max, float minDelta);

  /**
  * Starts the optimization
  * Add all your paramters using \c addDimension before calling this.
  */
  void start();

  /**
  * Checks whether the optimization has been started
  * @return \c true if the optimization has been started
  */
  bool isRunning() const {return points.size() > 0;}

  /**
  * States an evaluation of the currently used parameter set.
  * @param rating The evaluation
  */
  void setRating(float rating);

  /**
  * Returns the rating of the best parameter set found so far
  * @return The rating
  */
  float getBestRating() const {return bestRating;}

  /**
  * Finishes using the currently used parameter set and switches to another one.
  */
  void next();

private:
  class Dimension
  {
  public:
    float* variable;
    float min;
    float max;
    float minDelta;

    Dimension(float* variable, float min, float max, float minDelta) : variable(variable), min(min), max(max), minDelta(minDelta) {}
  };

  class Point
  {
  public:
    std::vector<float> position;
    float rating;

    inline bool operator<(const Point& other) const {return rating < other.rating;}
  };

  enum State
  {
    computeReflection,
    evaluateReflection,
    computeExpansion,
    evaluateExpansion,
    computeContraction,
    evaluateContraction,
    computeReduction,
  };

  std::vector<Dimension> dimensions;
  std::vector<Point> points;
  std::list<Point*> unratedPoints;
  float bestRating;
  State state;
  std::vector<float> centerOfGravity;
  Point reflectionPoint;
  Point expansionPoint;
  Point contractionPoint;
};
