/**
 * @file Tools/Optimization/ParticleSwarm.h
 * Declaration of a simple particle swarm optimization algorithm
 * @author Colin Graf
 */

#pragma once

#include <cstddef>
#include <vector>

/**
 * @class ParticleSwarm
 * An optimizer that uses a simple particle swarm to minimize a rating of a parameter set
 */
class ParticleSwarm
{
public:
  /**
   * Constructor
   * @param particleCount The particle count
   * @param velocityFactor A factor that controls the continuance of the velocity of a particle
   * @param bestPositionFactor A factor that controls the influence of the best position of a particle on the velocity of the particle
   * @param globalBestPositionFactor A factor that controls the influence of the best particle on the velocity of a particle
   * @param randomPositionFactor A factor that controls the influence of a random position on the velocity of a particle
   */
  ParticleSwarm(unsigned int particleCount = 8,
                float velocityFactor = 0.6f, float bestPositionFactor = 0.8f,
                float globalBestPositionFactor = 1.f, float randomPositionFactor = 0.02f);

  /**
   * Adds a parameter to the optimizer
   * @param variable A variable that stores a start value of the parameter and receives parameter values for evaluation
   * @param min The minimum value of the parameter
   * @param max The maximum value of the parameter
   * @param minDelta (not used)
   */
  void addDimension(float& variable, float min, float max, float minDelta);

  /**
   * Starts the optimization
   * Add all your parameters using \c addDimension before calling this.
   */
  void start();

  /**
   * Checks whether the optimization has been started
   * @return \c true if the optimization has been started
   */
  bool isRunning() const;

  /**
   * States an evaluation of the currently used parameter set.
   * @param rating The evaluation
   */
  void setRating(float rating);

  /**
   * Returns the rating of the best parameter set found so far
   * @return The rating
   */
  float getBestRating() const;

  /**
   * Finishes using the currently used parameter set and switches to another one.
   */
  void next();

private:
  class Particle
  {
  public:
    std::vector<float> position;
    std::vector<float> velocity;

    std::vector<float> bestPosition;
    float bestFitness;

    bool rated;
  };

  class Dimension
  {
  public:
    float* variable;
    float min;
    float max;
    float minDelta;

    Dimension(float* variable, float min, float max, float minDelta) :
      variable(variable), min(min), max(max), minDelta(minDelta)
    {}

    bool isInside(float t) const {return min <= max ? t >= min && t <= max : t >= min || t <= max;}
    float limit(float t) const {return t < min ? min : t > max ? max : t;}
  };

  unsigned int particleCount;
  float velocityFactor;
  float bestPositionFactor;
  float globalBestPositionFactor;
  float randomPositionFactor;

  std::vector<Dimension> dimensions;

  std::vector<Particle> particles;
  size_t bestParticleIndex;
  size_t currentParticleIndex;

  void updateParticle(Particle& particle);
};
