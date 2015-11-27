/**
 * @file Tools/Optimization/ParticleSwarm.cpp
 * Implementation of a simple particle swarm optimization algorithm
 * @author Colin Graf
 */

#include <limits>

#include "ParticleSwarm.h"
#include "Tools/Math/Random.h"
#include "Platform/BHAssert.h"

ParticleSwarm::ParticleSwarm(unsigned int particleCount, float velocityFactor, float bestPositionFactor,
                             float globalBestPositionFactor, float randomPositionFactor) :
  particleCount(particleCount), velocityFactor(velocityFactor), bestPositionFactor(bestPositionFactor),
  globalBestPositionFactor(globalBestPositionFactor), randomPositionFactor(randomPositionFactor)
{}

bool ParticleSwarm::isRunning() const
{
  return !particles.empty();
}

void ParticleSwarm::addDimension(float& variable, float min, float max, float minDelta)
{
  ASSERT(particles.empty());
  ASSERT(max > min);
  ASSERT(minDelta > 0.f);
  ASSERT(minDelta <= max - min);
  dimensions.push_back(Dimension(&variable, min, max, minDelta));
}

void ParticleSwarm::start()
{
  ASSERT(particles.empty());
  particles.resize(particleCount);
  for(unsigned int i = 0; i < particles.size(); ++i)
  {
    Particle& particle(particles[i]);
    particle.position.resize(dimensions.size());
    particle.velocity.resize(dimensions.size());
    for(unsigned int j = 0; j < dimensions.size(); ++j)
    {
      const Dimension& dimension = dimensions[j];
      float random = i == 0 ? 0.f : randomFloat(); // the first particles consists of the given start values
      particle.position[j] = *dimension.variable + (random >= 0.5f ?
                             (dimension.max - *dimension.variable) * (random - 0.5f) * 2.f : (dimension.min - *dimension.variable) * random * 2.f);
    }
    particle.bestPosition = particle.position;
    particle.bestFitness = std::numeric_limits<float>::max();
    particle.rated = false;
  }
  bestParticleIndex = 0;
  currentParticleIndex = particles.size() - 1;
}

void ParticleSwarm::next()
{
  currentParticleIndex = (currentParticleIndex + 1) % particles.size();
  Particle& particle = particles[currentParticleIndex];
  updateParticle(particle);
  for(size_t i = 0, count = dimensions.size(); i < count; ++i)
    *dimensions[i].variable = particle.position[i];
}

void ParticleSwarm::setRating(float rating)
{
  Particle& particle(particles[currentParticleIndex]);
  if(rating < particle.bestFitness)
  {
    particle.bestFitness = rating;
    particle.bestPosition = particle.position;
  }
  particle.rated = true;

  Particle& bestParticle(particles[bestParticleIndex]);
  if(rating < bestParticle.bestFitness)
    bestParticleIndex = currentParticleIndex;
}

float ParticleSwarm::getBestRating() const
{
  return particles[bestParticleIndex].bestFitness;
}

void ParticleSwarm::updateParticle(Particle& particle)
{
  if(!particle.rated)
    return;
  Particle& bestParticle(particles[bestParticleIndex]);
  for(unsigned int i = 0; i < particle.position.size(); ++i)
  {
    particle.velocity[i] = velocityFactor * particle.velocity[i]
      + bestPositionFactor * randomFloat() * (particle.bestPosition[i] - particle.position[i])
      + globalBestPositionFactor * randomFloat() * (bestParticle.bestPosition[i] - particle.position[i])
      + randomPositionFactor * randomFloat() * ((dimensions[i].min + randomFloat() * (dimensions[i].max - dimensions[i].min)) - particle.position[i]);
    if(dimensions[i].isInside(particle.position[i] + particle.velocity[i]))
      particle.position[i] += particle.velocity[i];
    else
    {
      particle.position[i] = dimensions[i].limit(particle.position[i] + particle.velocity[i]);
      particle.velocity[i] = 0.;
    }
  }
  particle.rated = false;
}
