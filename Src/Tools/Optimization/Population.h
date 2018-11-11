/**
 * @file Population.h
 * Population for Evolutionary Robotics.
 * It contains a generation of populations and provides functions to update the generation
 * @author Enno Röhrig
 */

#include "Tools/Range.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/EnumIndexedArray.h"

#pragma once

class Population
{
public:
  STREAMABLE(Configuration,
  {,
    //TODO better variable names
    (unsigned) total, // generation size
    (int) hallOfFamers,
    (int) numFeatures,
    (int) trialEvaluationMethod,  // 0 = Median | 1 = Mean
    (int)(3) trials,
    (float) sigma,
    (float) stopFitness,

    (std::vector<float>) initialization,
    (std::vector<Rangef>) limits,
    (std::string) path,
  });

  virtual ~Population() = default;

  virtual void init(const Configuration& configuration) = 0;
  virtual std::vector<float> getNextIndividual() = 0;
  virtual std::vector<float> getCurrentIndividual() const = 0;
  virtual void addCurrentIndividualFitness(float fitness) = 0;
  virtual float getCurrentIndividualFitness() = 0;
  virtual std::vector<float> getBestIndividual() const = 0;
  virtual bool save(const std::string& filename) = 0;
  virtual bool load(const std::string& filename) = 0;
  inline bool isInitialized() { return initialized; };
  unsigned getGenerationCount() { return generationCounter; };

protected:
  bool initialized = false;
  unsigned generationCounter = 1;
  Configuration configuration;
};
