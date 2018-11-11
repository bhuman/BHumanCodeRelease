/**
 * @file GeneticPopulation.h
 * Simple Genetic Population which uses mutation, crossover and a simple selection
 * @author Enno Röhrig
 */
#include <sstream>
#include "Tools/Optimization/Population.h"
#pragma once

template<typename I>
class GeneticPopulation : public Population
{
public:
  GeneticPopulation<I>() {};

  void init(const Configuration& configuration) override
  {
    conf = configuration;
  };
  std::vector<float> getNextIndividual() override;
  std::vector<float> getCurrentIndividual() const override;
  void addCurrentIndividualFitness(float fitness) override;
  float getCurrentIndividualFitness() override;

  bool save(const std::string& path) override { return false; };
  bool load(const std::string& path) override { return false;  };

private:
  std::vector<I> data;
  size_t total = 10;
  size_t curIndividualNr = -1;
  std::vector<float> rewards;
  Population::Configuration conf;
  I getWeightedRandomIndividual();
  void generationUpdate();
};

// Begin Implementation
template<typename I> std::vector<float> GeneticPopulation<I>::getNextIndividual()
{
  if(curIndividualNr != -1)
    data.at(curIndividualNr).fitness = getCurrentIndividualFitness();
  if(data.size() < total)
  {
    I newIndividual(conf);
    data.emplace_back(newIndividual);
    curIndividualNr = data.size() - 1;
  }
  else if(curIndividualNr == data.size() - 1)
  {
    generationUpdate();
    curIndividualNr = total / 2;
  }
  else
    curIndividualNr++;
  return data.at(curIndividualNr).toVector();
}
template<typename I>std::vector<float> GeneticPopulation<I>::getCurrentIndividual() const
{
  I ret = data.at(curIndividualNr); //copy individaul because the method is const
  return ret.toVector();
}
template<typename I> void GeneticPopulation<I>::addCurrentIndividualFitness(float reward)
{
  rewards.emplace_back(reward);
}
template<typename I> float GeneticPopulation<I>::getCurrentIndividualFitness()
{
  std::sort(rewards.begin(), rewards.end());
  return rewards[(rewards.size() - 1) / 2];
}
template<typename I> void GeneticPopulation<I>::generationUpdate()
{
  std::stringstream ss;
  ss << "Generation: " << generationCounter << std::endl;
  generationCounter++;
  std::sort(data.begin(), data.end(), I::compare);
  data.erase(data.end() - total / 2, data.end());
  for(I& i : data)
  {
    ss << i.fitness << std::endl;;
  }
  for(size_t i = data.size(); i < total; i++)
  {
    I parent1 = getWeightedRandomIndividual();
    I parent2 = getWeightedRandomIndividual();
    I child = parent1.crossover(parent2);
    child.mutate();
    data.push_back(child);
  }
  OUTPUT_TEXT(ss.str());
}

template<typename I> I GeneticPopulation<I>::getWeightedRandomIndividual()
{
  float fitness_sum = 0;
  for(const I& i : data)
    fitness_sum += i.fitness;

  float randomValue = (((float)rand()) / (float)RAND_MAX) * fitness_sum;

  fitness_sum = 0;
  for(const I& i : data)
  {
    fitness_sum += i.fitness;
    if(fitness_sum >= randomValue)
      return i;
  }
  assert(false); // No individual
  return data.front();
}
