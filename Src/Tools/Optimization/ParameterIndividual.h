/**
 * @file ParameterIndividual.h
 * Simple individual for Evolutionary Robotics.
 * A Vector of float which uses ParameterLearner for saving and loading
 * @author Enno Röhrig
 */

#pragma once

#include <algorithm>
#include <vector>
#include <cmath>
#include <time.h>

#include "Tools/Debugging/Debugging.h"
#include "Tools/Streams/AutoStreamable.h"

template<typename ParameterType>
STREAMABLE(StreamData,
{,
  (std::vector<float>) fitnessValues,
  (ParameterType) params,
});
template<typename ParameterLearner, typename ParameterType>
class ParameterIndividual: public Streamable
{
public:
  ParameterIndividual() {};
  ParameterIndividual(const Population::Configuration& conf);
  void setConfiguration(const Population::Configuration& conf);

  void mutate(float temperature = 0.1f);
  ParameterIndividual crossover(const ParameterIndividual& other) ;
  std::vector<float> getData() const;
  float getFitness() const;
  void addFitness(float fitness);
  void resetFitness();
  void serialize(In* in, Out* out) override;

  std::vector<float> data;
  std::vector<float> fitnessValues;

  static bool compare(const ParameterIndividual& i1, const ParameterIndividual& i2)
  {
    return i1.getFitness() > i2.getFitness();
  };
private:
  Population::Configuration conf;
};
template<typename ParameterLearner, typename ParameterType>
void ParameterIndividual<ParameterLearner, ParameterType>::setConfiguration(const Population::Configuration& conf)
{
  this->conf = conf;
}

template<typename ParameterLearner, typename ParameterType>
void ParameterIndividual<ParameterLearner, ParameterType>::serialize(In* in, Out* out)
{
  StreamData<ParameterType> sD;
  if(in != nullptr)
  {
    *in >> sD;
    fitnessValues = sD.fitnessValues;
    data = ParameterLearner::parameterToVector(sD.params);
  }
  if(out != nullptr)
  {
    sD.fitnessValues = fitnessValues;
    sD.params = ParameterLearner::vectorToParameter(data);
    *out << sD;
  }
}
template<typename ParameterLearner, typename ParameterType>
void ParameterIndividual<ParameterLearner, ParameterType>::addFitness(float fitness)
{
  fitnessValues.push_back(fitness);
}
template<typename ParameterLearner, typename ParameterType>
void ParameterIndividual<ParameterLearner, ParameterType>::resetFitness()
{
  fitnessValues.clear();
}
template<typename ParameterLearner, typename ParameterType>
float ParameterIndividual<ParameterLearner, ParameterType>::getFitness() const
{
  if(fitnessValues.size() == 0)
    return -999999999.f;  //to ensure that fitness of a tested individual is higher than from a dummy individual
  float sum = std::accumulate(fitnessValues.begin(), fitnessValues.end(), 0.f);
  return sum / fitnessValues.size();
}

template<typename ParameterLearner, typename ParameterType>
void ParameterIndividual<ParameterLearner, ParameterType>::mutate(float temperature)
{
  for(int i = 0; i < conf.numFeatures; i++)
  {
    std::random_device rd{};
    std::mt19937 gen{ rd() };
    std::normal_distribution<> d{ 0, temperature};
    float rand = static_cast<float>(d(gen));
    data[i] += (conf.limits[i].max - conf.limits[i].min) * rand;
    data[i] = conf.limits[i].limit(data[i]);
  }
}

template<typename ParameterLearner, typename ParameterType>
ParameterIndividual<ParameterLearner, ParameterType> ParameterIndividual<ParameterLearner, ParameterType>::crossover(const ParameterIndividual& other)
{
  srand(static_cast<unsigned>(time(NULL)));
  ParameterIndividual child(conf);
  for(int i = 0; i < conf.numFeatures; i++)
    if(((float)rand()) > 0.5)
      child.data[i] = other.data[i];
    else
      child.data[i] = data[i];
  return child;
}

template<typename ParameterLearner, typename ParameterType>
ParameterIndividual<ParameterLearner, ParameterType>::ParameterIndividual(const Population::Configuration& conf): conf(conf)
{
  srand(static_cast<unsigned>(time(NULL)));
  for(int i = 0; i < conf.numFeatures; i++)
  {
    float random = (float)rand() / (float)RAND_MAX;
    data.push_back(conf.limits[i].min + (conf.limits[i].max - conf.limits[i].min)*random);
  }
}

template<typename ParameterLearner, typename ParameterType>
std::vector<float> ParameterIndividual<ParameterLearner, ParameterType>::getData() const
{
  return std::vector<float>(std::begin(data), std::end(data));
}
