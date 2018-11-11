/**
 * @file Population.h
 * Population wich implements the MAPElites algorithm from Cully et. al (2015)
 * @author Enno Röhrig
 */

#include <map>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <vector>
#include <iomanip>
#include <ctime>
#include <cmath>

#include "Population.h"
#include "Platform/File.h"
#include "Tools/Math/Random.h"
#include "Tools/Streams/OutStreams.h"
#include "Tools/Streams/InStreams.h"

#pragma once

template<typename Individual>
STREAMABLE(MAPElitesFile,
{,
  (std::vector<std::string>) keys,
  (std::vector<Individual>) individuals,
});

template<typename Individual>
class MAPElitesPopulation : public Population
{
public:
  MAPElitesPopulation<Individual>() {};
  void init(const Configuration& configuration) override { this->configuration = configuration; };

  std::vector<float> getNextIndividual() override;
  std::vector<float> getCurrentIndividual() const override;
  void addCurrentIndividualFitness(float fitness) override;
  float getCurrentIndividualFitness() override;

  void saveIndividual(const std::vector<float>& descriptor);
  void debugOutput();
  std::vector<float> getBestIndividual() const override;
  bool save(const std::string& filename) override;
  bool load(const std::string& filename) override;
  bool include(const std::string& filename);

private:
  std::string generateKey(const std::vector<float>& descriptor);

  size_t total;
  Individual currentIndividual;
  std::map<std::string, Individual> data;
  std::vector<Individual> individualsToTest;
  std::vector<float> rewards;
  bool tryCurrentAgain = false;
  bool triedCurrentAgain = false;
  bool somethingChanged = true;
  unsigned testedSinceLastSave = 0;
};

// Begin Implementation
template<typename Individual> std::vector<float> MAPElitesPopulation<Individual>::getNextIndividual()
{
  if(tryCurrentAgain)
  {
    tryCurrentAgain = false;
    triedCurrentAgain = true;
    return getCurrentIndividual();
  }
  triedCurrentAgain = false;
  testedSinceLastSave++; //not completly correct, because a double checked individual counts for the first trial, but is only saved in the next trial

load_individuals:
  if(individualsToTest.size() > 0)
  {
    currentIndividual = individualsToTest.back();
    individualsToTest.pop_back();
    for(auto it = data.begin(); it != data.end(); it++)
    {
      std::vector<float> data1 = it->second.getData();
      std::vector<float> data2 = currentIndividual.getData();
      bool similar = true;
      for(unsigned i = 0; i < data2.size() && i < data1.size(); i++)
        if(std::abs(data1[i] - data2[i]) > 0.01)
          similar = false;
      if(similar)
        goto load_individuals;
    }
  }
  if(data.size() < configuration.total)
  {
    currentIndividual = Individual(configuration);
  }
  else
  {
    std::vector<float> fitnesses;
    for(auto it = data.begin(); it != data.end(); it++)
      fitnesses.push_back(it->second.getFitness());
    float avgFitness = std::accumulate(fitnesses.begin(), fitnesses.end(), 0.f) / fitnesses.size();
    float varianceFitness = 0.f;
    for(float f : fitnesses)
      varianceFitness += std::pow(f - avgFitness, 2);
    varianceFitness /= fitnesses.size();
    float minFitness = avgFitness - std::sqrt(varianceFitness) * configuration.sigma;

    auto  parent1 = data.begin();
    auto  parent2 = data.begin();
    do
    {
      parent1 = data.begin();
      parent2 = data.begin();
      std::advance(parent1, static_cast<int>(Random::uniform<float>(0.f, static_cast<float>(data.size() - 1))));
      std::advance(parent2, static_cast<int>(Random::uniform<float>(0.f, static_cast<float>(data.size() - 1))));
    }
    while(parent1->second.getFitness() < minFitness || parent2->second.getFitness() < minFitness);

    float found = 0;
    for(float f : fitnesses)
      if(f > minFitness)
        found++;
    OUTPUT_TEXT("Using the best " << found * 100 / fitnesses.size() << " %");

    currentIndividual = parent1->second.crossover(parent2->second);
    //currentIndividual = parent1->second;

    currentIndividual.mutate();
  }
  return currentIndividual.getData();
}
template<typename Individual> std::vector<float> MAPElitesPopulation<Individual>::getCurrentIndividual() const
{
  Individual ret = currentIndividual;
  return ret.getData();
}
template<typename Individual>  void MAPElitesPopulation<Individual>::addCurrentIndividualFitness(float fitness)
{
  currentIndividual.addFitness(fitness);
}
template<typename Individual>  float MAPElitesPopulation<Individual>::getCurrentIndividualFitness()
{
  return currentIndividual.getFitness();
}

template<typename Individual>  void MAPElitesPopulation<Individual>::saveIndividual(const std::vector<float>& descriptor)
{
  std::string key = generateKey(descriptor);
  if(currentIndividual.getFitness() > data[key].getFitness())
  {
    if(data.count(key) == 1 && !triedCurrentAgain)
    {
      tryCurrentAgain = true;
      OUTPUT_TEXT("That was good. Try it again");
    }
    else
    {
      data[key] = currentIndividual;
      somethingChanged = true;
    }
  }
}

template<typename Individual>  std::string MAPElitesPopulation<Individual>::generateKey(const std::vector<float>& descriptor)
{
  std::stringstream s;
  for(float v : descriptor)
    s << "," << std::to_string(int(v));
  return s.str();
}

template<typename Individual> void MAPElitesPopulation<Individual>::debugOutput()
{
  std::vector<float> fitnesses;
  for(auto it = data.begin(); it != data.end(); it++)
    fitnesses.push_back(it->second.getFitness());

  std::sort(fitnesses.rbegin(), fitnesses.rend());
  float avgFitness = std::accumulate(fitnesses.begin(), fitnesses.end(), 0.f) / fitnesses.size();
  std::vector<unsigned> nrOfElements = { 1, 10, 50, 100, 250, 500};
  std::vector<float> maxX;
  for(unsigned elements : nrOfElements)
  {
    if(elements > fitnesses.size())
      elements = static_cast<unsigned>(fitnesses.size());
    maxX.push_back(std::accumulate(fitnesses.begin(), fitnesses.begin() + elements, 0.f) / elements);
  }

  std::fstream log;
  log.open(std::string(File::getBHDir()) + "/Config/Logs/MapElitesLog.csv", std::fstream::app);
  if(log.is_open())
  {
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    log << std::put_time(&tm, "%H:%M") << ",";
    for(float v : maxX)
      log << std::to_string(v) << ",";
    log << std::to_string(avgFitness) << "," << std::to_string(data.size()) << "," << std::to_string(testedSinceLastSave) << "," << std::to_string(individualsToTest.size()) << std::endl;
    log.close();
  }
  testedSinceLastSave = 0;
  OUTPUT_TEXT("Best Fitness: " << std::to_string(maxX[0]));
  OUTPUT_TEXT("Avg Fitness: " << std::to_string(avgFitness));
  OUTPUT_TEXT("Found Descriptors: " << std::to_string(data.size()));
}
template<typename Individual> std::vector<float> MAPElitesPopulation<Individual>::getBestIndividual() const
{
  float maxFitness = 0;
  for(auto it = data.begin(); it != data.end(); it++)
    maxFitness = std::max(maxFitness, it->second.getFitness());
  for(auto it = data.begin(); it != data.end(); it++)
    if(maxFitness == it->second.getFitness())
      return it->second.getData();
  return std::vector<float>();
}
template<typename Individual> bool MAPElitesPopulation<Individual>::save(const std::string& filename)
{
  debugOutput();
  if(!somethingChanged)
    return false;
  somethingChanged = false;

  std::string dir = std::string(File::getBHDir()) + "/" + filename;
  {
    //only save the best one
    /*float maxFitness = 0;
    for(auto it = data.begin(); it != data.end(); it++)
      if(it->second.getFitness() > 20) continue; //get the best realistic one //TODO function: save only the best n individuals
      else  maxFitness = std::max(maxFitness, it->second.getFitness());*/

    OutMapFile stream(dir + ".part"); // to ensure data integrity even if program is terminitad while writing
    if(!stream.exists())
      return false;
    MAPElitesFile<Individual> v;
    for(auto it = data.begin(); it != data.end(); it++)
    {
      //if(it->second.getFitness() < maxFitness) continue;
      v.keys.push_back(it->first);
      v.individuals.push_back(it->second);
    }
    stream << v;
  }
  std::ifstream f(dir.c_str());
  bool fileExists = f.good();
  f.close();

  if(fileExists)
    std::remove(dir.c_str());
  std::rename(std::string(dir + ".part").c_str(), dir.c_str());
  return true;
}
template<typename Individual> bool MAPElitesPopulation<Individual>::load(const std::string& filename)
{
  std::string dir = std::string(File::getBHDir()) + "/" + filename;
  std::ifstream f(dir);
  if(!f.good())
    dir = dir + ".part";
  f.close();

  InMapFile stream(dir);
  if(!stream.exists())
    return false;
  MAPElitesFile<Individual> file;
  stream >> file;
  for(unsigned i = 0; i < file.keys.size(); i++)
  {
    Individual ind = file.individuals[i];
    ind.setConfiguration(configuration);
    data[file.keys[i]] = ind;
  }
  somethingChanged = false;
  return true;
}
template<typename Individual> bool MAPElitesPopulation<Individual>::include(const std::string& filename)
{
  InMapFile stream(std::string(File::getBHDir()) + "/" + filename);
  if(!stream.exists())
    return false;
  MAPElitesFile<Individual> file;
  stream >> file;
  for(unsigned i = 0; i < file.keys.size(); i++)
  {
    Individual individual = file.individuals[i];
    individual.resetFitness();
    individualsToTest.push_back(individual);
  }
  return true;
}
