/**
 * @file CMA_ES.h
 * Population for CMA-ES
 * It contains a generation of populations and provides functions to update the generation
 * @author Not Enno Röhrig := Bernd Popinga
 */
#include "Tools/Optimization/EigenPopulation.h"

#pragma once

class External_Population : public Population
{
public:
  void init(const Configuration& configuration) override;
  bool save(const std::string& path) override;
  bool load(const std::string& path) override;
  std::vector<float> getNextIndividual() override;
  std::vector<float> getCurrentIndividual() const override;
  void addCurrentIndividualFitness(float fitness) override;
  float getCurrentIndividualFitness() override; // const;
  std::vector<float> getBestIndividual() const override { return std::vector<float>(); } // TODO implement
private:
  std::vector<float> fitness;
  std::vector<float> currentIndividual;
  unsigned numFeatures;
  void updateIndividual();
  void generationUpdate();
  bool findFile(unsigned i, std::string name);
};
