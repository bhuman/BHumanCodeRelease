/**
 * @file Population.h
 * Population for Evolutionary Robotics.
 * It contains a generation of populations and provides functions to update the generation
 * @author Enno Röhrig
 */

#include "Tools/Math/Eigen.h"
#include "Population.h"

#pragma once

class EigenPopulation : public Population
{
public:
  //void init(const Configuration& configuration) = 0;
  std::vector<float> getNextIndividual() override;
  std::vector<float> getCurrentIndividual() const override;
  void addCurrentIndividualFitness(float fitness) override;
  float getCurrentIndividualFitness() override;
protected:
  std::vector<float> getIndividual(unsigned inidivualNr) const;
  virtual void generationUpdate() = 0;
  std::vector<unsigned> sortedIndices() const;
  VectorXf sort(const std::vector<unsigned>& indices, const VectorXf& vector, const int count = -1);
  MatrixXf sort(const std::vector<unsigned>& indices, const MatrixXf& matrix, const int count = -1);
  float getFitness(unsigned i)const;
  unsigned total;
  int curIndividualNr = -1;
  unsigned numFeatures;
  MatrixXf individuals;
  float bestIndividualFitness;
  std::vector<std::vector<float> > fitness;
  Configuration configuration;
};
