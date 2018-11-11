/**
 * @file Population.h
 * Population for Evolutionary Robotics.
 * It contains a generation of populations and provides functions to update the generation
 * @author Enno Röhrig
 */
#include "EigenPopulation.h"
#include "Tools/Debugging/Debugging.h"
#include <cmath>

std::vector<unsigned> EigenPopulation::sortedIndices() const
{
  std::vector<unsigned> ret(total);
  for(unsigned i = 0; i < total; i++)
    ret[i] = i;
  std::sort(ret.begin(), ret.end(), [&](unsigned i1, unsigned i2) { return getFitness(i1) > getFitness(i2); });
  return ret;
}

VectorXf EigenPopulation::sort(const std::vector<unsigned>& indices, const VectorXf& vector, const int count)
{
  VectorXf sorted(count == -1 ? vector.size() : count);
  for(int i = 0; i < sorted.size(); i++)
    sorted(i) = vector(static_cast<unsigned>(indices[i]));
  return sorted;
}

MatrixXf EigenPopulation::sort(const std::vector<unsigned>& indices, const MatrixXf& matrix, const int count)
{
  MatrixXf sorted(count == -1 ? matrix.rows() : count, matrix.cols());
  for(int i = 0; i < sorted.rows(); i++)
    sorted.row(i) = matrix.row(static_cast<unsigned>(indices[i]));
  return sorted;
}

std::vector<float> EigenPopulation::getNextIndividual()
{
  if(++curIndividualNr == static_cast<int>(total))
  {
    std::vector<unsigned> indices = sortedIndices();
    std::stringstream ss;
    ss << "Generation: " << generationCounter;
    ss << " | Best: " << getFitness(indices[0]);
    ss << " | Median: " << getFitness(indices[indices.size() / 2]);
    OUTPUT_TEXT(ss.str());
    generationCounter++;
    generationUpdate();
  }
  OUTPUT_TEXT("curIndividualNr: " << std::to_string(curIndividualNr) << "/" << std::to_string(total));
  return getCurrentIndividual();
}

std::vector<float> EigenPopulation::getCurrentIndividual() const
{
  return getIndividual(curIndividualNr);
}

std::vector<float> EigenPopulation::getIndividual(unsigned individualNr) const
{
  VectorXf individual = individuals.row(individualNr);
  ASSERT(!std::isnan(individual.sum()));
  std::vector<float> result;
  for(unsigned i = 0; i < numFeatures; i++)
    result.push_back(configuration.limits[i].min + (std::tanh(individual[i]) + 1.f) / 2.f * (configuration.limits[i].max - configuration.limits[i].min));
  return result;
}

void EigenPopulation::addCurrentIndividualFitness(float fitness)
{
  ASSERT(!std::isnan(fitness));
  this->fitness[curIndividualNr].push_back(fitness);
}

float EigenPopulation::getCurrentIndividualFitness() { return getFitness(curIndividualNr); }

float EigenPopulation::getFitness(unsigned i) const
{
  switch(static_cast<int>(configuration.trialEvaluationMethod))
  {
    case 0: // Median
      return fitness[i][fitness[i].size() / 2];
    case 1: // Mean
      return Eigen::Map<const Eigen::VectorXf>(fitness[i].data(), fitness[i].size()).mean();
    default:
      return NAN;
  }
}
