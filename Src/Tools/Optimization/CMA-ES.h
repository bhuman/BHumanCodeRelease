/**
 * @file CMA_ES.h
 * Population for CMA-ES
 * It contains a generation of populations and provides functions to update the generation
 * @author Not Enno Röhrig := Bernd Popinga
 */
#include "Tools/Optimization/EigenPopulation.h"

#pragma once

class CMA_ES_Population : public EigenPopulation
{
public:
  void init(const Configuration& configuration) override;
  bool save(const std::string& path) override;
  bool load(const std::string& path) override;
  std::vector<float> getBestIndividual() const override { return getIndividual(0); }

private:

  MatrixXf arz;
  MatrixXf arx;

  MatrixXf pc;
  MatrixXf ps;
  MatrixXf B;
  MatrixXf D;
  MatrixXf C;

  VectorXf xmean;
  VectorXf weights;
  float sigma;

  unsigned n;
  float stopfitness;
  float stopeval;
  unsigned lambda_; // = total;
  float mu;
  float mueff;
  float cc;
  float cs;
  float c1;
  float cmu;
  std::vector<float>  xmean_;
  float damps;
  int eigenval; // = 0;
  unsigned counteval;
  float chiN;
  //MatrixXf arz;
  //MatrixXf& arx = individuals;
  //unsigned& counteval = generationCounter;
  //unsigned& n = numFeatures;
  bool somethingChanged; //generation update after the last call of save
  void updateIndividuals();
  void generationUpdate() override;
  void debugOutput();
};
