/**
 * @file CMA_ES.cpp
 * Population for CMA-ES
 * It contains a generation of populations and provides functions to update the generation
 * @author Bert Poppinga
 */

#include "CMA-ES.h"
#include <fstream>
#include <random>
#include <cmath>
#include "Platform/File.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Streams/OutStreams.h"
#include "Tools/Debugging/Debugging.h"
#include <ctime>
#include <iomanip>

void CMA_ES_Population::updateIndividuals()
{
  arz = MatrixXf(lambda_, n);
  arx = MatrixXf(lambda_, n);

  arx.row(0) = xmean;
  arz.row(0) = 0.f * xmean;
  for(int i = 1; i < arz.rows(); i++)
  {
    for(int j = 0; j < arz.cols(); j++)
    {
      std::random_device rd{};
      std::mt19937 gen{ rd() };
      std::normal_distribution<> d{ 0, 1 };
      arz(i, j) = static_cast<float>(d(gen));
    }
    arx.row(i) = xmean + sigma * (B* D* arz.block(i, 0, 1, n).transpose()).col(0);
    ASSERT(!std::isnan(arx.row(i).sum()));
  }
  // clear fitness and copy individuals
  for(unsigned i = 0; i < total; i++)
    fitness[i].clear();

  individuals = arx;
}

void CMA_ES_Population::init(const Configuration& configuration)
{
  this->configuration = configuration;

  n = static_cast<unsigned>(configuration.initialization.size());
  sigma = configuration.sigma;
  stopfitness = configuration.stopFitness;
  stopeval = 1e3f * std::pow(static_cast<float>(n), 2.f);
  lambda_ = 4 + static_cast<unsigned>(floorf(3.f * std::log(static_cast<float>(n))));
  mu = lambda_ / 2.f;
  weights = VectorXf(static_cast<unsigned>(mu));
  for(int i = 1; i <= weights.size(); i++)
    weights[i - 1] = std::log(mu + 0.5f) - std::log(static_cast<float>(i));
  mu = floorf(mu);
  weights /= weights.sum();
  mueff = std::pow(weights.sum(), 2.f) / weights.array().pow(2).sum();
  cc = (4 + mueff / static_cast<float>(n)) / (static_cast<float>(n) + 4.f + 2.f * mueff / static_cast<float>(n));
  cs = (mueff + 2.f) / (static_cast<float>(n) + mueff + 5.f);
  c1 = 2.f / (std::pow(static_cast<float>(n) + 1.3f, 2.f) + mueff);
  cmu = 2.f * (mueff - 2.f + 1.f / mueff) / (std::pow(static_cast<float>(n) + 1.f, 2.f) + 2.f * mueff / 2.f);
  damps = 1.f + 2.f * std::max(0.f, std::sqrt((mueff - 1.f) / (static_cast<float>(n) + 1.f)) - 1.f) + cs;
  pc = MatrixXf::Zero(n, 1);
  ps = MatrixXf::Zero(n, 1);
  B = D = MatrixXf::Identity(n, n);
  C = B* D* (B* D).transpose();
  chiN = std::pow(static_cast<float>(n), 0.5f) * (1.f - 1.f / (4.f * static_cast<float>(n)) + 1.f / (21.f * std::pow(static_cast<float>(n), 2.f)));

  xmean = Eigen::Map<const Eigen::VectorXf>(configuration.initialization.data(), n);
  for(unsigned i = 0; i < n; i++)
  {
    xmean[i] = configuration.limits[i].limit(xmean[i]);
    xmean[i] = std::atanh((xmean[i] - configuration.limits[i].min) / (configuration.limits[i].max - configuration.limits[i].min) * 2.f - 1.f);
    xmean[i] = Rangef(-3.f, 3.f).limit(xmean[i]);
  }

  total = lambda_;
  numFeatures = n;
  fitness.resize(total);
  updateIndividuals();
  initialized = true;
  somethingChanged = true;
}

void CMA_ES_Population::generationUpdate()
{
  std::vector<unsigned> indices = sortedIndices();
  arx = sort(indices, arx, static_cast<unsigned>(mu));
  arz = sort(indices, arz, static_cast<unsigned>(mu));

  xmean = arx.transpose() * weights;
  VectorXf zmean = arz.transpose() * weights;

  ps = (1.f - cs) * ps + static_cast<float>(sqrt(cs * (2.f - cs) * mueff)) * (B*      zmean);
  pc = (1.f - cc) * pc + static_cast<float>(sqrt(cc * (2.f - cc) * mueff)) * (B* D* zmean);

  float hsig = ps.norm() / sqrt(1 - pow(1 - cs, 2 * counteval / lambda_)) / chiN < 1.4 + 2.f / (n + 1);

  C = (1 - c1 - cmu) * C
      + c1 * (pc * pc.transpose() + (1 - hsig) * cc * (2 - cc) * C)
      + cmu * (B* D* arz.transpose()) * weights.asDiagonal() * (B* D* arz.transpose()).transpose();
  //OUTPUT_WARNING("" << C);

  sigma *= exp((cs / damps) * (ps.norm() / chiN - 1));
  ASSERT(!std::isnan(sigma));
  if(counteval - eigenval > lambda_ / (c1 / cmu) / n / 10)
  {
    eigenval = counteval;
    //C.triangularView<Eigen::Lower>() = C.triangularView<Eigen::Upper>().transpose();
    Eigen::SelfAdjointEigenSolver<MatrixXf> es(C);
    //OUTPUT_WARNING("Success" << es.info());
    B = es.eigenvectors();
    D = sqrt(es.eigenvalues().array()).matrix().asDiagonal();
  }

  curIndividualNr = 0;
  configuration.initialization = getBestIndividual();
  //save("nn_balancer_gen_" + std::to_string(generationCounter) + "_fit_" + std::to_string(static_cast<int>(getFitness(indices[0]))) + ".evo");
  updateIndividuals();
  somethingChanged = true;
}

bool CMA_ES_Population::save(const std::string& path)
{
  if(!somethingChanged) // nothing to save
    return false;
  somethingChanged = false;
  debugOutput();

  std::string dir = std::string(File::getBHDir()) + "/" + path;
  {
    OutMapFile stream(dir + ".part"); //TODO save and load covariance
    if(!stream.exists())
      return false;
    stream << configuration;
  }
  std::ifstream f(dir);
  bool fileExists = f.good();
  f.close();

  if(fileExists)
    std::remove(dir.c_str());
  std::rename(std::string(dir + ".part").c_str(), dir.c_str());
  return true;
}
void CMA_ES_Population::debugOutput()
{
  std::fstream log;
  log.open(std::string(File::getBHDir()) + "/Config/Logs/CMA-ES.csv", std::fstream::app);
  if(log.is_open())
  {
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    log << std::put_time(&tm, "%H:%M") << ",";
    log << std::to_string(getFitness(0)) << ",";
    for(float v : configuration.initialization)
      log << v << ",";
    log << "begin_covariance,";
    for(int i = 0; i < C.rows(); i++)
      log << C(i, i) << ",";
    log << std::endl;
    log.close();
  }
}
bool CMA_ES_Population::load(const std::string& path)
{
  std::string dir = std::string(File::getBHDir()) + "/" + path;
  std::ifstream f(dir);
  if(!f.good())
    dir = dir + ".part";
  f.close();

  InMapFile stream(dir);
  if(!stream.exists())
    return false;
  stream >> configuration;
  init(configuration);
  //somethingChanged = false; TODO: reactivate
  return true;
}
