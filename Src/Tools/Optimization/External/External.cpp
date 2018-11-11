/**
 * @file Extern.cpp
 * Population for external learning
 * It contains a generation of populations and provides functions to update the generation
 * @author Not Enno Röhrig
 */

#include "External.h"
#include <random>
#include <cmath>
#include "Tools/Streams/InStreams.h"
#include "Tools/Streams/OutStreams.h"
#include "Tools/Debugging/Debugging.h"
#include "Platform/File.h"
#include <dirent.h>
#include <fstream>

bool External_Population::findFile(unsigned i, std::string name)
{
  std::string dirname = std::string(File::getBHDir()) + configuration.path;
  struct dirent* file;
  DIR* dir = opendir(dirname.c_str());
  ASSERT(dir);
  for(file = readdir(dir); file != nullptr && file->d_name != name + "_" + std::to_string(i) + ".dat"; file = readdir(dir));
  closedir(dir);
  if(file != nullptr && file->d_name == name + "_" + std::to_string(generationCounter) + ".dat")
    return true;
  return false;
}

void External_Population::updateIndividual()
{
  while(!findFile(generationCounter, "parameters"))
  {
    //OUTPUT_TEXT("Waiting for File");
    //std::this_thread::sleep_for(std::chrono::seconds(0.02));
  }

  std::string inputFileName = std::string(File::getBHDir()) + configuration.path + "/" + "parameters_" + std::to_string(generationCounter) + ".dat";
  std::ifstream inputFile(inputFileName);
  int y = 0;
  currentIndividual.clear();
  fitness.clear();
  while(inputFile)
  {
    std::string s;
    if(!getline(inputFile, s)) break;
    if(s[0] != '#')
    {
      std::istringstream ss(s);
      int x = 0;
      while(ss)
      {
        std::string line;
        if(!getline(ss, line, ','))
          break;
        try
        {
          currentIndividual.push_back(stof(line));
          x++;
          //OUTPUT_TEXT("" << (stof(line)));
        }
        catch(const std::invalid_argument e)
        {
          OUTPUT_TEXT("NaN found in file " << inputFileName << " line " << y << "\n");
          e.what();
        }
      }
      y++;
    }
  }

  if(!inputFile.eof())
  {
    OUTPUT_ERROR("Could not read file " << inputFileName << "\n");
  }
  inputFile.close();
}
void External_Population::init(const Configuration& configuration)
{
  this->configuration = configuration;
  initialized = true;
  numFeatures = (int)configuration.numFeatures;

  for(; findFile(generationCounter, "parameters") && findFile(generationCounter, "fitness"); generationCounter++);
  if(generationCounter == 1)
  {
    std::string dirname = std::string(File::getBHDir()) + configuration.path;
    std::string inputFileName = dirname + "/parameters_0.dat";
    std::ofstream outputFile(inputFileName);
    for(unsigned i = 0; i < configuration.initialization.size(); i++)
    {
      outputFile << configuration.initialization[i];
      if(i != configuration.initialization.size() - 1)
        outputFile << ", ";
    }
    outputFile.close();
  }
}

void External_Population::generationUpdate() { /* Nothing to do here. Bolero does everything. */ }

bool External_Population::save(const std::string& path)
{
  OutMapFile stream(path.c_str());
  ASSERT(stream.exists());
  stream << configuration;
  return true;
}
bool External_Population::load(const std::string& path)
{
  InMapFile stream(path.c_str());
  ASSERT(stream.exists());
  stream >> configuration;
  init(configuration);
  return true;
}

std::vector<float> External_Population::getNextIndividual()
{
  if(!currentIndividual.empty())
  {
    std::string dirname = std::string(File::getBHDir()) + configuration.path;
    std::string inputFileName = dirname + "/fitness_" + std::to_string(generationCounter) + ".dat";
    std::ofstream outputFile(inputFileName);
    //for(float f : fitness)
    outputFile << getCurrentIndividualFitness() << std::endl;
    outputFile.close();
    generationCounter++;
    OUTPUT_TEXT("Fitness: " << getCurrentIndividualFitness());
  }

  updateIndividual();

  return getCurrentIndividual();
}

std::vector<float> External_Population::getCurrentIndividual() const { return currentIndividual; }

void External_Population::addCurrentIndividualFitness(float fitness) { this->fitness.push_back(fitness); }

float External_Population::getCurrentIndividualFitness() //const
{
  std::vector<float> fitnessCopy = fitness;
  std::sort(fitnessCopy.begin(), fitnessCopy.end());

  switch(static_cast<int>(configuration.trialEvaluationMethod))
  {
    case 0: // Median
      return fitnessCopy[fitnessCopy.size() / 2];
    default:
      return NAN;
  }
}
