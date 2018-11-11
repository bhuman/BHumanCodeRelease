/**
 * @file ParemterLearner.h
 *
 * Abstract template class for general parameter learning in the b-human context
 *
 * @author <A href="mailto:enno.roehrig@gmx.de">Enno Röhrig</A>
 * @author <A href="mailto:poppinga@uni-bremen.de">Bernd Poppinga</A>
 *
 */

#pragma once

#include <chrono>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <regex>
#include "Platform/File.h"

#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Optimization/MachineLearningOutput.h"
#include "Tools/Debugging/DebugDataStreamer.h"
#include "Tools/MessageQueue/InMessage.h"
#include "Tools/MessageQueue/MessageQueue.h"
#include "Tools/Module/Module.h"
#include "Tools/Optimization/ParameterLearner.h"
#include "Tools/Optimization/Population.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/EnumIndexedArray.h"
#include "Tools/Streams/OutStreams.h"
#include "Tools/Streams/TypeInfo.h"

/* TODO
   - Use parameter from module instead of cfg. file
 */

using typed = std::pair<int, std::string>;

template<typename ParameterType>
class ParameterLearner
{
public:
  static constexpr int BUFFER_SIZE = 2000;
  ParameterLearner() {  queue.setSize(BUFFER_SIZE); };

  void handleState(MachineLearningOutput& output, const GameInfo& theGameInfo);
  Population::Configuration readConfiguration(const std::string& c_name);
  static ParameterType vectorToParameter(const std::vector<float>& vector);
  static std::vector<float> parameterToVector(const ParameterType& param);
  void setParameter(const std::vector<float>& parameter);
  void saveParameter(const std::vector<float>& parameter);

protected:
  virtual void nextIndividual(MachineLearningOutput& output) = 0;
  virtual void endIndividual(MachineLearningOutput& output) = 0;
  virtual void startEpisode(MachineLearningOutput& output) = 0;
  virtual void running(MachineLearningOutput& output) = 0;
  virtual void episodeEnded(MachineLearningOutput& output) = 0;

  int trailCounter = 0;
  int frameCounter = 0;
  int trialsPerEpisode = 1;
  static ParameterType stdParameter;

private:
  static char* streamToMemory(const ParameterType& param);
  void addParameter(const std::string& parameter,  Population::Configuration& conf, const std::string& min, const std::string& max, const std::string log = "False");

  static std::vector<Rangef> limits;
  static std::vector<bool> logarithmic;
  int lastGameState = -1;
  static MessageQueue queue;
  static std::vector<typed> parameterPositions;
  static std::string typeName;
  TypeInfo typeInfo;
  std::vector<typed> fillParameterStructure(const std::string& className, int& offset, char* example, std::string key = "");
  std::unordered_map<std::string, std::vector<typed>> parameterStructure;
};
template<typename ParameterType>
ParameterType ParameterLearner<ParameterType>::stdParameter;

template<typename ParameterType>
std::vector<typed> ParameterLearner<ParameterType>::parameterPositions;

template<typename ParameterType>
std::vector<bool> ParameterLearner<ParameterType>::logarithmic;

template<typename ParameterType>
std::vector<Rangef> ParameterLearner<ParameterType>::limits;

template<typename ParameterType>
MessageQueue ParameterLearner<ParameterType>::queue;

template<typename ParameterType>
std::string ParameterLearner<ParameterType>::typeName;

template<typename ParameterType>
char* ParameterLearner<ParameterType>::streamToMemory(const ParameterType& param)
{
  queue.clear();
  queue.out.bin << param; //remove headers
  return queue.getStreamedData() + 12; // 12 because of additional stuff in a streamable
}

template<typename ParameterType>
void ParameterLearner<ParameterType>::handleState(MachineLearningOutput& output, const GameInfo& theGameInfo)
{
  if(lastGameState == -1)     // first call
  {
    nextIndividual(output);
    startEpisode(output);
  }

  if(theGameInfo.state == STATE_PLAYING)
  {
    frameCounter++;
    running(output);
  }
  else if(lastGameState == STATE_PLAYING)    //end episode
  {
    trailCounter++;

    episodeEnded(output);
    if(trailCounter >= trialsPerEpisode)
    {
      trailCounter = 0;
      endIndividual(output);
      nextIndividual(output);
    }
    startEpisode(output);

    frameCounter = 0;
  }
  lastGameState = theGameInfo.state;
}

template<typename ParameterType>
Population::Configuration ParameterLearner<ParameterType>::readConfiguration(const std::string& conf_name)
{
  parameterPositions.clear();
  logarithmic.clear();
  limits.clear();

  // Read Configuration for Population
  Population::Configuration conf;
  InMapFile stream(std::string(File::getBHDir()) + "/Config/MachineLearning/" + conf_name + ".ev", false);
  if(stream.exists())
    stream >> conf;
  else
    OUTPUT_WARNING("Configuration File " << conf_name << " not found");
  trialsPerEpisode = conf.trials;
  // Read Configuration of training subject
  typeName = TypeRegistry::demangle(typeid(stdParameter).name());
  loadModuleParameters(stdParameter, typeName.substr(0, typeName.size() - 14).c_str(), nullptr);

  //analyse parameter type
  int offset = 0;
  fillParameterStructure(typeName, offset, streamToMemory(stdParameter));

  // Find Configuration of Child class
  std::string name = std::string(typeid(*this).name());
#ifdef TARGET_ROBOT
  name = name.substr(2, name.size() - 1) + ".cfg"; // "15" TODO check this for other datatypes
#else
  name = name.substr(6, name.size() - 1) + ".cfg"; // "class "
#endif // TARGET_ROBOT
  name[0] = static_cast<char>(tolower(name[0]));
  if(name.size() > 1 && isupper(name[1]))
    for(int i = 1; i + 1 < static_cast<int>(name.size()) && isupper(name[i + 1]); ++i)
      name[i] = static_cast<char>(tolower(name[i]));
  std::ifstream file;
  for(auto& path : File::getFullNames(name))
  {
    file = std::ifstream(path);
    if(!file.fail())
      break;
  }
  if(file.fail())
    OUTPUT_WARNING("Configuration File " << name << " missing");

  //Read Configuration of Child class
  std::string line, cfg;
  while(std::getline(file, line))
    cfg += line;
  file.close();
  cfg.erase(std::remove(cfg.begin(), cfg.end(), ' '), cfg.end());

  //Analyse Configuration of Child class
  std::smatch match;
  if(!std::regex_search(cfg, match, std::regex("parameters=\\[([^\\]]*)\\]")))
  {
    OUTPUT_WARNING("No configuration of the parameter, which should be learned is found. Please define it in " << name << " with one of the following styles: parameters=[p1,p2,p3], parameters= [{p1.sub1 = {min=0;max=1;log=1;};},{p2 = {min=-1;max=2;};}} or parameters=[all]");
    return conf;
  }
  std::string cfgParameter = std::string(match[1]);

  auto paramsToRead = std::count(cfgParameter.begin(), cfgParameter.end(), '{') / 2;
  if(paramsToRead * 2 != std::count(cfgParameter.begin(), cfgParameter.end(), '}'))
    OUTPUT_WARNING("ParemeterLearner: Cant read parameter in" << name << " because the numberr of closed and open brackets doesnt match");

  //min, max, log defined per parameter
  for(std::string parameter = cfgParameter; std::regex_search(parameter, match, std::regex("\\{([^=]+)=\\{min=([^;]+);max=([^;]+);log=([^;]+);\\}")); parameter = match.suffix(), paramsToRead -= 1)
    addParameter(match[1], conf, match[2], match[3], match[4]);

  //min, max defined per parameter
  for(std::string parameter = cfgParameter; std::regex_search(parameter, match, std::regex("\\{([^=]+)=\\{min=([^;]+);max=([^;]+);\\}")); parameter = match.suffix(), paramsToRead -= 1)
    addParameter(match[1], conf, match[2], match[3]);

  if(paramsToRead != 0)
    OUTPUT_WARNING("One or more parameter in " << name << " could not be read.");

  //only top-level parameter
  if(parameterPositions.size() == 0)
    for(std::string parameter = cfgParameter; std::regex_search(parameter, match, std::regex("([^,;\\[\\]]+)")); parameter = match.suffix())
      addParameter(match[1], conf, std::to_string(std::numeric_limits<float>::min()), std::to_string(std::numeric_limits<float>::max()));

  conf.numFeatures = static_cast<int>(parameterPositions.size());
  conf.initialization = parameterToVector(stdParameter);
  return conf;
}
template<typename ParameterType>
void ParameterLearner<ParameterType>::addParameter(const std::string& parameter, Population::Configuration& conf, const std::string& min, const std::string& max, const std::string log)
{
  if(parameterStructure.find(parameter) == parameterStructure.end())
  {
    OUTPUT_WARNING("ParameterLearner: " << parameter << " not found in " << typeName);
    return;
  }

  float minf = strtof(min.c_str(), 0);
  float maxf = strtof(max.c_str(), 0);
  bool logb = log == "true" || log == "True";
  for(auto p : parameterStructure.find(parameter)->second)
  {
    conf.limits.push_back(Rangef(0.f, 1.f));
    limits.push_back(Rangef(minf, maxf));
    logarithmic.push_back(logb ? 1 : 0);
    parameterPositions.push_back(p);
  }
}

template<typename ParameterType>
std::vector<typed> ParameterLearner<ParameterType>::fillParameterStructure(const std::string& className, int& offset, char* example, std::string key)
{
  std::vector<typed> positions;
  if(typeInfo.primitives.find(className) != typeInfo.primitives.end())
  {
    positions.push_back(typed(offset, className));
    if(className == "char")
      offset += sizeof(char);
    else if(className == "signed char")
      offset += sizeof(signed char);
    else if(className == "unsigned char")
      offset += sizeof(unsigned char);
    else if(className == "short")
      offset += sizeof(short);
    else if(className == "unsigned short")
      offset += sizeof(unsigned short);
    else if(className == "int")
      offset += sizeof(int);
    else if(className == "unsigned" || className == "unsigned int")
      offset += sizeof(unsigned int);
    else if(className == "float")
      offset += sizeof(float);
    else if(className == "double")
      offset += sizeof(double);
    else if(className == "bool")
      offset += sizeof(bool);
    else if(className == "Angle")
      offset += sizeof(Angle);
    else if(className.find("string") != std::string::npos)
      offset += sizeof(char);
    //TODO handle strings: find out size
    else
      FAIL(key << " is not a streamable type!");
  }
  else if(typeInfo.classes.find(className) != typeInfo.classes.end())
    for(TypeInfo::Attribute& attr : typeInfo.classes.find(className)->second)
    {
      std::string nextKey = key == "" ? attr.name : key + "." + attr.name;
      for(auto pos : fillParameterStructure(attr.type, offset, example, nextKey))
        positions.push_back(pos);
    }
  else if(className == "float*") //std::vector<float>
  {
    unsigned size = *((unsigned*)(example + offset));
    offset += sizeof(size);
    for(unsigned i = 0; i < size; i++)
    {
      positions.push_back(typed(offset, "float"));
      offset += sizeof(float);
    }
  }
  else
    FAIL(className << " in " << key << " is not a known type! Or it is an enum type, which is not handled yet in ParameterLearner.cpp");
  if(key == "")
    key = "all";
  parameterStructure.emplace(key, positions);
  if(offset > BUFFER_SIZE)
    OUTPUT_WARNING("Parameter Learner: Buffer Size in Parameter Learner to small, please increase!");
  return positions;
}

template<typename ParameterType>
ParameterType ParameterLearner<ParameterType>::vectorToParameter(const std::vector<float>& vector)
{
  ParameterType res;
  if(parameterPositions.size() == 0)
  {
    OUTPUT_WARNING("ParameterLearner: No Configuration for parameter type " << typeName << ". Call readConfiguration(string name) before using vectorToParameter()");
    return res;
  }
  if(parameterPositions.size() != vector.size())
  {
    OUTPUT_WARNING("ParameterLearner: Vector cannot be converted to " << typeName << ", because it has the wrong size, which should be " << std::to_string(parameterPositions.size()));
    return res;
  }
  char* mem_ptr =  streamToMemory(stdParameter);
  for(unsigned i = 0; i < parameterPositions.size(); i++)
  {
    float value = vector[i];
    if(logarithmic[i])
      value = std::pow(2.f, value) - 1.f;
    value = limits[i].min + value * (limits[i].max - limits[i].min);
    typed type = parameterPositions[i];
    void* ptr = mem_ptr + type.first;

    if(type.second == "char")
      *(char*)ptr = (char)(value);
    else if(type.second == "signed char")
      *(signed char*)ptr = (signed char)(value);
    else if(type.second == "unsigned char")
      *(unsigned char*)ptr = (unsigned char)(value);
    else if(type.second == "short")
      *(short*)ptr = (short)(value);
    else if(type.second == "unsigned short")
      *(unsigned short*)ptr = (unsigned short)(value);
    else if(type.second == "int")
      *(int*)ptr = (int)(value);
    else if(type.second == "unsigned" || type.second == "unsigned int")
      *(unsigned*)ptr = (unsigned)(value);
    else if(type.second == "float")
      *(float*)ptr = (float)(value);
    else if(type.second == "double")
      *(double*)ptr = (double)(value);
    else if(type.second == "bool")
      *(bool*)ptr = value > (limits[i].max + limits[i].min) / 2.f;
    else if(type.second == "Angle")
      *(Angle*)ptr = (Angle)(value);
  }

  InBinaryMemory memoryToParameter(mem_ptr, sizeof(res));
  memoryToParameter >> res;
  return res;
}

template<typename ParameterType>
std::vector<float> ParameterLearner<ParameterType>::parameterToVector(const ParameterType& param)
{
  std::vector<float> res;
  if(parameterPositions.size() == 0)
  {
    OUTPUT_WARNING("ParameterLearner: No Configuration for parameter type " << typeName << ". Call readConfiguration(string name) before using parameterToVector()");
    return res;
  }

  char* mem_ptr =  streamToMemory(param);

  for(unsigned i = 0; i < parameterPositions.size(); i++)
  {
    typed type = parameterPositions[i];
    float value = 0.f;
    void* ptr = mem_ptr + type.first;
    if(type.second == "char")
      value = (float)(*(char*)ptr);
    else if(type.second == "signed char")
      value = (float)(*(signed char*)ptr);
    else if(type.second == "unsigned char")
      value = (float)(*(unsigned char*)ptr);
    else if(type.second == "short")
      value = (float)(*(short*)ptr);
    else if(type.second == "unsigned short")
      value = (float)(*(unsigned short*)ptr);
    else if(type.second == "int")
      value = (float)(*(int*)ptr);
    else if(type.second == "unsigned" || type.second == "unsigned int")
      value = (float)(*(unsigned*)ptr);
    else if(type.second == "float")
      value = (*(float*)ptr);
    else if(type.second == "double")
      value = (float)(*(double*)ptr);
    else if(type.second == "bool")
      value = limits[i].min + ((*(bool*)ptr) ? 0.75f : 0.25f) * (limits[i].max - limits[i].min);
    else if(type.second == "Angle")
      value = (float)(*(Angle*)ptr);
    else
      ASSERT(false);

    value = (value - limits[i].min) / (limits[i].max - limits[i].min);
    if(logarithmic[i])
      value = std::log(value + 1.f) / std::log(2.f);
    res.push_back(value);
  }
  return res;
}

template<typename ParameterType>
void ParameterLearner<ParameterType>::setParameter(const std::vector<float>& parameter)
{
  ParameterType param = vectorToParameter(parameter);

  std::string name = "parameters:" + typeName.substr(0, typeName.size() - 14);
  queue.clear();
  queue.out.bin << name << char(1);
  queue.out.bin << param;
  queue.out.finishMessage(idDebugDataChangeRequest);
  Global::getDebugDataTable().processChangeRequest(queue.in);
}

template<typename ParameterType>
void ParameterLearner<ParameterType>::saveParameter(const std::vector<float>& parameter)
{
  saveModuleParameters(stdParameter, typeName.substr(0, typeName.size() - 14).c_str(), nullptr);
}
