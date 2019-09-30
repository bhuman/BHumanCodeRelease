/**
 * @file Utils/Tests/ModuleGraphCreator/ModuleGraphCreatorTest.h
 *
 * This file implements general functionality for ModuleGraphCreator tests.
 *
 * @author Jan Fiedler
 */

#pragma once

#include "Tools/FunctionList.h"
#include "Tools/Module/ModuleGraphCreator.h"

#include <gtest/gtest.h>

/**
 * The function creates a config with given parameters.
 *
 * @param threads The list of threads with all representation provider.
 * @param required The list of all requirements of all threads.
 */
static Configuration createConfig(const std::vector<std::vector<Configuration::RepresentationProvider>>& threads,
                                  const std::vector<std::string>& defaultRepresentations = {})
{
  Configuration config;
  config.defaultRepresentations = defaultRepresentations;
  for(size_t i = 0; i < threads.size(); ++i)
  {
    Configuration::Thread thread;
    thread.name = static_cast<char>('a' + i);
    thread.representationProviders = threads[i];
    config().emplace_back(thread);
  }
  return config;
}

/**
 * Input and output parameters of a test.
 */
struct Parameters
{
  Configuration config; // initial
  std::vector<std::vector<std::vector<const char*>>> received;
  std::vector<std::vector<std::vector<const char*>>> sent;
  Configuration config2; // update
  std::vector<std::vector<std::vector<const char*>>> received2;
  std::vector<std::vector<std::vector<const char*>>> sent2;
};

/**
 * @class ModuleGraphCreatorTest
 *
 * The class takes care about the environment of a test and provides util functions.
 */
class  ModuleGraphCreatorTest : public testing::TestWithParam<Parameters>
{
public:
  ModuleGraphCreatorTest() : moduleGraphCreator((FunctionList::execute(), GetParam().config)) {}

protected:
  ModuleGraphCreator moduleGraphCreator;
  Blackboard blackboard;

  void update(const Configuration& config)
  {
    OutBinaryMemory out(100);
    out << config;
    InBinaryMemory in(out.data());
    moduleGraphCreator.update(in);
  }

  bool calcShared(const Configuration& config)
  {
    for(auto& thread : moduleGraphCreator.sent)
    {
      for(auto& s : thread)
        s.clear();
    }
    for(auto& thread : moduleGraphCreator.received)
    {
      for(auto& r : thread)
        r.clear();
    }
    return moduleGraphCreator.calcShared(config);
  }

  const std::vector<std::vector<std::vector<const char*>>>& received() { return moduleGraphCreator.received; }
  const std::vector<std::vector<std::vector<const char*>>>& sent() { return moduleGraphCreator.sent; }
};

using ExpModuleGraphCreatorDeathTest = ModuleGraphCreatorTest;
using ModuleGraphCreatorSimple = ModuleGraphCreatorTest;
using ModuleGraphCreatorComplex = ModuleGraphCreatorTest;
