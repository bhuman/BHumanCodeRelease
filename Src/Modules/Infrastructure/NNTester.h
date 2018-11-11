/**
 * @file NNTester.h
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/NeuralNetwork/Model.h"
#include "Tools/NeuralNetwork/CompiledNN.h"
#include "Representations/Infrastructure/DummyRepresentation.h"
#include <string>

MODULE(NNTester,
{,
  PROVIDES(DummyRepresentation),
  DEFINES_PARAMETERS(
  {
    ENUM(TestMode,
    {,
      do_nothing,
      validation,
      benchmark,
    }),
    (std::string)("NNTests/meta.model") path,
    (TestMode)(validation) testMode,
    (unsigned int)(50) sampleSize,
  }),
});

class NNTester : public NNTesterBase
{
private:
  std::string loadedModel;
  NeuralNetwork::Model model;
  NeuralNetwork::CompiledNN compiledNN;
  std::vector<NeuralNetwork::Tensor3> random;
  bool testDone = false;
  bool compiled = false;

  void update(DummyRepresentation& dummy) override;

  void stressTest() const;
public:
  NNTester() {}
};
