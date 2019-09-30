/**
 * @author Felix Thielke
 */

#pragma once

#include "../ActivationFunctions.h"
#include "../CompiledNNImplBase.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    struct ActivationCompiler : public SISOOperationCompiler
    {
      struct Parameters
      {
        ActivationFunctionDescriptor activationDesc;
      };
      const Parameters p;

      ActivationCompiler(const CompilationSettings& settings, const Parameters& p) : SISOOperationCompiler(settings), p(p) {}

      inline bool canBeInplace() const override { return true; }

      void initialize() override {}
      void compile(x86::Assembler& a, ActivationFunctionHandler& afHandler, const TensorPointerXf& input, const TensorPointerXf& output) const override;
    };
  }
}
