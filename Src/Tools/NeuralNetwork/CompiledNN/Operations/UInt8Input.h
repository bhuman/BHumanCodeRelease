/**
 * @author Felix Thielke
 */

#pragma once

#include "../CompiledNNImplBase.h"
#include "BatchNormalization.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    struct UInt8InputCompiler : public SISOOperationCompiler
    {
      struct Parameters
      {
        const BatchNormalizationCompiler::Parameters* batchNormalization = nullptr;
      };
      const Parameters p;

      UInt8InputCompiler(const CompilationSettings& settings, const Parameters& p) : SISOOperationCompiler(settings), p(p) {}

      inline bool canBeInplace() const override { return false; }

      void initialize() override;
      void compile(x86::Assembler& a, ActivationFunctionHandler& afHandler, const TensorPointerXf& input, const TensorPointerXf& output) const override;

    private:
      unsigned int paramLength;
    };
  }
}
