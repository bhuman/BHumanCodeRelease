/**
 * @author Arne Hasselbring
 */

#pragma once

#include "../CompiledNNImplBase.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    struct ArithmeticCompiler : public OperationCompiler
    {
      enum OperationType
      {
        add,
        sub,
        mul,
        avg,
        max,
        min
      };
      struct Parameters
      {
        unsigned int inputSize = 0;
        OperationType op;
      };
      const Parameters p;
      ArithmeticCompiler(const CompilationSettings& settings, const Parameters& p) : OperationCompiler(settings), p(p) {}

      void initialize() override;
      void compile(x86::Assembler& a, ActivationFunctionHandler& afHandler, const std::vector<TensorPointerXf>& input, const std::vector<TensorPointerXf>& output) const override;

      inline std::vector<std::vector<unsigned int>> calcOutputDimensions(const std::vector<std::vector<unsigned int>>& inputDimensions) const override
      {
        ASSERT(inputDimensions.size() > 1);
        return {inputDimensions[0]};
      }

      inline std::vector<std::size_t> routeIO(const std::vector<std::size_t>& indices, const std::vector<std::vector<unsigned int>>& inputDimensions) const override
      {
        // This may break associativity of the original sum.
        // If associativity is needed, add && (indices[0] == 0 || indices[0] == 1)
        if(!indices.empty() && (p.op != sub || indices[0] == 0))
          return {indices[0]};
        return {};
      }
    };
  }
}
