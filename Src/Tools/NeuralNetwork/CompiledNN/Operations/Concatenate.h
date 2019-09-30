/**
 * @author Arne Hasselbring
 */

#pragma once

#include "../CompiledNNImplBase.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    struct ConcatenateCompiler : public OperationCompiler
    {
      struct Parameters
      {
        unsigned int dimension;
      };
      const Parameters p;

      ConcatenateCompiler(const CompilationSettings& settings, const Parameters& p) : OperationCompiler(settings), p(p) {}

      void initialize() override {}
      void compile(x86::Assembler& a, ActivationFunctionHandler& afHandler, const std::vector<TensorPointerXf>& input, const std::vector<TensorPointerXf>& output) const override;

      inline std::vector<std::vector<unsigned int>> calcOutputDimensions(const std::vector<std::vector<unsigned int>>& inputDimensions) const override
      {
        ASSERT(inputDimensions.size() > 1);
        std::vector<unsigned int> dimensions = inputDimensions[0];
        for(std::size_t i = 1; i < inputDimensions.size(); ++i)
          dimensions[p.dimension] += inputDimensions[i][p.dimension];
        return {dimensions};
      }

      inline std::vector<std::size_t> routeIO(const std::vector<std::size_t>& indices, const std::vector<std::vector<unsigned int>>& inputDimensions) const override
      {
        if(!indices.empty() && indices[0] == 0)
        {
          for(std::size_t i = 0; i < p.dimension; ++i)
            if(inputDimensions[0][i] != 1)
              return {};
          return {0};
        }
        return {};
      }

    private:
      void compileCopyPaste(x86::Assembler& a, const std::vector<TensorPointerXf>& input, const TensorPointerXf& output, std::size_t innerSize) const;
    };
  }
}
