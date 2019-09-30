/**
 * @author Felix Thielke
 */

#pragma once

#include "../CompiledNNImplBase.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    struct Pooling2DCompiler : public SISOOperationCompiler
    {
      struct Parameters
      {
        std::array<unsigned int, 2> kernelSize;
        std::array<unsigned int, 2> strides;
        PoolingMethod method;
        PaddingType padding;
      };
      const Parameters p;

      Pooling2DCompiler(const CompilationSettings& settings, const Parameters& p) : SISOOperationCompiler(settings), p(p) {}

      inline bool canBeInplace() const override
      {
        return p.strides[0] >= p.kernelSize[0] && p.strides[1] >= p.kernelSize[1];
      }

      void initialize() override;
      void compile(x86::Assembler& a, ActivationFunctionHandler& afHandler, const TensorPointerXf& input, const TensorPointerXf& output) const override;

      inline std::vector<unsigned int> calcOutputDimensions(const std::vector<unsigned int>& inputDimensions) const override
      {
        ASSERT(inputDimensions.size() == 3);
        return {{(inputDimensions[0] - (p.padding == PaddingType::valid ? p.kernelSize[0] - 1 : 0) + p.strides[0] - 1) / p.strides[0],
                 (inputDimensions[1] - (p.padding == PaddingType::valid ? p.kernelSize[1] - 1 : 0) + p.strides[1] - 1) / p.strides[1],
                 inputDimensions[2]}};
      }

    private:
      mutable bool helperRegInitialized = false;

      void pool(x86::Assembler& a, ActivationFunctionHandler& afHandler, const unsigned int paddingHorizontal, const unsigned int paddingVertical, const unsigned int inputWidth, const unsigned int channels) const;
      unsigned int poolRow(x86::Assembler& a, ActivationFunctionHandler& afHandler, const unsigned int paddingLeft, const unsigned int paddingVertical, const unsigned int inputWidth, const unsigned int outputWidth, const unsigned int channels) const;
    };
  }
}
