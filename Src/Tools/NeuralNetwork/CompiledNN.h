/**
 * Declares a class that compiles neural networks to optimized X86 machine
 * code and applies them on input data.
 *
 * @author Felix Thielke
 */

#pragma once

#include "Platform/BHAssert.h"
#include "Tensor.h"
#include "CompiledNN/CompilationSettings.h"
#include <string>
#include <array>

namespace NeuralNetwork
{
  struct Model;
  struct Layer;

  /**
   * A class that compiles neural networks into optimized X86 machine code and
   * applies them on input data.
   */
  class CompiledNN
  {
  private:
    typedef void(*FnType)();
    FnType applyFunction = nullptr;
    std::array<Tensor3, 2> tensors;
    static constexpr unsigned int inputTensorIx = 0;
    unsigned int outputTensorIx = 0;
    std::array<unsigned int, 3> inputDimensions;
    std::array<unsigned int, 3> outputDimensions;

  public:
    CompiledNN() = default;
    ~CompiledNN();

    /**
     * Compiles the net described by the given specification.
     */
    void compile(const Model& specification, const CompilationSettings& settings = CompilationSettings());

    /**
     * Compiles a net consisting of only the given layer.
     */
    void compile(const Layer& layer, const CompilationSettings& settings = CompilationSettings());

    /**
     * Compiles the net from the given file.
     */
    void compile(const std::string& filename, const CompilationSettings& settings = CompilationSettings());

    /**
     * Checks whether the net was successfully compiled.
     */
    inline bool valid() const { return static_cast<bool>(applyFunction); }

    /**
     * Returns a reference to the input tensor of the compiled net.
     * Reshaping the tensor will result in undefined behaviour.
     * Also note that calling apply() invalidates this tensor.
     * Have fun.
     */
    inline Tensor3& input()
    {
      if(inputTensorIx == outputTensorIx)
        tensors[inputTensorIx].reshape(inputDimensions);
      return tensors[inputTensorIx];
    }

    /**
     * Returns a reference to the output tensor of the compiled net.
     */
    inline Tensor3& output()
    {
      if(inputTensorIx == outputTensorIx)
        tensors[outputTensorIx].reshape(outputDimensions);
      return tensors[outputTensorIx];
    }

    /**
     * Applies the compiled net on the current input data.
     */
    inline void apply() const
    {
      ASSERT(valid());
      applyFunction();
    }
  };
}
