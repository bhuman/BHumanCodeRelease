/**
 * Declares a class that compiles neural networks to optimized X86 machine
 * code and applies them to input data.
 *
 * @author Felix Thielke
 * @author Arne Hasselbring
 */

#pragma once

#include "Model.h"
#include "Tensor.h"
#include "CompiledNN/CompilationSettings.h"
#include "Platform/BHAssert.h"
#include <list>
#include <memory>
#include <string>
#include <typeindex>
#include <unordered_map>
#include <vector>

namespace NeuralNetwork
{
  struct Model;
  struct Node;

  namespace CompiledNNImpl
  {
    class ActivationFunctionHandler;
    struct OperationCompiler;
  }

  /**
   * A class that compiles neural networks into optimized X86 machine code and
   * applies them on input data.
   */
  class CompiledNN
  {
  private:
    struct Operation;

    using CompilerList = std::vector<std::unique_ptr<CompiledNNImpl::OperationCompiler>>;
    using CompilerMap = std::unordered_map<std::type_index, CompilerList>;

    struct TensorLocationHasher
    {
      std::size_t operator()(const TensorLocation& tl) const
      {
        // Hash only the layer, since multiple nodes and multiple output tensors are not the main interest of this library.
        return std::hash<const Layer*>()(tl.layer);
      }
    };

    struct OperandLocation
    {
      Operation* provider;
      unsigned int tensorIndex;

      OperandLocation(Operation* provider, unsigned int tensorIndex) : provider(provider), tensorIndex(tensorIndex) {}

      bool operator==(const OperandLocation& other) const
      {
        return provider == other.provider && tensorIndex == other.tensorIndex;
      }
    };

    struct OperandPlaceholder
    {
      OperandLocation location;
      std::size_t requiredSize;
      std::size_t refCount;
      TensorXf* allocatedTensor = nullptr;

      OperandPlaceholder(const OperandLocation& location, std::size_t requiredSize, std::size_t refCount) :
          location(location), requiredSize(requiredSize), refCount(refCount)
      {}
    };

    struct Operation
    {
      const CompiledNNImpl::OperationCompiler* compiler;
      std::vector<OperandLocation> inputs;
      std::vector<OperandLocation> outputs;
      std::vector<std::vector<unsigned int>> inputDimensions;
      std::vector<std::vector<unsigned int>> outputDimensions;
      std::vector<OperandPlaceholder*> inputOperands;
      std::vector<OperandPlaceholder*> outputOperands;

      Operation(const CompiledNNImpl::OperationCompiler* compiler) : compiler(compiler) {}
    };

    /**
     * Obtains a compiler of a specific type with specific parameters. If a compiler with the same parameters already exists, that one is returned.
     */
    template<typename CompilerType>
    static CompiledNNImpl::OperationCompiler* getCompiler(const CompilationSettings& settings, const typename CompilerType::Parameters& p, CompilerMap& compilers);

    /**
     * Generates the compilers necessary to execute a given node (must be a sequential, atomic (i.e. non-mergeable) chain).
     */
    static std::vector<CompiledNNImpl::OperationCompiler*> generateCompilers(const CompilationSettings& settings, const Node& node, CompilerMap& compilers);

    /**
     * Assigns each symbolic variable a placeholder.
     */
    void assignOperands(std::list<Operation>& operations, const std::vector<OperandLocation>& inputLocations, const std::vector<OperandLocation>& outputLocations,
                        std::list<OperandPlaceholder>& operands, std::vector<OperandPlaceholder*>& inputPlaceholders, std::vector<OperandPlaceholder*>& outputPlaceholders);

    /**
     * Allocates actual tensors for all placeholders with their current required sizes.
     */
    void allocateTensors(std::list<OperandPlaceholder>& operands);

    /**
     * Generates code for all operations (in that order) in a list.
     */
    void generateCode(const std::list<Operation>& operations, const CompilerMap& compilers, CompiledNNImpl::ActivationFunctionHandler& afHandler);

    /**
     * Does the actual compilation process (given a list of operations, all compilers they use, input and output locations and the settings to use).
     */
    void compilerBackend(std::list<Operation>& operations, const CompilerMap& compilers,
                         const std::vector<OperandLocation>& inputLocations, const std::vector<OperandLocation>& outputLocations,
                         const CompilationSettings& settings);

    using FnType = void (*)();
    FnType applyFunction = nullptr;
    std::vector<TensorXf*> inputTensors, outputTensors;
    std::vector<std::vector<unsigned int>> inputDimensions, outputDimensions;
    std::vector<TensorXf> tensors;

  public:
    CompiledNN() = default;
    ~CompiledNN();

    /**
     * Compiles the net described by the given specification.
     */
    void compile(const Model& specification, const CompilationSettings& settings = CompilationSettings());

    /**
     * Compiles a net consisting of only the given node.
     */
    void compile(const Node& node, const CompilationSettings& settings = CompilationSettings());

    /**
     * Compiles the net from the given file.
     */
    void compile(const std::string& filename, const CompilationSettings& settings = CompilationSettings());

    /**
     * Checks whether the net was successfully compiled.
     */
    inline bool valid() const { return static_cast<bool>(applyFunction); }

    /**
     * Returns the number of input tensors of the compiled net.
     */
    inline std::size_t numOfInputs() const
    {
      return inputTensors.size();
    }

    /**
     * Returns a reference to an input tensor of the compiled net.
     * Reshaping the tensor will result in undefined behavior.
     * Also note that calling apply() or output() invalidates this tensor.
     * Have fun.
     */
    inline TensorXf& input(std::size_t index)
    {
      inputTensors[index]->reshape(inputDimensions[index]);
      return *inputTensors[index];
    }

    /**
     * Returns the number of output tensors of the compiled net.
     */
    inline std::size_t numOfOutputs() const
    {
      return outputTensors.size();
    }

    /**
     * Returns a reference to an output tensor of the compiled net.
     * Reshaping the tensor will result in undefined behavior.
     * Also not that calling input() invalidates this tensor.
     */
    inline TensorXf& output(std::size_t index)
    {
      outputTensors[index]->reshape(outputDimensions[index]);
      return *outputTensors[index];
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
