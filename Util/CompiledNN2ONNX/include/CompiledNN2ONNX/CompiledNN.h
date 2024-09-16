/**
 * This is a wrapper for ONNX runtime to mimic the behavior of CompiledNN.
 * It is used because CompiledNN is currently not supported on ARM Macs.
 * This file replaces the CompiledNN header CompiledNN.h.
 * @author Thomas Röfer
 */
#pragma once

#include <algorithm>
#include <span>
#include <asmjit/asmjit.h>
#include <onnxruntime_cxx_api.h>
#ifdef MACOS
#include <coreml_provider_factory.h>
#endif
#include "Model.h"

namespace NeuralNetworkONNX
{
  /** The compilation settings. Most are ignored. */
  struct CompilationSettings
  {
    bool useX64 = true;
    bool useSSE42 = true;
    bool useAVX2 = true;
    bool useExpApproxInSigmoid = true;
    bool useExpApproxInTanh = true;
    bool debug = false;
    bool useCoreML = false;
    int numOfThreads = 1;
  };

  /** The class for running neural networks.  */
  class CompiledNN
  {
    // On Windows, using the global environment deadlocks when ending the Simulator.
    // Therefore, a local environment is used.
#if defined WINDOWS || defined TARGET_ROBOT
    std::unique_ptr<Ort::Env> env;
#endif
    Ort::Session session {nullptr}; /**< The session for running a neural network. */
    Ort::AllocatorWithDefaultOptions allocator; /**< The allocator that handles memory. */
    std::vector<const char*> inputNames; /**< The input names required by Ort::Session::Run. */
    std::vector<const char*> outputNames; /**< The input names required by Ort::Session::Run. */
    std::vector<std::vector<int64_t>> inputDims; /**< The dimensions (shapes) required to support Tensor::rank and Tensor::dims for each input. */
    std::vector<std::vector<int64_t>> outputDims; /**< The dimensions (shapes) required to support Tensor::rank and Tensor::dims for each output. */
    std::vector<size_t> inputSizes; /**< The overall number of values for each input. */
    std::vector<size_t> outputSizes; /**< The overall number of values for each output. */
    std::vector<Ort::Value> inputTensors; /**< The pre-allocated tensors for each input. */
    std::vector<Ort::Value> outputTensors; /**< The pre-allocated tensors for each output. */
    std::vector<unsigned char*> uint8Buffers; /**< For each input that is encoded as unsigned chars, a buffer of the required size is provided. Otherwise, the entry is nullptr. */

    /**
     * Helper method to create a single ONNX environment that hosts the global
     * thread pool.
     */
    static Ort::Env& environment()
    {
      OrtThreadingOptions* threadingOptions = nullptr;
      Ort::GetApi().CreateThreadingOptions(&threadingOptions);
      static Ort::Env environment{threadingOptions};
      Ort::GetApi().ReleaseThreadingOptions(threadingOptions);
      return environment;
    }

    /** Clear all buffers. */
    void clear()
    {
      for(const char* inputName : inputNames)
        allocator.Free(const_cast<char*>(inputName));
      for(unsigned char* uint8Buffer : uint8Buffers)
        delete[] uint8Buffer;
      for(const char* outputName : outputNames)
        allocator.Free(const_cast<char*>(outputName));

      inputNames.clear();
      inputDims.clear();
      inputSizes.clear();
      inputTensors.clear();
      uint8Buffers.clear();
      outputNames.clear();
      outputDims.clear();
      outputSizes.clear();
      outputTensors.clear();
    }

  public:
    /**
     * A class that simulates the behavior of tensors returned by the
     * CompiledNN methods input() and output(). They support the
     * methods data(), rank(), and dims() as well as array access,
     * iteration, and copying the data from one tensor to another.
     */
    class Tensor : public std::span<float>
    {
      float* tensor; /**< The actual tensor data. */
      const std::vector<int64_t>& dimensions; /**< The dimensions (shape) of the tensor. */

      /**
       * Constructor.
       * @param tensor The actual tensor data.
       * @param size The overall number of values in the tensor.
       * @param dimensions The dimensions (shape) of the tensor.
       */
      Tensor(float* tensor, size_t size, const std::vector<int64_t>& dimensions)
        : span(tensor, size),

        tensor(tensor), dimensions(dimensions) {static_cast<void>(size);}

    public:
      /**
       * Returns the address of the actual tensor data.
       * @param The address of the data.
       */
      float* data() {return tensor;}

      /**
       * Returns the number of dimensions of the tensor.
       * @return The rank of the tensor. CompiledNN does not count the first dimension.
       */
      unsigned rank() const {return static_cast<unsigned>(dimensions.size() - 1);}

      /**
       * Returns the size of a certain dimension of the tensor.
       * @param dimension The number of the dimension. CompiledNN skips the first dimension.
       * @return The size of the dimension of the tensor.
       */
      unsigned dims(size_t dimension) const {return static_cast<unsigned>(dimensions[dimension + 1]);}

      /**
       * Copies the tensor data from another tensor to this one.
       * @param other The other tensor. It must have the same size as this one.
       * @return This tensor after the assignment.
       */
      Tensor& operator=(const Tensor& other)
      {
        std::copy(other.begin(), other.end(), begin());
        return *this;
      }

      friend class CompiledNN; /**< CompiledNN calls the private constructor. */
    };

    /** Constructor. The parameter is ignored. */
    explicit CompiledNN(asmjit::JitRuntime* = nullptr)
    {
#ifdef TARGET_ROBOT
      env = std::make_unique<Ort::Env>();
#elif defined WINDOWS
      OrtThreadingOptions* threadingOptions = nullptr;
      Ort::GetApi().CreateThreadingOptions(&threadingOptions);
      env = std::make_unique<Ort::Env>(threadingOptions);
      Ort::GetApi().ReleaseThreadingOptions(threadingOptions);
#endif
    }

    /** The destructor frees all buffers. */
    ~CompiledNN()
    {
      clear();
    }

    /**
     * Loads and compiles the model. Initializes all of the fields of
     * this class based on that model. The compilation settings are
     * ignored.
     * @param model The model to load and compile.
     */
    void compile(const Model& model, const CompilationSettings& settings = CompilationSettings())
    {
      clear();

      // Create an ONNX session. Use a global thread pool rather than local pools.
      Ort::SessionOptions sessionOptions;
#ifndef TARGET_ROBOT
      sessionOptions.DisablePerSessionThreads();
#else
      sessionOptions.SetExecutionMode(ORT_SEQUENTIAL);
      sessionOptions.SetIntraOpNumThreads(settings.numOfThreads);
#endif
#ifdef MACOS
      if(settings.useCoreML)
        Ort::ThrowOnError(OrtSessionOptionsAppendExecutionProvider_CoreML(sessionOptions, 0));
#else
      static_cast<void>(&settings);
#endif
#ifdef WINDOWS
      // Not sure whether this works with non-ASCII characters.
      std::wstring filename(model.filename.size(), L' ');
      std::copy(model.filename.begin(), model.filename.end(), filename.begin());
      session = Ort::Session(*env, filename.c_str(), sessionOptions);
#elif defined TARGET_ROBOT
      session = Ort::Session(*env, model.filename.c_str(), sessionOptions);
#else
      session = Ort::Session(environment(), model.filename.c_str(), sessionOptions);
#endif
      // Create the names, tensors, dimensions, sizes, and buffers for all inputs.
      for(size_t i = 0; i < session.GetInputCount(); i++)
      {
        inputNames.emplace_back(session.GetInputName(i, allocator));
        inputDims.emplace_back(session.GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
        size_t size = 1;
        for(int64_t& dim : inputDims.back())
        {
          dim = dim <= 0 ? 1 : dim;
          size *= static_cast<size_t>(dim);
        }
        inputSizes.emplace_back(size);
        inputTensors.emplace_back(Ort::Value::CreateTensor<float>(allocator, inputDims.back().data(), inputDims.back().size()));
        uint8Buffers.emplace_back(model.isUint8.find(i) != model.isUint8.end() ? new unsigned char[size] : nullptr);
      }

      // Create the names, tensors, dimensions, and sizes for all outputs.
      for(size_t i = 0; i < session.GetOutputCount(); i++)
      {
        outputNames.emplace_back(session.GetOutputName(i, allocator));
        outputDims.emplace_back(session.GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
        size_t size = 1;
        for(int64_t& dim : outputDims.back())
        {
          dim = dim <= 0 ? 1 : dim;
          size *= static_cast<size_t>(dim);
        }
        outputSizes.emplace_back(size);
        outputTensors.emplace_back(Ort::Value::CreateTensor<float>(allocator, outputDims.back().data(), outputDims.back().size()));
      }
    }

    /** Runs the network. */
    void apply()
    {
      // For each input with an unsigned chars buffer, the data is copied into
      // the actual float input tensor.
      for(size_t i = 0; i < uint8Buffers.size(); ++i)
        if(uint8Buffers[i])
        {
          float* pDest = inputTensors[i].GetTensorMutableData<float>();
          for(unsigned char* pSrc = uint8Buffers[i], * pEnd = pSrc + inputSizes[i]; pSrc < pEnd;)
            *pDest++ = *pSrc++;
        }

      // Run the network.
      session.Run(Ort::RunOptions{nullptr},
                  inputNames.data(), inputTensors.data(), inputTensors.size(),
                  outputNames.data(), outputTensors.data(), outputTensors.size());
    }

    /**
     * Was the network successfully compiled?
     * @return Always true after \c compile has been called, because ONNX would have terminated the program
     *         if something went wrong.
     */
    bool valid() const {return session;}

    /**
     * Returns the number of inputs.
     * @return The number of inputs.
     */
    size_t numOfInputs() const {return session.GetInputCount();}

    /**
     * Returns the number of outputs.
     * @return The number of outputs.
     */
    size_t numOfOutputs() const {return session.GetOutputCount();}

    /**
     * Returns the input tensor.
     * @return The input tensor.
     */
    Tensor input(size_t index)
    {
      if(uint8Buffers[index])
        return Tensor(reinterpret_cast<float*>(uint8Buffers[index]), inputSizes[index], inputDims[index]);
      else
        return Tensor(inputTensors[index].GetTensorMutableData<float>(), inputSizes[index], inputDims[index]);
    }

    /**
     * Returns the output tensor.
     * @return The output tensor.
     */
    Tensor output(size_t index) {return Tensor(outputTensors[index].GetTensorMutableData<float>(), outputSizes[index], outputDims[index]);}
  };
}
