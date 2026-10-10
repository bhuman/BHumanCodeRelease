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
#include <iostream>
#include <mutex>
#include "../../../Src/Libs/Platform/File.h"

#include "Model.h"

#if ORT_API_VERSION < 13
namespace Ort
{
  struct AllocatedStringPtr
  {
    char* ptr;
    AllocatedStringPtr(char* ptr) : ptr(ptr) {}
    ~AllocatedStringPtr() {if(ptr) {AllocatorWithDefaultOptions allocator; allocator.Free(ptr);}}
    char* get() {return ptr;}
  };
#define GetInputNameAllocated GetInputName
#define GetOutputNameAllocated GetOutputName
}
#endif

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
    bool useCUDA = false;
    bool useTensorRT = false;
    bool fp16 = false;
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
    std::vector<Ort::AllocatedStringPtr> inputNameAllocators; /**< The allocated input names. */
    std::vector<const char*> outputNames; /**< The output names required by Ort::Session::Run. */
    std::vector<Ort::AllocatedStringPtr> outputNameAllocators; /**< The allocated output names. */
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

    /**
     * A single mutex for all threads.
     * Only used to synchronize CoreML caching.
     * @return The mutex.
     */
    static std::mutex& mutex()
    {
      static std::mutex mutex;
      return mutex;
    }

    /** Clear all buffers. */
    void clear()
    {
      for(unsigned char* uint8Buffer : uint8Buffers)
        delete[] uint8Buffer;

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
#if defined MACOS
      if(settings.useCoreML)
      {
        std::unordered_map<std::string, std::string> providerOptions;
        providerOptions["ModelCacheDirectory"] = File::getBHDir() + std::string("/Config/NeuralNets/CoreMLCache");
        sessionOptions.AppendExecutionProvider("CoreML", providerOptions);
      }
#elif defined TARGET_ROBOT
      if(settings.useTensorRT)
      {
        const OrtApi* api = OrtGetApiBase()->GetApi(ORT_API_VERSION);
        OrtTensorRTProviderOptionsV2* options;
        Ort::ThrowOnError(api->CreateTensorRTProviderOptions(&options));
        const char* keys[] = {"trt_engine_cache_enable", "trt_engine_cache_path", "trt_fp16_enable"};
        const char* values[] = {"1", "/home/booster/Config/NeuralNets/TensorRTCache", settings.fp16 ? "1" : "0"};
        Ort::ThrowOnError(api->UpdateTensorRTProviderOptions(options, keys, values, sizeof(keys) / sizeof(*keys)));
        Ort::ThrowOnError(api->SessionOptionsAppendExecutionProvider_TensorRT_V2(sessionOptions, options));
        api->ReleaseTensorRTProviderOptions(options);
      }
      if(settings.useCUDA)
      {
        OrtCUDAProviderOptions options;
        options.device_id = 0;
        options.arena_extend_strategy = 0;
        options.gpu_mem_limit = 2 * 1024 * 1024 * 1024;
        options.cudnn_conv_algo_search = OrtCudnnConvAlgoSearchExhaustive;
        options.do_copy_in_default_stream = 1;
        Ort::ThrowOnError(Ort::GetApi().SessionOptionsAppendExecutionProvider_CUDA(sessionOptions, &options));
      }
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
      // CoreML caching does not work with parallelism
      if(settings.useCoreML)
        mutex().lock();
      session = Ort::Session(environment(), model.filename.c_str(), sessionOptions);
      if(settings.useCoreML)
        mutex().unlock();
#endif
      // Create the names, tensors, dimensions, sizes, and buffers for all inputs.
      for(size_t i = 0; i < session.GetInputCount(); i++)
      {
        inputNameAllocators.emplace_back(session.GetInputNameAllocated(i, allocator));
        inputNames.emplace_back(inputNameAllocators.back().get());
        if(session.GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
          ORT_CXX_API_THROW("Network inputs must be float values", ORT_FAIL);
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
        outputNameAllocators.emplace_back(session.GetOutputNameAllocated(i, allocator));
        outputNames.emplace_back(outputNameAllocators.back().get());
        if(session.GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
          ORT_CXX_API_THROW("Network outputs must be float values", ORT_FAIL);
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

#ifdef TARGET_ROBOT
      // With CUDA, run the network once to finish initialization.
      if(settings.useCUDA || settings.useTensorRT)
        apply();
#endif
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
