/**
 * @file NNTester.cpp
 */

#include "NNTester.h"
#include "Platform/File.h"
#include "Tools/Math/Random.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/NeuralNetwork/SimpleNN.h"
#include "Tools/Streams/InStreams.h"
#include <sstream>
#include <algorithm>

MAKE_MODULE(NNTester, cognitionInfrastructure)

using namespace NeuralNetwork;

template<size_t dim>
std::ostream& operator<<(std::ostream& s, const std::array<unsigned int, dim>& arr)
{
  for(size_t i = 0; i < dim; i++)
  {
    if(i != 0)
      s << "x";
    s << arr[i];
  }

  return s;
}

std::ostream& operator<<(std::ostream& s, const ActivationFunctionId fn)
{
  switch(fn)
  {
    case ActivationFunctionId::linear:
      s << "linear";
      break;
    case ActivationFunctionId::relu:
      s << "relu";
      break;
    case ActivationFunctionId::hardSigmoid:
      s << "hardSigmoid";
      break;
    case ActivationFunctionId::sigmoid:
      s << "sigmoid";
      break;
    case ActivationFunctionId::tanH:
      s << "tanh";
      break;
    case ActivationFunctionId::softmax:
      s << "softmax";
      break;
    case ActivationFunctionId::elu:
      s << "elu";
      break;
  }

  return s;
}

void NNTester::update(DummyRepresentation& dummy)
{
  DEBUG_RESPONSE_ONCE("module:NNTester:revalidate")
    testDone = false;

  if(loadedModel != path)
  {
    if(InBinaryFile(path).exists())
      model.load(path);
    loadedModel = path;
    testDone = false;
    compiled = false;
  }

  DEBUG_RESPONSE_ONCE("module:NNTester:stressTest")
    stressTest();

  DEBUG_RESPONSE_ONCE("module:NNTester:printModel")
  {
    std::stringstream output;

    output << "Input -> " << model.getLayers()[0]->inputDimensions << std::endl;
    for(const auto& layer : model.getLayers())
    {
      switch(layer->type)
      {
        case LayerType::dense:
          output << "Dense + " << static_cast<DenseLayer&>(*layer).activationId;
          break;
        case LayerType::conv2D:
          output << "Conv2D ";
          switch(static_cast<Conv2DLayer&>(*layer).padding)
          {
            case PaddingType::same:
              output << "same";
              break;
            case PaddingType::valid:
              output << "valid";
              break;
          }
          output << " " << static_cast<Conv2DLayer&>(*layer).weights.dims(0) << "x" << static_cast<Conv2DLayer&>(*layer).weights.dims(1)
                 << ", strides " << static_cast<Conv2DLayer&>(*layer).strides
                 << " + " << static_cast<Conv2DLayer&>(*layer).activationId;
          break;
        case LayerType::reshape:
          output << "Reshape";
          break;
        case LayerType::elu:
          output << "Elu";
          break;
        case LayerType::activation:
          output << "Activation " << static_cast<Conv2DLayer&>(*layer).activationId;
          break;
        case LayerType::pooling2D:
          output << (static_cast<Pooling2DLayer&>(*layer).method == Pooling2DLayer::PoolingMethod::max ? "Max" : "Avg") << "Pooling2D ";
          switch(static_cast<Pooling2DLayer&>(*layer).padding)
          {
            case PaddingType::same:
              output << "same";
              break;
            case PaddingType::valid:
              output << "valid";
              break;
          }
          output << " " << static_cast<Pooling2DLayer&>(*layer).kernelSize
                 << ", strides " << static_cast<Conv2DLayer&>(*layer).strides;
          break;
        case LayerType::batchNormalization:
          output << "BatchNormalization";
          break;
        case LayerType::softmax:
          output << "Softmax";
      }
      output << " -> " << layer->outputDimensions << std::endl;
    }
    OUTPUT_TEXT(output.str());
  }

  if(loadedModel.empty() || model.getLayers().size() == 0)
    return;

  if(testMode == TestMode::validation && !testDone)
  {
    std::stringstream output;
    Tensor3 testTensor(model.testInput);

    // Set compilation settings
    CompilationSettings settings;
    settings.useX64 = false;
    settings.useSSE42 = false;
    settings.useAVX2 = false;
    settings.useExpApproxInSigmoid = false;
    settings.useExpApproxInTanh = false;
    settings.debug = true;

    for(const auto& layer : model.getLayers())
    {
      // Compile layer
      compiledNN.compile(*layer, settings);

      // Do test
      compiledNN.input().copyFrom(testTensor);
      compiledNN.apply();
      SimpleNN::apply(Tensor3(testTensor), testTensor, *layer);

      // Compute error
      switch(layer->type)
      {
        case LayerType::dense:
          output << "Dense";
          break;
        case LayerType::conv2D:
          output << "Conv2D";
          break;
        case LayerType::reshape:
          output << "Reshape";
          break;
        case LayerType::elu:
          output << "Elu";
          break;
        case LayerType::activation:
          output << "Activation";
          break;
        case LayerType::pooling2D:
          output << (static_cast<Pooling2DLayer&>(*layer).method == Pooling2DLayer::PoolingMethod::max ? "Max" : "Avg") << "Pooling2D";
          break;
        case LayerType::batchNormalization:
          output << "BatchNormalization";
          break;
        case LayerType::softmax:
          output << "Softmax";
      }
      ASSERT(testTensor.size() == compiledNN.output().size());
      output << "Layer error: rel " << testTensor.relError(compiledNN.output(), true) << ", abs " << testTensor.absError(compiledNN.output(), true) << std::endl;
    }

    ASSERT(testTensor.size() == model.testResult.size());
    output << "Reference error: rel " << model.testResult.relError(testTensor, true) << ", abs " << model.testResult.absError(testTensor, true) << std::endl;

    // Test whole network
    compiledNN.compile(model, settings);
    compiledNN.input().copyFrom(model.testInput);
    compiledNN.apply();
    ASSERT(model.testResult.size() == compiledNN.output().size());
    output << "Total error: rel " << model.testResult.relError(compiledNN.output(), true) << ", abs " << model.testResult.absError(compiledNN.output(), true) << std::endl;
    OUTPUT_TEXT(output.str());

    testDone = true;
  }
  else if(testMode == TestMode::benchmark)
  {
    if(!compiled)
    {
      compiledNN.compile(model);
      compiled = true;
    }

    // Generate random inputs
    Random r;
    random.resize(sampleSize);
    for(Tensor3& t : random)
    {
      t.reshape(compiledNN.input().dims());
      float* p = t.data();
      for(size_t n = t.size(); n; --n)
        *(p++) = static_cast<float>(r.uniformInt(256));
    }

    // Measure execution time of the neural model
    STOPWATCH("module:NNTester:benchmark")
    {
      for(size_t i = 0; i < sampleSize; i++)
      {
        compiledNN.input().copyFrom(random[i]);
        compiledNN.apply();
      }
    }
  }
}

void NNTester::stressTest() const
{
  Random r;
  CompiledNN c;
  CompilationSettings settings;
  settings.useX64 = false;
  settings.useSSE42 = false;
  settings.useAVX2 = false;
  settings.debug = true;
  Tensor3 testTensor;
  std::stringstream output;

  output << "# Conv2D valid padding 32x32 image, 3x3 kernel" << std::endl;
  Conv2DLayer l;
  l.inputDimensions = { {32, 32, 1} };
  l.outputDimensions = { {30, 30, 1} };
  l.padding = PaddingType::valid;
  l.activationId = ActivationFunctionId::linear;
  l.strides = { {1, 1} };
  for(unsigned int inputSize = 1; inputSize < 40; inputSize++)
  {
    l.inputDimensions[2] = inputSize;

    for(unsigned int outputSize = 1; outputSize < 80; outputSize++)
    {
      l.outputDimensions[2] = outputSize;

      l.weights.reshape(3, 3, inputSize, outputSize);
      l.biases.resize(outputSize);

      const float limit = std::sqrt(6.f / static_cast<float>(3 * 3 * (inputSize + outputSize)));

      float absError = 0.f;
      float relError = 0.f;
      for(unsigned int i = 0; i < 5; i++)
      {
        for(auto p = l.weights.begin(); p < l.weights.end(); p++)
          *p = r.uniform(-limit, limit);

        for(auto p = l.biases.begin(); p < l.biases.end(); p++)
          *p = r.uniform(-limit, limit);

        c.compile(l, settings);

        for(auto p = c.input().begin(); p < c.input().end(); p++)
          *p = r.uniform(-1.f, 1.f);

        SimpleNN::apply(Tensor3(c.input()), testTensor, l);
        c.apply();

        const float err = testTensor.relError(c.output(), true);
        if(err > relError)
        {
          relError = err;
          absError = testTensor.absError(c.output(), true);
        }
      }

      if(relError > 0.2)
      {
        output << "input channels " << inputSize << ", output channels " << outputSize << ":" << std::endl;
        output << "abs: " << absError << ", rel: " << relError << std::endl;
      }
    }
  }

  OUTPUT_TEXT(output.str());
}
