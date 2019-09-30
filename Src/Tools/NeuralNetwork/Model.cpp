/**
 * Implements a methods to load neural network models from a file.
 *
 * @author Felix Thielke
 * @author Arne Hasselbring
 */

#include "Model.h"
#include "Platform/BHAssert.h"
#include "Platform/File.h"
#include "Platform/Thread.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Streams/SimpleMap.h"
#include <hdf5.h>
#include <cstring>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>
#include <utility>

namespace NeuralNetwork
{
  void Node::setDimensions()
  {
    inputDimensions.reserve(inputs.size());
    for(const TensorLocation& l : inputs)
      inputDimensions.push_back(l.layer->nodes[l.nodeIndex].outputDimensions[l.tensorIndex]);
    layer->calcOutputDimensions(*this);
  }

  void InputLayer::calcOutputDimensions(Node& node) const
  {
    ASSERT(node.inputDimensions.empty());
    node.outputDimensions.push_back(dimensions);
  }

  void DenseLayer::calcOutputDimensions(Node& node) const
  {
    ASSERT(node.inputDimensions.size() == 1);
    if(node.inputDimensions[0].size() != 1)
      FAIL("Dense layers can currently only be applied to flat tensors. Use a 1x1 convolution if it is really needed.");
    ASSERT(node.inputDimensions[0][0] == weights.dims(0));
    node.outputDimensions.push_back({weights.dims(1)});
  }

  void ActivationLayer::calcOutputDimensions(Node& node) const
  {
    if(node.inputDimensions.size() != 1)
      FAIL("Activation layers must currently have exactly one input tensor.");
    node.outputDimensions = node.inputDimensions;
  }

  void DropoutLayer::calcOutputDimensions(Node& node) const
  {
    node.outputDimensions = node.inputDimensions;
  }

  void FlattenLayer::calcOutputDimensions(Node& node) const
  {
    ASSERT(node.inputDimensions.size() == 1);
    node.outputDimensions.push_back({std::accumulate(node.inputDimensions[0].begin(), node.inputDimensions[0].end(), 1u, std::multiplies<>())});
  }

  void ReshapeLayer::calcOutputDimensions(Node& node) const
  {
    ASSERT(node.inputDimensions.size() == 1);
    ASSERT(std::accumulate(node.inputDimensions[0].begin(), node.inputDimensions[0].end(), 1u, std::multiplies<>()) ==
           std::accumulate(dimensions.begin(), dimensions.end(), 1u, std::multiplies<>()));
    node.outputDimensions.push_back(dimensions);
  }

  void Conv2DLayer::calcOutputDimensions(Node& node) const
  {
    ASSERT(node.inputDimensions.size() == 1);
    ASSERT(node.inputDimensions[0].size() == 3);
    ASSERT(padding == PaddingType::same || node.inputDimensions[0][0] >= weights.dims(0));
    ASSERT(padding == PaddingType::same || node.inputDimensions[0][1] >= weights.dims(1));
    ASSERT(node.inputDimensions[0][2] == weights.dims(2));
    node.outputDimensions.push_back({{(node.inputDimensions[0][0] - (padding == PaddingType::valid ? weights.dims(0) - 1 : 0) + strides[0] - 1) / strides[0],
                                      (node.inputDimensions[0][1] - (padding == PaddingType::valid ? weights.dims(1) - 1 : 0) + strides[1] - 1) / strides[1],
                                      weights.dims(3)}});
  }

  void SeparableConv2DLayer::calcOutputDimensions(Node& node) const
  {
    ASSERT(node.inputDimensions.size() == 1);
    ASSERT(node.inputDimensions[0].size() == 3);
    ASSERT(padding == PaddingType::same || node.inputDimensions[0][0] >= depthwiseWeights.dims(0));
    ASSERT(padding == PaddingType::same || node.inputDimensions[0][1] >= depthwiseWeights.dims(1));
    ASSERT(node.inputDimensions[0][2] == depthwiseWeights.dims(2));
    ASSERT(node.inputDimensions[0][2] * depthwiseWeights.dims(3) == pointwiseWeights.dims(2));
    node.outputDimensions.push_back({{(node.inputDimensions[0][0] - (padding == PaddingType::valid ? depthwiseWeights.dims(0) - 1 : 0) + strides[0] - 1) / strides[0],
                                      (node.inputDimensions[0][1] - (padding == PaddingType::valid ? depthwiseWeights.dims(1) - 1 : 0) + strides[1] - 1) / strides[1],
                                      pointwiseWeights.dims(3)}});
  }

  void DepthwiseConv2DLayer::calcOutputDimensions(Node& node) const
  {
    ASSERT(node.inputDimensions.size() == 1);
    ASSERT(node.inputDimensions[0].size() == 3);
    ASSERT(padding == PaddingType::same || node.inputDimensions[0][0] >= weights.dims(0));
    ASSERT(padding == PaddingType::same || node.inputDimensions[0][1] >= weights.dims(1));
    ASSERT(node.inputDimensions[0][2] == weights.dims(2));
    node.outputDimensions.push_back({{(node.inputDimensions[0][0] - (padding == PaddingType::valid ? weights.dims(0) - 1 : 0) + strides[0] - 1) / strides[0],
                                      (node.inputDimensions[0][1] - (padding == PaddingType::valid ? weights.dims(1) - 1 : 0) + strides[1] - 1) / strides[1],
                                      node.inputDimensions[0][2] * weights.dims(3)}});
  }

  void Cropping2DLayer::calcOutputDimensions(Node& node) const
  {
    ASSERT(node.inputDimensions.size() == 1);
    ASSERT(node.inputDimensions[0].size() == 3);
    node.outputDimensions = node.inputDimensions;
    node.outputDimensions[0][0] -= cropping[TOP] + cropping[BOTTOM];
    node.outputDimensions[0][1] -= cropping[LEFT] + cropping[RIGHT];
  }

  void UpSampling2DLayer::calcOutputDimensions(Node& node) const
  {
    ASSERT(node.inputDimensions.size() == 1);
    ASSERT(node.inputDimensions[0].size() == 3);
    node.outputDimensions = node.inputDimensions;
    node.outputDimensions[0][0] *= size[0];
    node.outputDimensions[0][1] *= size[1];
  }

  void ZeroPadding2DLayer::calcOutputDimensions(Node& node) const
  {
    ASSERT(node.inputDimensions.size() == 1);
    ASSERT(node.inputDimensions[0].size() == 3);
    node.outputDimensions = node.inputDimensions;
    node.outputDimensions[0][0] += padding[TOP] + padding[BOTTOM];
    node.outputDimensions[0][1] += padding[LEFT] + padding[RIGHT];
  }

  void Pooling2DLayer::calcOutputDimensions(Node& node) const
  {
    ASSERT(node.inputDimensions.size() == 1);
    ASSERT(node.inputDimensions[0].size() == 3);
    ASSERT(padding == PaddingType::same || node.inputDimensions[0][0] >= kernelSize[0]);
    ASSERT(padding == PaddingType::same || node.inputDimensions[0][1] >= kernelSize[1]);
    node.outputDimensions.push_back({{(node.inputDimensions[0][0] - (padding == PaddingType::valid ? kernelSize[0] - 1 : 0) + strides[0] - 1) / strides[0],
                                      (node.inputDimensions[0][1] - (padding == PaddingType::valid ? kernelSize[1] - 1 : 0) + strides[1] - 1) / strides[1],
                                      node.inputDimensions[0][2]}});
  }

  void GlobalPooling2DLayer::calcOutputDimensions(Node& node) const
  {
    ASSERT(node.inputDimensions.size() == 1);
    ASSERT(node.inputDimensions[0].size() == 3);
    node.outputDimensions.push_back({node.inputDimensions[0][2]});
  }

  void AddLayer::calcOutputDimensions(Node& node) const
  {
    ASSERT(node.inputDimensions.size() > 1);
#ifndef NDEBUG
    for(std::size_t i = 1; i < node.inputDimensions.size(); ++i)
      ASSERT(node.inputDimensions[i] == node.inputDimensions[0]);
#endif
    node.outputDimensions.push_back(node.inputDimensions[0]);
  }

  void SubtractLayer::calcOutputDimensions(Node& node) const
  {
    ASSERT(node.inputDimensions.size() == 2);
    ASSERT(node.inputDimensions[1] == node.inputDimensions[0]);
    node.outputDimensions.push_back(node.inputDimensions[0]);
  }

  void MultiplyLayer::calcOutputDimensions(Node& node) const
  {
    ASSERT(node.inputDimensions.size() > 1);
#ifndef NDEBUG
    for(std::size_t i = 1; i < node.inputDimensions.size(); ++i)
      ASSERT(node.inputDimensions[i] == node.inputDimensions[0]);
#endif
    node.outputDimensions.push_back(node.inputDimensions[0]);
  }

  void AverageLayer::calcOutputDimensions(Node& node) const
  {
    ASSERT(node.inputDimensions.size() > 1);
#ifndef NDEBUG
    for(std::size_t i = 1; i < node.inputDimensions.size(); ++i)
      ASSERT(node.inputDimensions[i] == node.inputDimensions[0]);
#endif
    node.outputDimensions.push_back(node.inputDimensions[0]);
  }

  void MaximumLayer::calcOutputDimensions(Node& node) const
  {
    ASSERT(node.inputDimensions.size() > 1);
#ifndef NDEBUG
    for(std::size_t i = 1; i < node.inputDimensions.size(); ++i)
      ASSERT(node.inputDimensions[i] == node.inputDimensions[0]);
#endif
    node.outputDimensions.push_back(node.inputDimensions[0]);
  }

  void MinimumLayer::calcOutputDimensions(Node& node) const
  {
    ASSERT(node.inputDimensions.size() > 1);
#ifndef NDEBUG
    for(std::size_t i = 1; i < node.inputDimensions.size(); ++i)
      ASSERT(node.inputDimensions[i] == node.inputDimensions[0]);
#endif
    node.outputDimensions.push_back(node.inputDimensions[0]);
  }

  void ConcatenateLayer::calcOutputDimensions(Node& node) const
  {
    ASSERT(node.inputDimensions.size() > 1);
    std::vector<unsigned int> dimensions = node.inputDimensions[0];
    ASSERT(static_cast<std::size_t>(axis >= 0 ? axis : -(axis + 1)) < dimensions.size());
    std::size_t realAxis = axis >= 0 ? axis : dimensions.size() + axis;
    for(std::size_t i = 1; i < node.inputDimensions.size(); ++i)
    {
      ASSERT(node.inputDimensions[i].size() == dimensions.size());
      for(std::size_t j = 0; j < dimensions.size(); ++j)
      {
        if(j == realAxis)
          dimensions[j] += node.inputDimensions[i][j];
        else
          ASSERT(dimensions[j] == node.inputDimensions[i][j]);
      }
    }
    node.outputDimensions.push_back(dimensions);
  }

  void LeakyReluLayer::calcOutputDimensions(Node& node) const
  {
    if(node.inputDimensions.size() != 1)
      FAIL("LeakyReLU layers must currently have exactly one input tensor.");
    node.outputDimensions = node.inputDimensions;
  }

  void EluLayer::calcOutputDimensions(Node& node) const
  {
    if(node.inputDimensions.size() != 1)
      FAIL("ELU layers must currently have exactly one input tensor.");
    node.outputDimensions = node.inputDimensions;
  }

  void ThresholdedReluLayer::calcOutputDimensions(Node& node) const
  {
    if(node.inputDimensions.size() != 1)
      FAIL("ThresholdedReLU layers must currently have exactly one input tensor.");
    node.outputDimensions = node.inputDimensions;
  }

  void SoftmaxLayer::calcOutputDimensions(Node& node) const
  {
    ASSERT(node.inputDimensions.size() == 1);
    node.outputDimensions = node.inputDimensions;
  }

  void ReluLayer::calcOutputDimensions(Node& node) const
  {
    if(node.inputDimensions.size() != 1)
      FAIL("ReLU layers must currently have exactly one input tensor.");
    node.outputDimensions = node.inputDimensions;
  }

  void BatchNormalizationLayer::calcOutputDimensions(Node& node) const
  {
    ASSERT(node.inputDimensions.size() == 1);
#ifndef NDEBUG
    if(axis >= 0)
    {
      ASSERT(static_cast<std::size_t>(axis) < node.inputDimensions[0].size());
      ASSERT(node.inputDimensions[0][axis] == factor.size());
    }
    else
    {
      ASSERT(static_cast<std::size_t>(-(axis + 1)) < node.inputDimensions[0].size());
      ASSERT(node.inputDimensions[0][node.inputDimensions[0].size() + axis] == factor.size());
    }
#endif
    node.outputDimensions = node.inputDimensions;
  }

  void Model::setInputUInt8(std::size_t index)
  {
    ASSERT(index < inputs.size());
    uint8Inputs.resize(index + 1, false);
    uint8Inputs[index] = true;
  }

  bool Model::isInputUInt8(std::size_t index) const
  {
    return index < uint8Inputs.size() && uint8Inputs[index];
  }

  template<typename T, typename S> inline T readFromFile(S& file)
  {
    T i;
    file.read(reinterpret_cast<char*>(&i), sizeof(T));
    return i;
  }

  template<typename T, typename S> inline void readFromFile(S& file, T* i, const size_t n = 1)
  {
    ASSERT(i);
    file.read(reinterpret_cast<char*>(i), sizeof(T) * n);
  }

  template<typename T, size_t n, typename S> inline void readFromFile(S& file, std::array<T, n>& i)
  {
    file.read(reinterpret_cast<char*>(i.data()), sizeof(T) * n);
  }

  template<typename T, typename S> inline void readFromFile(S& file, std::vector<T>& i)
  {
    file.read(reinterpret_cast<char*>(i.data()), sizeof(T) * i.size());
  }

  void Model::loadKerasify(const std::string& filename)
  {
    clear();

    // Open file
    InBinaryFile file(filename.c_str());
    ASSERT(file.exists());

    // Load test and result tensors
    std::vector<unsigned int> dims(3);
    readFromFile<>(file, dims);
    while(!dims.empty() && dims.back() == 0)
      dims.pop_back();
    ASSERT(!dims.empty());
    testInput.emplace_back(dims);
    readFromFile<float>(file, testInput[0].data(), testInput[0].size());
    dims.resize(3);
    readFromFile<>(file, dims);
    while(!dims.empty() && dims.back() == 0)
      dims.pop_back();
    ASSERT(!dims.empty());
    testResult.emplace_back(dims);
    readFromFile<float>(file, testResult[0].data(), testResult[0].size());

    // Add a "virtual" input layer
    {
      std::unique_ptr<InputLayer> inputLayer = std::make_unique<InputLayer>();
      inputLayer->dimensions = testInput[0].dims();
      inputLayer->nodes.emplace_back(inputLayer.get());
      inputLayer->nodes.back().outputDimensions.push_back(inputLayer->dimensions);
      inputLayer->nodes.back().outputs.emplace_back(inputLayer.get(), 0, 0);
      layers.push_back(std::move(inputLayer));
    }

    // Load layers
    for(unsigned int remainingLayers = readFromFile<unsigned int>(file); remainingLayers; --remainingLayers)
    {
      // Create the new layer
      std::unique_ptr<Layer> layer;
      switch(readFromFile<LayerType>(file))
      {
        case LayerType::dense:
          layer = std::make_unique<DenseLayer>();
          break;
        case LayerType::conv2D:
          layer = std::make_unique<Conv2DLayer>();
          break;
        case LayerType::depthwiseConv2D:
          layer = std::make_unique<DepthwiseConv2DLayer>();
          break;
        case LayerType::zeroPadding2D:
          layer = std::make_unique<ZeroPadding2DLayer>();
          break;
        case LayerType::reshape:
          layer = std::make_unique<ReshapeLayer>();
          break;
        case LayerType::elu:
          layer = std::make_unique<EluLayer>();
          break;
        case LayerType::activation:
          layer = std::make_unique<ActivationLayer>();
          break;
        case LayerType::maxPooling2D:
          layer = std::make_unique<MaxPooling2DLayer>();
          break;
        case LayerType::batchNormalization:
          layer = std::make_unique<BatchNormalizationLayer>();
          break;
        case LayerType::softmax:
          layer = std::make_unique<SoftmaxLayer>();
          break;
        default:
          FAIL("This layer type is not support by the kerasify format.");
      }

      // Add the one and only node to the layer
      ASSERT(!layers.empty());
      ASSERT(layers.back()->nodes.size() == 1);
      ASSERT(layers.back()->nodes[0].outputDimensions.size() == 1);
      layer->nodes.emplace_back(layer.get());
      Node& node = layer->nodes.back();
      node.inputs.emplace_back(layers.back().get(), 0, 0);
      node.inputDimensions.emplace_back(3);
      node.outputDimensions.emplace_back(3);
      node.outputs.emplace_back(layer.get(), 0, 0);

      // Read layer dimensions
      readFromFile<>(file, node.inputDimensions[0]);
      while(!node.inputDimensions[0].empty() && node.inputDimensions[0].back() == 0)
        node.inputDimensions[0].pop_back();
      ASSERT(!node.inputDimensions[0].empty());
      readFromFile<>(file, node.outputDimensions[0]);
      while(!node.outputDimensions[0].empty() && node.outputDimensions[0].back() == 0)
        node.outputDimensions[0].pop_back();
      ASSERT(!node.outputDimensions[0].empty());

      ASSERT(node.inputDimensions[0] == layers.back()->nodes[0].outputDimensions[0]);

      // Read layer parameters
      switch(layer->type)
      {
        case LayerType::dense:
        {
          DenseLayer& l = static_cast<DenseLayer&>(*layer);

          ASSERT(node.inputDimensions[0].size() == 1);
          ASSERT(node.outputDimensions[0].size() == 1);

          // Read weights shape
          std::array<unsigned int, 2> weightsShape;
          readFromFile<>(file, weightsShape);
          ASSERT(weightsShape[0] == node.inputDimensions[0][0]);
          ASSERT(weightsShape[1] == node.outputDimensions[0][0]);
          l.weights.reshape(weightsShape[0], weightsShape[1]);

          // Read biases shape
          const unsigned int biasesShape = readFromFile<unsigned int>(file);
          ASSERT(biasesShape == node.outputDimensions[0][0]);
          l.biases.resize(biasesShape);

          // Read weights and biases
          readFromFile<>(file, l.weights.data(), l.weights.size());
          readFromFile<>(file, l.biases);
          l.hasBiases = true;

          // Read activation function
          l.activationId = readFromFile<ActivationFunctionId>(file);
        }
        break;
        case LayerType::depthwiseConv2D:
        {
          DepthwiseConv2DLayer& l = static_cast<DepthwiseConv2DLayer&>(*layer);

          ASSERT(node.inputDimensions[0].size() == 3);
          ASSERT(node.outputDimensions[0].size() == 3);

          // Read weights shape
          std::vector<unsigned int> shape(4);
          readFromFile<>(file, shape);
          l.weights.reshape(shape);

          // Read strides
          readFromFile<>(file, l.strides);
          ASSERT(l.strides[0] > 0 && l.strides[1] > 0);

          // Read padding type
          l.padding = readFromFile<PaddingType>(file);

          // Read weights
          readFromFile(file, l.weights.data(), l.weights.size());

          l.activationId = ActivationFunctionId::linear;
          l.hasBiases = false;
        }
        break;
        case LayerType::zeroPadding2D:
        {
          ZeroPadding2DLayer& l = static_cast<ZeroPadding2DLayer&>(*layer);

          ASSERT(node.inputDimensions[0].size() == 3);
          ASSERT(node.outputDimensions[0].size() == 3);

          readFromFile<>(file, l.padding);
        }
        break;
        case LayerType::conv2D:
        {
          Conv2DLayer& l = static_cast<Conv2DLayer&>(*layer);

          ASSERT(node.inputDimensions[0].size() == 3);
          ASSERT(node.outputDimensions[0].size() == 3);

          // Read weights shape
          std::vector<unsigned int> shape(4);
          readFromFile<>(file, shape);
          ASSERT(shape[2] == node.inputDimensions[0][2]);
          ASSERT(shape[3] == node.outputDimensions[0][2]);
          ASSERT(shape[0] * shape[1] * shape[2] * shape[3] > 0);
          l.weights.reshape(shape);

          // Read biases shape
          const unsigned int biasesShape = readFromFile<unsigned int>(file);
          ASSERT(biasesShape == node.outputDimensions[0][2]);
          l.biases.resize(biasesShape);

          // Read strides
          readFromFile<>(file, l.strides);
          ASSERT(l.strides[0] > 0 && l.strides[1] > 0);

          // Read padding type
          l.padding = readFromFile<PaddingType>(file);

          // Read weights
          readFromFile(file, l.weights.data(), l.weights.size());

          // Read biases
          readFromFile<>(file, l.biases);
          l.hasBiases = true;

          // Read activation function
          l.activationId = readFromFile<ActivationFunctionId>(file);
        }
        break;
        case LayerType::reshape:
          break;
        case LayerType::elu:
        {
          EluLayer& l = static_cast<EluLayer&>(*layer);
          l.alpha = readFromFile<float>(file);
        }
        break;
        case LayerType::activation:
        {
          ActivationLayer& l = static_cast<ActivationLayer&>(*layer);
          l.activationId = readFromFile<ActivationFunctionId>(file);
        }
        break;
        case LayerType::maxPooling2D:
        {
          Pooling2DLayer& l = static_cast<Pooling2DLayer&>(*layer);

          ASSERT(node.inputDimensions[0].size() == 3);
          ASSERT(node.outputDimensions[0].size() == 3);

          l.method = readFromFile<PoolingMethod>(file);
          if(l.method == PoolingMethod::average)
            const_cast<LayerType&>(l.type) = LayerType::averagePooling2D;
          readFromFile<>(file, l.kernelSize);
          readFromFile<>(file, l.strides);
          l.padding = readFromFile<PaddingType>(file);

          ASSERT(l.method == PoolingMethod::average || l.method == PoolingMethod::max);
          ASSERT(node.inputDimensions[0][2] == node.outputDimensions[0][2]);
        }
        break;
        case LayerType::batchNormalization:
        {
          BatchNormalizationLayer& l = static_cast<BatchNormalizationLayer&>(*layer);

          // Read input size
          const unsigned int inputSize = readFromFile<unsigned int>(file);
          ASSERT(inputSize > 0);

          // Determine dimension that is normalized
          int d = static_cast<int>(node.inputDimensions[0].size() - 1);
          for(; d >= 0; d--)
            if(node.inputDimensions[0][d] == inputSize)
              break;
          ASSERT(d >= 0);
          l.axis = d;

          // Read normalization parameters
          const float epsilon = readFromFile<float>(file);
          std::vector<float> gamma, beta, mean, variance;
          gamma.resize(inputSize);
          beta.resize(inputSize);
          mean.resize(inputSize);
          variance.resize(inputSize);
          readFromFile<>(file, gamma);
          readFromFile<>(file, beta);
          readFromFile<>(file, mean);
          readFromFile<>(file, variance);

          // Calculate a factor and an offset from the parameters
          l.factor.resize(inputSize);
          l.offset.resize(inputSize);
          for(unsigned int i = 0; i < inputSize; i++)
          {
            l.factor[i] = gamma[i] / std::sqrt(variance[i] + epsilon);
            l.offset[i] = beta[i] - mean[i] * l.factor[i];
          }
        }
        break;
        case LayerType::softmax:
          break;
        default:
          ASSERT(false);
      }

      // Add layer to the vector
      layers.push_back(std::move(layer));
    }

    inputs.emplace_back(layers.front().get(), 0, 0);
    outputs.emplace_back(layers.back().get(), 0, 0);
  }

  template<typename T>
  const T* getRecordEntry(const SimpleMap::Record* record, const std::string& name)
  {
    SimpleMap::Record::const_iterator iter = record->find(name);
    ASSERT(iter != record->end());
    const T* result = dynamic_cast<const T*>(iter->second);
    ASSERT(result);
    return result;
  }

  template<typename T>
  const T* getArrayEntry(const SimpleMap::Array* array, std::size_t index)
  {
    const T* result = dynamic_cast<const T*>((*array)[index]);
    ASSERT(result);
    return result;
  }

  template<typename T>
  T getLiteral(const SimpleMap::Literal* literal)
  {
    T result;
    literal->operator In&() >> result;
    return result;
  }

  ActivationFunctionId parseActivation(const std::string& activation)
  {
    if(activation == "linear")
      return ActivationFunctionId::linear;
    else if(activation == "relu")
      return ActivationFunctionId::relu;
    else if(activation == "sigmoid")
      return ActivationFunctionId::sigmoid;
    else if(activation == "tanh")
      return ActivationFunctionId::tanH;
    else if(activation == "hard_sigmoid")
      return ActivationFunctionId::hardSigmoid;
    else if(activation == "softmax")
      return ActivationFunctionId::softmax;
    else if(activation == "elu")
      return ActivationFunctionId::elu;
    else if(activation == "selu")
      return ActivationFunctionId::selu;
    else if(activation == "exponential")
      return ActivationFunctionId::exponential;
    else if(activation == "softsign")
      return ActivationFunctionId::softsign;
    FAIL("The activation function \"" << activation << "\" is currently not implemented.");
    return ActivationFunctionId::linear;
  }

  PaddingType parsePadding(const std::string& padding)
  {
    if(padding == "valid")
      return PaddingType::valid;
    else if(padding == "same")
      return PaddingType::same;
    FAIL("The padding type \"" << padding << "\" is currently not implemented.");
    return PaddingType::valid;
  }

  InterpolationMethod parseInterpolation(const std::string& interpolation)
  {
    if(interpolation == "nearest")
      return InterpolationMethod::nearest;
    else if(interpolation == "bilinear")
      return InterpolationMethod::bilinear;
    FAIL("The interpolation method \"" << interpolation << "\" is currently not implemented.");
    return InterpolationMethod::nearest;
  }

  std::unique_ptr<Layer> parseInputLayer(const SimpleMap::Record* config, const Model::GetWeights2FuncType&)
  {
    const SimpleMap::Array* batchInputShape = getRecordEntry<SimpleMap::Array>(config, "batch_input_shape");
    const std::string dtype = getLiteral<std::string>(getRecordEntry<SimpleMap::Literal>(config, "dtype"));
    const bool sparse = getLiteral<bool>(getRecordEntry<SimpleMap::Literal>(config, "sparse"));

    if(dtype != "float32")
      FAIL("The datatype of the model input must be float32.");
    if(sparse)
      FAIL("Sparse inputs are not supported.");
    if(batchInputShape->size() < 2)
      FAIL("The input of a model must have at least 1 dimension (excluding the batch axis).");
    if(getLiteral<std::string>(getArrayEntry<SimpleMap::Literal>(batchInputShape, 0)) != "null")
      FAIL("The batch axis must be null.");

    std::unique_ptr<InputLayer> layer = std::make_unique<InputLayer>();
    layer->dimensions.resize(batchInputShape->size() - 1);
#ifndef NDEBUG
    std::size_t outputSize = 1;
#endif
    for(std::size_t i = 0; i < batchInputShape->size() - 1; ++i)
    {
      layer->dimensions[i] = getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(batchInputShape, i + 1));
#ifndef NDEBUG
      outputSize *= layer->dimensions[i];
#endif
    }
    ASSERT(outputSize > 0);

    // Special case for input layers: There is a "virtual" node without inputs.
    layer->nodes.emplace_back(layer.get());
    layer->nodes.back().outputDimensions.push_back(layer->dimensions);
    layer->nodes.back().outputs.emplace_back(layer.get(), 0, 0);

    return layer;
  }

  std::unique_ptr<Layer> parseDenseLayer(const SimpleMap::Record* config, const Model::GetWeights2FuncType& getWeights)
  {
#ifndef NDEBUG
    const unsigned int units = getLiteral<unsigned int>(getRecordEntry<SimpleMap::Literal>(config, "units"));
#endif
    const std::string activation = getLiteral<std::string>(getRecordEntry<SimpleMap::Literal>(config, "activation"));
    const bool useBias = getLiteral<bool>(getRecordEntry<SimpleMap::Literal>(config, "use_bias"));

    ASSERT(units > 0);

    std::unique_ptr<DenseLayer> layer = std::make_unique<DenseLayer>();
    layer->hasBiases = useBias;
    layer->activationId = parseActivation(activation);

    std::vector<float> weights;
    std::vector<unsigned int> dimensions;
    getWeights("kernel", weights, dimensions);
    ASSERT(dimensions.size() == 2);
    ASSERT(dimensions[1] == units);
    layer->weights.reshape(dimensions[0], dimensions[1]);
    std::copy(weights.begin(), weights.end(), layer->weights.begin());
    if(useBias)
    {
      getWeights("bias", weights, dimensions);
      ASSERT(dimensions.size() == 1);
      ASSERT(dimensions[0] == units);
      layer->biases = weights;
    }
    return layer;
  }

  std::unique_ptr<Layer> parseActivationLayer(const SimpleMap::Record* config, const Model::GetWeights2FuncType&)
  {
    const std::string activation = getLiteral<std::string>(getRecordEntry<SimpleMap::Literal>(config, "activation"));

    std::unique_ptr<ActivationLayer> layer = std::make_unique<ActivationLayer>();
    layer->activationId = parseActivation(activation);
    return layer;
  }

  std::unique_ptr<Layer> parseDropoutLayer(const SimpleMap::Record*, const Model::GetWeights2FuncType&)
  {
    return std::make_unique<DropoutLayer>();
  }

  std::unique_ptr<Layer> parseFlattenLayer(const SimpleMap::Record* config, const Model::GetWeights2FuncType&)
  {
    const std::string dataFormat = getLiteral<std::string>(getRecordEntry<SimpleMap::Literal>(config, "data_format"));
    if(dataFormat != "channels_last")
      FAIL("Data formats other than channels last are not supported.");

    return std::make_unique<FlattenLayer>();
  }

  std::unique_ptr<Layer> parseReshapeLayer(const SimpleMap::Record* config, const Model::GetWeights2FuncType&)
  {
    const SimpleMap::Array* targetShape = getRecordEntry<SimpleMap::Array>(config, "target_shape");
    ASSERT(!targetShape->empty());

    std::unique_ptr<ReshapeLayer> layer = std::make_unique<ReshapeLayer>();
    layer->dimensions.resize(targetShape->size());
#ifndef NDEBUG
    std::size_t outputSize = 1;
#endif
    for(std::size_t i = 0; i < targetShape->size(); ++i)
    {
      const int dim = getLiteral<int>(getArrayEntry<SimpleMap::Literal>(targetShape, i));
      if(dim == -1)
        FAIL("Shape inference of Reshape layers is not supported.");
      ASSERT(dim > 0);
      layer->dimensions[i] = static_cast<unsigned int>(dim);
#ifndef NDEBUG
      outputSize *= layer->dimensions[i];
#endif
    }
    ASSERT(outputSize > 0);
    return layer;
  }

  std::unique_ptr<Layer> parseConv2DLayer(const SimpleMap::Record* config, const Model::GetWeights2FuncType& getWeights)
  {
#ifndef NDEBUG
    const unsigned int filters = getLiteral<unsigned int>(getRecordEntry<SimpleMap::Literal>(config, "filters"));
    const SimpleMap::Array* kernelSize = getRecordEntry<SimpleMap::Array>(config, "kernel_size");
#endif
    const SimpleMap::Array* strides = getRecordEntry<SimpleMap::Array>(config, "strides");
    const std::string padding = getLiteral<std::string>(getRecordEntry<SimpleMap::Literal>(config, "padding"));
    const std::string dataFormat = getLiteral<std::string>(getRecordEntry<SimpleMap::Literal>(config, "data_format"));
    const SimpleMap::Array* dilationRate = getRecordEntry<SimpleMap::Array>(config, "dilation_rate");
    const std::string activation = getLiteral<std::string>(getRecordEntry<SimpleMap::Literal>(config, "activation"));
    const bool useBias = getLiteral<bool>(getRecordEntry<SimpleMap::Literal>(config, "use_bias"));

    ASSERT(filters > 0);
    ASSERT(kernelSize->size() == 2);
    ASSERT(strides->size() == 2);
    ASSERT(dilationRate->size() == 2);
    if(dataFormat != "channels_last")
      FAIL("Data formats other than channels last are not supported.");
    if(getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(dilationRate, 0)) != 1 ||
       getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(dilationRate, 1)) != 1)
      FAIL("Conv2D layers with a dilation rate other than (1, 1) are currently not supported.");
#ifndef NDEBUG
    const unsigned int kernelHeight = getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(kernelSize, 0));
    const unsigned int kernelWidth = getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(kernelSize, 1));
#endif
    const unsigned int strideVertical = getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(strides, 0));
    const unsigned int strideHorizontal = getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(strides, 1));
    ASSERT(kernelHeight > 0);
    ASSERT(kernelWidth > 0);
    ASSERT(strideVertical > 0);
    ASSERT(strideHorizontal > 0);

    std::unique_ptr<Conv2DLayer> layer = std::make_unique<Conv2DLayer>();
    layer->strides[0] = strideVertical;
    layer->strides[1] = strideHorizontal;
    layer->activationId = parseActivation(activation);
    layer->padding = parsePadding(padding);
    layer->hasBiases = useBias;

    std::vector<float> weights;
    std::vector<unsigned int> dimensions;
    getWeights("kernel", weights, dimensions);
    ASSERT(dimensions.size() == 4);
    ASSERT(dimensions[0] == kernelHeight);
    ASSERT(dimensions[1] == kernelWidth);
    ASSERT(dimensions[2] > 0);
    ASSERT(dimensions[3] == filters);
    layer->weights.reshape(dimensions[0], dimensions[1], dimensions[2], dimensions[3]);
    std::copy(weights.begin(), weights.end(), layer->weights.begin());
    if(useBias)
    {
      getWeights("bias", weights, dimensions);
      ASSERT(dimensions.size() == 1);
      ASSERT(dimensions[0] == filters);
      layer->biases = weights;
    }
    return layer;
  }

  std::unique_ptr<Layer> parseSeparableConv2DLayer(const SimpleMap::Record* config, const Model::GetWeights2FuncType& getWeights)
  {
#ifndef NDEBUG
    const unsigned int filters = getLiteral<unsigned int>(getRecordEntry<SimpleMap::Literal>(config, "filters"));
    const SimpleMap::Array* kernelSize = getRecordEntry<SimpleMap::Array>(config, "kernel_size");
#endif
    const SimpleMap::Array* strides = getRecordEntry<SimpleMap::Array>(config, "strides");
    const std::string padding = getLiteral<std::string>(getRecordEntry<SimpleMap::Literal>(config, "padding"));
    const std::string dataFormat = getLiteral<std::string>(getRecordEntry<SimpleMap::Literal>(config, "data_format"));
    const SimpleMap::Array* dilationRate = getRecordEntry<SimpleMap::Array>(config, "dilation_rate");
#ifndef NDEBUG
    const unsigned int depthMultiplier = getLiteral<unsigned int>(getRecordEntry<SimpleMap::Literal>(config, "depth_multiplier"));
#endif
    const std::string activation = getLiteral<std::string>(getRecordEntry<SimpleMap::Literal>(config, "activation"));
    const bool useBias = getLiteral<bool>(getRecordEntry<SimpleMap::Literal>(config, "use_bias"));

    ASSERT(filters > 0);
    ASSERT(kernelSize->size() == 2);
    ASSERT(strides->size() == 2);
    ASSERT(dilationRate->size() == 2);
    ASSERT(depthMultiplier > 0);
    if(dataFormat != "channels_last")
      FAIL("Data formats other than channels last are not supported.");
    if(getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(dilationRate, 0)) != 1 ||
       getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(dilationRate, 1)) != 1)
      FAIL("SeparableConv2D layers with a dilation rate other than (1, 1) are currently not supported.");
#ifndef NDEBUG
    const unsigned int kernelHeight = getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(kernelSize, 0));
    const unsigned int kernelWidth = getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(kernelSize, 1));
#endif
    const unsigned int strideVertical = getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(strides, 0));
    const unsigned int strideHorizontal = getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(strides, 1));
    ASSERT(kernelHeight > 0);
    ASSERT(kernelWidth > 0);
    ASSERT(strideVertical > 0);
    ASSERT(strideHorizontal > 0);

    std::unique_ptr<SeparableConv2DLayer> layer = std::make_unique<SeparableConv2DLayer>();
    layer->strides[0] = strideVertical;
    layer->strides[1] = strideHorizontal;
    layer->activationId = parseActivation(activation);
    layer->padding = parsePadding(padding);
    layer->hasBiases = useBias;

    std::vector<float> weights;
    std::vector<unsigned int> dimensions;
    getWeights("depthwise_kernel", weights, dimensions);
    ASSERT(dimensions.size() == 4);
    ASSERT(dimensions[0] == kernelHeight);
    ASSERT(dimensions[1] == kernelWidth);
    ASSERT(dimensions[2] > 0);
    ASSERT(dimensions[3] == depthMultiplier);
    layer->depthwiseWeights.reshape(dimensions[0], dimensions[1], dimensions[2], dimensions[3]);
    std::copy(weights.begin(), weights.end(), layer->depthwiseWeights.begin());
    getWeights("pointwise_kernel", weights, dimensions);
    ASSERT(dimensions.size() == 4);
    ASSERT(dimensions[0] == 1);
    ASSERT(dimensions[1] == 1);
    ASSERT(dimensions[2] == layer->depthwiseWeights.dims(2) * depthMultiplier);
    ASSERT(dimensions[3] == filters);
    layer->pointwiseWeights.reshape(dimensions[0], dimensions[1], dimensions[2], dimensions[3]);
    std::copy(weights.begin(), weights.end(), layer->pointwiseWeights.begin());
    if(useBias)
    {
      getWeights("bias", weights, dimensions);
      ASSERT(dimensions.size() == 1);
      ASSERT(dimensions[0] == filters);
      layer->biases = weights;
    }
    return layer;
  }

  std::unique_ptr<Layer> parseDepthwiseConv2DLayer(const SimpleMap::Record* config, const Model::GetWeights2FuncType& getWeights)
  {
#ifndef NDEBUG
    const SimpleMap::Array* kernelSize = getRecordEntry<SimpleMap::Array>(config, "kernel_size");
#endif
    const SimpleMap::Array* strides = getRecordEntry<SimpleMap::Array>(config, "strides");
    const std::string padding = getLiteral<std::string>(getRecordEntry<SimpleMap::Literal>(config, "padding"));
    const std::string dataFormat = getLiteral<std::string>(getRecordEntry<SimpleMap::Literal>(config, "data_format"));
    const SimpleMap::Array* dilationRate = getRecordEntry<SimpleMap::Array>(config, "dilation_rate");
#ifndef NDEBUG
    const unsigned int depthMultiplier = getLiteral<unsigned int>(getRecordEntry<SimpleMap::Literal>(config, "depth_multiplier"));
#endif
    const std::string activation = getLiteral<std::string>(getRecordEntry<SimpleMap::Literal>(config, "activation"));
    const bool useBias = getLiteral<bool>(getRecordEntry<SimpleMap::Literal>(config, "use_bias"));

    ASSERT(kernelSize->size() == 2);
    ASSERT(strides->size() == 2);
    ASSERT(dilationRate->size() == 2);
    ASSERT(depthMultiplier > 0);
    if(dataFormat != "channels_last")
      FAIL("Data formats other than channels last are not supported.");
    if(getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(dilationRate, 0)) != 1 ||
       getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(dilationRate, 1)) != 1)
      FAIL("DepthwiseConv2D layers with a dilation rate other than (1, 1) are currently not supported.");
#ifndef NDEBUG
    const unsigned int kernelHeight = getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(kernelSize, 0));
    const unsigned int kernelWidth = getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(kernelSize, 1));
#endif
    const unsigned int strideVertical = getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(strides, 0));
    const unsigned int strideHorizontal = getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(strides, 1));
    ASSERT(kernelHeight > 0);
    ASSERT(kernelWidth > 0);
    ASSERT(strideVertical > 0);
    ASSERT(strideHorizontal > 0);

    std::unique_ptr<DepthwiseConv2DLayer> layer = std::make_unique<DepthwiseConv2DLayer>();
    layer->strides[0] = strideVertical;
    layer->strides[1] = strideHorizontal;
    layer->activationId = parseActivation(activation);
    layer->padding = parsePadding(padding);
    layer->hasBiases = useBias;

    std::vector<float> weights;
    std::vector<unsigned int> dimensions;
    getWeights("depthwise_kernel", weights, dimensions);
    ASSERT(dimensions.size() == 4);
    ASSERT(dimensions[0] == kernelHeight);
    ASSERT(dimensions[1] == kernelWidth);
    ASSERT(dimensions[2] > 0);
    ASSERT(dimensions[3] == depthMultiplier);
    layer->weights.reshape(dimensions[0], dimensions[1], dimensions[2], dimensions[3]);
    std::copy(weights.begin(), weights.end(), layer->weights.begin());
    if(useBias)
    {
      getWeights("bias", weights, dimensions);
      ASSERT(dimensions.size() == 1);
      ASSERT(dimensions[0] == layer->weights.dims(2) * depthMultiplier);
      layer->biases = weights;
    }
    return layer;
  }

  std::unique_ptr<Layer> parseCropping2DLayer(const SimpleMap::Record* config, const Model::GetWeights2FuncType&)
  {
    const SimpleMap::Array* cropping = getRecordEntry<SimpleMap::Array>(config, "cropping");
    const std::string dataFormat = getLiteral<std::string>(getRecordEntry<SimpleMap::Literal>(config, "data_format"));

    if(dataFormat != "channels_last")
      FAIL("Data formats other than channels last are not supported.");
    ASSERT(cropping->size() == 2);
    const SimpleMap::Array* heightCropping = getArrayEntry<SimpleMap::Array>(cropping, 0);
    ASSERT(heightCropping->size() == 2);
    const SimpleMap::Array* widthCropping = getArrayEntry<SimpleMap::Array>(cropping, 1);
    ASSERT(widthCropping->size() == 2);
    const unsigned int topCropping = getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(heightCropping, 0));
    const unsigned int bottomCropping = getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(heightCropping, 1));
    const unsigned int leftCropping = getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(widthCropping, 0));
    const unsigned int rightCropping = getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(widthCropping, 1));

    std::unique_ptr<Cropping2DLayer> layer = std::make_unique<Cropping2DLayer>();
    layer->cropping[Cropping2DLayer::TOP] = topCropping;
    layer->cropping[Cropping2DLayer::BOTTOM] = bottomCropping;
    layer->cropping[Cropping2DLayer::LEFT] = leftCropping;
    layer->cropping[Cropping2DLayer::RIGHT] = rightCropping;
    return layer;
  }

  std::unique_ptr<Layer> parseUpSampling2DLayer(const SimpleMap::Record* config, const Model::GetWeights2FuncType&)
  {
    const SimpleMap::Array* size = getRecordEntry<SimpleMap::Array>(config, "size");
    const std::string dataFormat = getLiteral<std::string>(getRecordEntry<SimpleMap::Literal>(config, "data_format"));
    const std::string interpolation = getLiteral<std::string>(getRecordEntry<SimpleMap::Literal>(config, "interpolation"));

    if(dataFormat != "channels_last")
      FAIL("Data formats other than channels last are not supported.");
    ASSERT(size->size() == 2);
    const unsigned int sizeVertical = getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(size, 0));
    const unsigned int sizeHorizontal = getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(size, 1));
    ASSERT(sizeVertical > 0);
    ASSERT(sizeHorizontal > 0);

    std::unique_ptr<UpSampling2DLayer> layer = std::make_unique<UpSampling2DLayer>();
    layer->size[0] = sizeVertical;
    layer->size[1] = sizeHorizontal;
    layer->interpolation = parseInterpolation(interpolation);
    return layer;
  }

  std::unique_ptr<Layer> parseZeroPadding2DLayer(const SimpleMap::Record* config, const Model::GetWeights2FuncType&)
  {
    const SimpleMap::Array* padding = getRecordEntry<SimpleMap::Array>(config, "padding");
    const std::string dataFormat = getLiteral<std::string>(getRecordEntry<SimpleMap::Literal>(config, "data_format"));

    if(dataFormat != "channels_last")
      FAIL("Data formats other than channels last are not supported.");
    ASSERT(padding->size() == 2);
    const SimpleMap::Array* heightPadding = getArrayEntry<SimpleMap::Array>(padding, 0);
    ASSERT(heightPadding->size() == 2);
    const SimpleMap::Array* widthPadding = getArrayEntry<SimpleMap::Array>(padding, 1);
    ASSERT(widthPadding->size() == 2);
    const unsigned int topPadding = getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(heightPadding, 0));
    const unsigned int bottomPadding = getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(heightPadding, 1));
    const unsigned int leftPadding = getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(widthPadding, 0));
    const unsigned int rightPadding = getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(widthPadding, 1));

    std::unique_ptr<ZeroPadding2DLayer> layer = std::make_unique<ZeroPadding2DLayer>();
    layer->padding[ZeroPadding2DLayer::TOP] = topPadding;
    layer->padding[ZeroPadding2DLayer::BOTTOM] = bottomPadding;
    layer->padding[ZeroPadding2DLayer::LEFT] = leftPadding;
    layer->padding[ZeroPadding2DLayer::RIGHT] = rightPadding;
    return layer;
  }

  std::unique_ptr<Layer> parsePooling2DLayer(const SimpleMap::Record* config, PoolingMethod method)
  {
    const SimpleMap::Array* poolSize = getRecordEntry<SimpleMap::Array>(config, "pool_size");
    const std::string padding = getLiteral<std::string>(getRecordEntry<SimpleMap::Literal>(config, "padding"));
    const SimpleMap::Array* strides = getRecordEntry<SimpleMap::Array>(config, "strides");
    const std::string dataFormat = getLiteral<std::string>(getRecordEntry<SimpleMap::Literal>(config, "data_format"));

    ASSERT(poolSize->size() == 2);
    ASSERT(strides->size() == 2);
    if(dataFormat != "channels_last")
      FAIL("Data formats other than channels last are not supported.");
    const unsigned int poolVertical = getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(poolSize, 0));
    const unsigned int poolHorizontal = getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(poolSize, 1));
    const unsigned int strideVertical = getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(strides, 0));
    const unsigned int strideHorizontal = getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(strides, 1));
    ASSERT(poolVertical > 0);
    ASSERT(poolHorizontal > 0);
    ASSERT(strideVertical > 0);
    ASSERT(strideHorizontal > 0);

    std::unique_ptr<Pooling2DLayer> layer = std::make_unique<Pooling2DLayer>(method == PoolingMethod::max ? LayerType::maxPooling2D : LayerType::averagePooling2D, method);
    layer->method = method;
    layer->padding = parsePadding(padding);
    layer->kernelSize[0] = poolVertical;
    layer->kernelSize[1] = poolHorizontal;
    layer->strides[0] = strideVertical;
    layer->strides[1] = strideHorizontal;
    return layer;
  }

  std::unique_ptr<Layer> parseMaxPooling2DLayer(const SimpleMap::Record* config, const Model::GetWeights2FuncType&)
  {
    return parsePooling2DLayer(config, PoolingMethod::max);
  }

  std::unique_ptr<Layer> parseAveragePooling2DLayer(const SimpleMap::Record* config, const Model::GetWeights2FuncType&)
  {
    return parsePooling2DLayer(config, PoolingMethod::average);
  }

  std::unique_ptr<Layer> parseGlobalPooling2DLayer(const SimpleMap::Record* config, PoolingMethod method)
  {
    const std::string dataFormat = getLiteral<std::string>(getRecordEntry<SimpleMap::Literal>(config, "data_format"));

    if(dataFormat != "channels_last")
      FAIL("Data formats other than channels last are not supported.");

    std::unique_ptr<GlobalPooling2DLayer> layer = std::make_unique<GlobalPooling2DLayer>(method == PoolingMethod::max ? LayerType::globalMaxPooling2D : LayerType::globalAveragePooling2D, method);
    layer->method = method;
    return layer;
  }

  std::unique_ptr<Layer> parseGlobalMaxPooling2DLayer(const SimpleMap::Record* config, const Model::GetWeights2FuncType&)
  {
    return parseGlobalPooling2DLayer(config, PoolingMethod::max);
  }

  std::unique_ptr<Layer> parseGlobalAveragePooling2DLayer(const SimpleMap::Record* config, const Model::GetWeights2FuncType&)
  {
    return parseGlobalPooling2DLayer(config, PoolingMethod::average);
  }

  std::unique_ptr<Layer> parseAddLayer(const SimpleMap::Record*, const Model::GetWeights2FuncType&)
  {
    std::unique_ptr<AddLayer> layer = std::make_unique<AddLayer>();
    return layer;
  }

  std::unique_ptr<Layer> parseSubtractLayer(const SimpleMap::Record*, const Model::GetWeights2FuncType&)
  {
    std::unique_ptr<SubtractLayer> layer = std::make_unique<SubtractLayer>();
    return layer;
  }

  std::unique_ptr<Layer> parseMultiplyLayer(const SimpleMap::Record*, const Model::GetWeights2FuncType&)
  {
    std::unique_ptr<MultiplyLayer> layer = std::make_unique<MultiplyLayer>();
    return layer;
  }

  std::unique_ptr<Layer> parseAverageLayer(const SimpleMap::Record*, const Model::GetWeights2FuncType&)
  {
    std::unique_ptr<AverageLayer> layer = std::make_unique<AverageLayer>();
    return layer;
  }

  std::unique_ptr<Layer> parseMaximumLayer(const SimpleMap::Record*, const Model::GetWeights2FuncType&)
  {
    std::unique_ptr<MaximumLayer> layer = std::make_unique<MaximumLayer>();
    return layer;
  }

  std::unique_ptr<Layer> parseMinimumLayer(const SimpleMap::Record*, const Model::GetWeights2FuncType&)
  {
    std::unique_ptr<MinimumLayer> layer = std::make_unique<MinimumLayer>();
    return layer;
  }

  std::unique_ptr<Layer> parseConcatenateLayer(const SimpleMap::Record* config, const Model::GetWeights2FuncType&)
  {
    const int axis = getLiteral<int>(getRecordEntry<SimpleMap::Literal>(config, "axis"));
    ASSERT(axis != 0);

    std::unique_ptr<ConcatenateLayer> layer = std::make_unique<ConcatenateLayer>();
    layer->axis = axis > 0 ? axis - 1 : axis; // Remove batch axis.
    return layer;
  }

  std::unique_ptr<Layer> parseLeakyReluLayer(const SimpleMap::Record* config, const Model::GetWeights2FuncType&)
  {
    const float alpha = getLiteral<float>(getRecordEntry<SimpleMap::Literal>(config, "alpha"));
    ASSERT(alpha >= 0.f);

    std::unique_ptr<LeakyReluLayer> layer = std::make_unique<LeakyReluLayer>();
    layer->alpha = alpha;
    return layer;
  }

  std::unique_ptr<Layer> parseEluLayer(const SimpleMap::Record* config, const Model::GetWeights2FuncType&)
  {
    const float alpha = getLiteral<float>(getRecordEntry<SimpleMap::Literal>(config, "alpha"));

    std::unique_ptr<EluLayer> layer = std::make_unique<EluLayer>();
    layer->alpha = alpha;
    return layer;
  }

  std::unique_ptr<Layer> parseThresholdedReluLayer(const SimpleMap::Record* config, const Model::GetWeights2FuncType&)
  {
    const float theta = getLiteral<float>(getRecordEntry<SimpleMap::Literal>(config, "theta"));
    ASSERT(theta >= 0.f);

    std::unique_ptr<ThresholdedReluLayer> layer = std::make_unique<ThresholdedReluLayer>();
    layer->theta = theta;
    return layer;
  }

  std::unique_ptr<Layer> parseSoftmaxLayer(const SimpleMap::Record* config, const Model::GetWeights2FuncType&)
  {
    const int axis = getLiteral<int>(getRecordEntry<SimpleMap::Literal>(config, "axis"));
    ASSERT(axis != 0);

    std::unique_ptr<SoftmaxLayer> layer = std::make_unique<SoftmaxLayer>();
    layer->axis = axis > 0 ? axis - 1 : axis; // Remove batch axis.
    return layer;
  }

  std::unique_ptr<Layer> parseReluLayer(const SimpleMap::Record* config, const Model::GetWeights2FuncType&)
  {
    const float maxValue = getLiteral<std::string>(getRecordEntry<SimpleMap::Literal>(config, "max_value")) == "None"
                           ? std::numeric_limits<float>::max()
                           : getLiteral<float>(getRecordEntry<SimpleMap::Literal>(config, "max_value"));
    const float negativeSlope = getLiteral<float>(getRecordEntry<SimpleMap::Literal>(config, "negative_slope"));
    const float threshold = getLiteral<float>(getRecordEntry<SimpleMap::Literal>(config, "threshold"));

    std::unique_ptr<ReluLayer> layer = std::make_unique<ReluLayer>();
    layer->maxValue = maxValue;
    layer->negativeSlope = negativeSlope;
    layer->threshold = threshold;
    return layer;
  }

  std::unique_ptr<Layer> parseBatchNormalizationLayer(const SimpleMap::Record* config, const Model::GetWeights2FuncType& getWeights)
  {
    const int axis = getLiteral<int>(getRecordEntry<SimpleMap::Literal>(config, "axis"));
    const float epsilon = getLiteral<float>(getRecordEntry<SimpleMap::Literal>(config, "epsilon"));
    const bool center = getLiteral<bool>(getRecordEntry<SimpleMap::Literal>(config, "center"));
    const bool scale = getLiteral<bool>(getRecordEntry<SimpleMap::Literal>(config, "scale"));

    ASSERT(axis != 0);

    std::unique_ptr<BatchNormalizationLayer> layer = std::make_unique<BatchNormalizationLayer>();
    layer->axis = axis > 0 ? axis - 1 : axis; // Remove batch axis.

    std::vector<float> weights;
    std::vector<unsigned int> dimensions;
    getWeights("moving_variance", weights, dimensions);
    ASSERT(dimensions.size() == 1);
    layer->factor.resize(dimensions[0]);
    for(unsigned int i = 0; i < dimensions[0]; ++i)
      layer->factor[i] = 1.f / std::sqrt(weights[i] + epsilon);
    if(scale)
    {
      getWeights("gamma", weights, dimensions);
      ASSERT(dimensions.size() == 1);
      ASSERT(dimensions[0] == layer->factor.size());
      for(unsigned int i = 0; i < dimensions[0]; ++i)
        layer->factor[i] *= weights[i];
    }
    getWeights("moving_mean", weights, dimensions);
    ASSERT(dimensions.size() == 1);
    ASSERT(dimensions[0] == layer->factor.size());
    layer->offset.resize(dimensions[0]);
    for(unsigned int i = 0; i < dimensions[0]; ++i)
      layer->offset[i] = -weights[i] * layer->factor[i];
    if(center)
    {
      getWeights("beta", weights, dimensions);
      ASSERT(dimensions.size() == 1);
      ASSERT(dimensions[0] == layer->offset.size());
      for(unsigned int i = 0; i < dimensions[0]; ++i)
        layer->offset[i] += weights[i];
    }
    return layer;
  }

  void Model::parseJSONModel(In& stream, const std::string& fileName, const GetWeightsFuncType& getWeights)
  {
    // This function uses the following convention:
    // ASSERTs are used if the model is invalid (i.e. has not been exported correctly or with an incompatible version of keras).
    // FAILs are used if the model is valid, but uses a feature that is currently not supported.
    // Have fun eliminating all the FAILs ;-)

    SimpleMap map(stream, fileName, /* jsonMode: */ true);
    const SimpleMap::Record* root = dynamic_cast<const SimpleMap::Record*>(map.operator const SimpleMap::Value*());
    ASSERT(root);
    const SimpleMap::Record* config = getRecordEntry<SimpleMap::Record>(root, "config");
    const SimpleMap::Array* layers = getRecordEntry<SimpleMap::Array>(config, "layers");
    const SimpleMap::Array* inputLayers = getRecordEntry<SimpleMap::Array>(config, "input_layers");
    const SimpleMap::Array* outputLayers = getRecordEntry<SimpleMap::Array>(config, "output_layers");

    using ParseLayerFuncType = std::unique_ptr<Layer>(*)(const SimpleMap::Record*, const GetWeights2FuncType&);
    std::unordered_map<std::string, ParseLayerFuncType> layerParsers;
    // Input
    layerParsers.emplace("InputLayer", &parseInputLayer);
    // Core layers
    layerParsers.emplace("Dense", &parseDenseLayer);
    layerParsers.emplace("Activation", &parseActivationLayer);
    layerParsers.emplace("Dropout", &parseDropoutLayer);
    layerParsers.emplace("Flatten", &parseFlattenLayer);
    layerParsers.emplace("Reshape", &parseReshapeLayer);
    // Convolutional layers
    layerParsers.emplace("Conv2D", &parseConv2DLayer);
    layerParsers.emplace("SeparableConv2D", &parseSeparableConv2DLayer);
    layerParsers.emplace("DepthwiseConv2D", &parseDepthwiseConv2DLayer);
    layerParsers.emplace("Cropping2D", &parseCropping2DLayer);
    layerParsers.emplace("UpSampling2D", &parseUpSampling2DLayer);
    layerParsers.emplace("ZeroPadding2D", &parseZeroPadding2DLayer);
    // Pooling layers
    layerParsers.emplace("MaxPooling2D", &parseMaxPooling2DLayer);
    layerParsers.emplace("AveragePooling2D", &parseAveragePooling2DLayer);
    layerParsers.emplace("GlobalMaxPooling2D", &parseGlobalMaxPooling2DLayer);
    layerParsers.emplace("GlobalAveragePooling2D", &parseGlobalAveragePooling2DLayer);
    // Merge layers
    layerParsers.emplace("Add", &parseAddLayer);
    layerParsers.emplace("Subtract", &parseSubtractLayer);
    layerParsers.emplace("Multiply", &parseMultiplyLayer);
    layerParsers.emplace("Average", &parseAverageLayer);
    layerParsers.emplace("Maximum", &parseMaximumLayer);
    layerParsers.emplace("Minimum", &parseMinimumLayer);
    layerParsers.emplace("Concatenate", &parseConcatenateLayer);
    // Advanced Activation layers
    layerParsers.emplace("LeakyReLU", &parseLeakyReluLayer);
    layerParsers.emplace("ELU", &parseEluLayer);
    layerParsers.emplace("ThresholdedReLU", &parseThresholdedReluLayer);
    layerParsers.emplace("Softmax", &parseSoftmaxLayer);
    layerParsers.emplace("ReLU", &parseReluLayer);
    // Normalization layers
    layerParsers.emplace("BatchNormalization", &parseBatchNormalizationLayer);

    // This code is very much inspired by the original keras `Network.from_config` method:
    // https://github.com/keras-team/keras/blob/d78c982b326adeed6ac25200dc6892ff8f518ca6/keras/engine/network.py#L933
    std::unordered_map<std::string, std::unique_ptr<Layer>> createdLayers;
    std::unordered_map<std::string, std::vector<const SimpleMap::Array*>> unprocessedNodes;
    // It first instantiates all layers and adds all their nodes (each layer can have multiple nodes) to the unprocessed map.
    for(const SimpleMap::Value* value : *layers)
    {
      const SimpleMap::Record* layer = dynamic_cast<const SimpleMap::Record*>(value);
      ASSERT(layer);

      // A layer is an object with four members: name (a string, identifies the layer in the model),
      // class_name (a string, identifies the layer type), config (an object with layer-specific parameters),
      // and inbound_nodes (an array of nodes, which are in turn arrays).
      const std::string name = getLiteral<std::string>(getRecordEntry<SimpleMap::Literal>(layer, "name"));
      const std::string layerType = getLiteral<std::string>(getRecordEntry<SimpleMap::Literal>(layer, "class_name"));

      // Create a new layer with the parsing function for this type.
      auto it = layerParsers.find(layerType);
      if(it == layerParsers.end())
        FAIL("The layer type \"" << layerType << "\" is currently not implemented.");
      std::unique_ptr<Layer> newLayer = it->second(getRecordEntry<SimpleMap::Record>(layer, "config"),
                                                   std::bind(getWeights, name, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

      // Input layers are special because they have no explicit nodes, but an implicit one that is already created during layer creation.
      // Thus, they should not have any additional nodes.
      if(newLayer->type == LayerType::input && !getRecordEntry<SimpleMap::Array>(layer, "inbound_nodes")->empty())
        FAIL("Input layers that are called directly (i.e. `InputLayer(...)()` instead of `Input(...)`) are not supported.");
      createdLayers[name] = std::move(newLayer);

      // Add all inbound nodes of this layer to its unprocessed nodes array.
      for(const SimpleMap::Value* node : *getRecordEntry<SimpleMap::Array>(layer, "inbound_nodes"))
        unprocessedNodes[name].push_back(dynamic_cast<const SimpleMap::Array*>(node));
    }

    // After that, all nodes are processed (i.e. linked with their predecessors).
    // This code ensures that nodes are only created after all their inputs have been created, which is necessary to infer their input/output shapes.
    // It is also (probably) important that the layers array is processed in the same order as in the JSON file, thus the createdLayers map is not used.
    while(!unprocessedNodes.empty())
      for(const SimpleMap::Value* value : *layers)
      {
        const SimpleMap::Record* layer = dynamic_cast<const SimpleMap::Record*>(value);
        ASSERT(layer);
        const std::string name = getLiteral<std::string>(getRecordEntry<SimpleMap::Literal>(layer, "name"));

        // If all nodes of this layer have been created, go to the next one.
        auto it = unprocessedNodes.find(name);
        if(it == unprocessedNodes.end())
          continue;

        std::size_t i;
        for(i = 0; i < it->second.size(); ++i)
        {
          const SimpleMap::Array* node = it->second[i];
          ASSERT(node);

          // A node in this array is represented as an array of its inputs.
          // Each input is also an array with 3 or four elements: layer name (a string), the index of the node in the layer (a number),
          // the index of the tensor in the node (a number), and optionally a map of keyword arguments (an object).
          std::vector<TensorLocation> inputTensors;
          std::size_t j;
          for(j = 0; j < node->size(); ++j)
          {
            const SimpleMap::Array* input = getArrayEntry<SimpleMap::Array>(node, j);
            ASSERT(input);
            ASSERT(input->size() == 3 || input->size() == 4);
            const std::string inboundLayerName = getLiteral<std::string>(getArrayEntry<SimpleMap::Literal>(input, 0));
            const unsigned int inboundNodeIndex = getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(input, 1));
            const unsigned int inboundTensorIndex = getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(input, 2));
            if(input->size() > 3 && !getArrayEntry<SimpleMap::Record>(input, 3)->empty())
              FAIL("Keyword arguments of nodes are currently not supported.");
            const Layer* inboundLayer = createdLayers[inboundLayerName].get();
            // Check if the inbound node has been already created.
            // This can be checked that way since the nodes must be created in the order of their indices.
            // In case it has not been created, this node cannot be created.
            if(inboundLayer->nodes.size() <= inboundNodeIndex)
              break;
            inputTensors.emplace_back(inboundLayer, inboundNodeIndex, inboundTensorIndex);
          }
          if(j < node->size())
            break;
          else if(!inputTensors.empty())
          {
            Layer* layer = createdLayers[name].get();
            // Create a new node in the layer and set its input/output dimensions.
            layer->nodes.emplace_back(layer);
            Node& node = layer->nodes.back();
            node.inputs = inputTensors;
            node.setDimensions();
            node.outputs.reserve(node.outputDimensions.size());
            for(std::size_t k = 0; k < node.outputDimensions.size(); ++k)
              node.outputs.emplace_back(layer, static_cast<unsigned int>(layer->nodes.size() - 1), static_cast<unsigned int>(k));
          }
        }

        // If not all nodes of this layer could be created, only erase the ones which could be created.
        // Otherwise, remove the layer from the map (so the outer loop can terminate in the future).
        if(i < it->second.size())
          it->second.erase(it->second.begin(), it->second.begin() + i);
        else
          unprocessedNodes.erase(it);
      }

    for(std::size_t i = 0; i < inputLayers->size(); ++i)
    {
      const SimpleMap::Array* inputLayer = getArrayEntry<SimpleMap::Array>(inputLayers, i);
      ASSERT(inputLayer->size() == 3);
      const std::string layerName = getLiteral<std::string>(getArrayEntry<SimpleMap::Literal>(inputLayer, 0));
      const unsigned int nodeIndex = getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(inputLayer, 1));
      const unsigned int tensorIndex = getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(inputLayer, 2));
      auto layer = createdLayers.find(layerName);
      ASSERT(layer != createdLayers.end());
      ASSERT(nodeIndex < layer->second->nodes.size());
      ASSERT(tensorIndex < layer->second->nodes[nodeIndex].outputDimensions.size());
      if(layer->second->type != LayerType::input)
        FAIL("Inputs must be outputs of Input layers.");
      inputs.emplace_back(layer->second.get(), nodeIndex, tensorIndex);
    }

    for(std::size_t i = 0; i < outputLayers->size(); ++i)
    {
      const SimpleMap::Array* outputLayer = getArrayEntry<SimpleMap::Array>(outputLayers, i);
      ASSERT(outputLayer->size() == 3);
      const std::string layerName = getLiteral<std::string>(getArrayEntry<SimpleMap::Literal>(outputLayer, 0));
      const unsigned int nodeIndex = getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(outputLayer, 1));
      const unsigned int tensorIndex = getLiteral<unsigned int>(getArrayEntry<SimpleMap::Literal>(outputLayer, 2));
      ASSERT(createdLayers.find(layerName) != createdLayers.end());
      auto layer = createdLayers.find(layerName);
      ASSERT(layer != createdLayers.end());
      ASSERT(nodeIndex < layer->second->nodes.size());
      ASSERT(tensorIndex < layer->second->nodes[nodeIndex].outputDimensions.size());
      outputs.emplace_back(layer->second.get(), nodeIndex, tensorIndex);
    }

    // The layers list should also be in the same order as the list in the JSON file.
    this->layers.reserve(layers->size());
    for(const SimpleMap::Value* value : *layers)
    {
      const SimpleMap::Record* layer = dynamic_cast<const SimpleMap::Record*>(value);
      ASSERT(layer);
      const std::string name = getLiteral<std::string>(getRecordEntry<SimpleMap::Literal>(layer, "name"));
      this->layers.push_back(std::move(createdLayers[name]));
    }
  }

  void Model::loadKerasHDF5(const std::string& filename)
  {
    clear();

    // HDF5 is not necessarily thread-safe.
    static DECLARE_SYNC;
    SYNC;

    H5dont_atexit();
    const std::string fullPath = std::string(File::getBHDir()) + "/Config/" + filename;
    hid_t rootGroup = H5Fopen(fullPath.c_str(), H5F_ACC_RDONLY, H5P_DEFAULT);
    ASSERT(rootGroup >= 0);

    hid_t modelConfigAttribute = H5Aopen(rootGroup, "model_config", H5P_DEFAULT);
    ASSERT(modelConfigAttribute >= 0);

    hid_t modelConfigAttributeDataspace = H5Aget_space(modelConfigAttribute);
    ASSERT(modelConfigAttributeDataspace >= 0);

#ifndef NDEBUG
    ASSERT(H5Sis_simple(modelConfigAttributeDataspace) > 0);
    ASSERT(H5Sget_simple_extent_type(modelConfigAttributeDataspace) == H5S_SCALAR);

    hid_t modelConfigAttributeDatatype = H5Aget_type(modelConfigAttribute);
    ASSERT(H5Tget_class(modelConfigAttributeDatatype) == H5T_STRING);
    ASSERT(H5Tis_variable_str(modelConfigAttributeDatatype) > 0);
    ASSERT(H5Tget_cset(modelConfigAttributeDatatype) == H5T_CSET_ASCII);
    VERIFY(H5Tclose(modelConfigAttributeDatatype) >= 0);
#endif

    hid_t variableLengthStringDatatype = H5Tcopy(H5T_C_S1);
    ASSERT(variableLengthStringDatatype >= 0);
    VERIFY(H5Tset_size(variableLengthStringDatatype, H5T_VARIABLE) >= 0);

    hid_t floatDatatype = H5Tcopy(H5T_IEEE_F32LE);
    ASSERT(floatDatatype >= 0);

    hid_t modelWeightsGroup = H5Gopen2(rootGroup, "model_weights", H5P_DEFAULT);
    ASSERT(modelWeightsGroup >= 0);

    auto getWeights = [modelWeightsGroup, floatDatatype](const std::string& layerName, const std::string& weightName, std::vector<float>& weights, std::vector<unsigned int>& shape)
    {
      hid_t layerGroup = H5Gopen2(modelWeightsGroup, layerName.c_str(), H5P_DEFAULT);
      ASSERT(layerGroup >= 0);

      std::string mangledLayerName = layerName;
      hid_t weightNamesAttribute = H5Aopen(layerGroup, "weight_names", H5P_DEFAULT);
      ASSERT(weightNamesAttribute >= 0);
      hid_t weightNamesAttributeType = H5Aget_type(weightNamesAttribute);
      ASSERT(weightNamesAttributeType >= 0);
      hsize_t weightNamesAttributeSize = H5Aget_storage_size(weightNamesAttribute);
      ASSERT(weightNamesAttributeSize > 0);
      std::vector<char> weightNamesBuf(static_cast<size_t>(weightNamesAttributeSize));
      VERIFY(H5Aread(weightNamesAttribute, weightNamesAttributeType, weightNamesBuf.data()) >= 0);
      size_t weightNameLength = H5Tget_size(weightNamesAttributeType);
      ASSERT(weightNameLength > 0);
      VERIFY(H5Tclose(weightNamesAttributeType) >= 0);
      VERIFY(H5Aclose(weightNamesAttribute) >= 0);
      for(size_t i = 0; i < static_cast<size_t>(weightNamesAttributeSize); i += weightNameLength)
      {
        // This string might end with multiple \0 due to the zero-padding in HDF5, but it does not matter since
        // everything coming after the colon is irrelevant
        const std::string currentWeightLayerName(weightNamesBuf.begin() + i, weightNamesBuf.begin() + i + weightNameLength);
        auto posSlash = currentWeightLayerName.find('/');
        auto posColon = currentWeightLayerName.find(':');
        std::string currentWeightName = currentWeightLayerName.substr(posSlash + 1, posColon - posSlash - 1);
        if(currentWeightName == weightName)
        {
          mangledLayerName = currentWeightLayerName.substr(0, posSlash);
          break;
        }
      }

      hid_t weightsGroup = H5Gopen2(layerGroup, mangledLayerName.c_str(), H5P_DEFAULT);
      ASSERT(weightsGroup >= 0);
      const std::string mangledWeightName = weightName + ":0"; // TODO: Is this always :0? We will see.
      hid_t weightsDataset = H5Dopen2(weightsGroup, mangledWeightName.c_str(), H5P_DEFAULT);
      ASSERT(weightsDataset >= 0);
      hid_t weightsDatasetDataspace = H5Dget_space(weightsDataset);
      ASSERT(weightsDatasetDataspace >= 0);

      ASSERT(H5Sis_simple(weightsDatasetDataspace) > 0);
      ASSERT(H5Sget_simple_extent_type(weightsDatasetDataspace) == H5S_SIMPLE);
      const int ndims = H5Sget_simple_extent_ndims(weightsDatasetDataspace);
      shape.resize(static_cast<std::size_t>(ndims));
      std::vector<hsize_t> shapeAsHSize(static_cast<std::size_t>(ndims));
      VERIFY(H5Sget_simple_extent_dims(weightsDatasetDataspace, shapeAsHSize.data(), nullptr) == ndims);
      std::size_t numOfWeights = 1;
      for(int i = 0; i < ndims; ++i)
      {
        numOfWeights *= static_cast<std::size_t>(shapeAsHSize[i]);
        shape[i] = static_cast<unsigned int>(shapeAsHSize[i]);
      }
      weights.resize(numOfWeights);

      VERIFY(H5Dread(weightsDataset, floatDatatype, H5S_ALL, H5S_ALL, H5P_DEFAULT, weights.data()) >= 0);

      VERIFY(H5Sclose(weightsDatasetDataspace) >= 0);
      VERIFY(H5Dclose(weightsDataset) >= 0);
      VERIFY(H5Gclose(weightsGroup) >= 0);
      VERIFY(H5Gclose(layerGroup) >= 0);
    };

    char* modelConfig = nullptr;
    VERIFY(H5Aread(modelConfigAttribute, variableLengthStringDatatype, &modelConfig) >= 0);
    InBinaryMemory modelConfigStream(modelConfig, std::strlen(modelConfig));
    parseJSONModel(modelConfigStream, fullPath, getWeights);
    H5Dvlen_reclaim(variableLengthStringDatatype, modelConfigAttributeDataspace, H5P_DEFAULT, &modelConfig);

    VERIFY(H5Gclose(modelWeightsGroup) >= 0);
    VERIFY(H5Tclose(floatDatatype) >= 0);
    VERIFY(H5Tclose(variableLengthStringDatatype) >= 0);
    VERIFY(H5Sclose(modelConfigAttributeDataspace) >= 0);
    VERIFY(H5Aclose(modelConfigAttribute) >= 0);
    VERIFY(H5Fclose(rootGroup) >= 0);
  }
}
