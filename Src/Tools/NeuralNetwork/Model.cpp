/**
 * Implements a method to load neural network models from a file.
 *
 * @author Felix Thielke
 */

#include "Model.h"
#include "Tools/Streams/InStreams.h"
#include <cmath>
#include <utility>

namespace NeuralNetwork
{
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

  void Model::load(const std::string& filename)
  {
    layers.clear();

    // Open file
    InBinaryFile file(filename.c_str());
    ASSERT(file.exists());

    // Load test and result tensors
    std::array<unsigned int, 3> dims;
    readFromFile<>(file, dims);
    testInput.reshape(dims);
    readFromFile<float>(file, testInput.data(), testInput.size());
    readFromFile<>(file, dims);
    testResult.reshape(dims);
    readFromFile<float>(file, testResult.data(), testResult.size());

    // Load layers
    for(unsigned int remainingLayers = readFromFile<unsigned int>(file); remainingLayers; --remainingLayers)
    {
      // Create the new layer
      std::unique_ptr<Layer> layer, additionalLayer;
      switch(readFromFile<LayerType>(file))
      {
        case LayerType::dense:
          layer = std::make_unique<DenseLayer>();
          break;
        case LayerType::conv2D:
          layer = std::make_unique<Conv2DLayer>();
          break;
        case LayerType::sconv2D:
          layer = std::make_unique<SConv2DLayer>();
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
        case LayerType::pooling2D:
          layer = std::make_unique<Pooling2DLayer>();
          break;
        case LayerType::batchNormalization:
          layer = std::make_unique<BatchNormalizationLayer>();
          break;
        case LayerType::softmax:
          layer = std::make_unique<SoftmaxLayer>();
          break;
        default:
          ASSERT(false);
      }

      // Read layer dimensions
      readFromFile<>(file, layer->inputDimensions);
      readFromFile<>(file, layer->outputDimensions);

      // Read layer parameters
      switch(layer->type)
      {
        case LayerType::dense:
        {
          DenseLayer& l = static_cast<DenseLayer&>(*layer);

          ASSERT(l.inputDimensions[1] == 1);
          ASSERT(l.inputDimensions[2] == 1);
          ASSERT(l.outputDimensions[1] == 1);
          ASSERT(l.outputDimensions[2] == 1);

          // Read weights shape
          std::array<unsigned int, 2> weightsShape;
          readFromFile<>(file, weightsShape);
          ASSERT(weightsShape[0] == l.inputDimensions[0]);
          ASSERT(weightsShape[1] == l.outputDimensions[0]);
          l.weights.reshape(weightsShape[0], weightsShape[1]);

          // Read biases shape
          const unsigned int biasesShape = readFromFile<unsigned int>(file);
          ASSERT(biasesShape == l.outputDimensions[0]);
          l.biases.resize(biasesShape);

          // Read weights and biases
          readFromFile<>(file, l.weights.data(), l.weights.size());
          readFromFile<>(file, l.biases);

          // Read activation function
          l.activationId = readFromFile<ActivationFunctionId>(file);
          if(l.activationId == ActivationFunctionId::softmax || l.activationId == ActivationFunctionId::elu)
          {
            l.activationId = ActivationFunctionId::linear;

            if(l.activationId == ActivationFunctionId::elu)
            {
              additionalLayer = std::make_unique<EluLayer>();
              static_cast<EluLayer*>(additionalLayer.get())->alpha = 1.f;
            }
            else
              additionalLayer = std::make_unique<SoftmaxLayer>();
            additionalLayer->outputDimensions = additionalLayer->inputDimensions = layer->inputDimensions;
          }
        }
        break;
        case LayerType::sconv2D:
        {
          Conv2DLayer& l = static_cast<Conv2DLayer&>(*layer);

          // Read weights shape
          std::array<unsigned int, 4> shape;
          readFromFile<>(file, shape);
          //ASSERT(shape[2] == l.inputDimensions[2]);
          //ASSERT(shape[3] == l.outputDimensions[2]);
          //ASSERT(shape[0] * shape[1] * shape[2] * shape[3] > 0);
          l.weights.reshape(shape);

          // Read strides
          readFromFile<>(file, l.strides);
          ASSERT(l.strides[0] > 0 && l.strides[1] > 0);

          // Read padding type
          l.padding = readFromFile<PaddingType>(file);

          // Read weights
          readFromFile(file, l.weights.data(), l.weights.size());
        }
        break;
        case LayerType::conv2D:
        {
          Conv2DLayer& l = static_cast<Conv2DLayer&>(*layer);

          // Read weights shape
          std::array<unsigned int, 4> shape;
          readFromFile<>(file, shape);
          //ASSERT(shape[2] == l.inputDimensions[2]);
          //ASSERT(shape[3] == l.outputDimensions[2]);
          //ASSERT(shape[0] * shape[1] * shape[2] * shape[3] > 0);
          l.weights.reshape(shape);

          // Read biases shape
          const unsigned int biasesShape = readFromFile<unsigned int>(file);
          ASSERT(biasesShape == l.outputDimensions[2]);
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

          // Read activation function
          l.activationId = readFromFile<ActivationFunctionId>(file);
          if(l.activationId == ActivationFunctionId::softmax || l.activationId == ActivationFunctionId::elu)
          {
            l.activationId = ActivationFunctionId::linear;

            if(l.activationId == ActivationFunctionId::elu)
            {
              additionalLayer = std::make_unique<EluLayer>();
              static_cast<EluLayer*>(additionalLayer.get())->alpha = 1.f;
            }
            else
              additionalLayer = std::make_unique<SoftmaxLayer>();
            additionalLayer->outputDimensions = additionalLayer->inputDimensions = layer->inputDimensions;
          }

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

          if(l.activationId == ActivationFunctionId::softmax || l.activationId == ActivationFunctionId::elu)
          {
            std::unique_ptr<Layer> newLayer;

            if(l.activationId == ActivationFunctionId::elu)
            {
              newLayer = std::make_unique<EluLayer>();
              static_cast<EluLayer*>(newLayer.get())->alpha = 1.f;
            }
            else
              newLayer = std::make_unique<SoftmaxLayer>();
            newLayer->inputDimensions = layer->inputDimensions;
            newLayer->outputDimensions = layer->outputDimensions;
            layer.reset(newLayer.release());
          }
        }
        break;
        case LayerType::pooling2D:
        {
          Pooling2DLayer& l = static_cast<Pooling2DLayer&>(*layer);
          l.method = readFromFile<Pooling2DLayer::PoolingMethod>(file);
          readFromFile<>(file, l.kernelSize);
          readFromFile<>(file, l.strides);
          l.padding = readFromFile<PaddingType>(file);

          ASSERT(l.method == Pooling2DLayer::PoolingMethod::average || l.method == Pooling2DLayer::PoolingMethod::max);
          ASSERT(l.inputDimensions[2] == l.outputDimensions[2]);
        }
        break;
        case LayerType::batchNormalization:
        {
          BatchNormalizationLayer& l = static_cast<BatchNormalizationLayer&>(*layer);

          // Read input size
          l.inputSize = readFromFile<unsigned int>(file);
          ASSERT(l.inputSize > 0);

          // Determine dimension that is normalized
          int d = 2;
          for(; d >= 0; d--)
            if(l.inputDimensions[d] == l.inputSize)
              break;
          ASSERT(d >= 0);
          l.dimension = d;

          // Read normalization parameters
          const float epsilon = readFromFile<float>(file);
          std::vector<float> gamma, beta, mean, variance;
          gamma.resize(l.inputSize);
          beta.resize(l.inputSize);
          mean.resize(l.inputSize);
          variance.resize(l.inputSize);
          readFromFile<>(file, gamma);
          readFromFile<>(file, beta);
          readFromFile<>(file, mean);
          readFromFile<>(file, variance);

          // Calculate a factor and an offset from the parameters
          l.factor.resize(l.inputSize);
          l.offset.resize(l.inputSize);
          for(unsigned int i = 0; i < l.inputSize; i++)
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

      // Add layer(s) to the vector
      addLayer(std::move<>(layer));
      if(additionalLayer.get())
        addLayer(std::move<>(additionalLayer));
    }
  }

  std::vector<float> Model::getParameter(bool onlyLast) const
  {
    std::vector<float> parameter;
    for(auto it = layers.rbegin(); it != layers.rend(); it++)
    {
      it++;
      (*it)->getParameter(parameter);
      if(onlyLast)
        break;
    }
    return parameter;
  };

  void Model::setParameter(const std::vector<float>& parameter_, bool onlyLast)
  {
    std::vector<float> parameter = parameter_;
    for(auto it = layers.rbegin(); it != layers.rend(); it++)
    {
      it++;
      (*it)->setParameter(parameter);
      if(onlyLast)
        break;
    }
  }

  void DenseLayer::getParameter(std::vector<float>& parameter) const
  {
    parameter.insert(parameter.end(), weights.begin(), weights.end());
    parameter.insert(parameter.end(), biases.begin(), biases.end());
  }
  void DenseLayer::setParameter(std::vector<float>& parameter)
  {
    biases.assign(parameter.end() - biases.size(), parameter.end());
    parameter.erase(parameter.end() - biases.size(), parameter.end());
    std::copy(parameter.end() - weights.size(), parameter.end(), weights.data());
    parameter.erase(parameter.end() - weights.size(), parameter.end());
  }

  void Conv2DLayer::getParameter(std::vector<float>& parameter) const
  {
    parameter.insert(parameter.end(), weights.begin(), weights.end());
    parameter.insert(parameter.end(), biases.begin(), biases.end());
  }
  void Conv2DLayer::setParameter(std::vector<float>& parameter)
  {
    biases.assign(parameter.end() - biases.size(), parameter.end());
    parameter.erase(parameter.end() - biases.size(), parameter.end());
    std::copy(parameter.end() - weights.size(), parameter.end(), weights.data());
    parameter.erase(parameter.end() - weights.size(), parameter.end());
  }

  void BatchNormalizationLayer::getParameter(std::vector<float>& parameter) const
  {
    parameter.insert(parameter.end(), factor.begin(), factor.end());
    parameter.insert(parameter.end(), offset.begin(), offset.end());
  }
  void BatchNormalizationLayer::setParameter(std::vector<float>& parameter)
  {
    offset.assign(parameter.end() - offset.size(), parameter.end());
    parameter.erase(parameter.end() - offset.size(), parameter.end());
    factor.assign(parameter.end() - factor.size(), parameter.end());
    parameter.erase(parameter.end() - factor.size(), parameter.end());
  }
}
