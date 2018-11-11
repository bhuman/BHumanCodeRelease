/**
 * Implements a class that applies neural networks on input data without
 * optimizations. Do not use this in production; this class is only meant to
 * verify the results of optimized implementations.
 *
 * @author Felix Thielke
 */

#include "SimpleNN.h"
#include "Platform/BHAssert.h"
#include "Tools/Math/NeumaierSum.h"
#include <cmath>
#include <algorithm>

namespace NeuralNetwork
{
  namespace SimpleNN
  {
    namespace Impl
    {
      float applyActivationFunction(const float v, const ActivationFunctionId fn)
      {
        switch(fn)
        {
          case ActivationFunctionId::linear:
            return v;
          case ActivationFunctionId::relu:
            return std::max(0.f, v);
          case ActivationFunctionId::sigmoid:
            return 1.f / (1.f + std::exp(-v));
          case ActivationFunctionId::tanH:
            return tanhf(v);
          case ActivationFunctionId::hardSigmoid:
            return std::max(0.f, std::min(1.f, 0.2f * v + 0.5f));
          default:
            ASSERT(false);
            return 0.f;
        }
      }

      void apply(const Tensor3& input, Tensor3& output, const DenseLayer& layer)
      {
        for(unsigned int o = 0; o < output.dims(0); o++)
        {
          NeumaierSum<float> v(input(0, 0, 0) * layer.weights(0, o));
          for(unsigned int i = 1; i < input.dims(0); i++)
            v += input(i, 0, 0) * layer.weights(i, o);
          output(o, 0, 0) = applyActivationFunction(static_cast<float>(v) + layer.biases[o], layer.activationId);
        }
      }

      void apply(const Tensor3& input, Tensor3& output, const Conv2DLayer& layer)
      {
        const unsigned int paddingTop = layer.padding == PaddingType::valid ? 0 : ((layer.outputDimensions[0] - 1) * layer.strides[0] + layer.weights.dims(0) - layer.inputDimensions[0]) / 2;
        const unsigned int paddingLeft = layer.padding == PaddingType::valid ? 0 : ((layer.outputDimensions[1] - 1) * layer.strides[1] + layer.weights.dims(1) - layer.inputDimensions[1]) / 2;

        unsigned int outputY = 0;
        for(int y = -static_cast<int>(paddingTop); outputY < output.dims(0) ; y += layer.strides[0], outputY++)
        {
          unsigned int outputX = 0;
          for(int x = -static_cast<int>(paddingLeft); outputX < output.dims(1); x += layer.strides[1], outputX++)
          {
            std::vector<NeumaierSum<float>> outputChannels(output.dims(2));

            for(unsigned int filterY = 0; filterY < layer.weights.dims(0); filterY++)
            {
              for(unsigned int filterX = 0; filterX < layer.weights.dims(1); filterX++)
              {
                const int yIndex = y + filterY;
                const int xIndex = x + filterX;

                if(yIndex >= 0 && static_cast<unsigned int>(yIndex) < input.dims(0) && xIndex >= 0 && static_cast<unsigned int>(xIndex) < input.dims(1))
                {
                  for(unsigned int inputChannel = 0; inputChannel < input.dims(2); inputChannel++)
                  {
                    const float v = input(yIndex, xIndex, inputChannel);
                    for(unsigned int outputChannel = 0; outputChannel < output.dims(2); outputChannel++)
                      outputChannels[outputChannel] += layer.weights(filterY, filterX, inputChannel, outputChannel) * v;
                  }
                }
              }
            }

            for(unsigned int outputChannel = 0; outputChannel < output.dims(2); outputChannel++)
              output(outputY, outputX, outputChannel) = applyActivationFunction(static_cast<float>(outputChannels[outputChannel] += layer.biases[outputChannel]), layer.activationId);
          }
        }
      }

      void apply(const Tensor3& input, Tensor3& output, const SConv2DLayer& layer)
      {
        const unsigned int paddingTop = layer.padding == PaddingType::valid ? 0 : ((layer.outputDimensions[0] - 1) * layer.strides[0] + layer.weights.dims(0) - layer.inputDimensions[0]) / 2;
        const unsigned int paddingLeft = layer.padding == PaddingType::valid ? 0 : ((layer.outputDimensions[1] - 1) * layer.strides[1] + layer.weights.dims(1) - layer.inputDimensions[1]) / 2;

        unsigned int outputY = 0;
        for(int y = -static_cast<int>(paddingTop); outputY < output.dims(0); y += layer.strides[0], outputY++)
        {
          unsigned int outputX = 0;
          for(int x = -static_cast<int>(paddingLeft); outputX < output.dims(1); x += layer.strides[1], outputX++)
          {
            std::vector<NeumaierSum<float>> outputChannels(output.dims(2));

            for(unsigned int filterY = 0; filterY < layer.weights.dims(0); filterY++)
            {
              for(unsigned int filterX = 0; filterX < layer.weights.dims(1); filterX++)
              {
                const int yIndex = y + filterY;
                const int xIndex = x + filterX;

                if(yIndex >= 0 && static_cast<unsigned int>(yIndex) < input.dims(0) && xIndex >= 0 && static_cast<unsigned int>(xIndex) < input.dims(1))
                {
                  for(unsigned int inputChannel = 0; inputChannel < input.dims(2); inputChannel++)
                  {
                    const float v = input(yIndex, xIndex, inputChannel);
                    outputChannels[inputChannel] += layer.weights(filterY, filterX, inputChannel, 0) * v;
                  }
                }
              }
            }

            for(unsigned int outputChannel = 0; outputChannel < output.dims(2); outputChannel++)
              output(outputY, outputX, outputChannel) = outputChannels[outputChannel];
          }
        }
      }

      void apply(const Tensor3& input, Tensor3& output, const ReshapeLayer& layer)
      {
        output.copyFrom(input);
      }

      void apply(const Tensor3& input, Tensor3& output, const EluLayer& layer)
      {
        float* out = output.data();
        for(const float* in = input.data(); in < input.data() + input.size(); in++, out++)
        {
          const float v = *in;
          *out = v < 0.f ? layer.alpha * (std::exp(v) - 1.f) : v;
        }
      }

      void apply(const Tensor3& input, Tensor3& output, const ActivationLayer& layer)
      {
        float* out = output.data();
        for(const float* in = input.data(); in < input.data() + input.size(); in++, out++)
          *out = applyActivationFunction(*in, layer.activationId);
      }

      void apply(const Tensor3& input, Tensor3& output, const Pooling2DLayer& layer)
      {
        const unsigned int paddingTop = layer.padding == PaddingType::valid ? 0 : ((layer.outputDimensions[0] - 1) * layer.strides[0] + layer.kernelSize[0] - layer.inputDimensions[0]) / 2;
        const unsigned int paddingLeft = layer.padding == PaddingType::valid ? 0 : ((layer.outputDimensions[1] - 1) * layer.strides[1] + layer.kernelSize[1] - layer.inputDimensions[1]) / 2;

        const float filterSize = static_cast<float>(layer.kernelSize[0] * layer.kernelSize[1]);

        unsigned int outputY = 0;
        for(int y = -static_cast<int>(paddingTop); outputY < output.dims(0); y += layer.strides[0], outputY++)
        {
          unsigned int outputX = 0;
          for(int x = -static_cast<int>(paddingLeft); outputX < output.dims(1); x += layer.strides[1], outputX++)
          {
            if(y >= 0 && static_cast<unsigned int>(y) < input.dims(0) && x >= 0 && static_cast<unsigned int>(x) < input.dims(1) && layer.method == Pooling2DLayer::PoolingMethod::max)
            {
              for(unsigned int channel = 0; channel < output.dims(2); channel++)
                output(outputY, outputX, channel) = input(y, x, channel);
            }
            else if(layer.method == Pooling2DLayer::PoolingMethod::max)
            {
              for(unsigned int channel = 0; channel < output.dims(2); channel++)
                output(outputY, outputX, channel) = 0.f;
            }

            std::vector<NeumaierSum<float>> outputSums(output.dims(2));

            for(unsigned int filterY = 0; filterY < layer.kernelSize[0]; filterY++)
            {
              for(unsigned int filterX = 0; filterX < layer.kernelSize[1]; filterX++)
              {
                for(unsigned int channel = 0; channel < output.dims(2); channel++)
                {
                  const int yIndex = y + filterY;
                  const int xIndex = x + filterX;
                  if(yIndex >= 0 && static_cast<unsigned int>(yIndex) < input.dims(0) && xIndex >= 0 && static_cast<unsigned int>(xIndex) < input.dims(1))
                  {
                    if(layer.method == Pooling2DLayer::PoolingMethod::average)
                      outputSums[channel] += input(yIndex, xIndex, channel);
                    else if(layer.method == Pooling2DLayer::PoolingMethod::max)
                      output(outputY, outputX, channel) = std::max(output(outputY, outputX, channel), input(yIndex, xIndex, channel));
                    else
                      ASSERT(false);
                  }
                }
              }
            }

            if(layer.method == Pooling2DLayer::PoolingMethod::average)
            {
              for(unsigned int outputChannel = 0; outputChannel < output.dims(2); outputChannel++)
                output(outputY, outputX, outputChannel) = static_cast<float>(outputSums[outputChannel]) / filterSize;
            }
          }
        }
      }

      void apply(const Tensor3& input, Tensor3& output, const BatchNormalizationLayer& layer)
      {
        std::array<unsigned int, 3> i;

        for(i[0] = 0; i[0] < input.dims(0); i[0]++)
          for(i[1] = 0; i[1] < input.dims(1); i[1]++)
            for(i[2] = 0; i[2] < input.dims(2); i[2]++)
              output(i) = input(i) * layer.factor[i[layer.dimension]] + layer.offset[i[layer.dimension]];
      }

      void applySoftmax(const Tensor3& input, Tensor3& output)
      {
        NeumaierSum<float> sum = 0;
        float* out = output.data();
        for(const float in : input)
        {
          const float v = std::exp(in);
          sum += v;
          *out++ = v;
        }

        for(float& v : output)
          v /= static_cast<float>(sum);
      }
    }

    void apply(const Tensor3& input, Tensor3& output, const Layer& layer)
    {
      // Create output tensor
      output.reshape(layer.outputDimensions);

      // Apply layer
      switch(layer.type)
      {
        case LayerType::dense:
          Impl::apply(input, output, static_cast<const DenseLayer&>(layer));
          break;
        case LayerType::conv2D:
          Impl::apply(input, output, static_cast<const Conv2DLayer&>(layer));
          break;
        case LayerType::sconv2D:
          Impl::apply(input, output, static_cast<const SConv2DLayer&>(layer));
          break;
        case LayerType::reshape:
          Impl::apply(input, output, static_cast<const ReshapeLayer&>(layer));
          break;
        case LayerType::elu:
          Impl::apply(input, output, static_cast<const EluLayer&>(layer));
          break;
        case LayerType::activation:
          Impl::apply(input, output, static_cast<const ActivationLayer&>(layer));
          break;
        case LayerType::pooling2D:
          Impl::apply(input, output, static_cast<const Pooling2DLayer&>(layer));
          break;
        case LayerType::batchNormalization:
          Impl::apply(input, output, static_cast<const BatchNormalizationLayer&>(layer));
          break;
        case LayerType::softmax:
          Impl::applySoftmax(input, output);
          break;
      }
    }

    void apply(Tensor3& input, Tensor3& output, const Model& model)
    {
      bool flip = false;

      for(const auto& layer : model.getLayers())
      {
        if(flip)
          apply(output, input, *layer);
        else
          apply(input, output, *layer);

        flip = !flip;
      }

      if(!flip)
        output = input;
    }
  }
}
