/**
 * Implements a class that applies neural networks on input data without
 * optimizations. Do not use this in production; this class is only meant to
 * verify the results of optimized implementations.
 *
 * @author Felix Thielke
 * @author Arne Hasselbring
 */

#include "SimpleNN.h"
#include "Platform/BHAssert.h"
#include "Tools/Math/NeumaierSum.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <list>
#include <numeric>

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
            return std::tanh(v);
          case ActivationFunctionId::hardSigmoid:
            return std::max(0.f, std::min(1.f, 0.2f * v + 0.5f));
          case ActivationFunctionId::softmax:
            return std::exp(v);
          case ActivationFunctionId::elu:
            return v < 0.f ? std::exp(v) - 1.f : v;
          case ActivationFunctionId::selu:
            return 1.0507009873554804934193349852946f * (v < 0.f ? 1.6732632423543772848170429916717f * (std::exp(v) - 1.f) : v);
          case ActivationFunctionId::exponential:
            return std::exp(v);
          case ActivationFunctionId::softsign:
            return v / (std::abs(v) + 1.f);
          default:
            ASSERT(false);
            return 0.f;
        }
      }

      void apply(const TensorXf& input, TensorXf& output, const DenseLayer& layer)
      {
        ASSERT(input.rank() == 1);
        ASSERT(output.rank() == 1);

        NeumaierSum<float> softmaxSum = 0.f;
        for(unsigned int o = 0; o < output.dims(0); o++)
        {
          NeumaierSum<float> v(input(0) * layer.weights(0, o));
          for(unsigned int i = 1; i < input.dims(0); i++)
            v += input(i) * layer.weights(i, o);
          output(o) = applyActivationFunction(layer.hasBiases ? static_cast<float>(v += layer.biases[o]) : static_cast<float>(v), layer.activationId);
          if(layer.activationId == ActivationFunctionId::softmax)
            softmaxSum += output(o);
        }
        if(layer.activationId == ActivationFunctionId::softmax)
          for(float* out = output.data(); out < output.data() + output.size(); out++)
            *out /= softmaxSum;
      }

      void apply(const TensorXf& input, TensorXf& output, const ActivationLayer& layer)
      {
        if(layer.activationId != ActivationFunctionId::softmax)
        {
          float* out = output.data();
          for(const float* in = input.data(); in < input.data() + input.size(); in++, out++)
            *out = applyActivationFunction(*in, layer.activationId);
        }
        else
        {
          const unsigned int softmaxSize = input.dims().back();
          const float* in = input.data();
          float* out = output.data();
          for(std::size_t i = 0; i < output.size(); ++i)
          {
            NeumaierSum<float> softmaxSum = 0.f;
            float* sumOut = out;
            do
            {
              *out = applyActivationFunction(*in++, ActivationFunctionId::softmax);
              softmaxSum += *out++;
            }
            while(++i % softmaxSize);
            while(sumOut < out)
              *sumOut++ /= softmaxSum;
          }
        }
      }

      void apply(const std::vector<const TensorXf*>& input, std::vector<TensorXf*>& output, const DropoutLayer& layer)
      {
        for(std::size_t i = 0; i < input.size(); ++i)
          output[i]->copyFrom(*input[i]);
      }

      void apply(const TensorXf& input, TensorXf& output, const FlattenLayer& layer)
      {
        output.copyFrom(input);
      }

      void apply(const TensorXf& input, TensorXf& output, const ReshapeLayer& layer)
      {
        output.copyFrom(input);
      }

      void apply(const TensorXf& input, TensorXf& output, const Conv2DLayer& layer)
      {
        ASSERT(input.rank() == 3);
        ASSERT(output.rank() == 3);

        const unsigned int paddingTop = layer.padding == PaddingType::valid ? 0 : ((output.dims(0) - 1) * layer.strides[0] + layer.weights.dims(0) - input.dims(0)) / 2;
        const unsigned int paddingLeft = layer.padding == PaddingType::valid ? 0 : ((output.dims(1) - 1) * layer.strides[1] + layer.weights.dims(1) - input.dims(1)) / 2;

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
                    for(unsigned int outputChannel = 0; outputChannel < output.dims(2); outputChannel++)
                      outputChannels[outputChannel] += layer.weights(filterY, filterX, inputChannel, outputChannel) * v;
                  }
                }
              }
            }

            std::vector<NeumaierSum<float>> softmaxSums(output.dims(2));
            for(unsigned int outputChannel = 0; outputChannel < output.dims(2); outputChannel++)
            {
              output(outputY, outputX, outputChannel) = applyActivationFunction(layer.hasBiases ? static_cast<float>(outputChannels[outputChannel] += layer.biases[outputChannel])
                                                        : static_cast<float>(outputChannels[outputChannel]), layer.activationId);
              if(layer.activationId == ActivationFunctionId::softmax)
                softmaxSums[outputChannel] += output(outputY, outputX, outputChannel);
            }
            if(layer.activationId == ActivationFunctionId::softmax)
              for(unsigned int outputChannel = 0; outputChannel < output.dims(2); outputChannel++)
                output(outputY, outputX, outputChannel) /= softmaxSums[outputChannel];
          }
        }
      }

      void apply(const TensorXf& input, TensorXf& output, const SeparableConv2DLayer& layer)
      {
        ASSERT(input.rank() == 3);
        ASSERT(output.rank() == 3);

        TensorXf depthwiseOutput({(input.dims(0) - (layer.padding == PaddingType::valid ? layer.depthwiseWeights.dims(0) - 1 : 0) + layer.strides[0] - 1) / layer.strides[0],
                                  (input.dims(1) - (layer.padding == PaddingType::valid ? layer.depthwiseWeights.dims(1) - 1 : 0) + layer.strides[1] - 1) / layer.strides[1],
                                  input.dims(2) * layer.depthwiseWeights.dims(3)});

        const unsigned int paddingTop = layer.padding == PaddingType::valid ? 0 : ((depthwiseOutput.dims(0) - 1) * layer.strides[0] + layer.depthwiseWeights.dims(0) - input.dims(0)) / 2;
        const unsigned int paddingLeft = layer.padding == PaddingType::valid ? 0 : ((depthwiseOutput.dims(1) - 1) * layer.strides[1] + layer.depthwiseWeights.dims(1) - input.dims(1)) / 2;

        unsigned int outputY = 0;
        for(int y = -static_cast<int>(paddingTop); outputY < depthwiseOutput.dims(0); y += layer.strides[0], outputY++)
        {
          unsigned int outputX = 0;
          for(int x = -static_cast<int>(paddingLeft); outputX < depthwiseOutput.dims(1); x += layer.strides[1], outputX++)
          {
            std::vector<NeumaierSum<float>> outputChannels(input.dims(2) * layer.depthwiseWeights.dims(3));

            for(unsigned int filterY = 0; filterY < layer.depthwiseWeights.dims(0); filterY++)
            {
              for(unsigned int filterX = 0; filterX < layer.depthwiseWeights.dims(1); filterX++)
              {
                const int yIndex = y + filterY;
                const int xIndex = x + filterX;

                if(yIndex >= 0 && static_cast<unsigned int>(yIndex) < input.dims(0) && xIndex >= 0 && static_cast<unsigned int>(xIndex) < input.dims(1))
                {
                  for(unsigned int inputChannel = 0; inputChannel < input.dims(2); inputChannel++)
                  {
                    const float v = input(yIndex, xIndex, inputChannel);
                    for(unsigned int outputChannel = 0; outputChannel < layer.depthwiseWeights.dims(3); outputChannel++)
                      outputChannels[inputChannel * layer.depthwiseWeights.dims(3) + outputChannel] += layer.depthwiseWeights(filterY, filterX, inputChannel, outputChannel) * v;
                  }
                }
              }
            }

            for(unsigned int outputChannel = 0; outputChannel < depthwiseOutput.dims(2); outputChannel++)
              depthwiseOutput(outputY, outputX, outputChannel) = static_cast<float>(outputChannels[outputChannel]);
          }
        }

        for(unsigned int y = 0; y < output.dims(0); y++)
        {
          for(unsigned int x = 0; x < output.dims(1); x++)
          {
            std::vector<NeumaierSum<float>> outputChannels(output.dims(2));

            for(unsigned int inputChannel = 0; inputChannel < depthwiseOutput.dims(2); inputChannel++)
            {
              const float v = depthwiseOutput(y, x, inputChannel);
              for(unsigned int outputChannel = 0; outputChannel < output.dims(2); outputChannel++)
                outputChannels[outputChannel] += layer.pointwiseWeights(0, 0, inputChannel, outputChannel) * v;
            }

            std::vector<NeumaierSum<float>> softmaxSums(output.dims(2));
            for(unsigned int outputChannel = 0; outputChannel < output.dims(2); outputChannel++)
            {
              output(y, x, outputChannel) = applyActivationFunction(layer.hasBiases ? static_cast<float>(outputChannels[outputChannel] += layer.biases[outputChannel])
                                                                    : static_cast<float>(outputChannels[outputChannel]), layer.activationId);
              if(layer.activationId == ActivationFunctionId::softmax)
                softmaxSums[outputChannel] += output(y, x, outputChannel);
            }
            if(layer.activationId == ActivationFunctionId::softmax)
              for(unsigned int outputChannel = 0; outputChannel < output.dims(2); outputChannel++)
                output(y, x, outputChannel) /= softmaxSums[outputChannel];
          }
        }
      }

      void apply(const TensorXf& input, TensorXf& output, const DepthwiseConv2DLayer& layer)
      {
        ASSERT(input.rank() == 3);
        ASSERT(output.rank() == 3);

        const unsigned int paddingTop = layer.padding == PaddingType::valid ? 0 : ((output.dims(0) - 1) * layer.strides[0] + layer.weights.dims(0) - input.dims(0)) / 2;
        const unsigned int paddingLeft = layer.padding == PaddingType::valid ? 0 : ((output.dims(1) - 1) * layer.strides[1] + layer.weights.dims(1) - input.dims(1)) / 2;

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
                    for(unsigned int outputChannel = 0; outputChannel < layer.weights.dims(3); outputChannel++)
                      outputChannels[inputChannel * layer.weights.dims(3) + outputChannel] += layer.weights(filterY, filterX, inputChannel, outputChannel) * v;
                  }
                }
              }
            }

            std::vector<NeumaierSum<float>> softmaxSums(output.dims(2));
            for(unsigned int outputChannel = 0; outputChannel < output.dims(2); outputChannel++)
            {
              output(outputY, outputX, outputChannel) = applyActivationFunction(layer.hasBiases ? static_cast<float>(outputChannels[outputChannel] += layer.biases[outputChannel])
                                                        : static_cast<float>(outputChannels[outputChannel]), layer.activationId);
              if(layer.activationId == ActivationFunctionId::softmax)
                softmaxSums[outputChannel] += output(outputY, outputX, outputChannel);
            }
            if(layer.activationId == ActivationFunctionId::softmax)
              for(unsigned int outputChannel = 0; outputChannel < output.dims(2); outputChannel++)
                output(outputY, outputX, outputChannel) /= softmaxSums[outputChannel];
          }
        }
      }

      void apply(const TensorXf& input, TensorXf& output, const UpSampling2DLayer& layer)
      {
        ASSERT(input.rank() == 3);
        ASSERT(output.rank() == 3);
        ASSERT(layer.interpolation == InterpolationMethod::nearest);

        std::vector<unsigned int> i(3);

        for(i[0] = 0; i[0] < output.dims(0); i[0]++)
          for(i[1] = 0; i[1] < output.dims(1); i[1]++)
            for(i[2] = 0; i[2] < output.dims(2); i[2]++)
              output(i) = input(i[0] / layer.size[0], i[1] / layer.size[1], i[2]);
      }

      void apply(const TensorXf& input, TensorXf& output, const Cropping2DLayer& layer)
      {
        ASSERT(input.rank() == 3);
        ASSERT(output.rank() == 3);

        std::vector<unsigned int> iOut(3);
        std::vector<unsigned int> iIn(3);

        for(iOut[0] = 0; iOut[0] < output.dims(0); iOut[0]++)
        {
          iIn[0] = iOut[0] + layer.cropping[Cropping2DLayer::TOP];

          for(iOut[1] = 0; iOut[1] < output.dims(1); iOut[1]++)
          {
            iIn[1] = iOut[1] + layer.cropping[Cropping2DLayer::LEFT];

            for(iIn[2] = iOut[2] = 0; iOut[2] < output.dims(2); iIn[2]++, iOut[2]++)
              output(iOut) = input(iIn);
          }
        }
      }

      void apply(const TensorXf& input, TensorXf& output, const ZeroPadding2DLayer& layer)
      {
        ASSERT(input.rank() == 3);
        ASSERT(output.rank() == 3);

        std::vector<unsigned int> i(3);

        for(i[0] = 0; i[0] < output.dims(0); i[0]++)
          for(i[1] = 0; i[1] < output.dims(1); i[1]++)
          {
            if(i[1] < layer.padding[ZeroPadding2DLayer::LEFT] || i[0] < layer.padding[ZeroPadding2DLayer::TOP] ||
               output.dims(1) - i[1] <= layer.padding[ZeroPadding2DLayer::RIGHT] ||
               output.dims(0) - i[0] <= layer.padding[ZeroPadding2DLayer::BOTTOM])
            {
              for(i[2] = 0; i[2] < output.dims(2); i[2]++)
                output(i) = 0.f;
            }
            else
            {
              std::vector<unsigned int> i_ = i;
              i_[1] = i[1] - layer.padding[ZeroPadding2DLayer::LEFT];
              i_[0] = i[0] - layer.padding[ZeroPadding2DLayer::TOP];
              for(i_[2] = i[2] = 0; i_[2] < output.dims(2); i_[2]++, i[2]++)
                output(i) = input(i_);
            }
          }
      }

      void apply(const TensorXf& input, TensorXf& output, const Pooling2DLayer& layer)
      {
        ASSERT(input.rank() == 3);
        ASSERT(output.rank() == 3);

        const unsigned int paddingTop = layer.padding == PaddingType::valid ? 0 : ((output.dims(0) - 1) * layer.strides[0] + layer.kernelSize[0] - input.dims(0)) / 2;
        const unsigned int paddingLeft = layer.padding == PaddingType::valid ? 0 : ((output.dims(1) - 1) * layer.strides[1] + layer.kernelSize[1] - input.dims(1)) / 2;

        const float filterSize = static_cast<float>(layer.kernelSize[0] * layer.kernelSize[1]);

        unsigned int outputY = 0;
        for(int y = -static_cast<int>(paddingTop); outputY < output.dims(0); y += layer.strides[0], outputY++)
        {
          unsigned int outputX = 0;
          for(int x = -static_cast<int>(paddingLeft); outputX < output.dims(1); x += layer.strides[1], outputX++)
          {
            if(y >= 0 && static_cast<unsigned int>(y) < input.dims(0) && x >= 0 && static_cast<unsigned int>(x) < input.dims(1) && layer.method == PoolingMethod::max)
            {
              for(unsigned int channel = 0; channel < output.dims(2); channel++)
                output(outputY, outputX, channel) = input(y, x, channel);
            }
            else if(layer.method == PoolingMethod::max)
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
                    if(layer.method == PoolingMethod::average)
                      outputSums[channel] += input(yIndex, xIndex, channel);
                    else if(layer.method == PoolingMethod::max)
                      output(outputY, outputX, channel) = std::max(output(outputY, outputX, channel), input(yIndex, xIndex, channel));
                    else
                      ASSERT(false);
                  }
                }
              }
            }

            if(layer.method == PoolingMethod::average)
            {
              for(unsigned int outputChannel = 0; outputChannel < output.dims(2); outputChannel++)
                output(outputY, outputX, outputChannel) = static_cast<float>(outputSums[outputChannel]) / filterSize;
            }
          }
        }
      }

      void apply(const TensorXf& input, TensorXf& output, const GlobalPooling2DLayer& layer)
      {
        ASSERT(input.rank() == 3);
        ASSERT(output.rank() == 1);

        std::vector<NeumaierSum<float>> outputSums(output.dims(0));
        if(layer.method == PoolingMethod::max)
        {
          for(unsigned int channel = 0; channel < output.dims(0); channel++)
            output(channel) = input.data()[channel];
        }

        for(unsigned int channel = 0; channel < input.dims(2); channel++)
        {
          if(layer.method == PoolingMethod::max)
          {
            for(const float* in = input.begin() + channel + input.dims(2); in < input.end(); in += input.dims(2))
              output(channel) = std::max(output(channel), *in);
          }
          else
          {
            for(const float* in = input.begin() + channel; in < input.end(); in += input.dims(2))
              outputSums[channel] += *in;
          }
        }

        if(layer.method == PoolingMethod::average)
        {
          const float imageSize = static_cast<const float>(input.dims(0) * input.dims(1));
          for(unsigned int outputChannel = 0; outputChannel < output.dims(0); outputChannel++)
            output(outputChannel) = static_cast<float>(outputSums[outputChannel]) / imageSize;
        }
      }

      void apply(const std::vector<const TensorXf*>& input, TensorXf& output, const AddLayer&)
      {
        std::copy(input[0]->begin(), input[0]->end(), output.begin());
        for(std::size_t i = 1; i < input.size(); ++i)
        {
          float* out = output.data();
          for(const float* in = input[i]->data(); in < input[i]->data() + input[i]->size(); in++, out++)
            *out += *in;
        }
      }

      void apply(const std::vector<const TensorXf*>& input, TensorXf& output, const SubtractLayer&)
      {
        std::copy(input[0]->begin(), input[0]->end(), output.begin());
        float* out = output.data();
        for(const float* in = input[1]->data(); in < input[1]->data() + input[1]->size(); in++, out++)
          *out -= *in;
      }

      void apply(const std::vector<const TensorXf*>& input, TensorXf& output, const MultiplyLayer&)
      {
        std::copy(input[0]->begin(), input[0]->end(), output.begin());
        for(std::size_t i = 1; i < input.size(); ++i)
        {
          float* out = output.data();
          for(const float* in = input[i]->data(); in < input[i]->data() + input[i]->size(); in++, out++)
            *out *= *in;
        }
      }

      void apply(const std::vector<const TensorXf*>& input, TensorXf& output, const AverageLayer&)
      {
        std::copy(input[0]->begin(), input[0]->end(), output.begin());
        for(std::size_t i = 1; i < input.size() - 1; ++i)
        {
          float* out = output.data();
          for(const float* in = input[i]->data(); in < input[i]->data() + input[i]->size(); in++, out++)
            *out += *in;
        }
        const float rNumber = 1.f / static_cast<float>(input.size());
        float* out = output.data();
        for(const float* in = input.back()->data(); in < input.back()->data() + input.back()->size(); in++, out++)
        {
          *out += *in;
          *out *= rNumber;
        }
      }

      void apply(const std::vector<const TensorXf*>& input, TensorXf& output, const MaximumLayer&)
      {
        std::copy(input[0]->begin(), input[0]->end(), output.begin());
        for(std::size_t i = 1; i < input.size(); ++i)
        {
          float* out = output.data();
          for(const float* in = input[i]->data(); in < input[i]->data() + input[i]->size(); in++, out++)
            *out = std::max(*in, *out);
        }
      }

      void apply(const std::vector<const TensorXf*>& input, TensorXf& output, const MinimumLayer&)
      {
        std::copy(input[0]->begin(), input[0]->end(), output.begin());
        for(std::size_t i = 1; i < input.size(); ++i)
        {
          float* out = output.data();
          for(const float* in = input[i]->data(); in < input[i]->data() + input[i]->size(); in++, out++)
            *out = std::min(*in, *out);
        }
      }

      void apply(const std::vector<const TensorXf*>& input, TensorXf& output, const ConcatenateLayer& layer)
      {
        // We rely on the assertions that have been made in ConcatenateLayer::calcOutputDimensions.

        ASSERT(static_cast<std::size_t>(layer.axis >= 0 ? layer.axis : -(layer.axis + 1)) < output.rank());
        std::size_t realAxis = layer.axis >= 0 ? layer.axis : output.rank() + layer.axis;

        std::size_t outerSize = 1, innerSize = 1;
        for(std::size_t i = 0; i < realAxis; ++i)
          outerSize *= output.dims(i);
        for(std::size_t i = realAxis + 1; i < output.rank(); ++i)
          innerSize *= output.dims(i);

        float* outputPtr = output.data();
        for(std::size_t i = 0; i < outerSize; ++i)
          for(std::size_t j = 0; j < input.size(); ++j)
          {
            // This is the number of elements to copy
            const std::size_t s = innerSize * input[j]->dims(realAxis);
            std::copy(input[j]->data() + i * s, input[j]->data() + (i + 1) * s, outputPtr);
            outputPtr += s;
          }
      }

      void apply(const TensorXf& input, TensorXf& output, const LeakyReluLayer& layer)
      {
        float* out = output.data();
        for(const float* in = input.data(); in < input.data() + input.size(); in++, out++)
        {
          const float v = *in;
          *out = v < 0.f ? layer.alpha * v : v;
        }
      }

      void apply(const TensorXf& input, TensorXf& output, const EluLayer& layer)
      {
        float* out = output.data();
        for(const float* in = input.data(); in < input.data() + input.size(); in++, out++)
        {
          const float v = *in;
          *out = v < 0.f ? layer.alpha * (std::exp(v) - 1.f) : v;
        }
      }

      void apply(const TensorXf& input, TensorXf& output, const ThresholdedReluLayer& layer)
      {
        float* out = output.data();
        for(const float* in = input.data(); in < input.data() + input.size(); in++, out++)
        {
          const float v = *in;
          *out = v <= layer.theta ? 0.f : v;
        }
      }

      void apply(const TensorXf& input, TensorXf& output, const SoftmaxLayer& layer)
      {
        ASSERT(input.dims() == output.dims());

        ASSERT(static_cast<std::size_t>(layer.axis >= 0 ? layer.axis : -(layer.axis + 1)) < input.rank());
        const std::size_t realAxis = layer.axis >= 0 ? layer.axis : input.rank() + layer.axis;

        std::size_t parentSize = 1;                           // The combined dimensions of all axes before the softmax axis
        for(size_t i = 0; i < realAxis; i++)
          parentSize *= input.dims(i);
        const std::size_t softmaxSize = input.dims(realAxis); // The dimension of the softmax axis
        std::size_t childSize = 1;                            // The combined dimensions of all axes after the softmax axis
        for(size_t i = realAxis; i < input.rank(); i++)
          childSize *= input.dims(i);

        const float* in = input.begin();
        float* out = output.begin();
        for(size_t i = parentSize; i; in += (softmaxSize - 1) * childSize, out += (softmaxSize - 1) * childSize, i--)
        {
          for(size_t j = childSize; j; in++, out++, j--)
          {
            // Calculate the maximum
            float max = *in;
            for(std::size_t k = 1; k < softmaxSize; k++)
              max = std::max(max, in[k * childSize]);

            // Calculate exp(x - max) per value and the sum of the results
            NeumaierSum<float> sum;
            for(std::size_t k = 0; k < softmaxSize; k++)
            {
              const float v = std::exp(in[k * childSize] - max);
              sum += v;
              out[k * childSize] = v;
            }
            const float totalSum = static_cast<float>(sum);

            // Calculate the softmax
            for(std::size_t k = 0; k < softmaxSize; k++)
              out[k * childSize] /= totalSum;
          }
        }
      }

      void apply(const TensorXf& input, TensorXf& output, const ReluLayer& layer)
      {
        float* out = output.data();
        for(const float* in = input.data(); in < input.data() + input.size(); in++, out++)
        {
          const float v = *in;
          *out = v >= layer.maxValue ? layer.maxValue : (v >= layer.threshold ? v : layer.negativeSlope * (v - layer.threshold));
        }
      }

      void apply(const TensorXf& input, TensorXf& output, const BatchNormalizationLayer& layer)
      {
        ASSERT(input.dims() == output.dims());

        ASSERT(static_cast<std::size_t>(layer.axis >= 0 ? layer.axis : -(layer.axis + 1)) < input.rank());
        const std::size_t realAxis = layer.axis >= 0 ? layer.axis : input.rank() + layer.axis;
        ASSERT(input.dims(realAxis) == layer.factor.size());

        std::vector<unsigned int> i(input.rank(), 0);
        unsigned int& index = i[realAxis];

        while(true)
        {
          output(i) = input(i) * layer.factor[index] + layer.offset[index];
          unsigned int carryIndex = 0;
          while(++i[carryIndex] == input.dims(carryIndex))
          {
            i[carryIndex] = 0;
            carryIndex++;
            if(carryIndex == input.rank())
              return;
          }
        }
      }
    }

    void apply(const std::vector<const TensorXf*>& input, std::vector<TensorXf*>& output, const Node& node)
    {
      ASSERT(input.size() == node.inputDimensions.size());
      ASSERT(output.size() == node.outputDimensions.size());

      // Create output tensors
      for(std::size_t i = 0; i < output.size(); ++i)
        output[i]->reshape(node.outputDimensions[i]);

      // Apply layer
      switch(node.layer->type)
      {
        case LayerType::input:
          FAIL("Input layers cannot be applied.");
          break;
        case LayerType::dense:
          ASSERT(input.size() == 1);
          ASSERT(output.size() == 1);
          Impl::apply(*input[0], *output[0], *static_cast<const DenseLayer*>(node.layer));
          break;
        case LayerType::activation:
          ASSERT(input.size() == 1);
          ASSERT(output.size() == 1);
          Impl::apply(*input[0], *output[0], *static_cast<const ActivationLayer*>(node.layer));
          break;
        case LayerType::dropout:
          ASSERT(input.size() == output.size());
          Impl::apply(input, output, *static_cast<const DropoutLayer*>(node.layer));
          break;
        case LayerType::flatten:
          ASSERT(input.size() == 1);
          ASSERT(output.size() == 1);
          Impl::apply(*input[0], *output[0], *static_cast<const FlattenLayer*>(node.layer));
          break;
        case LayerType::reshape:
          ASSERT(input.size() == 1);
          ASSERT(output.size() == 1);
          Impl::apply(*input[0], *output[0], *static_cast<const ReshapeLayer*>(node.layer));
          break;
        case LayerType::conv2D:
          ASSERT(input.size() == 1);
          ASSERT(output.size() == 1);
          Impl::apply(*input[0], *output[0], *static_cast<const Conv2DLayer*>(node.layer));
          break;
        case LayerType::separableConv2D:
          ASSERT(input.size() == 1);
          ASSERT(output.size() == 1);
          Impl::apply(*input[0], *output[0], *static_cast<const SeparableConv2DLayer*>(node.layer));
          break;
        case LayerType::depthwiseConv2D:
          ASSERT(input.size() == 1);
          ASSERT(output.size() == 1);
          Impl::apply(*input[0], *output[0], *static_cast<const DepthwiseConv2DLayer*>(node.layer));
          break;
        case LayerType::cropping2D:
          ASSERT(input.size() == 1);
          ASSERT(output.size() == 1);
          Impl::apply(*input[0], *output[0], *static_cast<const Cropping2DLayer*>(node.layer));
          break;
        case LayerType::upSampling2D:
          ASSERT(input.size() == 1);
          ASSERT(output.size() == 1);
          Impl::apply(*input[0], *output[0], *static_cast<const UpSampling2DLayer*>(node.layer));
          break;
        case LayerType::zeroPadding2D:
          ASSERT(input.size() == 1);
          ASSERT(output.size() == 1);
          Impl::apply(*input[0], *output[0], *static_cast<const ZeroPadding2DLayer*>(node.layer));
          break;
        case LayerType::maxPooling2D:
        case LayerType::averagePooling2D:
          ASSERT(input.size() == 1);
          ASSERT(output.size() == 1);
          Impl::apply(*input[0], *output[0], *static_cast<const Pooling2DLayer*>(node.layer));
          break;
        case LayerType::globalMaxPooling2D:
        case LayerType::globalAveragePooling2D:
          ASSERT(input.size() == 1);
          ASSERT(output.size() == 1);
          Impl::apply(*input[0], *output[0], *static_cast<const GlobalPooling2DLayer*>(node.layer));
          break;
        case LayerType::add:
          ASSERT(input.size() > 1);
          ASSERT(output.size() == 1);
          Impl::apply(input, *output[0], *static_cast<const AddLayer*>(node.layer));
          break;
        case LayerType::subtract:
          ASSERT(input.size() == 2);
          ASSERT(output.size() == 1);
          Impl::apply(input, *output[0], *static_cast<const SubtractLayer*>(node.layer));
          break;
        case LayerType::multiply:
          ASSERT(input.size() > 1);
          ASSERT(output.size() == 1);
          Impl::apply(input, *output[0], *static_cast<const MultiplyLayer*>(node.layer));
          break;
        case LayerType::average:
          ASSERT(input.size() > 1);
          ASSERT(output.size() == 1);
          Impl::apply(input, *output[0], *static_cast<const AverageLayer*>(node.layer));
          break;
        case LayerType::maximum:
          ASSERT(input.size() > 1);
          ASSERT(output.size() == 1);
          Impl::apply(input, *output[0], *static_cast<const MaximumLayer*>(node.layer));
          break;
        case LayerType::minimum:
          ASSERT(input.size() > 1);
          ASSERT(output.size() == 1);
          Impl::apply(input, *output[0], *static_cast<const MinimumLayer*>(node.layer));
          break;
        case LayerType::concatenate:
          ASSERT(input.size() > 1);
          ASSERT(output.size() == 1);
          Impl::apply(input, *output[0], *static_cast<const ConcatenateLayer*>(node.layer));
          break;
        case LayerType::leakyRelu:
          ASSERT(input.size() == 1);
          ASSERT(output.size() == 1);
          Impl::apply(*input[0], *output[0], *static_cast<const LeakyReluLayer*>(node.layer));
          break;
        case LayerType::elu:
          ASSERT(input.size() == 1);
          ASSERT(output.size() == 1);
          Impl::apply(*input[0], *output[0], *static_cast<const EluLayer*>(node.layer));
          break;
        case LayerType::thresholdedRelu:
          ASSERT(input.size() == 1);
          ASSERT(output.size() == 1);
          Impl::apply(*input[0], *output[0], *static_cast<const ThresholdedReluLayer*>(node.layer));
          break;
        case LayerType::softmax:
          ASSERT(input.size() == 1);
          ASSERT(output.size() == 1);
          Impl::apply(*input[0], *output[0], *static_cast<const SoftmaxLayer*>(node.layer));
          break;
        case LayerType::relu:
          ASSERT(input.size() == 1);
          ASSERT(output.size() == 1);
          Impl::apply(*input[0], *output[0], *static_cast<const ReluLayer*>(node.layer));
          break;
        case LayerType::batchNormalization:
          ASSERT(input.size() == 1);
          ASSERT(output.size() == 1);
          Impl::apply(*input[0], *output[0], *static_cast<const BatchNormalizationLayer*>(node.layer));
          break;
      }
    }

    void apply(const std::vector<TensorXf>& input, std::vector<TensorXf>& output, const Node& node)
    {
      std::vector<const TensorXf*> inputPointers(input.size());
      std::vector<TensorXf*> outputPointers(output.size());
      for(std::size_t i = 0; i < input.size(); ++i)
        inputPointers[i] = &input[i];
      for(std::size_t i = 0; i < output.size(); ++i)
        outputPointers[i] = &output[i];
      apply(inputPointers, outputPointers, node);
    }

    void apply(std::vector<TensorXf>& input, std::vector<TensorXf>& output, const Model& model, const NodeCallback& nodeCallback)
    {
      const std::vector<TensorLocation>& modelInputs = model.getInputs();
      const std::vector<TensorLocation>& modelOutputs = model.getOutputs();
      ASSERT(input.size() == modelInputs.size());
      ASSERT(output.size() == modelOutputs.size());

      for(std::size_t i = 0; i < input.size(); ++i)
        if(model.isInputUInt8(i))
        {
          float* out = input[i].data() + input[i].size() - 1;
          for(const unsigned char* in = reinterpret_cast<unsigned char*>(input[i].data()) + input[i].size() - 1; out >= input[i].data(); --in, --out)
            *out = *in;
        }

      struct TensorPlaceholder
      {
        TensorLocation location;
        TensorXf* tensor;
        std::size_t refCount;
        const bool externalMemory = false;

        TensorPlaceholder(const TensorLocation& location, TensorXf* tensor, std::size_t refCount) :
          location(location), tensor(tensor), refCount(refCount), externalMemory(true)
        {}

        TensorPlaceholder(const TensorLocation& location, const std::vector<unsigned int>& dimensions, std::size_t refCount) :
          location(location), tensor(new TensorXf(dimensions)), refCount(refCount)
        {}

        TensorPlaceholder(TensorPlaceholder&& other) :
          location(std::move(other.location)),
          tensor(std::move(other.tensor)),
          refCount(std::move(other.refCount)),
          externalMemory(std::move(other.externalMemory))
        {
          other.tensor = nullptr;
        }

        ~TensorPlaceholder()
        {
          if(!externalMemory)
            delete tensor;
        }
      };

      std::list<const Node*> remainingNodes;
      for(const auto& layer : model.getLayers())
      {
#ifdef NDEBUG
        if(layer->type == LayerType::input)
          continue;
#endif
        for(const Node& node : layer->nodes)
        {
#ifndef NDEBUG
          // If a node provides any input, it must not have any inputs itself and all of its tensors must be inputs.
          // In that case, it is not added to the remaining nodes.
          int providesInput = 0;
          // Normally, either all outputs of a node are inputs or none.
          for(const TensorLocation& tl : node.outputs)
          {
            bool isInput = false;
            for(const TensorLocation& tl2 : modelInputs)
              if(tl == tl2)
              {
                ASSERT(node.inputs.empty());
                isInput = true;
                break;
              }
            if(!providesInput)
            {
              ASSERT(isInput == (layer->type == LayerType::input));
              providesInput = isInput ? 1 : -1;
            }
            else
              ASSERT(providesInput > 0 == isInput);
          }
          if(providesInput > 0)
            continue;
#endif
          remainingNodes.emplace_back(&node);
        }
      }

      // This function computes the number of times a tensor is named as input from the remaining nodes.
      auto getRefCount = [&remainingNodes, &modelOutputs](const TensorLocation& tl) -> std::size_t
      {
        std::size_t refCount = 0;
        for(const Node* node : remainingNodes)
          for(const TensorLocation& tl2 : node->inputs)
            if(tl == tl2)
              ++refCount;
        for(const TensorLocation& tl2 : modelOutputs)
          if(tl == tl2)
            ++refCount;
        return refCount;
      };

      // Add the input and output tensors that this function was given to the set of live tensors.
      std::vector<TensorPlaceholder> tensors;
      tensors.reserve(input.size() + output.size());
      // The output tensors must be first so they are always found first in lookupOutputTensor.
      for(std::size_t i = 0; i < output.size(); ++i)
        tensors.emplace_back(modelOutputs[i], &output[i], std::numeric_limits<std::size_t>::max());
      for(std::size_t i = 0; i < input.size(); ++i)
      {
        // Check if an input tensor is also an output of the net.
        // In that case, do not add a new placeholder, but copy the data to the existing one.
        for(std::size_t j = 0; j < output.size(); ++j)
          if(tensors[j].location == modelInputs[i])
          {
            tensors[j].tensor->reshape(input[i].dims());
            tensors[j].tensor->copyFrom(input[i]);
            tensors[j].refCount = getRefCount(modelInputs[i]);
            goto nextInput;
          }
        tensors.emplace_back(modelInputs[i], &input[i], getRefCount(modelInputs[i]));
      nextInput:
        ;
      }

      // This function obtains an arbitrary node that has all its dependencies satisfied.
      auto lookupNextNode = [&tensors, &remainingNodes]() -> const Node*
      {
        for(const Node* node : remainingNodes)
        {
          bool allFound = true;
          for(const TensorLocation& tl : node->inputs)
          {
            bool found = false;
            for(const TensorPlaceholder& tp : tensors)
              if(tp.refCount && tp.refCount != std::numeric_limits<std::size_t>::max() && tl == tp.location)
              {
                found = true;
                break;
              }
            if(!found)
            {
              allFound = false;
              break;
            }
          }
          if(allFound)
            return node;
        }
        return nullptr;
      };

      // This function decreases the reference count of all tensors that have been the input to a node.
      auto decreaseRefCounters = [&tensors](const std::vector<TensorLocation>& inputs)
      {
        for(const TensorLocation& tl : inputs)
          for(TensorPlaceholder& tp : tensors)
            if(tp.refCount && tl == tp.location)
              --tp.refCount;
      };

      // This function gets an existing input tensor at a specific location.
      auto lookupInputTensor = [&tensors](const TensorLocation& location) -> TensorXf*
      {
        for(const TensorPlaceholder& tp : tensors)
          if(tp.refCount && location == tp.location)
            return tp.tensor;
        return nullptr;
      };

      // This function finds a tensor which should hold the output of a node.
      auto lookupOutputTensor = [&tensors, &getRefCount](const TensorLocation& location, const std::vector<unsigned int>& dimensions) -> TensorXf*
      {
        TensorPlaceholder* minSizeTensor = nullptr;
        for(TensorPlaceholder& tp : tensors)
        {
          // If the output tensor is predetermined (because it is an output of the model), that one is used.
          if(tp.refCount == std::numeric_limits<std::size_t>::max() && tp.location == location)
          {
            tp.refCount = getRefCount(location);
            return tp.tensor;
          }
          // Do not overwrite used tensors.
          else if(tp.refCount)
            continue;
          // If there is a free tensor with enough capacity, use it.
          if(tp.tensor->capacity() >= std::accumulate(dimensions.begin(), dimensions.end(), 1u, std::multiplies<>()))
          {
            tp.location = location;
            tp.tensor->reshape(dimensions);
            tp.refCount = getRefCount(location);
            return tp.tensor;
          }
          // Otherwise, try to find the tensor with minimum size (so the amount that needs to be copied is minimum).
          if(!minSizeTensor || tp.tensor->size() < minSizeTensor->tensor->size())
            minSizeTensor = &tp;
        }
        if(minSizeTensor)
        {
          minSizeTensor->location = location;
          minSizeTensor->tensor->reshape(dimensions);
          minSizeTensor->refCount = getRefCount(location);
          return minSizeTensor->tensor;
        }
        // Create a new tensor.
        tensors.emplace_back(location, dimensions, getRefCount(location));
        return tensors.back().tensor;
      };

      // Execute all nodes.
      while(!remainingNodes.empty())
      {
        // Get a node of which all dependencies are satisfied.
        const Node* node = lookupNextNode();

        // Set references to input and output tensors.
        std::vector<const TensorXf*> inputPointers(node->inputs.size());
        for(std::size_t i = 0; i < node->inputs.size(); ++i)
          inputPointers[i] = lookupInputTensor(node->inputs[i]);
        std::vector<TensorXf*> outputPointers(node->outputs.size());
        for(std::size_t i = 0; i < node->outputs.size(); ++i)
          outputPointers[i] = lookupOutputTensor(node->outputs[i], node->outputDimensions[i]);

        // Let the node execute.
        apply(inputPointers, outputPointers, *node);

        // Invoke the callback.
        if(nodeCallback)
          nodeCallback(*node, inputPointers, outputPointers);

        // Decrease the reference counters of all tensors that have been used as input to this node.
        decreaseRefCounters(node->inputs);

        // Remove this node from the list of remaining nodes.
        std::remove(remainingNodes.begin(), remainingNodes.end(), node);
        remainingNodes.pop_back();
      }

      // All outputs should be computed now.
#ifndef NDEBUG
      std::size_t numOfComputedOutputs = 0;
      for(const TensorPlaceholder& tp : tensors)
      {
        bool isOutput = false;
        for(const TensorLocation& tl : modelOutputs)
          if(tl == tp.location)
          {
            isOutput = true;
            ++numOfComputedOutputs;
            break;
          }
        if(isOutput)
          ASSERT(tp.refCount == 1);
        else
          ASSERT(tp.refCount == 0);
      }
      // It cannot happen that the same tensor exists twice in the tensors array, thus this assertion asserts that all outputs have been computed.
      ASSERT(numOfComputedOutputs == modelOutputs.size());
#endif
    }
  }
}
