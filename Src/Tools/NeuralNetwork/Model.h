/**
 * Contains structs that define a neural network model as well as a method to
 * load such models from a file.
 *
 * @author Felix Thielke
 */

#pragma once

#include "Tensor.h"

#include "Platform/BHAssert.h"
#include <array>
#include <vector>
#include <memory>

namespace NeuralNetwork
{
  enum class LayerType : unsigned int
  {
    dense = 1,
    conv2D = 2,
    reshape = 3,
    elu = 4,
    activation = 5,
    pooling2D = 6,
    batchNormalization = 9,
    softmax = 10,
    sconv2D = 11,

  };

  struct Layer
  {
    const LayerType type;
    std::array<unsigned int, 3> inputDimensions;
    std::array<unsigned int, 3> outputDimensions;

    Layer() = delete;
    virtual ~Layer() = default;
    virtual void getParameter(std::vector<float>& parameter) const {};
    virtual void setParameter(std::vector<float>& parameter) {};
  protected:
    Layer(const LayerType type) : type(type) {}
  };

  /**
   * A struct that describes a neural network model.
   */
  struct Model
  {
  private:
    std::vector<std::unique_ptr<Layer>> layers;

  public:
    Tensor3 testInput;
    Tensor3 testResult;

    Model() = default;
    Model(const std::string& file) : Model() { load(file); }
    ~Model()
    {
      layers.clear();
    }

    /**
     * Returns a const reference to a vector of the layers that make up this NN.
     */
    inline const std::vector<std::unique_ptr<Layer>>& getLayers() const { return layers; }

    /**
     * Adds the given layer to this model, thereby giving this struct ownership
     * of the pointer to it.
     */
    void addLayer(std::unique_ptr<Layer>&& layer)
    {
      ASSERT(layers.empty() || layers.back()->outputDimensions == layer->inputDimensions);

      layers.emplace_back(std::move<>(layer));
    }

    /**
     * Removes all layers from this model.
     */
    void clear() { layers.clear(); }

    /**
     * Loads a neural network model from the given file.
     */
    void load(const std::string& file);

    std::vector<float> getParameter(bool onlyLast = true) const;
    void setParameter(const std::vector<float>& parameter_, bool onlyLast = true);
  };

  enum class ActivationFunctionId : unsigned int
  {
    linear = 1,
    relu = 2,
    sigmoid = 4,
    tanH = 5,
    hardSigmoid = 6,

    // These are only for the import; these activations are transformed into the corresponding layers
    softmax = 7,
    elu = 8
  };

  enum class PaddingType : unsigned int
  {
    valid = 1,
    same = 2
  };

  struct DenseLayer : Layer
  {
    Tensor<2, float, 1> weights;
    std::vector<float> biases;
    ActivationFunctionId activationId;

    DenseLayer() : Layer(LayerType::dense) {}
    void getParameter(std::vector<float>& parameter) const override;
    void setParameter(std::vector<float>& parameter) override;
  };

  struct Conv2DLayer : Layer
  {
    std::array<unsigned int, 2> strides;
    PaddingType padding;
    Tensor<4, float, 1> weights;
    std::vector<float> biases;
    ActivationFunctionId activationId;

    Conv2DLayer() : Layer(LayerType::conv2D) {}
    void getParameter(std::vector<float>& parameter) const override;
    void setParameter(std::vector<float>& parameter) override;
  };

  struct SConv2DLayer : Layer
  {
    std::array<unsigned int, 2> strides;
    PaddingType padding;
    Tensor<4, float, 1> weights;

    SConv2DLayer() : Layer(LayerType::sconv2D) {}
  };

  struct ReshapeLayer : Layer
  {
    ReshapeLayer() : Layer(LayerType::reshape) {}
  };

  struct EluLayer : Layer
  {
    float alpha;

    EluLayer() : Layer(LayerType::elu) {}
  };

  struct ActivationLayer : Layer
  {
    ActivationFunctionId activationId;

    ActivationLayer() : Layer(LayerType::activation) {}
  };

  struct Pooling2DLayer : Layer
  {
    enum PoolingMethod : unsigned int
    {
      average = 1,
      max = 2
    };

    PoolingMethod method;
    PaddingType padding;
    std::array<unsigned int, 2> kernelSize;
    std::array<unsigned int, 2> strides;

    Pooling2DLayer() : Layer(LayerType::pooling2D) {}
  };

  struct BatchNormalizationLayer : Layer
  {
    unsigned int dimension;
    unsigned int inputSize;
    std::vector<float> factor;
    std::vector<float> offset;

    BatchNormalizationLayer() : Layer(LayerType::batchNormalization) {}
    void getParameter(std::vector<float>& parameter) const override;
    void setParameter(std::vector<float>& parameter) override;
  };

  struct SoftmaxLayer : Layer
  {
    SoftmaxLayer() : Layer(LayerType::softmax) {}
  };
}
