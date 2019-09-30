/**
 * Contains structs that define a neural network model as well as a method to
 * load such models from a file.
 *
 * @author Felix Thielke
 */

#pragma once

#include "Tensor.h"
#include <array>
#include <functional>
#include <memory>
#include <string>
#include <vector>

class In;

namespace NeuralNetwork
{
  enum class LayerType : unsigned int
  {
    dense = 1,
    conv2D = 2,
    reshape = 3,
    elu = 4,
    activation = 5,
    maxPooling2D = 6,
    batchNormalization = 9,
    softmax = 10,
    depthwiseConv2D = 11,
    zeroPadding2D = 12,

    // These layers cannot occur in kerasify files.
    input = 100,
    dropout,
    flatten,
    separableConv2D,
    cropping2D,
    upSampling2D,
    averagePooling2D,
    globalMaxPooling2D,
    globalAveragePooling2D,
    add,
    subtract,
    multiply,
    average,
    maximum,
    minimum,
    concatenate,
    leakyRelu,
    thresholdedRelu,
    relu
  };

  struct Layer;

  /**
   * A struct that describes the location of a tensor in a network.
   */
  struct TensorLocation
  {
    const Layer* layer;
    unsigned int nodeIndex;
    unsigned int tensorIndex;

    TensorLocation(const Layer* layer, unsigned int nodeIndex, unsigned int tensorIndex) : layer(layer), nodeIndex(nodeIndex), tensorIndex(tensorIndex) {}

    bool operator==(const TensorLocation& other) const
    {
      return layer == other.layer && nodeIndex == other.nodeIndex && tensorIndex == other.tensorIndex;
    }
  };

  /**
   * A struct that describes a node in a network, i.e. an instance of a layer with known inputs and outputs.
   */
  struct Node
  {
    const Layer* const layer;
    std::vector<TensorLocation> inputs;
    std::vector<TensorLocation> outputs;
    std::vector<std::vector<unsigned int>> inputDimensions;
    std::vector<std::vector<unsigned int>> outputDimensions;

    void setDimensions();

    Node(const Layer* layer) : layer(layer) {}
  };

  struct Layer
  {
    const LayerType type;
    std::vector<Node> nodes;

    Layer() = delete;
    virtual ~Layer() = default;

    virtual void calcOutputDimensions(Node& node) const = 0;
  protected:
    Layer(const LayerType type) : type(type) {}
  };

  /**
   * A struct that describes a neural network model.
   */
  struct Model
  {
    /// getWeights with layer name as first parameter
    using GetWeightsFuncType = std::function<void(const std::string&, const std::string&, std::vector<float>&, std::vector<unsigned int>&)>;
    /// getWeights with bound layer name
    using GetWeights2FuncType = std::function<void(const std::string&, std::vector<float>&, std::vector<unsigned int>&)>;

  private:
    std::vector<std::unique_ptr<Layer>> layers;
    std::vector<bool> uint8Inputs;
    std::vector<TensorLocation> inputs;
    std::vector<TensorLocation> outputs;

    /**
     * Parses a model from a JSON description.
     */
    void parseJSONModel(In& stream, const std::string& fileName, const GetWeightsFuncType& getWeights);

  public:
    std::vector<TensorXf> testInput;
    std::vector<TensorXf> testResult;

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
     * Returns a const reference to a vector of the inputs that this NN has.
     */
    inline const std::vector<TensorLocation>& getInputs() const { return inputs; }

    /**
     * Returns a const reference to a vector of the outputs that this NN has.
     */
    inline const std::vector<TensorLocation>& getOutputs() const { return outputs; }

    /*
     * Indicates that an input with a specified index should be interpreted as a tensor of unsigned chars.
     */
    void setInputUInt8(std::size_t index);

    /*
     * Checks whether an input with a specified index should be interpreted as a tensor of unsigned chars.
     */
    bool isInputUInt8(std::size_t index) const;

    /**
     * Removes all layers from this model.
     */
    void clear() { layers.clear(); inputs.clear(); outputs.clear(); uint8Inputs.clear(); testInput.clear(), testResult.clear(); }

    /**
     * Loads a neural network model from the given file, determining the file format from the file name.
     */
    void load(const std::string& file)
    {
      if(!file.empty() && file.back() == '5')
        loadKerasHDF5(file);
      else
        loadKerasify(file);
    }

    /**
     * Loads a neural network model from the given file in the (extended) kerasify format.
     */
    void loadKerasify(const std::string& file);

    /**
     * Loads a neural network model from the given file in the native Keras HDF5 format.
     * Warning: Does not use the B-Human search path (i.e. must be relative to the Config/ directory).
     */
    void loadKerasHDF5(const std::string& file);
  };

  enum class ActivationFunctionId : unsigned int
  {
    linear = 1,
    relu = 2,
    sigmoid = 4,
    tanH = 5,
    hardSigmoid = 6,
    softmax = 7,
    elu = 8,
    selu = 10,
    exponential = 11,
    softsign = 12
  };

  enum class PaddingType : unsigned int
  {
    valid = 1,
    same = 2
  };

  enum class InterpolationMethod
  {
    nearest,
    bilinear
  };

  enum class PoolingMethod : unsigned int
  {
    average = 1,
    max = 2
  };

  struct InputLayer : Layer
  {
    std::vector<unsigned int> dimensions;

    InputLayer() : Layer(LayerType::input) {}

    void calcOutputDimensions(Node& node) const override;
  };

  struct DenseLayer : Layer
  {
    Tensor<float, 1> weights;
    std::vector<float> biases;
    bool hasBiases;
    ActivationFunctionId activationId;

    DenseLayer() : Layer(LayerType::dense) {}

    void calcOutputDimensions(Node& node) const override;
  };

  struct ActivationLayer : Layer
  {
    ActivationFunctionId activationId;

    ActivationLayer() : Layer(LayerType::activation) {}

    void calcOutputDimensions(Node& node) const override;
  };

  struct DropoutLayer : Layer
  {
    DropoutLayer() : Layer(LayerType::dropout) {}

    void calcOutputDimensions(Node& node) const override;
  };

  struct FlattenLayer : Layer
  {
    FlattenLayer() : Layer(LayerType::flatten) {}

    void calcOutputDimensions(Node& node) const override;
  };

  struct ReshapeLayer : Layer
  {
    std::vector<unsigned int> dimensions;

    ReshapeLayer() : Layer(LayerType::reshape) {}

    void calcOutputDimensions(Node& node) const override;
  };

  struct Conv2DLayer : Layer
  {
    std::array<unsigned int, 2> strides;
    Tensor<float, 1> weights;
    std::vector<float> biases;
    bool hasBiases;
    ActivationFunctionId activationId;
    PaddingType padding;

    Conv2DLayer() : Layer(LayerType::conv2D) {}

    void calcOutputDimensions(Node& node) const override;
  };

  struct SeparableConv2DLayer : Layer
  {
    std::array<unsigned int, 2> strides;
    Tensor<float, 1> depthwiseWeights;
    Tensor<float, 1> pointwiseWeights;
    std::vector<float> biases;
    bool hasBiases;
    ActivationFunctionId activationId;
    PaddingType padding;

    SeparableConv2DLayer() : Layer(LayerType::separableConv2D) {}

    void calcOutputDimensions(Node& node) const override;
  };

  struct DepthwiseConv2DLayer : Layer
  {
    std::array<unsigned int, 2> strides;
    Tensor<float, 1> weights;
    std::vector<float> biases;
    bool hasBiases;
    ActivationFunctionId activationId;
    PaddingType padding;

    DepthwiseConv2DLayer() : Layer(LayerType::depthwiseConv2D) {}

    void calcOutputDimensions(Node& node) const override;
  };

  struct Cropping2DLayer : Layer
  {
    enum Side
    {
      TOP,
      BOTTOM,
      LEFT,
      RIGHT,
    };
    std::array<unsigned int, 4> cropping;

    Cropping2DLayer() : Layer(LayerType::cropping2D) {}

    void calcOutputDimensions(Node& node) const override;
  };

  struct UpSampling2DLayer : Layer
  {
    std::array<unsigned int, 2> size;
    InterpolationMethod interpolation;

    UpSampling2DLayer() : Layer(LayerType::upSampling2D) {}

    void calcOutputDimensions(Node& node) const override;
  };

  struct ZeroPadding2DLayer : Layer
  {
    enum Side
    {
      TOP,
      BOTTOM,
      LEFT,
      RIGHT,
    };
    std::array<unsigned int, 4> padding;

    ZeroPadding2DLayer() : Layer(LayerType::zeroPadding2D) {}

    void calcOutputDimensions(Node& node) const override;
  };

  struct Pooling2DLayer : Layer
  {
    PoolingMethod method;
    PaddingType padding;
    std::array<unsigned int, 2> kernelSize;
    std::array<unsigned int, 2> strides;

    Pooling2DLayer(LayerType type, PoolingMethod method) : Layer(type), method(method) {}

    void calcOutputDimensions(Node& node) const override;
  };

  struct MaxPooling2DLayer : Pooling2DLayer
  {
    MaxPooling2DLayer() : Pooling2DLayer(LayerType::maxPooling2D, PoolingMethod::max) {}
  };

  struct AveragePooling2DLayer : Pooling2DLayer
  {
    AveragePooling2DLayer() : Pooling2DLayer(LayerType::averagePooling2D, PoolingMethod::average) {}
  };

  struct GlobalPooling2DLayer : Layer
  {
    PoolingMethod method;

    GlobalPooling2DLayer(LayerType type, PoolingMethod method) : Layer(type), method(method) {}

    void calcOutputDimensions(Node& node) const override;
  };

  struct GlobalMaxPooling2DLayer : GlobalPooling2DLayer
  {
    GlobalMaxPooling2DLayer() : GlobalPooling2DLayer(LayerType::globalMaxPooling2D, PoolingMethod::max) {}
  };

  struct GlobalAveragePooling2DLayer : GlobalPooling2DLayer
  {
    GlobalAveragePooling2DLayer() : GlobalPooling2DLayer(LayerType::globalAveragePooling2D, PoolingMethod::average) {}
  };

  struct AddLayer : Layer
  {
    AddLayer() : Layer(LayerType::add) {}

    void calcOutputDimensions(Node& node) const override;
  };

  struct SubtractLayer : Layer
  {
    SubtractLayer() : Layer(LayerType::subtract) {}

    void calcOutputDimensions(Node& node) const override;
  };

  struct MultiplyLayer : Layer
  {
    MultiplyLayer() : Layer(LayerType::multiply) {}

    void calcOutputDimensions(Node& node) const override;
  };

  struct AverageLayer : Layer
  {
    AverageLayer() : Layer(LayerType::average) {}

    void calcOutputDimensions(Node& node) const override;
  };

  struct MaximumLayer : Layer
  {
    MaximumLayer() : Layer(LayerType::maximum) {}

    void calcOutputDimensions(Node& node) const override;
  };

  struct MinimumLayer : Layer
  {
    MinimumLayer() : Layer(LayerType::minimum) {}

    void calcOutputDimensions(Node& node) const override;
  };

  struct ConcatenateLayer : Layer
  {
    int axis;

    ConcatenateLayer() : Layer(LayerType::concatenate) {}

    void calcOutputDimensions(Node& node) const override;
  };

  struct LeakyReluLayer : Layer
  {
    float alpha;

    LeakyReluLayer() : Layer(LayerType::leakyRelu) {}

    void calcOutputDimensions(Node& node) const override;
  };

  struct EluLayer : Layer
  {
    float alpha;

    EluLayer() : Layer(LayerType::elu) {}

    void calcOutputDimensions(Node& node) const override;
  };

  struct ThresholdedReluLayer : Layer
  {
    float theta;

    ThresholdedReluLayer() : Layer(LayerType::thresholdedRelu) {}

    void calcOutputDimensions(Node& node) const override;
  };

  struct SoftmaxLayer : Layer
  {
    int axis;

    SoftmaxLayer() : Layer(LayerType::softmax) {}

    void calcOutputDimensions(Node& node) const override;
  };

  struct ReluLayer : Layer
  {
    float maxValue;
    float negativeSlope;
    float threshold;

    ReluLayer() : Layer(LayerType::relu) {}

    void calcOutputDimensions(Node& node) const override;
  };

  struct BatchNormalizationLayer : Layer
  {
    int axis;
    std::vector<float> factor;
    std::vector<float> offset;

    BatchNormalizationLayer() : Layer(LayerType::batchNormalization) {}

    void calcOutputDimensions(Node& node) const override;
  };
}
