/**
 * Implements a class that compiles neural networks to optimized X86 machine
 * code and applies them to input data.
 *
 * @author Felix Thielke
 * @author Arne Hasselbring
 */

#include "CompiledNN.h"
#include "CompiledNN/CompiledNNImpl.h"
#include "Model.h"
#include "Tools/Global.h"
#include <cstring>
#include <numeric>
#include <unordered_map>

namespace NeuralNetwork
{
  using namespace asmjit;
  using namespace CompiledNNImpl;

  CompiledNN::~CompiledNN()
  {
    if(applyFunction)
      Global::getAsmjitRuntime().release(applyFunction);
  }

  template<typename CompilerType>
  OperationCompiler* CompiledNN::getCompiler(const CompilationSettings& settings, const typename CompilerType::Parameters& p, CompilerMap& compilers)
  {
    CompilerList& compilerType = compilers[typeid(CompilerType)];
    for(const std::unique_ptr<OperationCompiler>& compiler : compilerType)
      if(!std::memcmp(&static_cast<CompilerType*>(compiler.get())->p, &p, sizeof(p)))
      {
        ++compiler->refCount;
        return compiler.get();
      }
    std::unique_ptr<OperationCompiler> compiler = std::make_unique<CompilerType>(settings, p);
    OperationCompiler* compilerPtr = compiler.get();
    compilerType.emplace_back(std::move(compiler));
    return compilerPtr;
  }

  std::vector<OperationCompiler*> CompiledNN::generateCompilers(const CompilationSettings& settings, const Node& node, CompilerMap& compilers)
  {
    auto activationToCompiled = [&compilers, &node, &settings](ActivationFunctionId activationId, OperationCompiler*& extCompiler) -> CompiledActivationFunctionId
    {
      extCompiler = nullptr;
      switch(activationId)
      {
        case ActivationFunctionId::linear:
          return CompiledActivationFunctionId::linear;
        case ActivationFunctionId::relu:
          return CompiledActivationFunctionId::relu;
        case ActivationFunctionId::sigmoid:
          return CompiledActivationFunctionId::sigmoid;
        case ActivationFunctionId::tanH:
          return CompiledActivationFunctionId::tanH;
        case ActivationFunctionId::hardSigmoid:
          return CompiledActivationFunctionId::hardSigmoid;
        case ActivationFunctionId::softmax:
        {
          SoftmaxCompiler::Parameters p;
          ASSERT(node.inputDimensions.size() == 1);
          p.dimension = static_cast<unsigned int>(node.inputDimensions[0].size() - 1);
          extCompiler = getCompiler<SoftmaxCompiler>(settings, p, compilers);
          return CompiledActivationFunctionId::linear;
        }
        case ActivationFunctionId::elu:
          return CompiledActivationFunctionId::elu;
        case ActivationFunctionId::selu:
          return CompiledActivationFunctionId::selu;
        case ActivationFunctionId::exponential:
          return CompiledActivationFunctionId::exponential;
        case ActivationFunctionId::softsign:
          return CompiledActivationFunctionId::softsign;
        default:
          break;
      }
      FAIL("Unknown activation function.");
      return CompiledActivationFunctionId::linear;
    };

    auto getPadding = [&compilers, &node, &settings](const std::array<unsigned int, 2>& kernelSize, const std::array<unsigned int, 2>& strides) -> OperationCompiler*
    {
      ASSERT(node.inputDimensions.size() == 1);
      ASSERT(node.inputDimensions[0].size() == 3);
      const unsigned int verticalRemainder = node.inputDimensions[0][0] % strides[0];
      const unsigned int horizontalRemainder = node.inputDimensions[0][1] % strides[1];
      const unsigned int verticalPadding = std::max<int>(0, static_cast<int>(kernelSize[0]) -
                                                         (verticalRemainder ? verticalRemainder : strides[0]));
      const unsigned int horizontalPadding = std::max<int>(0, static_cast<int>(kernelSize[1]) -
                                                           (horizontalRemainder ? horizontalRemainder : strides[1]));
      if(!verticalPadding && !horizontalPadding)
        return nullptr;

      ZeroPadding2DCompiler::Parameters p;
      p.padding[ZeroPadding2DLayer::TOP] = verticalPadding / 2;
      p.padding[ZeroPadding2DLayer::BOTTOM] = (verticalPadding + 1) / 2;
      p.padding[ZeroPadding2DLayer::LEFT] = horizontalPadding / 2;
      p.padding[ZeroPadding2DLayer::RIGHT] = (horizontalPadding + 1) / 2;
      return getCompiler<ZeroPadding2DCompiler>(settings, p, compilers);
    };

    std::vector<OperationCompiler*> result;
    switch(node.layer->type)
    {
      case LayerType::input:
        break;
      case LayerType::dense:
      {
        const DenseLayer& layer = *static_cast<const DenseLayer*>(node.layer);
        DenseCompiler::Parameters p;
        p.weights = &layer.weights;
        p.biases = layer.hasBiases ? &layer.biases : nullptr;
        OperationCompiler* extActivation;
        p.activationDesc = activationToCompiled(layer.activationId, extActivation);
        result.push_back(getCompiler<DenseCompiler>(settings, p, compilers));
        if(extActivation)
          result.push_back(extActivation);
        break;
      }
      case LayerType::activation:
      {
        const ActivationLayer& layer = *static_cast<const ActivationLayer*>(node.layer);
        OperationCompiler* extActivation;
        const CompiledActivationFunctionId activationId = activationToCompiled(layer.activationId, extActivation);
        if(activationId != CompiledActivationFunctionId::linear)
        {
          ActivationCompiler::Parameters p;
          p.activationDesc = activationId;
          result.push_back(getCompiler<ActivationCompiler>(settings, p, compilers));
        }
        else if(extActivation)
          result.push_back(extActivation);
        break;
      }
      case LayerType::dropout:
        break;
      case LayerType::flatten:
        break;
      case LayerType::reshape:
        break;
      case LayerType::conv2D:
      {
        const Conv2DLayer& layer = *static_cast<const Conv2DLayer*>(node.layer);
        if(layer.padding == PaddingType::same)
        {
          OperationCompiler* extPadding = getPadding({{layer.weights.dims(0), layer.weights.dims(1)}}, layer.strides);
          if(extPadding)
            result.push_back(extPadding);
        }
        Conv2DCompiler::Parameters p;
        p.weights = &layer.weights;
        p.biases = layer.hasBiases ? &layer.biases : nullptr;
        p.strides = layer.strides;
        OperationCompiler* extActivation;
        p.activationDesc = activationToCompiled(layer.activationId, extActivation);
        result.push_back(getCompiler<Conv2DCompiler>(settings, p, compilers));
        if(extActivation)
          result.push_back(extActivation);
        break;
      }
      case LayerType::separableConv2D:
      {
        const SeparableConv2DLayer& layer = *static_cast<const SeparableConv2DLayer*>(node.layer);
        if(layer.padding == PaddingType::same)
        {
          OperationCompiler* extPadding = getPadding({{layer.depthwiseWeights.dims(0), layer.depthwiseWeights.dims(1)}}, layer.strides);
          if(extPadding)
            result.push_back(extPadding);
        }
        {
          DConv2DCompiler::Parameters p;
          p.weights = &layer.depthwiseWeights;
          p.strides = layer.strides;
          result.push_back(getCompiler<DConv2DCompiler>(settings, p, compilers));
        }
        Conv2DCompiler::Parameters p;
        p.weights = &layer.pointwiseWeights;
        p.biases = layer.hasBiases ? &layer.biases : nullptr;
        p.strides[0] = p.strides[1] = 1;
        OperationCompiler* extActivation;
        p.activationDesc = activationToCompiled(layer.activationId, extActivation);
        result.push_back(getCompiler<Conv2DCompiler>(settings, p, compilers));
        if(extActivation)
          result.push_back(extActivation);
        break;
      }
      case LayerType::depthwiseConv2D:
      {
        const DepthwiseConv2DLayer& layer = *static_cast<const DepthwiseConv2DLayer*>(node.layer);
        if(layer.padding == PaddingType::same)
        {
          OperationCompiler* extPadding = getPadding({{layer.weights.dims(0), layer.weights.dims(1)}}, layer.strides);
          if(extPadding)
            result.push_back(extPadding);
        }
        DConv2DCompiler::Parameters p;
        p.weights = &layer.weights;
        p.strides = layer.strides;
        if(layer.hasBiases)
          FAIL("CompiledNN does not support DepthwiseConv2D with biases.");
        if(layer.activationId != ActivationFunctionId::linear)
          FAIL("CompiledNN does not support DepthwiseConv2D with an activation function.");
        result.push_back(getCompiler<DConv2DCompiler>(settings, p, compilers));
        break;
      }
      case LayerType::cropping2D:
      {
        const Cropping2DLayer& layer = *static_cast<const Cropping2DLayer*>(node.layer);
        Cropping2DCompiler::Parameters p;
        p.cropping = layer.cropping;
        result.push_back(getCompiler<Cropping2DCompiler>(settings, p, compilers));
        break;
      }
      case LayerType::upSampling2D:
      {
        const UpSampling2DLayer& layer = *static_cast<const UpSampling2DLayer*>(node.layer);
        UpSampling2DCompiler::Parameters p;
        p.size = layer.size;
        if(layer.interpolation != InterpolationMethod::nearest)
          FAIL("CompiledNN does not support UpSampling2D with bilinear interpolation.");
        result.push_back(getCompiler<UpSampling2DCompiler>(settings, p, compilers));
        break;
      }
      case LayerType::zeroPadding2D:
      {
        const ZeroPadding2DLayer& layer = *static_cast<const ZeroPadding2DLayer*>(node.layer);
        ZeroPadding2DCompiler::Parameters p;
        p.padding = layer.padding;
        result.push_back(getCompiler<ZeroPadding2DCompiler>(settings, p, compilers));
        break;
      }
      case LayerType::maxPooling2D:
      case LayerType::averagePooling2D:
      {
        const Pooling2DLayer& layer = *static_cast<const Pooling2DLayer*>(node.layer);
        Pooling2DCompiler::Parameters p;
        p.padding = layer.padding;
        p.kernelSize = layer.kernelSize;
        p.strides = layer.strides;
        p.method = layer.method;
        result.push_back(getCompiler<Pooling2DCompiler>(settings, p, compilers));
        break;
      }
      case LayerType::globalMaxPooling2D:
      case LayerType::globalAveragePooling2D:
      {
        const GlobalPooling2DLayer& layer = *static_cast<const GlobalPooling2DLayer*>(node.layer);
        GlobalPooling2DCompiler::Parameters p;
        p.method = layer.method;
        p.imageSize = node.inputDimensions[0][0] * node.inputDimensions[0][1];
        result.push_back(getCompiler<GlobalPooling2DCompiler>(settings, p, compilers));
        break;
      }
      case LayerType::add:
      {
        ArithmeticCompiler::Parameters p;
        p.op = ArithmeticCompiler::add;
        result.push_back(getCompiler<ArithmeticCompiler>(settings, p, compilers));
        break;
      }
      case LayerType::subtract:
      {
        ArithmeticCompiler::Parameters p;
        p.op = ArithmeticCompiler::sub;
        result.push_back(getCompiler<ArithmeticCompiler>(settings, p, compilers));
        break;
      }
      case LayerType::multiply:
      {
        ArithmeticCompiler::Parameters p;
        p.op = ArithmeticCompiler::mul;
        result.push_back(getCompiler<ArithmeticCompiler>(settings, p, compilers));
        break;
      }
      case LayerType::average:
      {
        ArithmeticCompiler::Parameters p;
        p.op = ArithmeticCompiler::avg;
        p.inputSize = static_cast<unsigned int>(node.inputDimensions.size());
        result.push_back(getCompiler<ArithmeticCompiler>(settings, p, compilers));
        break;
      }
      case LayerType::maximum:
      {
        ArithmeticCompiler::Parameters p;
        p.op = ArithmeticCompiler::max;
        result.push_back(getCompiler<ArithmeticCompiler>(settings, p, compilers));
        break;
      }
      case LayerType::minimum:
      {
        ArithmeticCompiler::Parameters p;
        p.op = ArithmeticCompiler::min;
        result.push_back(getCompiler<ArithmeticCompiler>(settings, p, compilers));
        break;
      }
      case LayerType::concatenate:
      {
        const ConcatenateLayer& layer = *static_cast<const ConcatenateLayer*>(node.layer);
        ConcatenateCompiler::Parameters p;
        p.dimension = layer.axis >= 0 ? layer.axis : static_cast<unsigned int>(node.inputDimensions[0].size() + layer.axis);
        result.push_back(getCompiler<ConcatenateCompiler>(settings, p, compilers));
        break;
      }
      case LayerType::leakyRelu:
      {
        const LeakyReluLayer& layer = *static_cast<const LeakyReluLayer*>(node.layer);
        ActivationCompiler::Parameters p;
        p.activationDesc = ActivationFunctionDescriptor(CompiledActivationFunctionId::relu, ReluParameters(std::numeric_limits<float>::max(), layer.alpha, 0.f));
        result.push_back(getCompiler<ActivationCompiler>(settings, p, compilers));
        break;
      }
      case LayerType::elu:
      {
        const EluLayer& layer = *static_cast<const EluLayer*>(node.layer);
        ActivationCompiler::Parameters p;
        p.activationDesc = ActivationFunctionDescriptor(CompiledActivationFunctionId::elu, EluParameters(layer.alpha));
        result.push_back(getCompiler<ActivationCompiler>(settings, p, compilers));
        break;
      }
      case LayerType::thresholdedRelu:
      {
        const ThresholdedReluLayer& layer = *static_cast<const ThresholdedReluLayer*>(node.layer);
        ActivationCompiler::Parameters p;
        p.activationDesc = ActivationFunctionDescriptor(CompiledActivationFunctionId::relu, ReluParameters(std::numeric_limits<float>::max(), 0.f, layer.theta));
        result.push_back(getCompiler<ActivationCompiler>(settings, p, compilers));
        break;
      }
      case LayerType::softmax:
      {
        const SoftmaxLayer& layer = *static_cast<const SoftmaxLayer*>(node.layer);
        SoftmaxCompiler::Parameters p;
        p.dimension = layer.axis >= 0 ? layer.axis : static_cast<unsigned int>(node.inputDimensions[0].size() + layer.axis);
        result.push_back(getCompiler<SoftmaxCompiler>(settings, p, compilers));
        break;
      }
      case LayerType::relu:
      {
        const ReluLayer& layer = *static_cast<const ReluLayer*>(node.layer);
        ActivationCompiler::Parameters p;
        p.activationDesc = ActivationFunctionDescriptor(CompiledActivationFunctionId::relu, ReluParameters(layer.maxValue, layer.negativeSlope, layer.threshold));
        result.push_back(getCompiler<ActivationCompiler>(settings, p, compilers));
        break;
      }
      case LayerType::batchNormalization:
      {
        const BatchNormalizationLayer& layer = *static_cast<const BatchNormalizationLayer*>(node.layer);
        BatchNormalizationCompiler::Parameters p;
        p.factor = &layer.factor;
        p.offset = &layer.offset;
        p.inputSize = static_cast<unsigned int>(layer.factor.size());
        p.dimension = layer.axis >= 0 ? layer.axis : static_cast<unsigned int>(node.inputDimensions[0].size() + layer.axis);
        result.push_back(getCompiler<BatchNormalizationCompiler>(settings, p, compilers));
        break;
      }
      default:
        FAIL("CompiledNN does not support this layer.");
    }
    return result;
  }

  void CompiledNN::assignOperands(std::list<Operation>& operations, const std::vector<OperandLocation>& inputLocations, const std::vector<OperandLocation>& outputLocations,
                                  std::list<OperandPlaceholder>& operands, std::vector<OperandPlaceholder*>& inputPlaceholders, std::vector<OperandPlaceholder*>& outputPlaceholders)
  {
    operands.clear();

    // This function computes the number of times a tensor is named as input from the remaining nodes
    auto getRefCount = [&operations, &outputLocations](const OperandLocation& ol) -> std::size_t
    {
      std::size_t refCount = 0;
      for(const Operation& op : operations)
        for(const OperandLocation& ol2 : op.inputs)
          if(ol == ol2)
            ++refCount;
      for(const OperandLocation& ol2 : outputLocations)
        if(ol == ol2)
          ++refCount;
      return refCount;
    };

    // Create placeholders for input operands (has to be a list because there are pointers to it)
    for(std::size_t i = 0; i < inputDimensions.size(); ++i)
    {
      operands.emplace_back(inputLocations[i], std::accumulate(inputDimensions[i].begin(), inputDimensions[i].end(), 1u, std::multiplies<>()), getRefCount(inputLocations[i]));
      inputPlaceholders[i] = &operands.back();
      for(std::size_t j = 0; j < outputLocations.size(); ++j)
        if(outputLocations[j] == inputLocations[i])
          outputPlaceholders[j] = &operands.back();
    }

    // This function decreases the reference count of all tensors that have been the input to a node.
    auto decreaseRefCounters = [&operands](const std::vector<OperandLocation>& inputs)
    {
      for(const OperandLocation& ol : inputs)
        for(OperandPlaceholder& op : operands)
          if(op.refCount && ol == op.location)
            --op.refCount;
    };

    // This function gets an existing input tensor at a specific location
    auto lookupInputOperand = [&operands](const OperandLocation& location) -> OperandPlaceholder*
    {
      for(OperandPlaceholder& op : operands)
        if(op.refCount && location == op.location)
          return &op;
      return nullptr;
    };

    // This function gets a free tensor that can be used to write the output of an operation to it
    auto lookupOutputOperand = [&operands, &getRefCount, &outputLocations, &outputPlaceholders](const OperandLocation& location, const std::vector<unsigned int>& dimensions) -> OperandPlaceholder*
    {
      const std::size_t requiredSize = std::accumulate(dimensions.begin(), dimensions.end(), 1u, std::multiplies<>());
      OperandPlaceholder* maxCapacityOperand = nullptr;
      for(OperandPlaceholder& op : operands)
      {
        // Do not overwrite used tensors
        if(op.refCount)
          continue;
        // If there is a free tensor with enough capacity, use it
        if(op.requiredSize >= requiredSize)
        {
          op.location = location;
          for(std::size_t i = 0; i < outputLocations.size(); ++i)
            if(outputLocations[i] == location)
              outputPlaceholders[i] = &op;
          op.refCount = getRefCount(location);
          return &op;
        }
        // Otherwise, try to find the tensor with maximum size (so overall memory usage is minimized)
        if(!maxCapacityOperand || op.requiredSize > maxCapacityOperand->requiredSize)
          maxCapacityOperand = &op;
      }
      if(maxCapacityOperand)
      {
        maxCapacityOperand->location = location;
        maxCapacityOperand->requiredSize = requiredSize;
        maxCapacityOperand->refCount = getRefCount(location);
        for(std::size_t i = 0; i < outputLocations.size(); ++i)
          if(outputLocations[i] == location)
            outputPlaceholders[i] = maxCapacityOperand;
        return maxCapacityOperand;
      }
      // Create a new tensor
      operands.emplace_back(location, requiredSize, getRefCount(location));
      for(std::size_t i = 0; i < outputLocations.size(); ++i)
        if(outputLocations[i] == location)
          outputPlaceholders[i] = &operands.back();
      return &operands.back();
    };

    // Assign operands to all operations (and determine their required size)
    for(Operation& op : operations)
    {
      op.inputOperands.resize(op.inputs.size());
      for(std::size_t i = 0; i < op.inputs.size(); ++i)
        op.inputOperands[i] = lookupInputOperand(op.inputs[i]);

      // Check which inputs the compiler wants to reuse as outputs, but only offer it inputs that will not be used by other nodes anymore
      std::vector<std::size_t> inputIndices;
      for(std::size_t i = 0; i < op.inputs.size(); ++i)
        if(op.inputOperands[i]->refCount == 1)
          inputIndices.push_back(i);
      auto outputMapping = op.compiler->routeIO(inputIndices, op.inputDimensions);

      // Either alias the inputs as outputs as requested by the compiler or request a new operand
      op.outputOperands.resize(op.outputs.size());
      for(std::size_t i = 0; i < op.outputs.size(); ++i)
      {
        if(i < outputMapping.size() && outputMapping[i] < op.inputs.size())
        {
          ASSERT(op.inputOperands[outputMapping[i]]->refCount == 1);
          op.outputOperands[i] = op.inputOperands[outputMapping[i]];
          const std::size_t requiredSize = std::accumulate(op.outputDimensions[i].begin(), op.outputDimensions[i].end(), 1u, std::multiplies<>());
          if(requiredSize > op.outputOperands[i]->requiredSize)
            op.outputOperands[i]->requiredSize = requiredSize;
          op.outputOperands[i]->location = op.outputs[i];
          for(std::size_t j = 0; j < outputLocations.size(); ++j)
            if(outputLocations[j] == op.outputs[i])
              outputPlaceholders[j] = op.outputOperands[i];
          op.outputOperands[i]->refCount = getRefCount(op.outputs[i]);
        }
        else
          op.outputOperands[i] = lookupOutputOperand(op.outputs[i], op.outputDimensions[i]);
      }

      // Decrease the reference counters of all tensors that have been used as input to this node
      decreaseRefCounters(op.inputs);
    }
  }

  void CompiledNN::allocateTensors(std::list<OperandPlaceholder>& operands)
  {
    std::size_t i = 0;
    tensors.resize(operands.size());
    for(OperandPlaceholder& operand : operands)
    {
      tensors[i].reserve(operand.requiredSize + 3);
      operand.allocatedTensor = &tensors[i];
      ++i;
    }
  }

  class CompilationErrorHandler : public ErrorHandler
  {
    void handleError(Error err, const char* message, BaseEmitter* origin) override
    {
      FAIL(message);
    }
  };

  void CompiledNN::generateCode(const std::list<Operation>& operations, const CompilerMap& compilers, ActivationFunctionHandler& afHandler)
  {
    // Initialize assembler
    CodeHolder code;
    code.init(Global::getAsmjitRuntime().codeInfo());
    x86::Assembler a(&code);
    CompilationErrorHandler errorHandler;
    a.setErrorHandler(&errorHandler);

    // Emit Prolog
    if(!operations.empty())
    {
      a.enter(imm(24u), imm(0u)); // Reserve stack space for up to six 32-bit variables, indexed as a.ptr_zbp(-i*4,4)
      a.push(a.zbx());
#if ASMJIT_ARCH_X86 != 64 || defined(_WIN32)
      // CDECL or Windows64
      a.push(a.zdi());
      a.push(a.zsi());
#endif
    }

    // Declare constant labels
    for(auto& compilerType : compilers)
      for(auto& compiler : compilerType.second)
      {
        if(!compiler->refCount)
          continue;
        for(NetworkConstants& cs : compiler->constants)
          if(cs.data.size())
            cs.label = a.newLabel();
      }

    // Compile operations
    for(const Operation& op : operations)
    {
      // Set references to operands
      std::vector<TensorPointerXf> inputPointers(op.inputOperands.size());
      for(std::size_t i = 0; i < op.inputOperands.size(); ++i)
      {
        op.inputOperands[i]->allocatedTensor->reshape(op.inputDimensions[i]);
        inputPointers[i] = TensorPointerXf(*op.inputOperands[i]->allocatedTensor);
      }
      std::vector<TensorPointerXf> outputPointers(op.outputOperands.size());
      for(std::size_t i = 0; i < op.outputs.size(); ++i)
      {
        op.outputOperands[i]->allocatedTensor->reshape(op.outputDimensions[i]);
        outputPointers[i] = TensorPointerXf(*op.outputOperands[i]->allocatedTensor);
      }

      // Compile the operation
      op.compiler->compile(a, afHandler, inputPointers, outputPointers);
    }

    // Emit epilog
    if(!operations.empty())
    {
#if ASMJIT_ARCH_X86 != 64 || defined(_WIN32)
      a.pop(a.zsi());
      a.pop(a.zdi());
#endif
      a.pop(a.zbx());
      a.leave();
    }
    a.ret();

    // Store constants
    afHandler.compileData(a);
    for(auto& compilerType : compilers)
      for(auto& compiler : compilerType.second)
      {
        if(!compiler->refCount)
          continue;
        for(NetworkConstants& cs : compiler->constants)
          if(!cs.data.empty())
          {
            a.align(AlignMode::kAlignZero, 16);
            a.bind(cs.label);
            for(const float c : cs.data)
              a.dfloat(c);
          }
      }

    // Bind function
    VERIFY(static_cast<ErrorCode>(Global::getAsmjitRuntime().add<FnType>(&applyFunction, &code)) == ErrorCode::kErrorOk);
  }

  void CompiledNN::compilerBackend(std::list<Operation>& operations, const CompilerMap& compilers,
                                   const std::vector<OperandLocation>& inputLocations, const std::vector<OperandLocation>& outputLocations,
                                   const CompilationSettings& settings)
  {
    // Assign operands to placeholders
    std::list<OperandPlaceholder> operands;
    std::vector<OperandPlaceholder*> inputPlaceholders(inputLocations.size()), outputPlaceholders(outputLocations.size());
    assignOperands(operations, inputLocations, outputLocations, operands, inputPlaceholders, outputPlaceholders);

    // Actually allocate memory for all tensors and link it to the operands
    allocateTensors(operands);

    // Initialize compilers
    for(auto& compilerType : compilers)
      for(auto& compiler : compilerType.second)
        if(compiler->refCount)
          compiler->initialize();

    // Initialize activation functions
    ActivationFunctionHandler afHandler(settings);

    // Generate the function
    generateCode(operations, compilers, afHandler);

    // Set input/output pointers
    inputTensors.resize(inputPlaceholders.size());
    outputTensors.resize(outputPlaceholders.size());
    for(std::size_t i = 0; i < inputTensors.size(); ++i)
      inputTensors[i] = inputPlaceholders[i]->allocatedTensor;
    for(std::size_t i = 0; i < outputTensors.size(); ++i)
      outputTensors[i] = outputPlaceholders[i]->allocatedTensor;
  }

  void CompiledNN::compile(const std::string& filename, const CompilationSettings& settings)
  {
    compile(Model(filename), settings);
  }

  void CompiledNN::compile(const Model& specification, const CompilationSettings& settings)
  {
    // Reset attributes
    if(applyFunction)
    {
      Global::getAsmjitRuntime().release(applyFunction);
      applyFunction = nullptr;
    }

    // Constrict settings to CPU features
    const CompilationSettings effSettings = settings.constricted();

    // Set network input/output dimensions
    const std::vector<TensorLocation>& inputs = specification.getInputs();
    const std::vector<TensorLocation>& outputs = specification.getOutputs();
    inputDimensions.resize(inputs.size());
    for(std::size_t i = 0; i < inputs.size(); ++i)
      inputDimensions[i] = inputs[i].layer->nodes[inputs[i].nodeIndex].outputDimensions[inputs[i].tensorIndex];
    outputDimensions.resize(outputs.size());
    for(std::size_t i = 0; i < outputs.size(); ++i)
      outputDimensions[i] = outputs[i].layer->nodes[outputs[i].nodeIndex].outputDimensions[outputs[i].tensorIndex];

    // Create symbolic locations of the inputs (it is safe to assume that they are actually not aliased since Keras does not allow to create such models (and it does not make sense))
    std::vector<OperandLocation> inputLocations;
    for(std::size_t i = 0; i < inputs.size(); ++i)
      inputLocations.emplace_back(nullptr, static_cast<unsigned int>(i));

    // Create operations for input converters (if required) and initialize mapping from operand locations to tensor locations
    CompilerMap compilers;
    std::list<Operation> operations;
    std::unordered_map<TensorLocation, OperandLocation, TensorLocationHasher> locationMap;
    for(std::size_t i = 0; i < inputs.size(); ++i)
    {
      if(specification.isInputUInt8(i))
      {
        UInt8InputCompiler::Parameters p;
        p.batchNormalization = nullptr;
        const OperationCompiler* uInt8InputCompiler = getCompiler<UInt8InputCompiler>(effSettings, p, compilers);
        operations.emplace_back(uInt8InputCompiler);
        Operation& operation = operations.back();
        operation.inputs = {inputLocations[i]};
        operation.inputDimensions = {inputDimensions[i]};
        operation.outputDimensions = uInt8InputCompiler->calcOutputDimensions(operation.inputDimensions);
        ASSERT(operation.inputDimensions == operation.outputDimensions);
        operation.outputs.emplace_back(&operation, 0);
        locationMap.emplace(inputs[i], operation.outputs.back());
      }
      else
        locationMap.emplace(inputs[i], inputLocations[i]);
    }

    // Collect all nodes and initialize the number of times each tensor is required
    std::list<const Node*> remainingNodes;
    std::unordered_map<TensorLocation, unsigned int, TensorLocationHasher> locationRefCounters;
    for(auto& layer : specification.getLayers())
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
          for(const TensorLocation& tl2 : inputs)
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

        for(std::size_t i = 0; i < node.inputs.size(); ++i)
          ++locationRefCounters[node.inputs[i]];

        remainingNodes.emplace_back(&node);
      }
    }

    // If a location is an output, it is required once more
    for(std::size_t i = 0; i < outputs.size(); ++i)
      ++locationRefCounters[outputs[i]];

    // Create operations for all nodes
    for(const Node* node : remainingNodes)
    {
      std::vector<OperandLocation> nodeInputs;
      nodeInputs.reserve(node->inputs.size());
      for(std::size_t i = 0; i < node->inputs.size(); ++i)
      {
        auto it = locationMap.find(node->inputs[i]);
        // TODO: This happens if the layers are not topologically sorted and there are forward references to other nodes.
        if(it == locationMap.end())
          FAIL("This net is not supported yet.");
        nodeInputs.push_back(it->second);
      }

      auto opCompilers = generateCompilers(effSettings, *node, compilers);

      // Eliminate operations if they can be integrated into previous ones
      std::size_t compilerOffset;
      for(compilerOffset = 0; compilerOffset < opCompilers.size(); ++compilerOffset)
      {
        const BatchNormalizationCompiler* bnCompiler = dynamic_cast<const BatchNormalizationCompiler*>(opCompilers[compilerOffset]);
        if(bnCompiler && node->inputs.size() == 1 && locationRefCounters[node->inputs[0]] == 1 && nodeInputs[0].provider)
        {
          const UInt8InputCompiler* uInt8InputCompiler = dynamic_cast<const UInt8InputCompiler*>(nodeInputs[0].provider->compiler);
          if(uInt8InputCompiler && !uInt8InputCompiler->p.batchNormalization && bnCompiler->p.dimension == 2)
          {
            --bnCompiler->refCount;
            --uInt8InputCompiler->refCount;
            UInt8InputCompiler::Parameters p = uInt8InputCompiler->p;
            p.batchNormalization = &bnCompiler->p;
            nodeInputs[0].provider->compiler = getCompiler<UInt8InputCompiler>(effSettings, p, compilers);
            continue;
          }
          const DenseCompiler* denseCompiler = dynamic_cast<const DenseCompiler*>(nodeInputs[0].provider->compiler);
          if(denseCompiler && !denseCompiler->p.postBatchNormalization)
          {
            --bnCompiler->refCount;
            --denseCompiler->refCount;
            DenseCompiler::Parameters p = denseCompiler->p;
            p.postBatchNormalization = &bnCompiler->p;
            nodeInputs[0].provider->compiler = getCompiler<DenseCompiler>(effSettings, p, compilers);
            continue;
          }
          const Conv2DCompiler* conv2DCompiler = dynamic_cast<const Conv2DCompiler*>(nodeInputs[0].provider->compiler);
          if(conv2DCompiler && !conv2DCompiler->p.batchNormalization && bnCompiler->p.dimension == 2)
          {
            --bnCompiler->refCount;
            --conv2DCompiler->refCount;
            Conv2DCompiler::Parameters p = conv2DCompiler->p;
            p.batchNormalization = &bnCompiler->p;
            nodeInputs[0].provider->compiler = getCompiler<Conv2DCompiler>(effSettings, p, compilers);
            continue;
          }
        }

        const ActivationCompiler* activationCompiler = dynamic_cast<const ActivationCompiler*>(opCompilers[compilerOffset]);
        if(activationCompiler && node->inputs.size() == 1 && locationRefCounters[node->inputs[0]] == 1 && nodeInputs[0].provider)
        {
          const DenseCompiler* denseCompiler = dynamic_cast<const DenseCompiler*>(nodeInputs[0].provider->compiler);
          if(denseCompiler && denseCompiler->p.postActivation.id == CompiledActivationFunctionId::linear)
          {
            --activationCompiler->refCount;
            --denseCompiler->refCount;
            DenseCompiler::Parameters p = denseCompiler->p;
            p.postActivation = activationCompiler->p.activationDesc;
            nodeInputs[0].provider->compiler = getCompiler<DenseCompiler>(effSettings, p, compilers);
            continue;
          }
          const Conv2DCompiler* conv2DCompiler = dynamic_cast<const Conv2DCompiler*>(nodeInputs[0].provider->compiler);
          if(conv2DCompiler && conv2DCompiler->p.postActivation.id == CompiledActivationFunctionId::linear)
          {
            --activationCompiler->refCount;
            --conv2DCompiler->refCount;
            Conv2DCompiler::Parameters p = conv2DCompiler->p;
            p.postActivation = activationCompiler->p.activationDesc;
            nodeInputs[0].provider->compiler = getCompiler<Conv2DCompiler>(effSettings, p, compilers);
            continue;
          }
        }

        break;
      }

      for(std::size_t i = compilerOffset; i < opCompilers.size(); ++i)
      {
        const Operation* prevOperation = (i == compilerOffset) ? nullptr : &operations.back();
        operations.emplace_back(opCompilers[i]);
        Operation& operation = operations.back();
        if(i == compilerOffset)
        {
          operation.inputs = nodeInputs;
          operation.inputDimensions = node->inputDimensions;
        }
        else
        {
          operation.inputs = prevOperation->outputs;
          operation.inputDimensions = prevOperation->outputDimensions;
        }
        operation.outputDimensions = opCompilers[i]->calcOutputDimensions(operation.inputDimensions);
        for(std::size_t j = 0; j < operation.outputDimensions.size(); ++j)
          operation.outputs.emplace_back(&operations.back(), static_cast<unsigned int>(j));
      }

      if(compilerOffset == opCompilers.size())
      {
        ASSERT(node->inputs.size() == node->outputs.size());
        for(std::size_t i = 0; i < node->outputs.size(); ++i)
          locationMap.emplace(node->outputs[i], nodeInputs[i]);
      }
      else
      {
        ASSERT(operations.back().outputDimensions == node->outputDimensions);
        for(std::size_t i = 0; i < node->outputs.size(); ++i)
          locationMap.emplace(node->outputs[i], operations.back().outputs[i]);
      }

      for(std::size_t i = 0; i < node->inputs.size(); ++i)
        --locationRefCounters[node->inputs[i]];
    }

    std::vector<OperandLocation> outputLocations;
    outputLocations.reserve(outputs.size());
    for(const TensorLocation& output : outputs)
    {
      auto it = locationMap.find(output);
      ASSERT(it != locationMap.end());
      outputLocations.push_back(it->second);
    }

    // Do the actual compilation process
    compilerBackend(operations, compilers, inputLocations, outputLocations, effSettings);
  }

  void CompiledNN::compile(const Node& node, const CompilationSettings& settings)
  {
    // Reset attributes
    if(applyFunction)
    {
      Global::getAsmjitRuntime().release(applyFunction);
      applyFunction = nullptr;
    }

    // Constrict settings to CPU features
    const CompilationSettings effSettings = settings.constricted();

    // Set network input/output dimensions
    inputDimensions = node.inputDimensions;
    outputDimensions = node.outputDimensions;

    // Create symbolic locations of the inputs
    std::vector<OperandLocation> inputLocations;
    for(std::size_t i = 0; i < node.inputs.size(); ++i)
      inputLocations.emplace_back(nullptr, static_cast<unsigned int>(i));

    // Create compilers for the operations that the node requires
    CompilerMap compilers;
    std::vector<OperationCompiler*> opCompilers = generateCompilers(effSettings, node, compilers);

    // Create a representation of an operation for each sub-compiled operation
    std::list<Operation> operations;
    bool first = true;
    for(OperationCompiler* compiler : opCompilers)
    {
      const Operation* prevOperation = first ? nullptr : &operations.back();
      operations.emplace_back(compiler);
      Operation& operation = operations.back();
      if(first)
      {
        operation.inputs = inputLocations;
        operation.inputDimensions = inputDimensions;
      }
      else
      {
        operation.inputs = prevOperation->outputs;
        operation.inputDimensions = prevOperation->outputDimensions;
      }
      operation.outputDimensions = compiler->calcOutputDimensions(operation.inputDimensions);
      for(std::size_t i = 0; i < operation.outputDimensions.size(); ++i)
        operation.outputs.emplace_back(&operations.back(), static_cast<unsigned int>(i));
      first = false;
    }

    std::vector<OperandLocation> outputLocations;
    if(operations.empty())
    {
      ASSERT(node.inputs.size() == node.outputs.size());
      outputLocations = inputLocations;
    }
    else
    {
      ASSERT(operations.back().outputDimensions == outputDimensions);
      for(std::size_t i = 0; i < node.outputs.size(); ++i)
        outputLocations.emplace_back(&operations.back(), static_cast<unsigned int>(i));
    }

    // Do the actual compilation process
    compilerBackend(operations, compilers, inputLocations, outputLocations, settings);
  }
}
