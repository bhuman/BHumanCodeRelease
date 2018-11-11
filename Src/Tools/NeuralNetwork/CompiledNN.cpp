/**
 * Implements a class that compiles neural networks to optimized X86 machine
 * code and applies them on input data.
 *
 * @author Felix Thielke
 */

#include "CompiledNN.h"
#include "Model.h"
#include "CompiledNN/CompiledNNImpl.h"
#include "Tools/Global.h"

namespace NeuralNetwork
{
  using namespace asmjit;
  using namespace CompiledNNImpl;

  CompiledNN::~CompiledNN()
  {
    if(applyFunction)
      Global::getAsmjitRuntime().release(applyFunction);
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

    // Set network input/output dimensions
    inputDimensions = specification.getLayers().front()->inputDimensions;
    outputDimensions = specification.getLayers().back()->outputDimensions;
    size_t bufferSize = static_cast<size_t>(inputDimensions[0] * inputDimensions[1] * inputDimensions[2]) + 3;

    // Constrict settings to CPU features
    const CompilationSettings effSettings = settings.constricted();

    // Initialize layer compilers
    std::vector<std::unique_ptr<LayerCompiler>> compilers;
    for(auto it = specification.getLayers().cbegin(); it != specification.getLayers().cend(); it++)
    {
      const Layer& layer = **it;

      if(layer.type == LayerType::batchNormalization)
      {
        if(effSettings.uint8Input && it == specification.getLayers().cbegin())
          compilers.push_back(std::make_unique<UInt8InputCompiler>(effSettings, static_cast<const BatchNormalizationLayer*>(&layer)));
        else
          compilers.push_back(std::make_unique<BatchNormalizationLayerCompiler>(effSettings, static_cast<const BatchNormalizationLayer&>(layer)));
      }
      else
      {
        if(effSettings.uint8Input && it == specification.getLayers().cbegin())
        {
          compilers.push_back(std::make_unique<UInt8InputCompiler>(effSettings, nullptr, layer.inputDimensions));
          compilers.back()->initialize();
        }

        switch(layer.type)
        {
          case LayerType::dense:
            compilers.push_back(std::make_unique<DenseLayerCompiler>(effSettings, static_cast<const DenseLayer&>(layer)));
            if(it + 1 != specification.getLayers().cend() && (*(it + 1))->type == LayerType::batchNormalization)
            {
              it++;
              static_cast<DenseLayerCompiler&>(*compilers.back()).batchNormalization = static_cast<const BatchNormalizationLayer*>(it->get());
            }
            if(it + 1 != specification.getLayers().cend() && (*(it + 1))->type == LayerType::activation)
            {
              it++;
              static_cast<DenseLayerCompiler&>(*compilers.back()).postActivation = static_cast<const ActivationLayer&>(**it).activationId;
            }
            /*
            // TODO
            if (it + 1 != specification.getLayers().cend() && (*(it + 1))->type == LayerType::elu)
            {
              it++;
              static_cast<DenseLayerCompiler&>(*compilers.back()).elu = static_cast<const EluLayer*>(it->get());
            }
             */
            break;
          case LayerType::conv2D:
            compilers.push_back(std::make_unique<Conv2DLayerCompiler>(effSettings, static_cast<const Conv2DLayer&>(layer)));
            bufferSize = std::max(bufferSize, static_cast<Conv2DLayerCompiler&>(*compilers.back()).bufferSizeNeeded());
            if(static_cast<const Conv2DLayer&>(layer).activationId == ActivationFunctionId::linear)
            {
              // TODO this breaks for some reason, if it's not meant to be compiled together
              if(it + 1 != specification.getLayers().cend() && (*(it + 1))->type == LayerType::batchNormalization)
              {
                it++;
                static_cast<Conv2DLayerCompiler&>(*compilers.back()).batchNormalization = static_cast<const BatchNormalizationLayer*>(it->get());
              }
              if(it + 1 != specification.getLayers().cend() && (*(it + 1))->type == LayerType::activation)
              {
                it++;
                static_cast<Conv2DLayerCompiler&>(*compilers.back()).postActivation = static_cast<const ActivationLayer&>(**it).activationId;
              }
              /*
              // TODO
              if (it + 1 != specification.getLayers().cend() && (*(it + 1))->type == LayerType::elu)
              {
              it++;
              static_cast<Conv2DLayerCompiler&>(*compilers.back()).elu = static_cast<const EluLayer*>(it->get());
              }
               */
            }
            break;
          case LayerType::sconv2D:
            compilers.push_back(std::make_unique<SConv2DLayerCompiler>(effSettings, static_cast<const SConv2DLayer&>(layer)));
            bufferSize = std::max(bufferSize, static_cast<SConv2DLayerCompiler&>(*compilers.back()).bufferSizeNeeded());
            break;
          case LayerType::reshape:
            // Do nothing
            continue;
          case LayerType::elu:
            // TODO
            ASSERT(false);
            break;
          case LayerType::activation:
            compilers.push_back(std::make_unique<ActivationLayerCompiler>(effSettings, static_cast<const ActivationLayer&>(layer)));
            break;
          case LayerType::pooling2D:
            compilers.push_back(std::make_unique<Pooling2DLayerCompiler>(effSettings, static_cast<const Pooling2DLayer&>(layer)));
            break;
          case LayerType::batchNormalization:
            compilers.push_back(std::make_unique<BatchNormalizationLayerCompiler>(effSettings, static_cast<const BatchNormalizationLayer&>(layer)));
            break;
          case LayerType::softmax:
            compilers.push_back(std::make_unique<SoftmaxLayerCompiler>(effSettings, static_cast<const SoftmaxLayer&>(layer)));
            break;
          default:
            ASSERT(false);
        }
      }

      compilers.back()->initialize();
      bufferSize = std::max(bufferSize, static_cast<size_t>(layer.outputDimensions[0] * layer.outputDimensions[1] * layer.outputDimensions[2]) + 3);
    }

    // Reserve tensor size
    for(Tensor3& t : tensors)
      t.reserve(bufferSize);

    // Initialize activation functions
    ActivationFunctionHandler afHandler(effSettings);

    // Initialize assembler
    CodeHolder code;
    code.init(Global::getAsmjitRuntime().getCodeInfo());
    X86Assembler a(&code);

    // Emit Prolog
    a.enter(imm_u(24), imm_u(0)); // Reserve stack space for up to six 32-bit variables, indexed as a.ptr_zbp(-i*4,4)
    a.push(a.zbx());
#if !ASMJIT_ARCH_64BIT || ASMJIT_OS_WINDOWS
    // CDECL or Windows64
    a.push(a.zdi());
    a.push(a.zsi());
#endif

    // Declare constant labels
    for(auto& compiler : compilers)
      for(NetworkConstants& cs : compiler->constants)
        if(cs.data.size())
          cs.label = a.newLabel();

    // Compile layers
    unsigned int currentTensor = inputTensorIx;
    for(auto& compiler : compilers)
    {
      if(compiler->isInplace())
        compiler->compile(a, afHandler, tensors[currentTensor].data());
      else
      {
        const unsigned int nextTensor = (currentTensor + 1) % tensors.size();
        compiler->compile(a, afHandler, tensors[currentTensor].data(), tensors[nextTensor].data());
        currentTensor = nextTensor;
      }
    }
    outputTensorIx = currentTensor;
    if(inputTensorIx != outputTensorIx)
    {
      tensors[inputTensorIx].reshape(inputDimensions);
      tensors[outputTensorIx].reshape(outputDimensions);
    }

    // Emit epilog
#if !ASMJIT_ARCH_64BIT || ASMJIT_OS_WINDOWS
    a.pop(a.zsi());
    a.pop(a.zdi());
#endif
    a.pop(a.zbx());
    a.leave();
    a.ret();

    // Store constants
    afHandler.compileData(a);
    for(auto& compiler : compilers)
    {
      for(NetworkConstants& cs : compiler->constants)
      {
        if(cs.data.size())
        {
          a.align(AlignMode::kAlignZero, 16);
          a.bind(cs.label);
          for(const float c : cs.data)
            a.dfloat(c);
        }
      }
    }

    // Delete compilers
    compilers.clear();

    // Bind function
    ErrorCode err = static_cast<ErrorCode>(a.getLastError());
    ASSERT(err == ErrorCode::kErrorOk);
    err = static_cast<ErrorCode>(Global::getAsmjitRuntime().add<FnType>(&applyFunction, &code));
    ASSERT(err == ErrorCode::kErrorOk);
    if(err != ErrorCode::kErrorOk)
      applyFunction = nullptr;
  }

  void CompiledNN::compile(const Layer& layer, const CompilationSettings& settings)
  {
    // Reset attributes
    if(applyFunction)
    {
      Global::getAsmjitRuntime().release(applyFunction);
      applyFunction = nullptr;
    }

    // Set network input/output dimensions
    inputDimensions = layer.inputDimensions;
    outputDimensions = layer.outputDimensions;

    // Constrict settings to CPU features
    const CompilationSettings effSettings = settings.constricted();

    // Initialize compiler
    std::unique_ptr<LayerCompiler> compiler;
    switch(layer.type)
    {
      case LayerType::dense:
        compiler = std::make_unique<DenseLayerCompiler>(effSettings, static_cast<const DenseLayer&>(layer));
        break;
      case LayerType::conv2D:
        compiler = std::make_unique<Conv2DLayerCompiler>(effSettings, static_cast<const Conv2DLayer&>(layer));
        break;
      case LayerType::sconv2D:
        compiler = std::make_unique<SConv2DLayerCompiler>(effSettings, static_cast<const SConv2DLayer&>(layer));
        break;
      case LayerType::reshape:
        // Do nothing
        break;
      case LayerType::elu:
        // TODO
        ASSERT(false);
        break;
      case LayerType::activation:
        compiler = std::make_unique<ActivationLayerCompiler>(effSettings, static_cast<const ActivationLayer&>(layer));
        break;
      case LayerType::pooling2D:
        compiler = std::make_unique<Pooling2DLayerCompiler>(effSettings, static_cast<const Pooling2DLayer&>(layer));
        break;
      case LayerType::batchNormalization:
        compiler = std::make_unique<BatchNormalizationLayerCompiler>(effSettings, static_cast<const BatchNormalizationLayer&>(layer));
        break;
      case LayerType::softmax:
        compiler = std::make_unique<SoftmaxLayerCompiler>(effSettings, static_cast<const SoftmaxLayer&>(layer));
        break;
      default:
        ASSERT(false);
    }
    if(compiler.get())
      compiler->initialize();

    // Reserve tensor size
    size_t bufferSize = static_cast<size_t>(std::max(outputDimensions[0] * outputDimensions[1] * outputDimensions[2], inputDimensions[0] * inputDimensions[1] * inputDimensions[2]) + 3);
    if(layer.type == LayerType::conv2D)
      bufferSize = std::max(bufferSize, static_cast<const Conv2DLayerCompiler&>(*compiler).bufferSizeNeeded());
    if(layer.type == LayerType::sconv2D)
      bufferSize = std::max(bufferSize, static_cast<const SConv2DLayerCompiler&>(*compiler).bufferSizeNeeded());
    if(!compiler.get() || compiler->isInplace())
      tensors[inputTensorIx].reserve(bufferSize);
    else
    {
      for(Tensor3& t : tensors)
        t.reserve(bufferSize);
    }

    // Initialize activation functions
    ActivationFunctionHandler afHandler(effSettings);

    // Initialize assembler
    CodeHolder code;
    code.init(Global::getAsmjitRuntime().getCodeInfo());
    X86Assembler a(&code);

    // Emit Prolog
    a.enter(imm_u(24), imm_u(0)); // Reserve stack space for up to six 32-bit variables, indexed as a.ptr_zbp(-i*4,4)
    a.push(a.zbx());
#if !ASMJIT_ARCH_64BIT || ASMJIT_OS_WINDOWS
    // CDECL or Windows64
    a.push(a.zdi());
    a.push(a.zsi());
#endif

    if(compiler.get())
    {
      // Declare constant labels
      for(NetworkConstants& cs : compiler->constants)
        if(cs.data.size())
          cs.label = a.newLabel();

      // Compile layer
      if(compiler->isInplace())
      {
        outputTensorIx = inputTensorIx;
        compiler->compile(a, afHandler, tensors[inputTensorIx].data());
      }
      else
      {
        outputTensorIx = (inputTensorIx + 1) % tensors.size();
        compiler->compile(a, afHandler, tensors[inputTensorIx].data(), tensors[outputTensorIx].data());
      }

      if(inputTensorIx != outputTensorIx)
      {
        tensors[inputTensorIx].reshape(inputDimensions);
        tensors[outputTensorIx].reshape(outputDimensions);
      }
    }
    else
      outputTensorIx = inputTensorIx;

    // Emit epilog
#if !ASMJIT_ARCH_64BIT || ASMJIT_OS_WINDOWS
    a.pop(a.zsi());
    a.pop(a.zdi());
#endif
    a.pop(a.zbx());
    a.leave();
    a.ret();

    // Store constants
    afHandler.compileData(a);
    if(compiler.get())
    {
      for(NetworkConstants& cs : compiler->constants)
      {
        if(cs.data.size())
        {
          a.align(AlignMode::kAlignZero, 16);
          a.bind(cs.label);
          for(const float c : cs.data)
            a.dfloat(c);
        }
      }
    }

    // Bind function
    ErrorCode err = static_cast<ErrorCode>(a.getLastError());
    ASSERT(err == ErrorCode::kErrorOk);
    err = static_cast<ErrorCode>(Global::getAsmjitRuntime().add<FnType>(&applyFunction, &code));
    ASSERT(err == ErrorCode::kErrorOk);
    if(err != ErrorCode::kErrorOk)
      applyFunction = nullptr;
  }
}
