/**
 * @author Felix Thielke
 */

#pragma once

#include "CompiledNNImplBase.h"
#include "CompilationSettings.h"
#include <functional>

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    enum class CompiledActivationFunctionId
    {
      linear,
      relu,
      tanH,
      sigmoid,
      hardSigmoid,
      elu,
      selu,
      exponential,
      softsign
    };

    class ActivationFunctionParameters
    {
    public:
      virtual ~ActivationFunctionParameters() = default;
      virtual bool operator==(const ActivationFunctionParameters* other) const = 0;

      template<class T>
      inline const T& asType() const
      {
        const T* p = dynamic_cast<const T*>(this);
        ASSERT(p);
        return *p;
      }
    };

    class ReluParameters : public ActivationFunctionParameters
    {
    public:
      const float maxValue;
      const float negativeSlope;
      const float threshold;

      ReluParameters() : maxValue(std::numeric_limits<float>::max()), negativeSlope(0.f), threshold(0.f) {}
      ReluParameters(const float maxValue, const float negativeSlope, const float threshold) : maxValue(maxValue), negativeSlope(negativeSlope), threshold(threshold) {}

      bool operator==(const ActivationFunctionParameters* other) const override
      {
        const ReluParameters* rhs = dynamic_cast<const ReluParameters*>(other);
        return rhs != nullptr && rhs->maxValue == maxValue && rhs->negativeSlope == negativeSlope && rhs->threshold == threshold;
      }
    };

    class EluParameters : public ActivationFunctionParameters
    {
    public:
      const float alpha;

      EluParameters() : alpha(1.f) {}
      EluParameters(const float alpha) : alpha(alpha) {}

      bool operator==(const ActivationFunctionParameters* other) const override
      {
        const EluParameters* rhs = dynamic_cast<const EluParameters*>(other);
        return rhs != nullptr && rhs->alpha == alpha;
      }
    };

    struct ActivationFunctionDescriptor
    {
      const CompiledActivationFunctionId id;
      const ActivationFunctionParameters* const p;

      ActivationFunctionDescriptor() : ActivationFunctionDescriptor(CompiledActivationFunctionId::linear) {}
      ActivationFunctionDescriptor(const ActivationFunctionDescriptor& other) : ActivationFunctionDescriptor(other.id, *(other.p)) {}
      ActivationFunctionDescriptor(const CompiledActivationFunctionId id) : id(id), p(createParameters(id, nullptr)) {}
      ActivationFunctionDescriptor(const CompiledActivationFunctionId id, const ActivationFunctionParameters& p) : id(id), p(createParameters(id, &p)) {}
      ~ActivationFunctionDescriptor()
      {
        if(p)
          delete p;
      }

      bool operator==(const CompiledActivationFunctionId id) const
      {
        return *this == ActivationFunctionDescriptor(id);
      }

      bool operator!=(const CompiledActivationFunctionId id) const
      {
        return !(*this == id);
      }

      bool operator==(const ActivationFunctionDescriptor& other) const
      {
        return id == other.id && (p == nullptr ? other.p == nullptr : *p == other.p);
      }

      bool operator!=(const ActivationFunctionDescriptor& other) const
      {
        return !(*this == other);
      }

      ActivationFunctionDescriptor& operator=(const CompiledActivationFunctionId id)
      {
        return *this = ActivationFunctionDescriptor(id);
      }

      ActivationFunctionDescriptor& operator=(const ActivationFunctionDescriptor& other)
      {
        *const_cast<CompiledActivationFunctionId*>(&id) = other.id;
        *const_cast<const ActivationFunctionParameters**>(&p) = createParameters(other.id, other.p);
        return *this;
      }

    private:
      static const ActivationFunctionParameters* createParameters(const CompiledActivationFunctionId id, const ActivationFunctionParameters* const p)
      {
        switch(id)
        {
          case CompiledActivationFunctionId::relu:
          {
            if(!p)
              return new ReluParameters();
            return new ReluParameters(p->asType<ReluParameters>());
          }
          case CompiledActivationFunctionId::elu:
          {
            if(!p)
              return new EluParameters();
            return new EluParameters(p->asType<EluParameters>());
          }
          default:
            ASSERT(!p);
            return nullptr;
        }
      }
    };

    class ActivationFn
    {
    private:
      using DefineDataFnType = std::function<void(std::vector<float>&, const ActivationFunctionParameters* const)>;
      using InitializeFnType = std::function<void(x86::Assembler&, const ActivationFunctionParameters* const, const Label&, const std::vector<x86::Xmm>&)>;
      using ApplyFnType = std::function<void(x86::Assembler&, const ActivationFunctionParameters* const, const Label&, const std::vector<x86::Xmm>&, const std::vector<x86::Xmm>&)>;

      NetworkConstants constants;
      std::vector<x86::Xmm> spares;
      std::vector<x86::Xmm> values;
      const ActivationFunctionDescriptor& desc;
      const DefineDataFnType defineDataFn;
      const InitializeFnType initializeFn;
      const ApplyFnType applyFn;

      ActivationFn(const ActivationFunctionDescriptor& desc, const DefineDataFnType defineDataFn, const InitializeFnType initializeFn, const ApplyFnType applyFn) : desc(desc), defineDataFn(defineDataFn), initializeFn(initializeFn), applyFn(applyFn) {}
      ActivationFn(ActivationFn& other) = delete;
      ActivationFn(ActivationFn&& other) = delete;

      void prepare(const std::initializer_list<x86::Xmm> spares, const std::initializer_list<x86::Xmm> values)
      {
        this->spares = spares;
        this->values = values;
      }

      void defineData(x86::Assembler& a)
      {
        constants.data.clear();
        defineDataFn(constants.data, desc.p);
        if(constants.data.size())
          constants.label = a.newLabel();
      }

      friend class ActivationFunctionHandler;

    public:
      inline void addSpare(const x86::Xmm reg) { spares.push_back(reg); }
      inline void addValue(const x86::Xmm reg) { values.push_back(reg); }

      inline void initialize(x86::Assembler& a) { initializeFn(a, desc.p, constants.label, spares); }
      inline void apply(x86::Assembler& a) { applyFn(a, desc.p, constants.label, spares, values); }
    };

    class ActivationFunctionHandler
    {
    private:
      using DefineDataFnType = ActivationFn::DefineDataFnType;
      using InitializeFnType = ActivationFn::InitializeFnType;
      using ApplyFnType = ActivationFn::ApplyFnType;

      struct ActivationData
      {
        const ActivationFunctionDescriptor desc;
        const bool single;
        ActivationFn fn;
        ActivationData(const ActivationFunctionDescriptor& desc, const bool single, const DefineDataFnType defineDataFn, const InitializeFnType initializeFn, const ApplyFnType applyFn) : desc(desc), single(single), fn(this->desc, defineDataFn, initializeFn, applyFn) {}
        ActivationData(const ActivationData& other) : ActivationData(other.desc, other.single, other.fn.defineDataFn, other.fn.initializeFn, other.fn.applyFn) {}
        ActivationData(const ActivationData&& other) = delete;
      };
      std::list<ActivationData> functionData;
      const CompilationSettings& settings;

    public:
      ActivationFunctionHandler(const CompilationSettings& settings) : settings(settings) {}

      ActivationFn& prepare(const ActivationFunctionDescriptor& desc, const bool single, x86::Assembler& a, const std::initializer_list<x86::Xmm> spares, const std::initializer_list<x86::Xmm> values);
      void compileData(x86::Assembler& a) const;

      static unsigned int neededSpares(const ActivationFunctionDescriptor& desc);
    };
  }
}
