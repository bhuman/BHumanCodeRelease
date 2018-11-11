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
    class ActivationFn
    {
    private:
      NetworkConstants constants;
      std::vector<X86Xmm> spares;
      std::vector<X86Xmm> values;
      const std::function<void(X86Assembler&, const Label&, const std::vector<X86Xmm>&)> initializeFn;
      const std::function<void(X86Assembler&, const Label&, const std::vector<X86Xmm>&, const std::vector<X86Xmm>&)> applyFn;

      ActivationFn(X86Assembler& a, const std::function<void(std::vector<float>& data)> defineDataFn, const std::function<void(X86Assembler&, const Label&, const std::vector<X86Xmm>&)> initializeFn, const std::function<void(X86Assembler&, const Label&, const std::vector<X86Xmm>&, const std::vector<X86Xmm>&)> applyFn) : initializeFn(initializeFn), applyFn(applyFn)
      {
        constants.data.clear();
        defineDataFn(constants.data);
        if(constants.data.size())
          constants.label = a.newLabel();
      }

      void prepare(const std::initializer_list<X86Xmm> spares, const std::initializer_list<X86Xmm> values)
      {
        this->spares = spares;
        this->values = values;
      }

      friend class ActivationFunctionHandler;

    public:
      inline void addSpare(const X86Xmm reg) { spares.push_back(reg); }
      inline void addValue(const X86Xmm reg) { values.push_back(reg); }

      inline void initialize(X86Assembler& a) { initializeFn(a, constants.label, spares); }
      inline void apply(X86Assembler& a) { applyFn(a, constants.label, spares, values); }

      inline ActivationFn& operator=(const ActivationFn& other)
      {
        constants = other.constants;
        spares = other.spares;
        values = other.values;
        *const_cast<std::function<void(X86Assembler&, const Label&, const std::vector<X86Xmm>&)>*>(&initializeFn) = other.initializeFn;
        *const_cast<std::function<void(X86Assembler&, const Label&, const std::vector<X86Xmm>&, const std::vector<X86Xmm>&)>*>(&applyFn) = other.applyFn;
        return *this;
      }
    };

    class ActivationFunctionHandler
    {
    private:
      struct ActivationData
      {
        const ActivationFunctionId id;
        const bool single;
        ActivationFn fn;
        ActivationData(const ActivationFunctionId id, const bool single, X86Assembler& a, const std::function<void(std::vector<float>&)> defineDataFn, const std::function<void(X86Assembler&, const Label&, const std::vector<X86Xmm>&)> initializeFn, const std::function<void(X86Assembler&, const Label&, const std::vector<X86Xmm>&, const std::vector<X86Xmm>&)> applyFn) : id(id), single(single), fn(a, defineDataFn, initializeFn, applyFn) {}
      };
      std::vector<ActivationData> functionData;
      const CompilationSettings& settings;

    public:
      ActivationFunctionHandler(const CompilationSettings& settings) : settings(settings) {}

      ActivationFn& prepare(const ActivationFunctionId id, const bool single, X86Assembler& a, const std::initializer_list<X86Xmm> spares, const std::initializer_list<X86Xmm> values);
      void compileData(X86Assembler& a) const;

      static unsigned int neededSpares(const ActivationFunctionId id);
    };
  }
}
