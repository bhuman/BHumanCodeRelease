/**
 * @author Felix Thielke
 */

#include "CompiledNNImpl.h"
#include "Tools/NeuralNetwork/CompiledNN/Util/ExpApprox.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    template<bool single>
    void sigmoidExpApproxDefineData(std::vector<float>& data)
    {
      constexpr int count = single ? 1 : 4;

      const float factor = ExpApprox::factor();
      const int offset = ExpApprox::offset();
      for(int i = count; i; --i)
        data.emplace_back(factor);
      for(int i = count; i; --i)
        data.emplace_back(*reinterpret_cast<const float*>(&offset));
      for(int i = count; i; --i)
        data.emplace_back(1.f);
    }

    template<bool single>
    void sigmoidExpApproxInitialize(X86Assembler& a, const Label& label, const std::vector<X86Xmm>& spares)
    {
      ASSERT(spares.size() >= 3);

      for(unsigned int i = 0; i < (spares.size() == 3 ? 2u : 3u); i++)
      {
        if(single)
          a.movss(spares[i], x86::ptr(label, sizeof(float) * i));
        else
          a.movaps(spares[i], x86::ptr(label, 4 * sizeof(float) * i));
      }
    }

    template<bool tanH, bool single>
    void sigmoidExpApproxApply(X86Assembler& a, const Label& label, const std::vector<X86Xmm>& spares, const std::vector<X86Xmm>& values)
    {
      // Calculate sigmoid as exp(x) / (exp(x) + 1) or tanh as (exp(2x) - 1) / (exp(2x) + 1), approximating exp(x)
      if(tanH)
      {
        for(const X86Xmm& value : values)
          a.addps(value, value);
      }

      ExpApprox::apply(a, values, spares[0], spares[1]);

      if(spares.size() - 3 >= values.size())
      {
        for(unsigned int i = 0; i < values.size(); i++)
        {
          if(single)
            a.movss(spares[i + 3], values[i]);
          else
            a.movaps(spares[i + 3], values[i]);
        }
        for(unsigned int i = 0; i < values.size(); i++)
        {
          if(single)
            a.addss(spares[i + 3], spares[2]);
          else
            a.addps(spares[i + 3], spares[2]);
        }

        if(tanH)
        {
          for(unsigned int i = 0; i < values.size(); i++)
          {
            if(single)
              a.subss(values[i], spares[2]);
            else
              a.subps(values[i], spares[2]);
          }
        }

        for(unsigned int i = 0; i < values.size(); i++)
        {
          if(single)
            a.divss(values[i], spares[i + 3]);
          else
            a.divps(values[i], spares[i + 3]);
        }
      }
      else
      {
        for(const X86Xmm value : values)
        {
          const X86Xmm reg = spares.size() > 3 ? spares[3] : spares[2];
          if(single)
            a.movss(reg, value);
          else
            a.movaps(reg, value);

          if(spares.size() > 3)
          {
            if(single)
              a.addss(reg, spares[2]);
            else
              a.addps(reg, spares[2]);

            if(tanH)
            {
              if(single)
                a.subss(value, spares[2]);
              else
                a.subps(value, spares[2]);
            }
          }
          else
          {
            if(single)
              a.addss(reg, x86::ptr(label, sizeof(float) * 2));
            else
              a.addps(reg, x86::ptr(label, 4 * sizeof(float) * 2));

            if(tanH)
            {
              if(single)
                a.subss(value, x86::ptr(label, sizeof(float) * 2));
              else
                a.subps(value, x86::ptr(label, 4 * sizeof(float) * 2));
            }
          }

          if(single)
            a.divss(value, reg);
          else
            a.divps(value, reg);
        }
      }
    }

    template<bool single>
    void tanhDefineData(std::vector<float>& data)
    {
      constexpr int count = single ? 1 : 4;
      for(int i = count; i; --i)
        data.emplace_back(36.f);
      for(int i = count; i; --i)
        data.emplace_back(630.f);
      for(int i = count; i; --i)
        data.emplace_back(6930.f);
      for(int i = count; i; --i)
        data.emplace_back(51975.f);
      for(int i = count; i; --i)
        data.emplace_back(270270.f);
      for(int i = count; i; --i)
        data.emplace_back(945945.f);
      for(int i = count; i; --i)
        data.emplace_back(2027025.f);
    }

    template<bool single>
    void tanhInitialize(X86Assembler& a, const Label& label, const std::vector<X86Xmm>& spares)
    {
      ASSERT(spares.size() >= 3);

      for(unsigned int i = 3; i < spares.size(); i++)
      {
        if(single)
          a.movss(spares[i], x86::ptr(label, sizeof(float) * (i - 3)));
        else
          a.movaps(spares[i], x86::ptr(label, 4 * sizeof(float) * (i - 3)));
      }
    }

    void tanhApprox(X86Assembler& a, const Label& label, const std::vector<X86Xmm>& spares, const X86Xmm& value, const size_t dataOffset = 0)
    {
      // Approximate tanh(x) as ((((36.f * x^2 + 6930.f) * x^2 + 270270.f) * x^2 + 2027025.f) * x) / ((((x^2 + 630.f) * x^2 + 51975.f) * x^2 + 945945.f) * x^2 + 2027025.f)
      // (this has an error of about 1e-7)
      a.movaps(spares[0], value);
      a.mulps(spares[0], spares[0]); // spares[0] = x^2

      // Try to optimize CPU throughput by alternating calculations for dividend and divisor
      if(spares.size() > 3 + dataOffset)
        a.movaps(spares[1], spares[3 + dataOffset]); // spares[1] = 36.f
      else
        a.movaps(spares[1], x86::ptr(label, static_cast<int>(4 * sizeof(float) * dataOffset))); // spares[1] = 36.f
      a.movaps(spares[2], spares[0]); // spares[2] = x^2
      if(spares.size() > 4 + dataOffset)
        a.addps(spares[2], spares[4 + dataOffset]); // spares[2] = x^2 + 630.f
      else
        a.addps(spares[2], x86::ptr(label, static_cast<int>(4 * sizeof(float) * (dataOffset + 1)))); // spares[2] = x^2 + 630.f
      a.mulps(spares[1], spares[0]); // spares[1] = 36.f * x^2
      a.mulps(spares[2], spares[0]); // spares[2] = (x^2 + 630.f) * x^2
      if(spares.size() > 5 + dataOffset)
        a.addps(spares[1], spares[5 + dataOffset]); // spares[1] = 36.f * x^2 + 6930.f
      else
        a.addps(spares[1], x86::ptr(label, static_cast<int>(4 * sizeof(float) * (dataOffset + 2)))); // spares[1] = 36.f * x^2 + 6930.f
      if(spares.size() > 6 + dataOffset)
        a.addps(spares[2], spares[6 + dataOffset]); // spares[2] = (x^2 + 630.f) * x^2 + 51975.f
      else
        a.addps(spares[2], x86::ptr(label, static_cast<int>(4 * sizeof(float) * (dataOffset + 3)))); // spares[2] = (x^2 + 630.f) * x^2 + 51975.f
      a.mulps(spares[1], spares[0]); // spares[1] = (36.f * x^2 + 6930.f) * x^2
      a.mulps(spares[2], spares[0]); // spares[2] = ((x^2 + 630.f) * x^2 + 51975.f) * x^2
      a.addps(spares[1], x86::ptr(label, static_cast<int>(4 * sizeof(float) * (dataOffset + 4)))); // spares[1] = (36.f * x^2 + 6930.f) * x^2 + 270270.f
      a.addps(spares[2], x86::ptr(label, static_cast<int>(4 * sizeof(float) * (dataOffset + 5)))); // spares[2] = ((x^2 + 630.f) * x^2 + 51975.f) * x^2 + 945945.f
      a.mulps(spares[1], spares[0]); // spares[1] = ((36.f * x^2 + 6930.f) * x^2 + 270270.f) * x^2
      a.mulps(spares[2], spares[0]); // spares[2] = (((x^2 + 630.f) * x^2 + 51975.f) * x^2 + 945945.f) * x^2
      a.addps(spares[1], x86::ptr(label, static_cast<int>(4 * sizeof(float) * (dataOffset + 6)))); // spares[1] = ((36.f * x^2 + 6930.f) * x^2 + 270270.f) * x^2 + 2027025.f
      a.addps(spares[2], x86::ptr(label, static_cast<int>(4 * sizeof(float) * (dataOffset + 6)))); // spares[2] = (((x^2 + 630.f) * x^2 + 51975.f) * x^2 + 945945.f) * x^2 + 2027025.f
      a.mulps(value, spares[1]); // value = (((36.f * x^2 + 6930.f) * x^2 + 270270.f) * x^2 + 2027025.f) * x
      a.rcpps(spares[2], spares[2]); // spares[2] = 1 / ((((x^2 + 630.f) * x^2 + 51975.f) * x^2 + 945945.f) * x^2 + 2027025.f)
      a.mulps(value, spares[2]); // value = ((((36.f * x^2 + 6930.f) * x^2 + 270270.f) * x^2 + 2027025.f) * x) / ((((x^2 + 630.f) * x^2 + 51975.f) * x^2 + 945945.f) * x^2 + 2027025.f)
    }

    void tanhApproxSingle(X86Assembler& a, const Label& label, const std::vector<X86Xmm>& spares, const X86Xmm& value, const size_t dataOffset = 0)
    {
      // Approximate tanh(x) as ((((36.f * x^2 + 6930.f) * x^2 + 270270.f) * x^2 + 2027025.f) * x) / ((((x^2 + 630.f) * x^2 + 51975.f) * x^2 + 945945.f) * x^2 + 2027025.f)
      // (this has an error of about 1e-7)
      a.movss(spares[0], value);
      a.mulss(spares[0], spares[0]); // spares[0] = x^2

      // Try to optimize CPU throughput by alternating calculations for dividend and divisor
      if(spares.size() > 3 + dataOffset)
        a.movss(spares[1], spares[3 + dataOffset]); // spares[1] = 36.f
      else
        a.movss(spares[1], x86::ptr(label, static_cast<int>(sizeof(float) * dataOffset))); // spares[1] = 36.f
      a.movss(spares[2], spares[0]); // spares[2] = x^2
      if(spares.size() > 4 + dataOffset)
        a.addss(spares[2], spares[4 + dataOffset]); // spares[2] = x^2 + 630.f
      else
        a.addss(spares[2], x86::ptr(label, static_cast<int>(sizeof(float) * (dataOffset + 1)))); // spares[2] = x^2 + 630.f
      a.mulss(spares[1], spares[0]); // spares[1] = 36.f * x^2
      a.mulss(spares[2], spares[0]); // spares[2] = (x^2 + 630.f) * x^2
      if(spares.size() > 5 + dataOffset)
        a.addss(spares[1], spares[5 + dataOffset]); // spares[1] = 36.f * x^2 + 6930.f
      else
        a.addss(spares[1], x86::ptr(label, static_cast<int>(sizeof(float) * (dataOffset + 2)))); // spares[1] = 36.f * x^2 + 6930.f
      if(spares.size() > 6 + dataOffset)
        a.addss(spares[2], spares[6 + dataOffset]); // spares[2] = (x^2 + 630.f) * x^2 + 51975.f
      else
        a.addss(spares[2], x86::ptr(label, static_cast<int>(sizeof(float) * (dataOffset + 3)))); // spares[2] = (x^2 + 630.f) * x^2 + 51975.f
      a.mulss(spares[1], spares[0]); // spares[1] = (36.f * x^2 + 6930.f) * x^2
      a.mulss(spares[2], spares[0]); // spares[2] = ((x^2 + 630.f) * x^2 + 51975.f) * x^2
      a.addss(spares[1], x86::ptr(label, static_cast<int>(sizeof(float) * (dataOffset + 4)))); // spares[1] = (36.f * x^2 + 6930.f) * x^2 + 270270.f
      a.addss(spares[2], x86::ptr(label, static_cast<int>(sizeof(float) * (dataOffset + 5)))); // spares[2] = ((x^2 + 630.f) * x^2 + 51975.f) * x^2 + 945945.f
      a.mulss(spares[1], spares[0]); // spares[1] = ((36.f * x^2 + 6930.f) * x^2 + 270270.f) * x^2
      a.mulss(spares[2], spares[0]); // spares[2] = (((x^2 + 630.f) * x^2 + 51975.f) * x^2 + 945945.f) * x^2
      a.addss(spares[1], x86::ptr(label, static_cast<int>(sizeof(float) * (dataOffset + 6)))); // spares[1] = ((36.f * x^2 + 6930.f) * x^2 + 270270.f) * x^2 + 2027025.f
      a.addss(spares[2], x86::ptr(label, static_cast<int>(sizeof(float) * (dataOffset + 6)))); // spares[2] = (((x^2 + 630.f) * x^2 + 51975.f) * x^2 + 945945.f) * x^2 + 2027025.f
      a.mulss(value, spares[1]); // value = (((36.f * x^2 + 6930.f) * x^2 + 270270.f) * x^2 + 2027025.f) * x
      a.rcpss(spares[2], spares[2]); // spares[2] = 1 / ((((x^2 + 630.f) * x^2 + 51975.f) * x^2 + 945945.f) * x^2 + 2027025.f)
      a.mulss(value, spares[2]); // value = ((((36.f * x^2 + 6930.f) * x^2 + 270270.f) * x^2 + 2027025.f) * x) / ((((x^2 + 630.f) * x^2 + 51975.f) * x^2 + 945945.f) * x^2 + 2027025.f)
    }

    template<bool single>
    void tanhApply(X86Assembler& a, const Label& label, const std::vector<X86Xmm>& spares, const std::vector<X86Xmm>& values)
    {
      for(const X86Xmm& value : values)
      {
        if(single)
          tanhApproxSingle(a, label, spares, value);
        else
          tanhApprox(a, label, spares, value);
      }
    }

    template<bool single>
    void sigmoidDefineData(std::vector<float>& data)
    {
      constexpr int count = single ? 1 : 4;
      for(int i = count; i; --i)
        data.emplace_back(0.5f);

      tanhDefineData<single>(data);
    }

    template<bool single>
    void sigmoidInitialize(X86Assembler& a, const Label& label, const std::vector<X86Xmm>& spares)
    {
      tanhInitialize<single>(a, label, spares);
    }

    template<bool single>
    void sigmoidApply(X86Assembler& a, const Label& label, const std::vector<X86Xmm>& spares, const std::vector<X86Xmm>& values)
    {
      for(const X86Xmm& value : values)
      {
        // sigmoid(x) = 0.5f * tanh(x * 0.5f) + 0.5f
        if(single)
        {
          if(spares.size() > 3)
            a.mulss(value, spares[3]);
          else
            a.mulss(value, x86::ptr(label));

          tanhApproxSingle(a, label, spares, value, 1);

          if(spares.size() > 3)
          {
            a.mulss(value, spares[3]);
            a.addss(value, spares[3]);
          }
          else
          {
            a.movss(spares[0], x86::ptr(label));
            a.mulss(value, spares[0]);
            a.addss(value, spares[0]);
          }
        }
        else
        {
          if(spares.size() > 3)
            a.mulps(value, spares[3]);
          else
            a.mulps(value, x86::ptr(label));

          tanhApprox(a, label, spares, value, 1);

          if(spares.size() > 3)
          {
            a.mulps(value, spares[3]);
            a.addps(value, spares[3]);
          }
          else
          {
            a.movaps(spares[0], x86::ptr(label));
            a.mulps(value, spares[0]);
            a.addps(value, spares[0]);
          }
        }
      }
    }

    template<bool single>
    void hardSigmoidDefineData(std::vector<float>& data)
    {
      constexpr int count = single ? 1 : 4;
      for(int i = count; i; --i)
        data.emplace_back(0.2f);
      for(int i = count; i; --i)
        data.emplace_back(0.5f);
      for(int i = count; i; --i)
        data.emplace_back(1.f);
    }

    template<bool single>
    void hardSigmoidInitialize(X86Assembler& a, const Label& label, const std::vector<X86Xmm>& spares)
    {
      ASSERT(spares.size());
      a.xorps(spares[0], spares[0]);

      for(size_t i = 1; i < spares.size(); i++)
      {
        if(single)
          a.movss(spares[i], x86::ptr(label, static_cast<unsigned int>((i - 1) * sizeof(float))));
        else
          a.movaps(spares[i], x86::ptr(label, static_cast<unsigned int>((i - 1) * 4 * sizeof(float))));
      }
    }

    template<bool single>
    void hardSigmoidApply(X86Assembler& a, const Label& label, const std::vector<X86Xmm>& spares, const std::vector<X86Xmm>& values)
    {
      for(const X86Xmm& value : values)
      {
        if(single)
        {
          spares.size() > 1 ? a.mulss(value, spares[1]) : a.mulss(value, x86::ptr(label));
          spares.size() > 2 ? a.addss(value, spares[2]) : a.addss(value, x86::ptr(label, sizeof(float)));
          a.maxss(value, spares[0]);
          spares.size() > 3 ? a.minss(value, spares[3]) : a.minss(value, x86::ptr(label, 2 * sizeof(float)));
        }
        else
        {
          spares.size() > 1 ? a.mulps(value, spares[1]) : a.mulps(value, x86::ptr(label));
          spares.size() > 2 ? a.addps(value, spares[2]) : a.addps(value, x86::ptr(label, 4 * sizeof(float)));
          a.maxps(value, spares[0]);
          spares.size() > 3 ? a.minps(value, spares[3]) : a.minps(value, x86::ptr(label, 2 * 4 * sizeof(float)));
        }
      }
    }

    unsigned int ActivationFunctionHandler::neededSpares(const ActivationFunctionId id)
    {
      switch(id)
      {
        case ActivationFunctionId::linear:
          return 0u;
        case ActivationFunctionId::relu:
          return 1u;
        case ActivationFunctionId::tanH:
          return 3u;
        case ActivationFunctionId::sigmoid:
          return 3u;
        case ActivationFunctionId::hardSigmoid:
          return 1u;
        default:
          ASSERT(false);
      }

      return 0u;
    }

    ActivationFn& ActivationFunctionHandler::prepare(const ActivationFunctionId id, const bool single, X86Assembler& a, const std::initializer_list<X86Xmm> spares, const std::initializer_list<X86Xmm> values)
    {
      // Look up function
      for(ActivationData& fnData : functionData)
      {
        if(fnData.id == id && fnData.single == single)
        {
          fnData.fn.prepare(spares, values);
          return fnData.fn;
        }
      }

      // Create entry for function if it does not yet exist
      switch(id)
      {
        case ActivationFunctionId::linear:
          functionData.emplace_back(
            id, single, a,
          [](std::vector<float>& data) {},
          [](X86Assembler & a, const Label& label, const std::vector<X86Xmm>& spares) {},
          [](X86Assembler & a, const Label& label, const std::vector<X86Xmm>& spares, const std::vector<X86Xmm>& values) {}
          );
          break;
        case ActivationFunctionId::relu:
        {
          std::function<void(X86Assembler&, const Label&, const std::vector<X86Xmm>&, const std::vector<X86Xmm>&)> applyFn;
          if(single)
          {
            applyFn = [](X86Assembler & a, const Label& label, const std::vector<X86Xmm>& spares, const std::vector<X86Xmm>& values)
            {
              for(const X86Xmm& value : values)
                a.maxss(value, spares[0]);
            };
          }
          else
          {
            applyFn = [](X86Assembler & a, const Label& label, const std::vector<X86Xmm>& spares, const std::vector<X86Xmm>& values)
            {
              for(const X86Xmm& value : values)
                a.maxps(value, spares[0]);
            };
          }
          functionData.emplace_back(
            id, single, a,
          [](std::vector<float>& data) {},
          [](X86Assembler & a, const Label& label, const std::vector<X86Xmm>& spares)
          {
            ASSERT(spares.size());
            a.xorps(spares[0], spares[0]);
          },
          applyFn
          );
        }
        break;
        case ActivationFunctionId::tanH:
          if(settings.useExpApproxInTanh)
          {
            functionData.emplace_back(
              id, single, a,
              single ? sigmoidExpApproxDefineData<true> : sigmoidExpApproxDefineData<false>,
              single ? sigmoidExpApproxInitialize<true> : sigmoidExpApproxInitialize<false>,
              single ? sigmoidExpApproxApply<true, true> : sigmoidExpApproxApply<true, false>
            );
          }
          else
          {
            functionData.emplace_back(
              id, single, a,
              single ? tanhDefineData<true> : tanhDefineData<false>,
              single ? tanhInitialize<true> : tanhInitialize<false>,
              single ? tanhApply<true> : tanhApply<false>
            );
          }
          break;
        case ActivationFunctionId::sigmoid:
          if(settings.useExpApproxInSigmoid)
          {
            functionData.emplace_back(
              id, single, a,
              single ? sigmoidExpApproxDefineData<true> : sigmoidExpApproxDefineData<false>,
              single ? sigmoidExpApproxInitialize<true> : sigmoidExpApproxInitialize<false>,
              single ? sigmoidExpApproxApply<false, true> : sigmoidExpApproxApply<false, false>
            );
          }
          else
          {
            functionData.emplace_back(
              id, single, a,
              single ? sigmoidDefineData<true> : sigmoidDefineData<false>,
              single ? sigmoidInitialize<true> : sigmoidInitialize<false>,
              single ? sigmoidApply<true> : sigmoidApply<false>
            );
          }
          break;
        case ActivationFunctionId::hardSigmoid:
          functionData.emplace_back(
            id, single, a,
            single ? hardSigmoidDefineData<true> : hardSigmoidDefineData<false>,
            single ? hardSigmoidInitialize<true> : hardSigmoidInitialize<false>,
            single ? hardSigmoidApply<true> : hardSigmoidApply<false>
          );
          break;
        default:
          ASSERT(false);
      }

      functionData.back().fn.prepare(spares, values);
      return functionData.back().fn;
    }

    void ActivationFunctionHandler::compileData(X86Assembler& a) const
    {
      for(const ActivationData& fnData : functionData)
      {
        if(fnData.fn.constants.data.size())
        {
          a.align(AlignMode::kAlignZero, 16);
          a.bind(fnData.fn.constants.label);
          for(const float c : fnData.fn.constants.data)
            a.dfloat(c);
        }
      }
    }
  }
}
