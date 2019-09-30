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
    void reluDefineData(std::vector<float>& data, const ActivationFunctionParameters* const params)
    {
      const ReluParameters& p = params->asType<ReluParameters>();
      constexpr int count = single ? 1 : 4;

      if(p.threshold != 0.f)
      {
        for(int i = count; i; --i)
          data.emplace_back(p.threshold);
      }
      if(p.negativeSlope != 0.f)
      {
        ASSERT(p.negativeSlope > 0.f);
        ASSERT(p.negativeSlope < 1.f);
        for(int i = count; i; --i)
          data.emplace_back(p.negativeSlope);
      }
      if(p.maxValue != std::numeric_limits<float>::max())
      {
        for(int i = count; i; --i)
          data.emplace_back(p.maxValue);
      }
    }

    template<bool single>
    void reluInitialize(x86::Assembler& a, const ActivationFunctionParameters* const params, const Label& label, const std::vector<x86::Xmm>& spares)
    {
      const ReluParameters& p = params->asType<ReluParameters>();
      ASSERT(!spares.empty());

      if(p.negativeSlope == 0.f && p.threshold == 0.f)
      {
        a.xorps(spares[0], spares[0]);

        if(p.maxValue != std::numeric_limits<float>::max())
        {
          if(spares.size() > 1)
          {
            if(single)
              a.movss(spares[1], x86::ptr(label));
            else
              a.movaps(spares[1], x86::ptr(label));
          }
        }
      }
      else
      {
        const size_t maxConstantSpares = (p.negativeSlope != 0.f ? 1 : 0) + (p.threshold != 0.f ? 1 : 0) + (p.maxValue != std::numeric_limits<float>::max() ? 1 : 0);
        for(size_t offset = 0, i = spares.size() <= maxConstantSpares ? 1 : spares.size() - maxConstantSpares; i < spares.size(); offset++, i++)
        {
          if(single)
            a.movss(spares[i], x86::ptr(label, static_cast<unsigned int>(offset * sizeof(float))));
          else
            a.movaps(spares[i], x86::ptr(label, static_cast<unsigned int>(offset * 4 * sizeof(float))));
        }
      }
    }

    template<bool single>
    void reluApply(x86::Assembler& a, const ActivationFunctionParameters* const params, const Label& label, const std::vector<x86::Xmm>& spares, const std::vector<x86::Xmm>& values)
    {
      const ReluParameters& p = params->asType<ReluParameters>();

      if(p.negativeSlope == 0.f && p.threshold == 0.f)
      {
        for(const x86::Xmm& value : values)
        {
          if(single)
            a.maxss(value, spares[0]);
          else
            a.maxps(value, spares[0]);
        }

        if(p.maxValue != std::numeric_limits<float>::max())
        {
          for(const x86::Xmm& value : values)
          {
            if(single)
              spares.size() > 1 ? a.minss(value, spares[1]) : a.minss(value, x86::ptr(label));
            else
              spares.size() > 1 ? a.minps(value, spares[1]) : a.minps(value, x86::ptr(label));
          }
        }
      }
      else
      {
        const size_t maxConstantSpares = (p.negativeSlope != 0.f ? 1 : 0) + (p.threshold != 0.f ? 1 : 0) + (p.maxValue != std::numeric_limits<float>::max() ? 1 : 0);
        const size_t constantOffset = spares.size() <= maxConstantSpares ? 1 : spares.size() - maxConstantSpares;

        int thresholdOffset = -1;
        int negativeSlopeOffset = -1;
        int maxValueOffset = -1;
        {
          int curOffset = 0;
          if(p.threshold != 0.f)
          {
            thresholdOffset = curOffset;
            curOffset++;
          }
          if(p.negativeSlope != 0.f)
          {
            negativeSlopeOffset = curOffset;
            curOffset++;
          }
          if(p.maxValue != std::numeric_limits<float>::max())
            maxValueOffset = curOffset;
        }

        if(spares.size() >= values.size() + maxConstantSpares)
        {
          for(size_t step = 0; step < values.size(); step++)
          {
            if(single)
              a.movss(spares[step], values[step]);
            else
              a.movaps(spares[step], values[step]);
          }

          if(thresholdOffset >= 0)
          {
            for(size_t step = 0; step < values.size(); step++)
            {
              if(single)
                a.subss(spares[step], spares[constantOffset + thresholdOffset]);
              else
                a.subps(spares[step], spares[constantOffset + thresholdOffset]);
            }
          }

          if(negativeSlopeOffset >= 0)
          {
            for(size_t step = 0; step < values.size(); step++)
            {
              if(single)
                a.mulss(spares[step], spares[constantOffset + negativeSlopeOffset]);
              else
                a.mulps(spares[step], spares[constantOffset + negativeSlopeOffset]);
            }
          }

          for(size_t step = 0; step < values.size(); step++)
          {
            if(single)
              a.maxss(values[step], spares[step]);
            else
              a.maxps(values[step], spares[step]);
          }
        }
        else
        {
          for(const x86::Xmm& value : values)
          {
            if(single)
              a.movss(spares[0], value);
            else
              a.movaps(spares[0], value);

            if(thresholdOffset >= 0)
            {
              if(single)
                constantOffset + thresholdOffset < spares.size() ? a.subss(spares[0], spares[constantOffset + thresholdOffset]) : a.subss(spares[0], x86::ptr(label, thresholdOffset * sizeof(float)));
              else
                constantOffset + thresholdOffset < spares.size() ? a.subps(spares[0], spares[constantOffset + thresholdOffset]) : a.subps(spares[0], x86::ptr(label, thresholdOffset * 4 * sizeof(float)));
            }

            if(negativeSlopeOffset >= 0)
            {
              if(single)
                constantOffset + negativeSlopeOffset < spares.size() ? a.mulss(spares[0], spares[constantOffset + negativeSlopeOffset]) : a.mulss(spares[0], x86::ptr(label, negativeSlopeOffset * sizeof(float)));
              else
                constantOffset + negativeSlopeOffset < spares.size() ? a.mulps(spares[0], spares[constantOffset + negativeSlopeOffset]) : a.mulps(spares[0], x86::ptr(label, negativeSlopeOffset * 4 * sizeof(float)));
            }

            if(single)
              a.maxss(value, spares[0]);
            else
              a.maxps(value, spares[0]);
          }
        }

        if(maxValueOffset >= 0)
        {
          for(const x86::Xmm& value : values)
          {
            if(single)
              constantOffset + maxValueOffset < spares.size() ? a.minss(value, spares[constantOffset + maxValueOffset]) : a.minss(value, x86::ptr(label, maxValueOffset * sizeof(float)));
            else
              constantOffset + maxValueOffset < spares.size() ? a.minps(value, spares[constantOffset + maxValueOffset]) : a.minps(value, x86::ptr(label, maxValueOffset * 4 * sizeof(float)));
          }
        }
      }
    }

    template<bool single>
    void sigmoidExpApproxDefineData(std::vector<float>& data, const ActivationFunctionParameters* const params)
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
    void sigmoidExpApproxInitialize(x86::Assembler& a, const ActivationFunctionParameters* const params, const Label& label, const std::vector<x86::Xmm>& spares)
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
    void sigmoidExpApproxApply(x86::Assembler& a, const ActivationFunctionParameters* const params, const Label& label, const std::vector<x86::Xmm>& spares, const std::vector<x86::Xmm>& values)
    {
      // Calculate sigmoid as exp(x) / (exp(x) + 1) or tanh as (exp(2x) - 1) / (exp(2x) + 1), approximating exp(x)
      if(tanH)
      {
        for(const x86::Xmm& value : values)
          a.addps(value, value);
      }

      ExpApprox::apply<single>(a, values, spares[0], spares[1]);

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
        for(const x86::Xmm value : values)
        {
          const x86::Xmm reg = spares.size() > 3 ? spares[3] : spares[2];
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
    void tanhDefineData(std::vector<float>& data, const ActivationFunctionParameters* const params)
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
    void tanhInitialize(x86::Assembler& a, const ActivationFunctionParameters* const params, const Label& label, const std::vector<x86::Xmm>& spares)
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

    void tanhApprox(x86::Assembler& a, const Label& label, const std::vector<x86::Xmm>& spares, const x86::Xmm& value, const size_t dataOffset = 0)
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

    void tanhApproxSingle(x86::Assembler& a, const Label& label, const std::vector<x86::Xmm>& spares, const x86::Xmm& value, const size_t dataOffset = 0)
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
    void tanhApply(x86::Assembler& a, const ActivationFunctionParameters* const params, const Label& label, const std::vector<x86::Xmm>& spares, const std::vector<x86::Xmm>& values)
    {
      for(const x86::Xmm& value : values)
      {
        if(single)
          tanhApproxSingle(a, label, spares, value);
        else
          tanhApprox(a, label, spares, value);
      }
    }

    template<bool single>
    void sigmoidDefineData(std::vector<float>& data, const ActivationFunctionParameters* const params)
    {
      constexpr int count = single ? 1 : 4;
      for(int i = count; i; --i)
        data.emplace_back(0.5f);

      tanhDefineData<single>(data, params);
    }

    template<bool single>
    void sigmoidInitialize(x86::Assembler& a, const ActivationFunctionParameters* const params, const Label& label, const std::vector<x86::Xmm>& spares)
    {
      tanhInitialize<single>(a, params, label, spares);
    }

    template<bool single>
    void sigmoidApply(x86::Assembler& a, const ActivationFunctionParameters* const params, const Label& label, const std::vector<x86::Xmm>& spares, const std::vector<x86::Xmm>& values)
    {
      for(const x86::Xmm& value : values)
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
    void hardSigmoidDefineData(std::vector<float>& data, const ActivationFunctionParameters* const params)
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
    void hardSigmoidInitialize(x86::Assembler& a, const ActivationFunctionParameters* const params, const Label& label, const std::vector<x86::Xmm>& spares)
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
    void hardSigmoidApply(x86::Assembler& a, const ActivationFunctionParameters* const params, const Label& label, const std::vector<x86::Xmm>& spares, const std::vector<x86::Xmm>& values)
    {
      for(const x86::Xmm& value : values)
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

    template<bool single>
    void eluDefineData(std::vector<float>& data, const ActivationFunctionParameters* const params)
    {
      const EluParameters& p = params->asType<EluParameters>();
      ASSERT(p.alpha > 0.f);

      constexpr int count = single ? 1 : 4;

      const float factor = ExpApprox::factor();
      const int offset = ExpApprox::offset();
      const unsigned int signBit = 1u << 31;

      for(int i = count; i; --i)
        data.emplace_back(*reinterpret_cast<const float*>(&offset));
      for(int i = count; i; --i)
        data.emplace_back(*reinterpret_cast<const float*>(&signBit));
      for(int i = count; i; --i)
        data.emplace_back(factor);
      for(int i = count; i; --i)
        data.emplace_back(-p.alpha);
    }

    template<bool single>
    void eluInitialize(x86::Assembler& a, const ActivationFunctionParameters* const params, const Label& label, const std::vector<x86::Xmm>& spares)
    {
      ASSERT(spares.size() >= 3);

      for(size_t offset = 0, i = spares.size() <= 4 ? 1 : spares.size() - 4; i < spares.size(); offset++, i++)
      {
        if(single)
          a.movss(spares[i], x86::ptr(label, static_cast<unsigned int>(offset * sizeof(float))));
        else
          a.movaps(spares[i], x86::ptr(label, static_cast<unsigned int>(offset * 4 * sizeof(float))));
      }
    }

    template<bool single>
    void eluApply(x86::Assembler& a, const ActivationFunctionParameters* const params, const Label& label, const std::vector<x86::Xmm>& spares, const std::vector<x86::Xmm>& values)
    {
      const EluParameters& p = params->asType<EluParameters>();

      // elu(x) = sign(x) * max(-alpha * exp(x) + alpha, x)
      const size_t constOffset = spares.size() <= 4 ? 1 : spares.size() - 4;

      if(spares.size() >= values.size() + 4)
      {
        for(unsigned int step = 0; step < values.size(); step++)
        {
          if(single)
            a.movss(spares[step], values[step]);
          else
            a.movaps(spares[step], values[step]);
        }
        std::vector<x86::Xmm> expRegs;
        for(unsigned int step = 0; step < values.size(); step++)
          expRegs.emplace_back(spares[step]);
        ExpApprox::apply<single>(a, expRegs, spares[constOffset + 2], spares[constOffset]);
        if(p.alpha == 1.f)
        {
          // Calculate -x = (x XOR 1<<31)
          for(unsigned int step = 0; step < values.size(); step++)
            a.xorps(spares[step], spares[constOffset + 1]);
        }
        else
        {
          for(unsigned int step = 0; step < values.size(); step++)
          {
            if(single)
              a.mulss(spares[step], spares[constOffset + 3]);
            else
              a.mulps(spares[step], spares[constOffset + 3]);
          }
        }
        for(unsigned int step = 0; step < values.size(); step++)
        {
          if(single)
            a.subss(spares[step], spares[constOffset + 3]);
          else
            a.subps(spares[step], spares[constOffset + 3]);
        }
        for(unsigned int step = 0; step < values.size(); step++)
        {
          if(single)
            a.maxss(spares[step], values[step]);
          else
            a.maxps(spares[step], values[step]);
        }
        for(unsigned int step = 0; step < values.size(); step++)
          a.andps(values[step], spares[constOffset + 1]);
        for(unsigned int step = 0; step < values.size(); step++)
          a.xorps(values[step], spares[step]);
      }
      else
      {
        for(const x86::Xmm& value : values)
        {
          if(single)
            a.movss(spares[0], value);
          else
            a.movaps(spares[0], value);
          if(spares.size() > 3)
            ExpApprox::apply<single>(a, { spares[0] }, spares[constOffset + 2], spares[constOffset]);
          else
            ExpApprox::apply<single>(a, { spares[0] }, x86::ptr(label, static_cast<unsigned int>((single ? 2 : 2 * 4) * sizeof(float))), spares[constOffset]);

          if(p.alpha == 1.f)
          {
            a.xorps(spares[0], spares[constOffset + 1]);
          }
          else
          {
            if(single)
              spares.size() > 4 ? a.mulss(spares[0], spares[constOffset + 3]) : a.mulss(spares[0], x86::ptr(label, static_cast<unsigned int>(3 * sizeof(float))));
            else
              spares.size() > 4 ? a.mulps(spares[0], spares[constOffset + 3]) : a.mulps(spares[0], x86::ptr(label, static_cast<unsigned int>(3 * sizeof(float))));
          }

          if(single)
          {
            spares.size() > 4 ? a.subss(spares[0], spares[constOffset + 3]) : a.subss(spares[0], x86::ptr(label, static_cast<unsigned int>(3 * sizeof(float))));
            a.maxss(spares[0], value);
          }
          else
          {
            spares.size() > 4 ? a.subps(spares[0], spares[constOffset + 3]) : a.subps(spares[0], x86::ptr(label, static_cast<unsigned int>(3 * 4 * sizeof(float))));
            a.maxps(spares[0], value);
          }
          a.andps(value, spares[constOffset + 1]);
          a.xorps(value, spares[0]);
        }
      }
    }

    template<bool single>
    void seluDefineData(std::vector<float>& data, const ActivationFunctionParameters* const params)
    {
      constexpr int count = single ? 1 : 4;

      const float factor = ExpApprox::factor();
      const int offset = ExpApprox::offset();
      const unsigned int signBit = 1u << 31;
      const float alpha = 1.6732632423543772848170429916717f;
      const float scale = 1.0507009873554804934193349852946f;

      for(int i = count; i; --i)
        data.emplace_back(*reinterpret_cast<const float*>(&offset));
      for(int i = count; i; --i)
        data.emplace_back(*reinterpret_cast<const float*>(&signBit));
      for(int i = count; i; --i)
        data.emplace_back(factor);
      for(int i = count; i; --i)
        data.emplace_back(-alpha);
      for(int i = count; i; --i)
        data.emplace_back(scale);
    }

    template<bool single>
    void seluInitialize(x86::Assembler& a, const ActivationFunctionParameters* const params, const Label& label, const std::vector<x86::Xmm>& spares)
    {
      ASSERT(spares.size() >= 3);

      for(size_t offset = 0, i = spares.size() <= 5 ? 1 : spares.size() - 5; i < spares.size(); offset++, i++)
      {
        if(single)
          a.movss(spares[i], x86::ptr(label, static_cast<unsigned int>(offset * sizeof(float))));
        else
          a.movaps(spares[i], x86::ptr(label, static_cast<unsigned int>(offset * 4 * sizeof(float))));
      }
    }

    template<bool single>
    void seluApply(x86::Assembler& a, const ActivationFunctionParameters* const params, const Label& label, const std::vector<x86::Xmm>& spares, const std::vector<x86::Xmm>& values)
    {
      // selu(x) = scale * sign(x) * max(-alpha * exp(x) + alpha, x)
      const size_t constOffset = spares.size() <= 5 ? 1 : spares.size() - 5;

      if(spares.size() >= values.size() + 5)
      {
        for(unsigned int step = 0; step < values.size(); step++)
        {
          if(single)
            a.movss(spares[step], values[step]);
          else
            a.movaps(spares[step], values[step]);
        }
        std::vector<x86::Xmm> expRegs;
        for(unsigned int step = 0; step < values.size(); step++)
          expRegs.emplace_back(spares[step]);
        ExpApprox::apply<single>(a, expRegs, spares[constOffset + 2], spares[constOffset]);
        for(unsigned int step = 0; step < values.size(); step++)
        {
          if(single)
            a.mulss(spares[step], spares[constOffset + 3]);
          else
            a.mulps(spares[step], spares[constOffset + 3]);
        }
        for(unsigned int step = 0; step < values.size(); step++)
        {
          if(single)
            a.subss(spares[step], spares[constOffset + 3]);
          else
            a.subps(spares[step], spares[constOffset + 3]);
        }
        for(unsigned int step = 0; step < values.size(); step++)
        {
          if(single)
            a.maxss(spares[step], values[step]);
          else
            a.maxps(spares[step], values[step]);
        }
        for(unsigned int step = 0; step < values.size(); step++)
          a.andps(values[step], spares[constOffset + 1]);
        for(unsigned int step = 0; step < values.size(); step++)
          a.xorps(values[step], spares[step]);
        for(unsigned int step = 0; step < values.size(); step++)
        {
          if(single)
            a.mulss(values[step], spares[constOffset + 4]);
          else
            a.mulps(values[step], spares[constOffset + 4]);
        }
      }
      else
      {
        for(const x86::Xmm& value : values)
        {
          if(single)
            a.movss(spares[0], value);
          else
            a.movaps(spares[0], value);
          if(spares.size() > 3)
            ExpApprox::apply<single>(a, { spares[0] }, spares[constOffset + 2], spares[constOffset]);
          else
            ExpApprox::apply<single>(a, { spares[0] }, x86::ptr(label, static_cast<unsigned int>((single ? 2 : 2 * 4) * sizeof(float))), spares[constOffset]);

          if(single)
          {
            spares.size() > 4 ? a.mulss(spares[0], spares[constOffset + 3]) : a.mulss(spares[0], x86::ptr(label, static_cast<unsigned int>(3 * sizeof(float))));
            spares.size() > 4 ? a.subss(spares[0], spares[constOffset + 3]) : a.subss(spares[0], x86::ptr(label, static_cast<unsigned int>(3 * sizeof(float))));
            a.maxss(spares[0], value);
          }
          else
          {
            spares.size() > 4 ? a.mulps(spares[0], spares[constOffset + 3]) : a.mulps(spares[0], x86::ptr(label, static_cast<unsigned int>(3 * 4 * sizeof(float))));
            spares.size() > 4 ? a.subps(spares[0], spares[constOffset + 3]) : a.subps(spares[0], x86::ptr(label, static_cast<unsigned int>(3 * 4 * sizeof(float))));
            a.maxps(spares[0], value);
          }

          a.andps(value, spares[constOffset + 1]);
          a.xorps(value, spares[0]);
          if(single)
            spares.size() > 5 ? a.mulss(value, spares[constOffset + 4]) : a.mulss(value, x86::ptr(label, static_cast<unsigned int>(4 * sizeof(float))));
          else
            spares.size() > 5 ? a.mulps(value, spares[constOffset + 4]) : a.mulps(value, x86::ptr(label, static_cast<unsigned int>(4 * 4 * sizeof(float))));
        }
      }
    }

    template<bool single>
    void exponentialDefineData(std::vector<float>& data, const ActivationFunctionParameters* const params)
    {
      constexpr int count = single ? 1 : 4;

      const float factor = ExpApprox::factor();
      const int offset = ExpApprox::offset();

      for(int i = count; i; --i)
        data.emplace_back(*reinterpret_cast<const float*>(&offset));
      for(int i = count; i; --i)
        data.emplace_back(factor);
    }

    template<bool single>
    void exponentialInitialize(x86::Assembler& a, const ActivationFunctionParameters* const params, const Label& label, const std::vector<x86::Xmm>& spares)
    {
      for(size_t offset = 0, i = spares.size() < 2 ? 0 : spares.size() - 2; i < spares.size(); offset++, i++)
      {
        if(single)
          a.movss(spares[i], x86::ptr(label, static_cast<unsigned int>(offset * sizeof(float))));
        else
          a.movaps(spares[i], x86::ptr(label, static_cast<unsigned int>(offset * 4 * sizeof(float))));
      }
    }

    template<bool single>
    void exponentialApply(x86::Assembler& a, const ActivationFunctionParameters* const params, const Label& label, const std::vector<x86::Xmm>& spares, const std::vector<x86::Xmm>& values)
    {
      if(spares.size() == 0)
        ExpApprox::apply<single>(a, values, x86::ptr(label, static_cast<unsigned int>((single ? 1 : 4) * sizeof(float))), x86::ptr(label));
      else if(spares.size() == 1)
        ExpApprox::apply<single>(a, values, x86::ptr(label, static_cast<unsigned int>((single ? 1 : 4) * sizeof(float))), spares[0]);
      else
        ExpApprox::apply<single>(a, values, spares[spares.size() - 1], spares[spares.size() - 2]);
    }

    template<bool single>
    void softsignDefineData(std::vector<float>& data, const ActivationFunctionParameters* const params)
    {
      constexpr int count = single ? 1 : 4;

      const unsigned int signMask = ~(1u << 31);

      for(int i = count; i; --i)
        data.emplace_back(*reinterpret_cast<const float*>(&signMask));
      for(int i = count; i; --i)
        data.emplace_back(1.f);
    }

    template<bool single>
    void softsignInitialize(x86::Assembler& a, const ActivationFunctionParameters* const params, const Label& label, const std::vector<x86::Xmm>& spares)
    {
      ASSERT(spares.size() >= 2);

      for(size_t offset = 0, i = spares.size() < 2 ? 1 : spares.size() - 2; i < spares.size(); offset++, i++)
      {
        if(single)
          a.movss(spares[i], x86::ptr(label, static_cast<unsigned int>(offset * sizeof(float))));
        else
          a.movaps(spares[i], x86::ptr(label, static_cast<unsigned int>(offset * 4 * sizeof(float))));
      }
    }

    template<bool single>
    void softsignApply(x86::Assembler& a, const ActivationFunctionParameters* const params, const Label& label, const std::vector<x86::Xmm>& spares, const std::vector<x86::Xmm>& values)
    {
      // softsign(x) = x / (abs(x) + 1)
      const size_t constOffset = spares.size() < 2 ? 1 : spares.size() - 2;

      if(spares.size() >= values.size() + 2)
      {
        for(size_t step = 0; step < values.size(); step++)
        {
          if(single)
            a.movss(spares[step], values[step]);
          else
            a.movaps(spares[step], values[step]);
        }

        for(size_t step = 0; step < values.size(); step++)
          a.andps(spares[step], spares[constOffset]);

        for(size_t step = 0; step < values.size(); step++)
        {
          if(single)
            a.addss(spares[step], spares[constOffset + 1]);
          else
            a.addps(spares[step], spares[constOffset + 1]);
        }

        for(size_t step = 0; step < values.size(); step++)
        {
          if(single)
            a.divss(values[step], spares[step]);
          else
            a.divps(values[step], spares[step]);
        }
      }
      else
      {
        for(const x86::Xmm& value : values)
        {
          if(single)
          {
            a.movss(spares[0], value);
            a.andps(spares[0], spares[constOffset]);
            spares.size() > 2 ? a.addss(spares[0], spares[constOffset + 1]) : a.addss(spares[0], static_cast<unsigned int>(sizeof(float)));
            a.divss(value, spares[0]);
          }
          else
          {
            a.movaps(spares[0], value);
            a.andps(spares[0], spares[constOffset]);
            spares.size() > 2 ? a.addps(spares[0], spares[constOffset + 1]) : a.addps(spares[0], static_cast<unsigned int>(4 * sizeof(float)));
            a.divps(value, spares[0]);
          }
        }
      }
    }

    unsigned int ActivationFunctionHandler::neededSpares(const ActivationFunctionDescriptor& desc)
    {
      switch(desc.id)
      {
        case CompiledActivationFunctionId::linear:
          return 0u;
        case CompiledActivationFunctionId::relu:
          return 1u;
        case CompiledActivationFunctionId::tanH:
          return 3u;
        case CompiledActivationFunctionId::sigmoid:
          return 3u;
        case CompiledActivationFunctionId::hardSigmoid:
          return 1u;
        case CompiledActivationFunctionId::elu:
          return 3u;
        case CompiledActivationFunctionId::selu:
          return 3u;
        case CompiledActivationFunctionId::exponential:
          return 0u;
        case CompiledActivationFunctionId::softsign:
          return 2u;
        default:
          ASSERT(false);
      }

      return 0u;
    }

    ActivationFn& ActivationFunctionHandler::prepare(const ActivationFunctionDescriptor& desc, const bool single, x86::Assembler& a, const std::initializer_list<x86::Xmm> spares, const std::initializer_list<x86::Xmm> values)
    {
      // Look up function
      for(ActivationData& fnData : functionData)
      {
        if(fnData.desc == desc && fnData.single == single)
        {
          fnData.fn.prepare(spares, values);
          return fnData.fn;
        }
      }

      // Create entry for function if it does not yet exist
      switch(desc.id)
      {
        case CompiledActivationFunctionId::linear:
          functionData.emplace_back(
            desc, single,
          [](std::vector<float>& data, const ActivationFunctionParameters* const) {},
          [](x86::Assembler & a, const ActivationFunctionParameters* const params, const Label& label, const std::vector<x86::Xmm>& spares) {},
          [](x86::Assembler & a, const ActivationFunctionParameters* const params, const Label& label, const std::vector<x86::Xmm>& spares, const std::vector<x86::Xmm>& values) {}
          );
          break;
        case CompiledActivationFunctionId::relu:
          functionData.emplace_back(
            desc, single,
            single ? reluDefineData<true> : reluDefineData<false>,
            single ? reluInitialize<true> : reluInitialize<false>,
            single ? reluApply<true> : reluApply<false>
          );
          break;
        case CompiledActivationFunctionId::tanH:
          if(settings.useExpApproxInTanh)
          {
            functionData.emplace_back(
              desc, single,
              single ? sigmoidExpApproxDefineData<true> : sigmoidExpApproxDefineData<false>,
              single ? sigmoidExpApproxInitialize<true> : sigmoidExpApproxInitialize<false>,
              single ? sigmoidExpApproxApply<true, true> : sigmoidExpApproxApply<true, false>
            );
          }
          else
          {
            functionData.emplace_back(
              desc, single,
              single ? tanhDefineData<true> : tanhDefineData<false>,
              single ? tanhInitialize<true> : tanhInitialize<false>,
              single ? tanhApply<true> : tanhApply<false>
            );
          }
          break;
        case CompiledActivationFunctionId::sigmoid:
          if(settings.useExpApproxInSigmoid)
          {
            functionData.emplace_back(
              desc, single,
              single ? sigmoidExpApproxDefineData<true> : sigmoidExpApproxDefineData<false>,
              single ? sigmoidExpApproxInitialize<true> : sigmoidExpApproxInitialize<false>,
              single ? sigmoidExpApproxApply<false, true> : sigmoidExpApproxApply<false, false>
            );
          }
          else
          {
            functionData.emplace_back(
              desc, single,
              single ? sigmoidDefineData<true> : sigmoidDefineData<false>,
              single ? sigmoidInitialize<true> : sigmoidInitialize<false>,
              single ? sigmoidApply<true> : sigmoidApply<false>
            );
          }
          break;
        case CompiledActivationFunctionId::hardSigmoid:
          functionData.emplace_back(
            desc, single,
            single ? hardSigmoidDefineData<true> : hardSigmoidDefineData<false>,
            single ? hardSigmoidInitialize<true> : hardSigmoidInitialize<false>,
            single ? hardSigmoidApply<true> : hardSigmoidApply<false>
          );
          break;
        case CompiledActivationFunctionId::elu:
          functionData.emplace_back(
            desc, single,
            single ? eluDefineData<true> : eluDefineData<false>,
            single ? eluInitialize<true> : eluInitialize<false>,
            single ? eluApply<true> : eluApply<false>
          );
          break;
        case CompiledActivationFunctionId::selu:
          functionData.emplace_back(
            desc, single,
            single ? seluDefineData<true> : seluDefineData<false>,
            single ? seluInitialize<true> : seluInitialize<false>,
            single ? seluApply<true> : seluApply<false>
          );
        case CompiledActivationFunctionId::exponential:
          functionData.emplace_back(
            desc, single,
            single ? exponentialDefineData<true> : exponentialDefineData<false>,
            single ? exponentialInitialize<true> : exponentialInitialize<false>,
            single ? exponentialApply<true> : exponentialApply<false>
          );
        case CompiledActivationFunctionId::softsign:
          functionData.emplace_back(
            desc, single,
            single ? softsignDefineData<true> : softsignDefineData<false>,
            single ? softsignInitialize<true> : softsignInitialize<false>,
            single ? softsignApply<true> : softsignApply<false>
          );
          break;
        default:
          ASSERT(false);
      }

      functionData.back().fn.defineData(a);
      functionData.back().fn.prepare(spares, values);
      return functionData.back().fn;
    }

    void ActivationFunctionHandler::compileData(x86::Assembler& a) const
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
