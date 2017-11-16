/**
 * @file Tools/Math/FixedPoint.h
 * A class to represent FixedPoint numbers
 * @author <a href="mailto:alexists@tzi.de">Alexis Tsogias</a>
 */

#pragma once

#include <cstdint>

template<int32_t precisionBits>
class FixedPoint
{
public:
  constexpr FixedPoint() = default;
  constexpr FixedPoint(const FixedPoint& other) : raw(other.raw) {}
  explicit constexpr FixedPoint(int32_t value) : raw(fromInt32(value)) {};
  explicit constexpr FixedPoint(float value) : raw(fromFloat(value)) {};

  static constexpr FixedPoint fromRaw(int32_t raw) { return FixedPoint(RawInt(raw)); }

  FixedPoint& operator=(const FixedPoint& other) { raw = other.raw; return *this; }
  FixedPoint& operator=(int32_t value) { raw = fromInt32(value); return *this; }
  FixedPoint& operator=(float value) { raw = fromFloat(value); return *this; }

  constexpr operator int32_t () const { return raw >> precisionBits; }
  constexpr operator float() const { return raw / static_cast<float>(one); }

  constexpr int32_t getRaw() const { return raw; }

private:
  int32_t raw = 0;

  struct RawInt
  {
    int32_t raw;
    RawInt(int32_t raw) : raw(raw) {}
  };
  constexpr FixedPoint(RawInt rawInt) : raw(rawInt.raw) {}

  static const constexpr int32_t one = 1 << precisionBits;

  static constexpr int32_t fromInt32(int32_t value)
  {
    return value << precisionBits;
  }

  static constexpr int32_t fromFloat(float value)
  {
    return static_cast<int32_t>(value * static_cast<float>(one) + (value < 0 ? -0.5f : 0.5f));
  }
};

using FixedPoint5 = FixedPoint<5>;
using FixedPoint7 = FixedPoint<7>;
