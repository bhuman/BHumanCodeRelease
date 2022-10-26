/**
 * @file KeyframeMotionParameters.h
 * Declaration of a struct for representing the phases for keyframe based motions
 * @author <a href="mailto:s_ksfo6n@uni-bremen.de">Philip Reichenberg</a>
 */

#pragma once
#include "Streaming/Enum.h"
#include "Streaming/EnumIndexedArray.h"
#include "Math/Range.h"

namespace KeyframeMotionPhases
{
  ENUM(KeyframeMotionPhase,
  {,
    UnknownPhase,
    PushingWithArmsFromGround,
    ReduceVel,
    Split,
    HalfSplit,
    Sit,
    Stand,
    StandStatic,
  });
}

// Angle and float as one struct
STREAMABLE(DirectionValue,
{
  DirectionValue() = default;
  DirectionValue(const Rangea& pitch, const Rangea& roll);
  void assignDirectionValue(const DirectionValue& from);
  void assignIgnoreValue(const Angle& defaultValue);
  void replaceIgnoreDirectionValue(const DirectionValue& from);
  DirectionValue operator-(const DirectionValue& other) const;
  DirectionValue operator+(const DirectionValue& other) const;
  DirectionValue operator*(const float value) const,

  (Rangea)(Rangea(-2000_deg, 2000_deg)) pitchDirection, // value for pitch direction (relative to the robots origin)
  (Rangea)(Rangea(-2000_deg, 2000_deg)) rollDirection, // value for roll direction (relative to the robots origin)
});

inline DirectionValue DirectionValue::operator-(const DirectionValue& other) const
{
  DirectionValue dv;
  dv.pitchDirection = Rangea(this->pitchDirection.min - other.pitchDirection.min, this->pitchDirection.max - other.pitchDirection.max);
  dv.rollDirection = Rangea(this->rollDirection.min - other.rollDirection.min, this->rollDirection.max - other.rollDirection.max);
  return dv;
}
inline DirectionValue DirectionValue::operator+(const DirectionValue& other) const
{
  DirectionValue dv;
  dv.pitchDirection = Rangea(this->pitchDirection.min + other.pitchDirection.min, this->pitchDirection.max + other.pitchDirection.max);
  dv.rollDirection = Rangea(this->rollDirection.min + other.rollDirection.min, this->rollDirection.max + other.rollDirection.max);
  return dv;
}
inline DirectionValue DirectionValue::operator*(const float value) const
{
  DirectionValue dv;
  dv.pitchDirection = Rangea(this->pitchDirection.min * value, this->pitchDirection.max * value);
  dv.rollDirection = Rangea(this->rollDirection.min * value, this->rollDirection.max * value);
  return dv;
}

inline void DirectionValue::assignDirectionValue(const DirectionValue& from)
{
  if(from.pitchDirection.min != JointAngles::ignore)
    this->pitchDirection.min = from.pitchDirection.min;
  if(from.pitchDirection.max != JointAngles::ignore)
    this->pitchDirection.max = from.pitchDirection.max;
  if(from.rollDirection.min != JointAngles::ignore)
    this->rollDirection.min = from.rollDirection.min;
  if(from.rollDirection.max != JointAngles::ignore)
    this->rollDirection.max = from.rollDirection.max;
}

inline void DirectionValue::replaceIgnoreDirectionValue(const DirectionValue& from)
{
  if(this->pitchDirection.min == JointAngles::ignore)
    this->pitchDirection.min = from.pitchDirection.min;
  if(this->pitchDirection.max == JointAngles::ignore)
    this->pitchDirection.max = from.pitchDirection.max;
  if(this->rollDirection.min == JointAngles::ignore)
    this->rollDirection.min = from.rollDirection.min;
  if(this->rollDirection.max == JointAngles::ignore)
    this->rollDirection.max = from.rollDirection.max;
}

inline void DirectionValue::assignIgnoreValue(const Angle& defaultValue)
{
  if(this->pitchDirection.min == JointAngles::ignore)
    this->pitchDirection.min = -defaultValue;
  if(this->pitchDirection.max == JointAngles::ignore)
    this->pitchDirection.max = defaultValue;
  if(this->rollDirection.min == JointAngles::ignore)
    this->rollDirection.min = -defaultValue;
  if(this->rollDirection.max == JointAngles::ignore)
    this->rollDirection.max = defaultValue;
}

STREAMABLE(KeyframeMotionParameters,
{
  STREAMABLE(PIDParameter,
  {
    PIDParameter() = default;
    PIDParameter(const float base, const float maxAdditional),

    (float)(0.f) baseValue, // base PID parameter
    (float)(0.f) maxAdditionalValue, // max this value is additionally added on top of the baseValue
  });

  STREAMABLE(BalanceFactors,
  {,
    (DirectionValue) maxValAdditionalScaling, // increaseFactorCap
    (DirectionValue) exponentFactorScaling, // powDegree
    (PIDParameter)(0.f, 0.f) pidPPitchParameter,  // borderPIDPX,
    (PIDParameter)(0.f, 0.f) pidPRollParameter, // borderPIDPY,
    (PIDParameter)(0.f, 0.f) pidDPitchParameter, // borderPIDDX,
    (PIDParameter)(0.f, 0.f) pidDRollParameter, // borderPIDDY,
  });

  STREAMABLE(BalancingParameter,
  {,
    (BalanceFactors) balanceFactor,
    (DirectionValue)(Rangea(-2000.f, 2000.f), Rangea(-2000.f, 2000.f)) comDiffBase,
  }),

  (ENUM_INDEXED_ARRAY(BalancingParameter, KeyframeMotionPhases::KeyframeMotionPhase)) balanceList,
});

inline DirectionValue::DirectionValue(const Rangea& pitch, const Rangea& roll) : pitchDirection(pitch), rollDirection(roll) {}
inline KeyframeMotionParameters::PIDParameter::PIDParameter(const float base, const float maxAdditional) : baseValue(base), maxAdditionalValue(maxAdditional) {}
