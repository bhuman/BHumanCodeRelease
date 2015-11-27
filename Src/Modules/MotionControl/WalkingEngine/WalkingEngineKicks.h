/**
 * @file WalkingEngineKicks.h
 * Declaration of walking engine kicks and tools to use them
 * @author Colin Graf
 */

#pragma once

#include "Representations/MotionControl/WalkRequest.h"
#include "Tools/Joints.h"
#include "Tools/Math/Pose3f.h"

class WalkingEngineKick
{
public:
  enum Track
  {
    headYaw = Joints::headYaw,
    headPitch = Joints::headPitch,
    lShoulderPitch = Joints::lShoulderPitch,
    lShoulderRoll = Joints::lShoulderRoll,
    lElbowYaw = Joints::lElbowYaw,
    lElbowRoll = Joints::lElbowRoll,
    rShoulderPitch = Joints::rShoulderPitch,
    rShoulderRoll = Joints::rShoulderRoll,
    rElbowYaw = Joints::rElbowYaw,
    rElbowRoll = Joints::rElbowRoll,
    numOfJointTracks,
    footTranslationX = numOfJointTracks,
    footTranslationY,
    footTranslationZ,
    footRotationX,
    footRotationY,
    footRotationZ,
    numOfTracks,
  };

  WalkingEngineKick();
  ~WalkingEngineKick();

  bool load(const char* filePath);
  bool load(const char* filePath, char* data);
  void getPreStepSize(float& rotation, Vector3f& translation) const;
  void getStepSize(float& rotation, Vector3f& translation) const;
  float getStepDuration() const;
  float getRefX(float defaultValue) const;

private:
  class String
  {
  public:
    template<int N>String(const char(&ptr)[N]) : ptr(ptr), len(N - 1) {}
    String(const char* ptr, size_t len) : ptr(ptr), len(len) {}
    bool operator==(const String& other) const;
  private:
    const char* ptr;
    size_t len;
  };

  class Value
  {
  public:
    Value(WalkingEngineKick& kick) : next(kick.firstValue) {kick.firstValue = this;}
    virtual ~Value() = default;

    virtual float evaluate() const = 0;

  protected:
    float value;

  private:
    Value* next;

    friend class WalkingEngineKick;
  };

  class ConstantValue : public Value
  {
  public:
    ConstantValue(float value, WalkingEngineKick& kick) : Value(kick) {this->value = value;}

  private:
    virtual float evaluate() const {return value;}
  };

  class BinaryExpression : public Value
  {
  public:
    BinaryExpression(Value& operand1, Value& operand2, WalkingEngineKick& kick) : Value(kick), operand1(operand1), operand2(operand2) {}

  protected:
    Value& operand1;
    Value& operand2;
  };

  class PlusExpression : public BinaryExpression
  {
  public:
    PlusExpression(Value& operand1, Value& operand2, WalkingEngineKick& kick) : BinaryExpression(operand1, operand2, kick) {}

  private:
    virtual float evaluate() const {return operand1.evaluate() + operand2.evaluate();}
  };

  class MinusExpression : public BinaryExpression
  {
  public:
    MinusExpression(Value& operand1, Value& operand2, WalkingEngineKick& kick) : BinaryExpression(operand1, operand2, kick) {}

  private:
    virtual float evaluate() const {return operand1.evaluate() - operand2.evaluate();}
  };

  class TimesExpression : public BinaryExpression
  {
  public:
    TimesExpression(Value& operand1, Value& operand2, WalkingEngineKick& kick) : BinaryExpression(operand1, operand2, kick) {}

  private:
    virtual float evaluate() const {return operand1.evaluate() * operand2.evaluate();}
  };

  class DivExpression : public BinaryExpression
  {
  public:
    DivExpression(Value& operand1, Value& operand2, WalkingEngineKick& kick) : BinaryExpression(operand1, operand2, kick) {}

  private:
    virtual float evaluate() const {return operand1.evaluate() / operand2.evaluate();}
  };

  class ParameterValue : public Value
  {
  public:
    ParameterValue(unsigned int index, WalkingEngineKick& kick) : Value(kick), index(index), kick(kick) {}

  private:
    unsigned int index;
    WalkingEngineKick& kick;

    virtual float evaluate() const {return kick.getParameterValue(index);}
  };

  class ParseException
  {
  public:
    ParseException(const char* message) : message(message) {}
    const char* message;
  };

  class PhaseInfo
  {
  public:
    Value* posValue;
    Value* lengthValue;

    PhaseInfo(Value* pos) : posValue(pos), lengthValue(0) {}
  };

  Value* firstValue;

  Value* preStepSizeRValue;
  Value* preStepSizeXValue;
  Value* preStepSizeYValue;
  Value* preStepSizeZValue;
  Value* stepSizeRValue;
  Value* stepSizeXValue;
  Value* stepSizeYValue;
  Value* stepSizeZValue;
  Value* durationValue;
  Value* refXValue;
  std::vector<PhaseInfo> tracks[numOfTracks];

  float getParameterValue(unsigned int index) {return 0.f;}

  void addPhase(Track track, Value* value);

  String readString(char*& buf);
  unsigned int readUInt(char*& buf);
  float readFloat(char*& buf);
  Value* readValue(char*& buf);
  Value* readPlusFormula(char*& buf);
  Value* readMultFormula(char*& buf);

  friend class WalkingEngineKickPlayer;
};

class WalkingEngineKicks
{
public:

  void load();
  void load(WalkRequest::KickType type, char* data);

  bool isKickMirrored(WalkRequest::KickType type) const {return (type - 1) % 2 != 0;}
  void getKickStepSize(WalkRequest::KickType type, float& rotation, Vector3f& translation) const;
  void getKickPreStepSize(WalkRequest::KickType type, float& rotation, Vector3f& translation) const;
  float getKickStepDuration(WalkRequest::KickType type) const;
  float getKickRefX(WalkRequest::KickType type, float defaultValue) const;
  const WalkingEngineKick* getKick(WalkRequest::KickType type) const;

private:
  WalkingEngineKick kicks[(WalkRequest::numOfKickTypes - 1) / 2];
};

class WalkingEngineKickPlayer
{
public:
  WalkingEngineKickPlayer() : kick(0) {}

  /**
  * Starts replaying a kick motion
  * @param kick The kick motion to replay
  * @param mirrored Whether the replayed kick should be mirrored
  */
  void start(const WalkingEngineKick& kick, bool mirrored);

  /**
  * Stops replaying a kick
  */
  void stop() {kick = 0;}

  /**
  * Moves the player to the given position
  * @param t The position in seconds to move the player to
  */
  void seekTo(float t);

  /**
  * Returns the length of the currently replayed kick
  * @return The length of the kick in seconds
  */
  float getLength() const {return length * 0.001f;}

  void applyFoot(Pose3f& leftOriginToFoot, Pose3f& rightOriginToFoot);
  void applyHeadAndArms(float headJointAngles[2], float leftArmJointAngles[4], float rightArmJointAngles[4]);

  /**
  * Whether the kick player is currently replaying a kick
  * @return true if so
  */
  bool isActive() const {return !!kick;}

private:
  class Phase
  {
  public:
    bool posEvaluated;
    float pos;
    float velocity;
    float start;
    float end;
    float length;
    const WalkingEngineKick::PhaseInfo* info;

    void evaluatePos(float externValue)
    {
      if(!posEvaluated)
      {
        pos = info->posValue ? info->posValue->evaluate() : externValue;
        posEvaluated = true;
      }
    }
  };

  const WalkingEngineKick* kick;
  bool mirrored;

  std::vector<Phase> tracks[WalkingEngineKick::numOfTracks];
  int currentPhases[WalkingEngineKick::numOfTracks];
  float currentPosition;
  float length; /**< The length of the currently executed kick in ms */

  float getValue(WalkingEngineKick::Track track, float externValue);
};
