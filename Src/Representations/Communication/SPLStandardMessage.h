/**
 * @file SPLStandardMessage.h
 * The file declares a struct that encapsulates the structure SPLStandardMessage
 * defined in the file SPLStandardMessage.h that is provided with the GameController.
 * @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
 */

#pragma once

#include "Representations/Infrastructure/RoboCupGameControlData.h"
#include "Tools/MessageQueue/MessageQueue.h"
#include "Tools/Streams/Streamable.h"

struct SPLStandardMessage : public RoboCup::SPLStandardMessage, public Streamable, public MessageHandler
{
private:
  struct BHumanHeader
  {
    unsigned timestamp;
    unsigned ballTimeWhenLastSeen;
    unsigned ballTimeWhenDisappeared;
    short ballLastPerceptX;
    short ballLastPerceptY;
    std::array<float, 3> ballCovariance;
    float robotPoseDeviation;
    std::array<float, 6> robotPoseCovariance;
    unsigned char robotPoseValidity;
    unsigned char magicNumber;
    char sizeMarker; /**< Helper to determine size without padding. Must be last field. */
  };

  MessageQueue theOutMsgData;

public:
  static const size_t bhumanHeaderSize = offsetof(BHumanHeader, sizeMarker);

  SPLStandardMessage();
  SPLStandardMessage(const SPLStandardMessage& other);
  SPLStandardMessage(MessageQueue& out);

  SPLStandardMessage& operator=(const SPLStandardMessage& other);

  void toMessageQueue(MessageQueue& in, const unsigned remoteIp, const unsigned realNumOfDataBytes);

private:
  bool handleMessage(InMessage& message);

  virtual void serialize(In* in, Out* out);
};

struct SPLStandardMessageWithoutData : public SPLStandardMessage
{
public:
  SPLStandardMessageWithoutData() = default;
  SPLStandardMessageWithoutData& operator=(const SPLStandardMessage& other);
  SPLStandardMessageWithoutData& operator=(const SPLStandardMessageWithoutData& other) = default;

private:
  virtual void serialize(In* in, Out* out);
};
