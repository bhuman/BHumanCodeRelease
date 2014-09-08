/**
 * @file SPLStandardMessage.h
 * The file declares a class that encapsulates the structure SPLStandardMessage
 * defined in the file SPLStandardMessage.h that is provided with the GameController.
 * @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
 */

#pragma once

#include "Representations/Infrastructure/RoboCupGameControlData.h"
#include "Tools/MessageQueue/MessageQueue.h"
#include "Tools/Streams/Streamable.h"

class SPLStandardMessage : public RoboCup::SPLStandardMessage, public Streamable, public MessageHandler
{
private:
  struct BHumanHeader
  {
    unsigned timestamp;
    unsigned short messageSize;
    unsigned ballTimeWhenLastSeen;
    unsigned ballTimeWhenDisappeared;
    short ballLastPerceptX;
    short ballLastPerceptY;
    float robotPoseDeviation;
    unsigned char robotPoseValidity;
    char sizeMarker; /**< Helper to determine size without padding. Must be last field. */
  };

  MessageQueue theOutMsgData;

  bool handleMessage(InMessage& message);

  virtual void serialize(In* in, Out* out);

public:
  SPLStandardMessage();

  void toMessageQueue(MessageQueue& in, const unsigned remoteIp);
  unsigned fromMessageQueue(MessageQueue& out);
};

class SPLStandardMessageWithoutData : public SPLStandardMessage
{
private:
  virtual void serialize(In* in, Out* out);

public:
  SPLStandardMessageWithoutData() : SPLStandardMessage()
  {}
  SPLStandardMessageWithoutData(const SPLStandardMessage& other);

  SPLStandardMessageWithoutData(const SPLStandardMessageWithoutData& other);

  SPLStandardMessageWithoutData& operator=(const SPLStandardMessage& other);
  SPLStandardMessageWithoutData& operator=(const SPLStandardMessageWithoutData& other);
};