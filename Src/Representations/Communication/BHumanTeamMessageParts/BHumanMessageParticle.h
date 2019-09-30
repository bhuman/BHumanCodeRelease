/**
 * @file BHumanMessageParticle.h
 *
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 */

#pragma once

#include "Representations/Communication/BHumanMessage.h"
#include "Tools/Streams/Streamable.h"
#include "Tools/MessageQueue/InMessage.h"
#include "Tools/MessageQueue/MessageIDs.h"
#include <functional>

template<MessageID ID>
struct BHumanMessageParticle
{
  static MessageID id() { return ID; }

  virtual void operator>>(BHumanMessage& m) const = 0;
  virtual void operator<<(const BHumanMessage& m) { ; };
  virtual bool handleArbitraryMessage(InMessage& m, const std::function<unsigned(unsigned u)>& toLocalTimestamp) { return false; };
};

template<MessageID ID>
In& operator>>(In& in, BHumanMessageParticle<ID>& particle)
{
  return in >> dynamic_cast<Streamable&>(particle);
}

template<MessageID ID>
Out& operator<<(Out& out, const BHumanMessageParticle<ID>& particle)
{
  return out << dynamic_cast<const Streamable&>(particle);
}

template<MessageID ID>
struct PureBHumanArbitraryMessageParticle : public BHumanMessageParticle<ID>
{
  void operator>>(BHumanMessage& m) const override
  {
    m.theBHumanArbitraryMessage.queue.out.bin << *this;
    m.theBHumanArbitraryMessage.queue.out.finishMessage(this->id());
  }

  void operator<<(const BHumanMessage& m) override { ; }

  bool handleArbitraryMessage(InMessage& m, const std::function<unsigned(unsigned u)>& toLocalTimestamp) override
  {
    ASSERT(m.getMessageID() == this->id());
    m.bin >> *this;
    return true;
  }
};
