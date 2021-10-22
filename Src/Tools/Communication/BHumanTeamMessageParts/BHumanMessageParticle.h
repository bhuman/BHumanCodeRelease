/**
 * @file BHumanMessageParticle.h
 *
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 */

#pragma once

#include "Platform/BHAssert.h"
#include "Representations/Communication/BHumanMessage.h"
#include "Tools/Streams/Streamable.h"
#include "Tools/MessageQueue/InMessage.h"
#include "Tools/MessageQueue/MessageIDs.h"
#include <functional>

template<MessageID ID>
struct BHumanMessageParticle
{
  static MessageID id() { return ID; }

  virtual void operator>>(BHumanMessage&) const = 0;
  virtual void operator<<(const BHumanMessage&) {}
  virtual bool handleArbitraryMessage(InMessage&, const std::function<unsigned(unsigned u)>& toLocalTimestamp);
};

template<MessageID ID>
inline bool BHumanMessageParticle<ID>::handleArbitraryMessage(InMessage&, const std::function<unsigned(unsigned u)>&) { return false; }

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

  void operator<<(const BHumanMessage&) override {}

  bool handleArbitraryMessage(InMessage& m, const std::function<unsigned(unsigned u)>&) override
  {
    ASSERT(m.getMessageID() == this->id());
    m.bin >> *this;
    return true;
  }
};

template<typename Message>
struct BHumanCompressedMessageParticle : public BHumanMessageParticle<undefined>
{
  BHumanCompressedMessageParticle() :
    _typeName("the" + TypeRegistry::demangle(typeid(Message).name()))
  {}

  void operator>>(BHumanMessage& m) const override
  {
    Streaming::streamIt(*m.theBHumanStandardMessage.out, _typeName.c_str(), *this);
  }

  void operator<<(const BHumanMessage& m) override
  {
    Streaming::streamIt(*m.theBHumanStandardMessage.in, _typeName.c_str(), *this);
  }

private:
  std::string _typeName;
};
