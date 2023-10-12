/**
 * @file BHumanMessageParticle.h
 *
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 */

#pragma once

#include "Streaming/InOut.h"
#include "Streaming/Streamable.h"
#include "Streaming/TypeRegistry.h"
#include "Tools/Communication/BHumanMessage.h"

struct BHumanMessageParticle
{
  virtual void operator>>(BHumanMessage&) const = 0;
  virtual void operator<<(const BHumanMessage&) = 0;
};

template<typename Message>
struct BHumanCompressedMessageParticle : public BHumanMessageParticle
{
  BHumanCompressedMessageParticle() :
    _typeName("the" + TypeRegistry::demangle(typeid(Message).name()))
  {}

  void operator>>(BHumanMessage& m) const override
  {
    Streaming::streamIt(*m.out, _typeName.c_str(), static_cast<const Message&>(*this));
  }

  void operator<<(const BHumanMessage& m) override
  {
    Streaming::streamIt(*m.in, _typeName.c_str(), static_cast<Message&>(*this));
  }

private:
  std::string _typeName;
};
