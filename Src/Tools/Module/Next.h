#pragma once

#include "Platform/SystemCall.h"
#include "Tools/Streams/Streamable.h"

template<class T>
class Next : public Streamable
{
private:
  T next;
  unsigned readCounter = 0;
  unsigned writeCounter = 0;

public:
  Next() = default;
  Next(const Next& other) = delete;

  Next& operator=(const Next& other) = delete;

  void setNext(const T& next)
  {
    ++writeCounter;
    this->next = next;
  }

  bool hasNext() const
  {
    return writeCounter > readCounter;
  }

  T getNext()
  {
    readCounter = writeCounter;
    return next;
  }

private:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(next);
    STREAM(readCounter);
    STREAM(writeCounter);
    STREAM_REGISTER_FINISH;
  }
};