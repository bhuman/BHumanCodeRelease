#pragma once

#include "Tools/Streams/AutoStreamable.h"

template<typename T> STREAMABLE(Next,
{
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
  },

  (T) next,
  (unsigned)(0) readCounter,
  (unsigned)(0) writeCounter,
});
