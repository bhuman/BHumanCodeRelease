#pragma once

#include "Tools/Streams/AutoStreamable.h"

template<typename T> STREAMABLE(Next,
{
  void setNext(const T& next)
  {
    updated = true;
    this->next = next;
  }

  bool hasNext() const {return updated;}

  const T& getNext() const
  {
    const_cast<Next<T>*>(this)->updated = false;
    return next;
  }

private:,
  (T) next,
  (bool)(false) updated,
});
