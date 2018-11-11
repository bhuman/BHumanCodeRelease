/**
 * @file History.h
 * @author Andreas Stolpmann
 */

#include <vector>
#include "Platform/BHAssert.h"

template<typename T>
class History
{
private:
  std::vector<T> history;
  size_t current = 0;
  size_t last = 0;

public:
  void add(const T& t, const bool ignoreIfEqualToLast = true)
  {
    if(ignoreIfEqualToLast && current > 0 && history[current - 1] == t)
      return;

    if(current >= history.size())
      history.emplace_back(t);
    else
      history[current] = t;

    ++current;
    last = current;
  }

  void undo(T& t)
  {
    if(!undoable())
    {
      ASSERT(false);
      return;
    }
    --current;
    t = history[current - 1];
  }

  void redo(T& t)
  {
    if(!redoable())
    {
      ASSERT(false);
      return;
    }
    t = history[current];
    ++current;
  }

  bool undoable() const
  {
    return current > 1;
  }

  bool redoable() const
  {
    return current < last;
  }
};
