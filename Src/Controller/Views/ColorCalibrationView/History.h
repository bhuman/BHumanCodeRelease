/**
* @file History.h
* @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
*/

#include <vector>
#include "Platform/BHAssert.h"

template <typename T>
class History
{
private:
  std::vector<T> history;
  size_t current, last;

public:
  History() : current(0), last(0)
  {}

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
