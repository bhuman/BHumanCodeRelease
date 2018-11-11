/**
 * @file FunctionList.cpp
 *
 * This file implements a class that allows creating a global list of functions that
 * is created at the launch of the application. All functions in the list can be
 * called afterwards.
 *
 * @author Thomas Röfer
 */

#include "FunctionList.h"
#include <list>

const FunctionList::Element* FunctionList::first = nullptr;

void FunctionList::execute()
{
  // Functions were always added to the front, i.e. the list is reversed.
  // Reverse it again to execute function in the sequence they were defined.
  std::list<const Element*> elements;
  for(const Element* element = first; element; element = element->next)
    elements.push_front(element);
  for(const Element* element : elements)
    element->function();
}
