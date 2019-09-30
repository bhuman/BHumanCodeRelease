/**
 * @file FunctionList.h
 *
 * This file declares a class that allows creating a global list of functions that
 * is created at the launch of the application. All functions in the list can be
 * called afterwards.
 *
 * @author Thomas Röfer
 */

#pragma once

class FunctionList
{
  /** A list element. */
  struct Element
  {
    void (*const function)(); /**< The function inserted into the list. */
    const Element* const next; /**< The next entry in the list or nullptr in case there is none. */

    /**
     * Insert a new element into the list.
     * @param function The function to be inserted.
     */
    Element(void (*const function)()) : function(function), next(first)
    {
      first = this;
    }
  };

  static const Element* first; /**< The first entry in the list or nullptr in case there is none. */

public:
  /**
   * Creating an instance of this class adds a function to the list.
   * @tparam function The function inserted into the list.
   */
  template<void (*const function)()> class Registrar
  {
    static Element element; /**< The list element actually created. */

  public:
    /** This constructor is required to create a dependency to "element" for the linker. */
    Registrar()
    {
      static_cast<void>(&Registrar::element);
    }
  };

  /** Execute all functions in the list. */
  static void execute();
};

template<void (*const function)()> FunctionList::Element FunctionList::Registrar<function>::element = FunctionList::Element(function);

/**
 * Add a global/static function to the global function list to be executed at launch.
 * The function must have global linkage, i.e. it must be either a static method within
 * a class or a non-static function outside a class. While functions normally can
 * publish themselves, this does not work if they are members of template classes. In
 * that can, the PUBLISH statement must be inside a method that will definitely be
 * instantiated.
 */
#define PUBLISH(...) {static FunctionList::Registrar<__VA_ARGS__> publish;}
