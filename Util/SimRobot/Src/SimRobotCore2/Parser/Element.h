/**
* @file Simulation/Element.h
* Declaration of class Element
* @author Colin Graf
*/

#pragma once

/**
* @class Element
* An abstract representation of a ros2-file xml element
*/
class Element
{
public:
  /** Constructor */
  Element();

  /** Destructor */
  virtual ~Element() = default;

  /**
  * Registers an element as parent
  * @param element The element to register
  */
  virtual void addParent(Element& element) {}
};
