/**
 * @file TypeInfo.h
 *
 * This file declares a class that stores type information about primitive data types,
 * enumerations, and classes/structures. In contrast to the class TypeRegistry, this
 * class contains all type information in demangled and unified form. It is also
 * streamable.
 *
 * In their demangled form, type names can have two types of extensions at their end:
 *
 * '[<n>]' The type is an arrays with a fixed number of <n> elements. Such entries can
 *         either result from a normal C array or from std::array.
 *
 * '*'     The type is a dynamic array with an arbitrary number of elements. The actual
 *         number of elements will be part of the data (preceding the elements). Such
 *         entries can result from either std::vector or std::list.
 *
 * @todo Support representation of more complex types (std::array<std::array<int, 1>, 2>)
 * in the correct order. Currently, the order of extensions is inverted.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "InOut.h"
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

struct TypeInfo
{
  /** An attribute of a class. */
  struct Attribute
  {
    std::string type; /**< The type of the attribute. */
    std::string name; /**< The name of the attribute. */

    /** Keep default constructor. */
    Attribute() = default;

    /**
     * Constructor that initializes all attributes from parameters.
     * @param type The type of the attribute.
     * @param name The name of the attribute.
     */
    Attribute(const std::string& type, const std::string& name) : type(type), name(name) {}
  };

  std::unordered_set<std::string> primitives; /**< All primitive data types. */
  std::unordered_map<std::string, std::vector<std::string>> enums; /**< All enumeration types. */
  std::unordered_map<std::string, std::vector<Attribute>> classes; /**< All classes and structures. */

  /**
   * Default constructor.
   * @param fromTypeRegistry Automatically fills this instance from type registry.
   */
  TypeInfo(bool fromTypeRegistry = true);

  /**
   * Checks whether a type for another type information is deeply equal to
   * a type for this type information.
   * @param other The other type information.
   * @param thisType The type name for this type information.
   * @param otherType The type name for the other type information.
   */
  bool areTypesEqual(const TypeInfo& other, const std::string& thisType, const std::string& otherType) const;
};

/**
 * Writes the type information to a stream.
 * @param out The stream the type information is written to.
 * @param typeInfo The type information written.
 * @return The stream after the type information was written.
 */
Out& operator<<(Out& out, const TypeInfo& typeInfo);

/**
 * Reads the type information from a stream.
 * @param out The stream the type information is read from.
 * @param typeInfo The type information read.
 * @return The stream after the type information was read.
 */
In& operator>>(In& in, TypeInfo& typeInfo);
