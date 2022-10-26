/**
 * @file TypeRegistry.h
 *
 * This file declares a class that stores type information about primitive data types,
 * enumerations, and classes/structures. The avoid a large overhead, the class often
 * uses "const char*" pointers as parameters. It assumes that these stem from a global
 * string pool so that they exist throughout the execution of the program and that
 * each string only exists once, i.e. its address is sufficient to identify it.
 *
 * @author Thomas Röfer
 */

#pragma once

#include <string>
#include <typeinfo>

struct TypeInfo;

class TypeRegistry
{
public:
  /**
   * Add the name of an enumeration to the registry.
   * Must only be called once for each enumeration type.
   * @param enumeration The name of the enumeration type.
   */
  static void addEnum(const char* enumeration);

  /**
   * Add a constant of an enumeration type to the registry.
   * The enumeration type must already have been registered when calling this function.
   * All enumeration constants must be registered in the sequence they are defined. In
   * addition, their numerical values must start with 0 and continue in increments of 1.
   * @param enumeration The name of the enumeration type.
   * @param name The name of the enumeration constant.
   */
  static void addEnumConstant(const char* enumeration, const char* name);

  /**
   * Add the name of a class or structure to the registry.
   * Must only be called once for each class or structure.
   * @param theClass The name of the class or structure.
   */
  static void addClass(const char* theClass, const char* base = nullptr);

  /**
   * Add an attribute of a class or structure to the registry.
   * The class or structure must already have been registered when calling this function.
   * All attributes must be registered in the sequence they are defined.
   * @param theClass The name of the class or structure.
   * @param type The name of the type of the attribute.
   * @param attribute The name of the attribute.
   */
  static void addAttribute(const char* theClass, const char* type, const char* attribute);

  /**
   * Determine the name of an enumeration constant from its numerical value.
   * @param enumeration The name of the enumeration type.
   * @param value The numerical value of the enumeration constant.
   * @return The name of the constant or nullptr, if index is not within the value range
   *         of the enumeration type.
   */
  static const char* getEnumName(const char* enumeration, int value);

  /**
   * Determine the name of an enumeration constant from its numerical value.
   * @param value The numerical value of the enumeration constant.
   * @return The name of the constant or nullptr, if index is not within the value range
   *         of the enumeration type.
   */
  template<typename E> static const char* getEnumName(E value)
  {
    return getEnumName(typeid(E).name(), value);
  }

  /**
   * Determine the numerical value of an enumeration constant from its name.
   * @param enumeration The name of the enumeration type.
   * @param name The name of the enumeration constant.
   * @return The numerical value of the constant or -1 if it is not a valid constant
   *         of the enumeration type.
   */
  static int getEnumValue(const char* enumeration, const std::string& name);

  /**
   * Converts a string returned by typeid().name() into a platform independent one.
   * @param type A type name created on this platform.
   * @return A platform independent, human-readable type name.
   */
  static std::string demangle(std::string type);

  /** Print all type information stored to the console for debugging purposes. */
  static void print();

  /**
   * Copies demangled type data to a streamable object.
   * @param typeInfo The object that receives the data. It must initially be empty.
   */
  static void fill(TypeInfo& typeInfo);
};
