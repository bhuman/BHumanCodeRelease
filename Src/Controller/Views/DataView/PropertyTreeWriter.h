/**
 * This file declares a helper class to write the property tree to a stream.
 *
 * @author Thomas Röfer
 */

#pragma once

#include <vector>
#include "PropertyManager.h"
#include "Platform/BHAssert.h"
#include "Tools/Streams/InOut.h"

/**
 * Helper class to write the property tree to a stream.
 */
class PropertyTreeWriter : public In
{
private:
  /**
   * An entry representing a position in the ConfigMap.
   */
  class Entry
  {
  public:
    const QtProperty* property; /**< The current property in the tree. */
    int type; /**< The type of the entry. -2: value or record, -1: array , >= 0: array element index. */
    int index; /**< The index of the current entry. Is increased for each sub entry. */

    /**
     * @param property The current property in the tree.
     * @param type The type of the entry. -2: value or record, -1: array , >= 0: array element index.
     */
    Entry(const QtProperty* property, int type) : property(property), type(type), index(0) {}
  };

  std::vector<Entry> stack; /**< The hierarchy of values to read. */
  const PropertyManager& propertyManager; /**< Helps converting data to the desired type. */

  /**
   * The method reads an entry from the config map.
   * The entry has been selected by select() before.
   * @param value The value that is read.
   */
  template<class T> void in(T& value)
  {
    Entry& e = stack.back();
    value = propertyManager.value(e.property).value<T>();
  }

protected:
  virtual void inBool(bool& value) { in(value); }
  virtual void inChar(char& value) { in(value); }
  virtual void inSChar(signed char& value) { in(value); }
  virtual void inUChar(unsigned char& value) { in(value); }
  virtual void inShort(short& value) { in(value); }
  virtual void inUShort(unsigned short& value) { in(value); }
  virtual void inInt(int& value) { in(value); }
  virtual void inUInt(unsigned int& value);
  virtual void inFloat(float& value) { in(value); }
  virtual void inDouble(double& value) { in(value); }
  virtual void inString(std::string& value) { in(value); }
  virtual void inAngle(Angle& value);
  virtual void inEndL() {}

public:
  PropertyTreeWriter(const PropertyManager& propertyManager, QtProperty* root) :
    propertyManager(propertyManager)
  {
    stack.push_back(Entry(root, -2));
  }

  /**
   * Select an entry for reading.
   * @param name The name of the entry if type == -2, otherwise 0.
   * @param type The type of the entry.
   *             -2: value or record,
   *             -1: array,
   *             >= 0: array element index.
   * @param enumToString A function that translates an enum to a string.
   */
  virtual void select(const char* name, int type, const char* (*enumToString)(int));

  /** Deselects a field for reading. */
  virtual void deselect() { stack.pop_back(); }

  /** Not allowed for this stream! */
  virtual void read(void* p, size_t size) { ASSERT(false); }

  /** Not allowed for this stream! */
  virtual void skip(size_t size) { ASSERT(false); }

  /** Not allowed for this stream! */
  virtual bool eof() const { ASSERT(false); return false; }
};
