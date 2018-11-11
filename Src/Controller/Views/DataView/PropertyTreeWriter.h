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
  template<typename T> void in(T& value)
  {
    Entry& e = stack.back();
    value = propertyManager.value(e.property).value<T>();
  }

protected:
  void inBool(bool& value) override { in(value); }
  void inChar(char& value) override { in(value); }
  void inSChar(signed char& value) override { in(value); }
  void inUChar(unsigned char& value) override { in(value); }
  void inShort(short& value) override { in(value); }
  void inUShort(unsigned short& value) override { in(value); }
  void inInt(int& value) override { in(value); }
  void inUInt(unsigned int& value) override;
  void inFloat(float& value) override { in(value); }
  void inDouble(double& value) override { in(value); }
  void inString(std::string& value) override { in(value); }
  void inAngle(Angle& value) override;
  void inEndL() override {}

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
   * @param enumType A function that translates an enum to a string.
   */
  void select(const char* name, int type, const char* enumType) override;

  /** Deselects a field for reading. */
  void deselect() override { stack.pop_back(); }

  /** Not allowed for this stream! */
  void read(void* p, size_t size) override { ASSERT(false); }

  /** Not allowed for this stream! */
  void skip(size_t size) override { ASSERT(false); }

  /** Not allowed for this stream! */
  bool eof() const override { ASSERT(false); return false; }
};
