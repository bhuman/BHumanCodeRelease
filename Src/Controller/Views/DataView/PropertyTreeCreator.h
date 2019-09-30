/**
 * The file declares a helper class to create and update the property tree.
 *
 * @author Thomas Röfer
 */

#pragma once

#include <vector>
#include <cstdio>
#include "DataView.h"
#include "Platform/BHAssert.h"
#include "Tools/Streams/InOut.h"

/**
 * Helper class to create and update the property tree.
 */
class PropertyTreeCreator : public Out
{
  /**
   * An entry representing the current state in the creation process.
   */
  class Entry
  {
  public:
    int type; /**< The type of the entry. -2: value or record, -1: array, >= 0: array element index. */
    const char* enumType; /**< A function that translates an enum to a string. */
    QtVariantProperty* parent; /**< The parent in the property tree. Can be 0 if there is none (yet). */
    QtVariantProperty* property = nullptr; /**< The current node in the property tree if already created. Otherwise 0. */
    std::string path; /**< The path to this node including its name. */
    std::string name; /**< The name of this node. */

    /**
     * @param type The type of the entry. -2: value or record, -1: array, >= 0: array element index.
     * @param enumType A function that translates an enum to a string.
     * @param parent The parent in the property tree. Can be 0 if there is none (yet).
     * @param path The path to this node including its name.
     * @param name The name of this node.
     */
    Entry(int type, const char* enumType, QtVariantProperty* parent, const std::string& path, const char* name) :
      type(type), enumType(enumType), parent(parent)
    {
      if(type >= 0)
        this->name = std::to_string(type);
      else
        this->name = name;
      this->path = (path == "" ? "" : path + ".") + this->name;
    }
  };

  DataView& view; /**< The data view using this object. */
  std::vector<Entry> stack; /**< The hierarchy of nodes that are created. */

protected:
  /** Helper to create a node. */
  template<typename T> void out(const T& value)
  {
    Entry& e = stack.back();
    ASSERT(!e.property);
    e.property = view.getProperty(e.path, TypeDescriptor::getTypeId<T>(), e.name.c_str(), e.parent);
    e.property->setValue(value);
  }

  void outChar(char value) override { out(static_cast<int>(value)); }
  void outSChar(signed char value) override { out(static_cast<int>(value)); }
  void outUChar(unsigned char value) override;
  void outShort(short value) override { out(value); }
  void outUShort(unsigned short value) override { out(value); }
  void outInt(int value) override { out(value); }
  void outUInt(unsigned int value) override;
  void outFloat(float value) override { out(value); }
  void outDouble(double value) override { out(value); }
  void outBool(bool value) override { out(value); }
  void outString(const char* value) override { out(QString(value)); }
  void outAngle(const Angle& value) override;
  void outEndL() override {}

public:
  QtVariantProperty* root = nullptr; /**< The root of the property tree. */

  /**
   * @param view The data view using this object.
   */
  PropertyTreeCreator(DataView& view) : view(view) {}

  /**
   * Select an entry for writing.
   * @param name The name of the entry if type == -2, otherwise 0.
   * @param type The type of the entry.
   *             -2: value or record,
   *             -1: array,
   *             >= 0: array element index.
   * @param enumType A function that translates an enum to a string.
   */
  void select(const char* name, int type, const char* enumType) override;

  /** Deselects a field for writing. */
  void deselect() override;

  /** Writing raw data is not supported. Do not call. */
  void write(const void* p, size_t size) override { ASSERT(false); }
};
