/**
* @file ConfigMap.h
*
* Declaration of the ConfigMap class hierarchy.
*
* @author <a href="mailto::ojlr@informatik.uni-bremen.de">Ole Jan Lars Riemann</a>
*/

#pragma once
#include <streambuf>
#include <fstream>
#include <map>
#include <vector>
#include <stdexcept>
#include <sstream>
#include <cstring>

#include "Tools/Streams/Streamable.h"

#ifdef __clang__ // workaround for clang bug
#pragma clang system_header
#endif

#define STR(s) #s

/**
 * Represents an access to an invalid key of a ConfigMap.
 */
class invalid_key : public std::logic_error
{
  public:
    explicit invalid_key (const std::string& what_arg) : logic_error(what_arg)
    { }
};

/**
 * Represents a bad cast inside a ConfigValue. It is thrown if a operation an a
 * ConfigValue object ist executed which does not match the inner type of the
 * ConfigValue.
 */
class bad_cm_cast : public std::bad_cast
{
  std::string msg;
public:
  bad_cm_cast(const std::string &reason)
    : msg(reason)
  { }
  const char* what() const throw()
  {
    return msg.c_str();
  }
  virtual ~bad_cm_cast() throw() { }
};

class ConfigValue;
class ConfigValueBase;
class ConfigMap;
class ListConfigValue;
class ConfigValueProxy;
class PlainConfigValue;

/**
 * @class ConfigValue
 * ConfigValue interface
 */
class ConfigValue {
protected:
  virtual const ConfigValue* getParent() const = 0;
  virtual void setParent(ConfigValue *cv) = 0;
  virtual ConfigValue* deepCopy() const = 0;
public:
  virtual ~ConfigValue() { }
  enum Type
  {
    PLAIN,
    LIST,
    MAP
  };
  static std::string type2MapStr(Type type);
  virtual Type getType() const = 0;
  virtual bool isReadOnly() const = 0;
  virtual operator PlainConfigValue&() = 0;
  virtual operator ListConfigValue&() = 0;
  virtual operator ConfigMap&() = 0;
  virtual operator const PlainConfigValue&() const = 0;
  virtual operator const ListConfigValue&() const = 0;
  virtual operator const ConfigMap&() const = 0;
  virtual void setComment(const std::string &c) = 0;
  virtual const std::string& getComment() const = 0;
  virtual size_t length() const = 0;
  virtual std::ostream& write(std::ostream &os, bool comment = true, std::string indentation = "") const = 0;
  virtual ConfigValueProxy operator[](size_t index);
  virtual ConfigValueProxy operator[](const std::string &key);
  virtual const ConfigValue& operator[](size_t index) const;
  virtual const ConfigValue& operator[](const std::string &key) const;
  virtual ConfigValue& operator+=(const ConfigValue &rhs) = 0;
  std::string str() const;

  friend class ListConfigValue;
  friend class ConfigMap;
  friend class ConfigValueProxy;
};

/**
 * Specializes operator<< for the delivered type. The declaration part. Place it
 * into the header.
 * @param vType The type for which the operator should be specialized. Do not
 *              add an extra const since it is done by the macro.
 */
#define CONFIGMAP_STREAM_IN_DELCARE(vType) \
template<> ConfigValue& operator<<<vType>(ConfigValue &cv, const vType &value); \
template<> ConfigValue* createConfigValueFor<vType>(const vType &value);

/**
 * Specializes operator<< for the delivered type. The implementation part. Place
 * it wherever is works.
 * @param cvType The subtype of ConfigValue, which you desire for your value
 *               type.
 * @param vType  The type for which the operator should be specialized. Do not
 *               add an extra const since it is done by the macro. The type has
 *               to match to the one of the declaration part. ;-)
 */
#define CONFIGMAP_STREAM_IN(cvType, vType) \
template<> ConfigValue& operator<<<vType>(ConfigValue &cv, const vType &value) \
{ \
  cvType &ccv = cv; \
  return ccv << value; \
} \
template<> ConfigValue* createConfigValueFor<vType>(const vType &value) \
{ \
  return new cvType(); \
}

/**
 * Specializes operator>> for the delivered type. The declaration part. Place it
 * into the header.
 * @param vType The type for which the operator should be specialized.
 */
#define CONFIGMAP_STREAM_OUT_DELCARE(vType) \
template<> const ConfigValue& operator>><vType>(const ConfigValue &cv, vType &value);

/**
 * Specializes operator>> for the delivered type. The implementation part. Place
 * it wherever is works.
 * @param cvType The subtype of ConfigValue, which you desire for your value
 *               type.
 * @param vType  The type for which the operator should be specialized. The type
 *               has to match to the one of the declaration part. ;-)
 */
#define CONFIGMAP_STREAM_OUT(cvType, vType) \
template<> const ConfigValue& operator>><vType>(const ConfigValue &cv, vType &value) \
{ \
  const cvType &ccv = cv; \
  return ccv >> value; \
}

class ConfigValueBase : public ConfigValue {
  std::string comment;
  ConfigValue *parent;
protected:
  ConfigValueBase()
    : comment(""),
      parent(0)
  { }
  const ConfigValue* getParent() const;
  void setParent(ConfigValue *cv);
public:
  ConfigValueBase& operator=(const ConfigValue &other);
  bool isReadOnly() const;
  virtual operator PlainConfigValue&()
  {
    throw bad_cm_cast(__FILE__ " Line " STR(__LINE__ )": Expected PlainConfigValue, got " + type2MapStr(getType()));
  }
  virtual operator ListConfigValue&()
  {
    throw bad_cm_cast(__FILE__ " Line " STR(__LINE__ )": Expected ListConfigValue, got " + type2MapStr(getType()));
  }
  virtual operator ConfigMap&()
  {
    throw bad_cm_cast(__FILE__ " Line " STR(__LINE__ )": Expected ConfigMap, got " + type2MapStr(getType()));
  }
  virtual operator const PlainConfigValue&() const
  {
    throw bad_cm_cast(__FILE__ " Line " STR(__LINE__ )": Expected PlainConfigValue, got " + type2MapStr(getType()));
  }
  virtual operator const ListConfigValue&() const
  {
    throw bad_cm_cast(__FILE__ " Line " STR(__LINE__ )": Expected ListConfigValue, got " + type2MapStr(getType()));
  }
  virtual operator const ConfigMap&() const
  {
    throw bad_cm_cast(__FILE__ " Line " STR(__LINE__ )": Expected ConfigMap, got " + type2MapStr(getType()));
  }
  virtual void setComment(const std::string &c);
  virtual const std::string& getComment() const;
};

class PlainConfigValue : public ConfigValueBase {
  std::string strn;
protected:
  virtual ConfigValue* deepCopy() const
  {
    return new PlainConfigValue(*this);
  }
public:
  PlainConfigValue();
  PlainConfigValue(const std::string &v);
  PlainConfigValue(const PlainConfigValue &other);
  ~PlainConfigValue() { /* nothing to do */ }
  inline Type getType() const { return ConfigValue::PLAIN; }
  size_t length() const;
  std::ostream& write(std::ostream &os, bool comment = true, std::string indentation = "") const;

  template<typename T> const PlainConfigValue& operator>>(T &value) const
  {
    try
    {
      std::stringstream buf(std::stringstream::in | std::stringstream::out);
      buf.exceptions(std::stringstream::failbit);
      buf << strn;
      buf >> value;
      return *this;
    }
    catch (std::stringstream::failure f)
    {
      throw std::invalid_argument("Cannot convert value to requested type.");
    }
  }
  const PlainConfigValue& operator>>(std::string &value) const;

  template<typename T> PlainConfigValue& operator<<(const T &value)
  {
    try
    {
      std::stringstream buf(std::stringstream::in | std::stringstream::out);
      buf.exceptions(std::stringstream::failbit);
      buf << value;
      strn = buf.str();
      return *this;
    }
    catch (std::stringstream::failure f)
    {
      throw std::invalid_argument("Cannot convert value to requested type.");
    }
  }
  PlainConfigValue& operator<<(const std::string &value);

  virtual operator PlainConfigValue&()
  {
    return *this;
  }
  virtual operator const PlainConfigValue&() const
  {
    return *this;
  }

  ConfigValue& operator+=(const ConfigValue &rhs);

  friend class ConfigValue;
};

class ListConfigValue : public ConfigValueBase {
  std::vector<ConfigValue*> list;
  /** Checks if the delivered index would be the new last index in the list.
   * Throws an exception if not.
   * @return An interal pointer to the newly appended value. Do not hand it to
   * any user, since they should always get references.
   */
  ConfigValue* append(size_t index, const ConfigValue &cv);
  ConfigValue* replace(size_t index, const ConfigValue &cv);
  void clear();
protected:
  virtual ConfigValue* deepCopy() const
  {
    return new ListConfigValue(*this);
  }
public:
  ListConfigValue();
  ListConfigValue(const ListConfigValue &other);
  ~ListConfigValue();
  inline Type getType() const { return ConfigValue::LIST; }
  explicit ListConfigValue(std::vector<ConfigValue*> values);
  size_t length() const;
  std::ostream& write(std::ostream &os, bool comment = true, std::string indentation = "") const;

  template<typename T> const ListConfigValue& operator>>(std::vector<T> &value) const
  {
    value.clear();
    for (std::vector<ConfigValue*>::const_iterator i = list.begin();
         i != list.end();
         ++i)
    {
      T tmp;
      **i >> tmp;
      value.push_back(tmp);
    }
    return *this;
  }

  template<typename T> ListConfigValue& operator<<(const std::vector<T> &value);

  virtual operator ListConfigValue&()
  {
    return *this;
  }

  virtual operator const ListConfigValue&() const
  {
    return *this;
  }

  ConfigValueProxy operator[](size_t index);
  ConfigValueProxy operator[](const std::string &key);
  const ConfigValue& operator[](size_t index) const;
  const ConfigValue& operator[](const std::string &key) const;
  ListConfigValue& append(const ConfigValue &cv);
  ConfigValue& operator+=(const ConfigValue &rhs);

  friend class ConfigValueProxy;
};

class ConfigMap : public ConfigValueBase {
  std::map<std::string, ConfigValue*> dict;
  std::vector<std::string> keyOrder;
  unsigned int flags;
  std::string error;
  ConfigValue* set(const std::string &key, const ConfigValue &cv);
  ConfigValue* replace(const std::string &key, const ConfigValue &cv);
protected:
  virtual ConfigValue* deepCopy() const
  {
    return new ConfigMap(*this);
  }
public:
  static const unsigned int READONLY = 0x1;

  /** Used as return values of @link read if it wants to indicate an error.
   */
  enum Error
  {
    /** Indicates that the requested filename was not found or cannot be read. */
    E_FILE   = -1,

    /** Indicates that the file does not match the syntax of a config map. */
    E_SYNTAX = -2
  };

  ConfigMap();
  ConfigMap(const ConfigMap& other);
  ConfigMap(unsigned int flags);
  ~ConfigMap();
  inline Type getType() const { return ConfigValue::MAP; }
  bool isReadOnly() const;
  size_t length() const;
  void write(const std::string *filename, bool withComment = true, std::string indentation = "") const;
  std::ostream& write(std::ostream &os, bool withComment = true, std::string indentation = "") const;
  virtual operator ConfigMap&()
  {
    return *this;
  }

  virtual operator const ConfigMap&() const
  {
    return *this;
  }

  ConfigValueProxy operator[](const std::string &key);
  const ConfigValue& operator[](const std::string &key) const;

  /** Uses a ConfigMapParser to read a ConfigMap from a file.
   * @param filename The filename from which should be read.
   *                 If is is a relative path, the first existent element of
   *                 File::getFullNames is used.
   * @param verbose  If true, report errors with OUTPUT_ERROR. default: false;
   * @param onError  The function which should be called on an error. Use it if
   *                 you want a verbose read but OUTPUT_ERROR does not suit your
   *                 needs.
   * @return  The number of added keys or a negative value if an error occures.
   *          @see ConfigMap::Error::E_FILE
   *          @see ConfigMap::Error::E_SYNTAX
   */
  int read(const std::string& filename, bool verbose = false, void (*onError)(const std::string&) = 0);
  int read(std::istream& input, bool verbose = false, void (*onError)(const std::string&) = 0);

  /** Prints the delivered error message on std::cerr.
   * @param msg The error message
   */
  static void printOnErr(const std::string &msg);

  /**
   * Returns the last error message, occured during a read.
   * Returns an empty string if no error occured.
   * @return "" or an error message
   */
  std::string getErrorMessage() const;
  std::vector<std::string> getKeys(bool deep = false) const;

  void setFlags(unsigned int flags);
  unsigned int getFlags() const;

  bool hasKey(const std::string &key, bool deep = false) const;

  ConfigValue& operator+=(const ConfigValue &rhs);

  friend class ConfigValueProxy;
};

template<typename T> ConfigValue* createConfigValueFor(const T &value)
{
  return new PlainConfigValue();
}

/**
 * @class ConfigValueProxy
 * Used by the operator[] to distinct between reads and writes.
 */
class ConfigValueProxy : public ConfigValue {
  const union
  {
    ListConfigValue *list;
    ConfigMap *map;
  };
  int index;
  std::string *key;
  ConfigValue *configValue;
  ConfigValueProxy(ListConfigValue &lcv, int index);
  ConfigValueProxy(ListConfigValue &lcv, int index, ConfigValue *configValue);
  ConfigValueProxy(ConfigMap &cm, const std::string &key);
  ConfigValueProxy(ConfigMap &cm, const std::string &key, ConfigValue *configValue);
private:
  virtual const ConfigValue* getParent() const;
  virtual void setParent(ConfigValue *cv);
  virtual ConfigValue* deepCopy() const;
public:
  ~ConfigValueProxy();
  Type getType() const;
  ConfigValue* operator&();
  ConfigValueProxy& operator=(const ConfigValue &otherValue);
  virtual bool isReadOnly() const;
  virtual operator PlainConfigValue&();
  virtual operator ListConfigValue&();
  virtual operator ConfigMap&();
  virtual operator const PlainConfigValue&() const;
  virtual operator const ListConfigValue&() const;
  virtual operator const ConfigMap&() const;
  virtual void setComment(const std::string &c);
  virtual const std::string& getComment() const;
  virtual size_t length() const;
  virtual std::ostream& write(std::ostream &os, bool withComment = true, std::string indentation = "") const;

  template<typename T> ConfigValue& operator<<(const T &value)
  {
    if (!configValue)
    {
      if (key)
        configValue = map->set(*key, *createConfigValueFor(value));
      else
        configValue = list->append(index, *createConfigValueFor(value));
    }
    return *configValue << value;
  }

  template<typename T> ConfigValue& operator<<(const std::vector<T> &value)
  {
    if (!configValue)
    {
      if (key)
        configValue = map->set(*key, ListConfigValue());
      else
        configValue = list->append(index, ListConfigValue());
    }
    return *configValue << value;
  }

  ConfigValue& operator+=(const ConfigValue &rhs);

  friend class ListConfigValue;
  friend class ConfigMap;
};

template<typename T> const ConfigValue& operator>>(const ConfigValue &cv, T &value)
{
  const PlainConfigValue &pcv = cv;
  return pcv >> value;
}

template<typename T> const ConfigValue& operator>>(const ConfigValue &cv, std::vector<T> &value)
{
  const ListConfigValue &lcv = cv;
  return lcv >> value;
}

template<typename T> ConfigValue& operator<<(ConfigValue &cv, const T &value)
{
  PlainConfigValue &pcv = cv;
  return pcv << value;
}

template<typename T> ConfigValue& operator<<(ConfigValue &cv, const std::vector<T> &value)
{
  ListConfigValue &lcv = cv;
  return lcv << value;
}

template<typename T> ListConfigValue& ListConfigValue::operator<<(const std::vector<T> &value)
{
  clear();
  list.reserve(value.size());
  for (size_t i = 0; i < value.size(); ++i)
    (*this)[i] << value[i];
  return *this;
}

std::ostream& operator<<(std::ostream &os, const ConfigValue &cv);
