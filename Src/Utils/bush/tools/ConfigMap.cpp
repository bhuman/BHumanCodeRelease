/**
* @file ConfigMap.cpp
*
* Implementation of the ConfigMap class hierarchy.
*
* @author <a href="mailto::ojlr@informatik.uni-bremen.de">Ole Jan Lars Riemann</a>
*/

#include <algorithm>
#include <string>
#include <iostream>
#include <typeinfo>
#include <cstdlib>
#include <cctype>
#include "Platform/File.h"
#include "ConfigMap.h"
#include "ConfigMapParser.h"

/*
 * ConfigValue
 */
std::string ConfigValue::type2MapStr(Type type)
{
  switch(type)
  {
  case PLAIN:
    return "PlainConfigValue";
  case LIST:
    return "ListConfigValue";
  case MAP:
    return "ConfigMap";
  default:
    return "UnknownValue";
  }
}

ConfigValueProxy ConfigValue::operator[](size_t index)
{
  ListConfigValue& lcv = *this;
  return lcv[index];
}

const ConfigValue& ConfigValue::operator[](size_t index) const
{
  const ListConfigValue& lcv = *this;
  return lcv[index];
}

bool startsWithNumber(const std::string& key)
{
  for(std::string::const_iterator i = key.begin();
      i != key.end();
      ++i)
  {
    if(*i == '.' && i != key.begin())
      return true;
    else if(!isdigit(*i))
      return false;
  }
  return true;
}

ConfigValueProxy ConfigValue::operator[](const std::string& key)
{
  if(startsWithNumber(key))
  {
    ListConfigValue& lcv = *this;
    return lcv[key];
  }
  else
  {
    ConfigMap& cmv = *this;
    return cmv[key];
  }
}

const ConfigValue& ConfigValue::operator[](const std::string& key) const
{
  if(startsWithNumber(key))
  {
    const ListConfigValue& lcv = *this;
    return lcv[key];
  }
  else
  {
    const ConfigMap& cmv = *this;
    return cmv[key];
  }
}

/*
 * ConfigValueBase
 */
ConfigValueBase& ConfigValueBase::operator=(const ConfigValue& other)
{
  comment = other.getComment();
  // do not modify parent
  return *this;
}

const ConfigValue* ConfigValueBase::getParent() const
{
  return parent;
}

void ConfigValueBase::setParent(ConfigValue* cv)
{
  this->parent = cv;
}

bool ConfigValueBase::isReadOnly() const
{
  const ConfigValue* p = getParent();
  return p && p->isReadOnly();
}

void ConfigValueBase::setComment(const std::string& c)
{
  this->comment = c;
}

const std::string& ConfigValueBase::getComment() const
{
  return this->comment;
}

std::string ConfigValue::str() const
{
  std::stringstream buf;
  buf << *this;
  return buf.str();
}

/*
 * PlainConfigValue
 */

PlainConfigValue::PlainConfigValue()
  : ConfigValueBase(),
    strn("")
{ }

PlainConfigValue::PlainConfigValue(const std::string& v)
  : strn(v)
{ }

PlainConfigValue::PlainConfigValue(const PlainConfigValue& other)
  : ConfigValueBase(other),
    strn(other.strn)
{ }

size_t PlainConfigValue::length() const
{
  return strn.length();
}

/**
 * Encloses a string value in "" if necessary and escapes '"' and '\'.
 */
std::string escape(const std::string& in)
{
  bool escape = in.empty();
  for(std::string::const_iterator i = in.begin();
      i != in.end();
      ++i)
  {
    if(!(isalnum(*i) || *i == '_' || *i == '+' || *i == '-' || *i == '.') || isspace(*i))
    {
      escape = true;
      break;
    }
  }

  if(escape)
  {
    std::stringstream out;
    out << '"';
    for(std::string::const_iterator i = in.begin();
        i != in.end();
        ++i)
    {
      if(*i == '"' || *i == '\\')
        out << '\\' << *i;
      else
        out << *i;
    }
    out << '"';
    return out.str();
  }
  return in;
}

std::ostream& PlainConfigValue::write(std::ostream& os, bool comment, std::string indentation) const
{
  os << escape(strn);
  return os;
}

const PlainConfigValue& PlainConfigValue::operator>>(std::string& value) const
{
  value = this->strn;
  return *this;
}

PlainConfigValue& PlainConfigValue::operator<<(const std::string& value)
{
  this->strn = value;
  return *this;
}

/*
 * ListConfigValue
 */

ListConfigValue::ListConfigValue()
  : list()
{ }

ListConfigValue::ListConfigValue(const ListConfigValue& other)
  : ConfigValueBase(other),
    list(other.list)
{
  for(size_t i = 0; i < this->list.size(); ++i)
  {
    ConfigValue* cv = this->list[i]->deepCopy();
    cv->setParent(this);
    this->list[i] = cv;
  }
}

ListConfigValue::ListConfigValue(std::vector<ConfigValue*> values)
  : list(values)
{ }

ListConfigValue::~ListConfigValue()
{
  clear();
}

void ListConfigValue::clear()
{
  for(std::vector<ConfigValue*>::iterator i = list.begin();
      i != list.end();
      ++i)
    delete *i;
  list.clear();
}

size_t ListConfigValue::length() const
{
  return this->list.size();
}

bool hasComplexValue(const std::vector<ConfigValue*> &list)
{
  for(size_t i = 0; i < list.size(); ++i)
  {
    ConfigValue* cv = list[i];
    if(cv->getType() == ConfigValue::MAP
       || (cv->getType() == ConfigValue::LIST && cv->length() > 0))
      return true;
  }
  return false;
}

std::ostream& ListConfigValue::write(std::ostream& os, bool comment, std::string indentation) const
{
  bool wrap = list.size() > 10 || hasComplexValue(list);
  std::string currIndent = indentation + "  ";
  os << "[";
  if(wrap)
    os << std::endl << currIndent;
  bool sep = false;
  for(std::vector<ConfigValue*>::const_iterator i = list.begin();
      i != list.end();
      ++i)
  {
    if(sep)
    {
      os << ",";
      if(wrap)
        os << std::endl << currIndent;
      else
        os << " ";
    }
    (*i)->write(os, comment, currIndent);
    sep = true;
  }
  if(wrap) os << std::endl << indentation;
  os << "]";
  return os;
}

ConfigValueProxy ListConfigValue::operator[](size_t index)
{
  if(index >= list.size())
  {
    char buf[20];
    sprintf(buf, "%zu", index);
    char size[20];
    sprintf(size, "%zu", list.size());
    if(isReadOnly())
      throw invalid_key("Index " + std::string(buf) + " is not set and "
                        "cannot be created since this list is read-only");
    else if(index != list.size())
      throw invalid_key("Index " + std::string(buf) + " is not set and "
                        "cannot be appended since the next index has to be "
                        + std::string(size));
    return ConfigValueProxy(*this, index);
  }
  return ConfigValueProxy(*this, index, list[index]);
}

const ConfigValue& ListConfigValue::operator[](size_t index) const
{
  if(index >= list.size())
  {
    char buf[20];
    sprintf(buf, "%zu", index);
    throw invalid_key("Index " + std::string(buf) + " is not set and "
                      "cannot be created since this list is read-only (const)");
  }
  return *list[index];
}

ConfigValueProxy ListConfigValue::operator[](const std::string& key)
{
  try
  {
    std::stringstream buf(std::stringstream::in | std::stringstream::out);
    buf.exceptions(std::stringstream::failbit);
    size_t pos = key.find_first_of('.');
    buf.str(key.substr(0, pos));
    int index;
    buf >> index;
    if(pos == std::string::npos)
      return (*this)[index];
    else
      return (*this)[index][key.substr(pos + 1)];
  }
  catch(std::stringstream::failure f)
  {
    throw invalid_key("\"" + key + "\" not found.");
  }
}

const ConfigValue& ListConfigValue::operator[](const std::string& key) const
{
  try
  {
    std::stringstream buf(std::stringstream::in | std::stringstream::out);
    buf.exceptions(std::stringstream::failbit);
    size_t pos = key.find_first_of('.');
    buf.str(key.substr(0, pos));
    int index;
    buf >> index;
    return (*this)[index];
  }
  catch(std::stringstream::failure f)
  {
    throw invalid_key("\"" + key + "\" not found.");
  }
}

ListConfigValue& ListConfigValue::append(const ConfigValue& cv)
{
  if(isReadOnly())
  {
    throw std::invalid_argument("Cannot append to readonly list.");
  }
  list.push_back(cv.deepCopy());
  return *this;
}

ConfigValue* ListConfigValue::append(size_t index, const ConfigValue& cv)
{
  if(list.size() != index) throw invalid_key("Cannot append!");
  ConfigValue* internal = cv.deepCopy();
  internal->setParent(this);
  list.push_back(internal);
  return internal;
}

ConfigValue* ListConfigValue::replace(size_t index, const ConfigValue& cv)
{
  ConfigValue* internal = list[index];
  delete internal;
  internal = cv.deepCopy();
  return internal;
}

/*
 * ConfigMap
 */
ConfigMap::ConfigMap()
  : dict(), flags(0), error("")
{ }

ConfigMap::ConfigMap(const ConfigMap& other)
  : ConfigValueBase(other),
    dict(),
    keyOrder(other.keyOrder),
    flags(other.flags),
    error(other.error)
{
  for(std::map<std::string, ConfigValue*>::const_iterator i = other.dict.begin();
      i != other.dict.end();
      ++i)
  {
    ConfigValue* cv = i->second->deepCopy();
    cv->setParent(this);
    dict[i->first] = cv;
  }
}

ConfigMap::ConfigMap(unsigned int flags)
  : dict(), flags(flags), error("")
{ }

ConfigMap::~ConfigMap()
{
  for(std::map<std::string, ConfigValue*>::const_iterator i = dict.begin();
      i != dict.end();
      ++i)
    delete i->second;
  dict.clear();
}

ConfigValue* ConfigMap::set(const std::string& key, const ConfigValue& cv)
{
  if(isReadOnly()) throw invalid_key("Cannot set `" + key + "` since the map is read-only.");
  if(!hasKey(key))
    keyOrder.push_back(key);
  ConfigValue* internal = cv.deepCopy();
  internal->setParent(this);
  dict[key] = internal;
  return internal;
}

ConfigValue* ConfigMap::replace(const std::string& key, const ConfigValue& cv)
{
  ConfigValue* internal = dict[key];
  delete internal;
  internal = cv.deepCopy();
  if(hasKey(key, false))
    keyOrder.erase(std::remove(keyOrder.begin(), keyOrder.end(), key), keyOrder.end());
  keyOrder.push_back(key);
  dict[key] = internal;
  return internal;
}

bool ConfigMap::isReadOnly() const
{
  if(flags & READONLY)
  {
    return true;
  }
  else
  {
    const ConfigValue* p = getParent();
    return p && p->isReadOnly();
  }
}

size_t ConfigMap::length() const
{
  return this->dict.size();
}

ConfigValueProxy ConfigMap::operator[](const std::string& key)
{
  if(key.size() < 1 || !isalpha(key[0]))
    throw invalid_key("\"" + key + "\" does not start with an alphabetical character which is not a valid key for a map.");
  std::string k;
  size_t pos = key.find_first_of('.');
  if(pos == std::string::npos)
    k = key;
  else
    k = key.substr(0, pos);
  std::map<std::string, ConfigValue*>::const_iterator i = dict.find(k);
  if(i == dict.end())
  {
    if(isReadOnly())
      throw invalid_key("\"" + k + "\" not found.");
    if(pos == std::string::npos)
      return ConfigValueProxy(*this, k);
    else
      return ConfigValueProxy(*this, k)[key.substr(pos + 1)];
  }
  if(pos == std::string::npos)
    return ConfigValueProxy(*this, k, i->second);
  else
    return (*i->second)[key.substr(pos + 1)];
}

const ConfigValue& ConfigMap::operator[](const std::string& key) const
{
  if(key.size() < 1 || !isalpha(key[0]))
    throw invalid_key("\"" + key + "\" does not start with an alphabetical character which is not a valid key for a map.");
  std::string k;
  size_t pos = key.find_first_of('.');
  if(pos == std::string::npos)
    k = key;
  else
    k = key.substr(0, pos);
  std::map<std::string, ConfigValue*>::const_iterator i = dict.find(k);
  if(i == dict.end())
  {
    throw invalid_key("\"" + k + "\" not found.");
  }
  if(pos == std::string::npos)
    return *i->second;
  else
  {
    const ConfigValue& cv = *i->second;
    return cv[key.substr(pos + 1)];
  }
}

int ConfigMap::read(const std::string& filename, bool verbose, void (*onError)(const std::string&))
{
  int result = E_FILE;
  std::list<std::string> names = File::getFullNames(filename);
  for(std::list<std::string>::const_iterator i = names.begin(); result == E_FILE && i != names.end(); ++i)
  {
    if(onError && error != "")
      onError("Trying " + (*i) + " instead.");
    error = "";
    ConfigMapParser parser(this, *i);
    if(verbose)
      parser.setVerbose();
    result = parser.parse(true);
    if (result < 0)
      error = parser.getError()->what();
  }
  if(result < 0 && onError)
    onError(error);
  return result;
}

int ConfigMap::read(std::istream& input, bool verbose, void (*onError)(const std::string&))
{
  int result = E_FILE;
  error = "";
  ConfigMapParser parser(this, &input);
  if(verbose)
    parser.setVerbose();

  result = parser.parse(true);
  if(result < 0 && onError)
  {
    error = parser.getError()->what();
    onError(error);
  }
  return result;
}

void ConfigMap::printOnErr(const std::string& msg)
{
  std::cerr << "Error: " << msg << std::endl;
}

std::string ConfigMap::getErrorMessage() const
{
  return error;
}

bool hasComplexValue(const std::map<std::string, ConfigValue*> &map)
{
  for(std::map<std::string, ConfigValue*>::const_iterator i = map.begin();
      i != map.end();
      ++i)
    if(i->second->getType() != ConfigValue::PLAIN)
      return true;
  return false;
}

std::ostream& ConfigMap::write(std::ostream& os, bool withComment, std::string indentation) const
{

  std::string currIndent;
  bool wrap = dict.size() > 3 || hasComplexValue(dict);
  const ConfigValue* p = getParent();
  if(p)
  {
    currIndent = indentation + "  ";
    os << "{";
    if(wrap)
      os << std::endl << currIndent;
  }
  else
    currIndent = "";
  bool sep = false;
  for(std::vector<std::string>::const_iterator key = keyOrder.begin();
      key != keyOrder.end();
      ++key)
  {
    std::map<std::string, ConfigValue*>::const_iterator i = dict.find(*key);
    const std::string& comment = i->second->getComment();
    if(withComment && comment.size() > 0)
    {
      if(sep)
        os << std::endl << currIndent;
      os << "/*" << comment << "*/" << std::endl;
    }
    else if(sep && wrap)
      os << std::endl << currIndent;
    else if(sep)
      os << " ";
    os << i->first << " = ";
    i->second->write(os, withComment, currIndent);
    os << ";";
    sep = true;
  }
  if(wrap)
    os << std::endl << indentation;
  if(p) os << "}";

  return os;
}

std::vector<std::string> ConfigMap::getKeys(bool deep) const
{
  std::vector<std::string> keys;
  keys.reserve(dict.size());
  for(std::map<std::string, ConfigValue*>::const_iterator i = dict.begin();
      i != dict.end();
      ++i)
  {
    keys.push_back(i->first);
    if(deep && i->second->getType() == ConfigValue::MAP)
    {
      const ConfigMap& cmv = *i->second;
      const std::vector<std::string> &cKeys = cmv.getKeys();
      for(std::vector<std::string>::const_iterator j = cKeys.begin();
          j != cKeys.end();
          ++j)
      {
        keys.push_back(*j);
      }
    }
  }
  return keys;
}

bool ConfigMap::hasKey(const std::string& key, bool deep) const
{
  if(!deep)
    return dict.find(key) != dict.end();
  std::vector<std::string> keys = getKeys(true);
  for(size_t i = 0; i < keys.size(); ++i)
    if(key == keys[i])
      return true;
  return false;
}

/*
 * ConfigValueProxy
 */

ConfigValueProxy::ConfigValueProxy(ListConfigValue& lcv,
                                   int index)
  : list(&lcv),
    index(index),
    key(0),
    configValue(0)
{ }

ConfigValueProxy::ConfigValueProxy(ListConfigValue& lcv,
                                   int index,
                                   ConfigValue* configValue)
  : list(&lcv),
    index(index),
    key(0),
    configValue(configValue)
{ }

ConfigValueProxy::ConfigValueProxy(ConfigMap& cm,
                                   const std::string& key)
  : map(&cm),
    index(-1),
    key(new std::string(key)),
    configValue(0)
{ }

ConfigValueProxy::ConfigValueProxy(ConfigMap& cm,
                                   const std::string& key,
                                   ConfigValue* configValue)
  : map(&cm),
    index(-1),
    key(new std::string(key)),
    configValue(configValue)
{ }

ConfigValueProxy::~ConfigValueProxy()
{
  if(key) delete key;
}

ConfigValue::Type ConfigValueProxy::getType() const
{
  if(!configValue) return static_cast<ConfigValue::Type>(-1);
  return configValue->getType();
}

ConfigValue* ConfigValueProxy::operator&()
{
  if(!configValue)
  {
    std::stringstream buf;
    if(key)
      buf << *key;
    else
      buf << index;
    throw invalid_key("`" + buf.str() + "`not found");
  }
  return configValue;
}

ConfigValueProxy& ConfigValueProxy::operator=(const ConfigValue& otherValue)
{
  if(!configValue)
  {
    if(key)
    {
      configValue = map->set(*key, otherValue);
    }
    else
    {
      configValue = list->append(index, otherValue);
    }
  }
  else
  {
    if(key)
    {
      configValue = map->replace(*key, otherValue);
    }
    else
    {
      configValue = list->replace(index, otherValue);
    }
  }
  return *this;
}

const ConfigValue* ConfigValueProxy::getParent() const
{
  if(key)
    return map;
  else
    return list;
}

void ConfigValueProxy::setParent(ConfigValue* cv)
{
  if(!configValue) throw invalid_key("Cannot access or create " + std::string(1, index));
  configValue->setParent(cv);
}

ConfigValue* ConfigValueProxy::deepCopy() const
{
  if(!configValue) throw invalid_key("Cannot access or create " + std::string(1, index));
  return configValue->deepCopy();
}

bool ConfigValueProxy::isReadOnly() const
{
  if(configValue)
    return configValue->isReadOnly();
  else
    return key ? map->isReadOnly() : list->isReadOnly();
}

ConfigValueProxy::operator PlainConfigValue& ()
{
  if(configValue)
    return *configValue;

  if(key)
    configValue = map->set(*key, PlainConfigValue(""));
  else
  {
    configValue = list->append(index, PlainConfigValue(""));
  }
  return *configValue;
}

ConfigValueProxy::operator ListConfigValue& ()
{
  if(configValue)
    return *configValue;

  if(key)
    configValue = map->set(*key, ListConfigValue());
  else
    configValue = list->append(index, ListConfigValue());
  return *configValue;
}

ConfigValueProxy::operator ConfigMap& ()
{
  if(configValue)
    return *configValue;

  if(key)
    configValue = map->set(*key, ConfigMap());
  else
    configValue = list->append(index, ConfigMap());
  return *configValue;
}

ConfigValueProxy::operator const PlainConfigValue& () const
{
  if(!configValue) throw invalid_key("Cannot access or create " + std::string(1, index));
  return *configValue;
}

ConfigValueProxy::operator const ListConfigValue& () const
{
  if(!configValue) throw invalid_key("Cannot access or create " + std::string(1, index));
  return *configValue;
}

ConfigValueProxy::operator const ConfigMap& () const
{
  if(!configValue) throw invalid_key("Cannot access or create " + std::string(1, index));
  return *configValue;
}

void ConfigValueProxy::setComment(const std::string& c)
{
  if(!configValue) throw invalid_key("Cannot access or create " + std::string(1, index));
  configValue->setComment(c);
}

const std::string& ConfigValueProxy::getComment() const
{
  if(!configValue) throw invalid_key("Cannot access or create " + std::string(1, index));
  return configValue->getComment();
}

size_t ConfigValueProxy::length() const
{
  return configValue ? configValue->length() : 0;
}

std::ostream& ConfigValueProxy::write(std::ostream& os, bool withComment, std::string indentation) const
{
  if(!configValue) throw invalid_key("Cannot access or create " + std::string(1, index));
  return configValue->write(os, withComment);
}

std::ostream& operator<<(std::ostream& os, const ConfigValue& cv)
{
  return cv.write(os);
}

void ConfigMap::setFlags(unsigned int flags)
{
  this->flags = flags;
}

unsigned int ConfigMap::getFlags() const
{
  return this->flags;
}

void ConfigMap::write(const std::string* filename, bool withComment, std::string indentation) const
{
  if(!filename)
  {
    write(std::cout);
  }
  else
  {
    std::list<std::string> names = File::getFullNames(*filename);
    bool tryMore = true;
    for(std::list<std::string>::const_iterator i = names.begin(); tryMore && i != names.end(); ++i)
    {
      File f(i->c_str(), "w");
      if(f.exists())
      {
        std::fstream fout(i->c_str(), std::fstream::out);
        write(fout, withComment);
        fout.close();
        tryMore = false;
      }
    }
  }
}

ConfigValue& ConfigValueProxy::operator+=(const ConfigValue &rhs)
{
  if(!configValue) throw invalid_key("Cannot access or create " + std::string(1, index));
  return (*configValue) += rhs;
}

ConfigValue& ListConfigValue::operator+=(const ConfigValue &rhs)
{
  const ListConfigValue &rhsList = rhs;
  if (list.size() != rhsList.list.size())
  {
    // replace entire list
    clear();
    for (size_t i = 0; i < rhsList.list.size(); ++i)
    {
      append(i, *rhsList.list[i]);
    }
  }
  else
  {
    for (size_t i = 0; i < list.size(); ++i)
    {
      ConfigValue *left = list[i];
      const ConfigValue *right = rhsList.list[i];
      if (left->getType() != right->getType())
      {
        (*this)[i] = rhsList[i];
      }
      else
      {
        *left += *right;
      }
    }
  }
  return *this;
}

ConfigValue& PlainConfigValue::operator+=(const ConfigValue &rhs)
{
  const PlainConfigValue &rhsPlain = rhs;
  strn = rhsPlain.strn;
  return *this;
}

ConfigValue& ConfigMap::operator+=(const ConfigValue &rhs)
{
  const ConfigMap &rhsMap = rhs;
  if (rhsMap.length() == 0)
  {
    for (std::map<std::string, ConfigValue*>::iterator i = dict.begin();
         i != dict.end();
         ++i)
    {
      delete i->second;
    }
    dict.clear();
    keyOrder.clear();
  }
  else
  {
    for (std::map<std::string, ConfigValue*>::const_iterator i = rhsMap.dict.begin();
         i != rhsMap.dict.end();
         ++i)
    {
      ConfigValue *left = 0;
      if (hasKey(i->first))
      {
        left = dict.find(i->first)->second;
      }
      const ConfigValue *right = i->second;
      if (left == 0 || left->getType() != right->getType())
      {
        (*this)[i->first] = *i->second;
      }
      else
      {
        *left += *right;
      }
    }
  }
  return *this;
}
