/**
 * @file Logs.cpp
 *
 * This file implements some Python bindings to go through logs.
 *
 * @author Arne Hasselbring
 * @author Jan Fiedler
 */

#include "Debugging/DebugDataStreamer.h"
#include "Framework/LoggingTools.h"
#include "Math/Angle.h"
#include "Platform/File.h"
#include "Platform/SystemCall.h"
#include "Streaming/FunctionList.h"
#include "Streaming/MessageQueue.h"
#include "Streaming/MessageIDs.h"
#include "Streaming/InStreams.h"
#include "Streaming/TypeInfo.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <snappy-c.h>
#include <algorithm>
#include <memory>
#include <stack>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

namespace py = pybind11;

using LiteralVariant = std::variant<bool, long, unsigned long, double, std::string, std::vector<char>>;

class Value;

// For typings instead of `py::object`.
using PyTypeVariant = std::variant<py::bool_, py::int_, py::float_, py::str, py::bytes, Value*>;

/** A base class for all value types for usage in the same STL-containers. */
class Value
{
public:
  virtual ~Value() = default;

  PyTypeVariant toVariant();
};
/** Allow vector modifications to be passed between C++ and Python. Maybe useless here? */
PYBIND11_MAKE_OPAQUE(std::vector<Value*>)

/** Type conversion class for basic types. */
class Literal : public Value, public LiteralVariant
{
public:
  using LiteralVariant::variant;

  operator PyTypeVariant()
  {
    if(std::holds_alternative<bool>(*this))
      return py::bool_(std::get<bool>(*this));
    else if(std::holds_alternative<long>(*this))
      return py::int_(std::get<long>(*this));
    else if(std::holds_alternative<unsigned long>(*this))
      return py::int_(std::get<unsigned long>(*this));
    else if(std::holds_alternative<double>(*this))
      return py::float_(std::get<double>(*this));
    else if(std::holds_alternative<std::string>(*this))
      return py::str(std::get<std::string>(*this));
    else
    {
      auto& bytes = std::get<std::vector<char>>(*this);
      return py::bytes(bytes.data(), bytes.size());
    }
  }
};

class Array : public Value, public std::vector<Value*>
{
public:
  using std::vector<Value*>::vector;

  ~Array()
  {
    for(auto* value : *this)
      delete value;
  }
};

/** A collection of named attributes (i.e. a streamable class). */
class Record : public Value
{
public:
  Record() = default;

  Record(Record&& other) :
    attributes(std::move(other.attributes))
  {}

  ~Record()
  {
    for(const auto& pair : attributes)
      delete pair.second;
  }

  /** Returns the attribute with the given name. */
  PyTypeVariant getattr(const std::string& name)
  {
    auto it = attributes.find(name);
    if(it == attributes.end())
      throw py::attribute_error("Record has no attribute '" + name + "'");
    return it->second->toVariant();
  }

  /** Returns all attribute names of this Record. */
  const std::vector<std::string>& getKeys()
  {
    if(keys.empty())
    {
      keys.reserve(attributes.size());
      for(const auto& pair : attributes)
        keys.push_back(pair.first);
    }
    return keys;
  }

  std::unordered_map<std::string, Value*> attributes;
  std::vector<std::string> keys;
};

PyTypeVariant Value::toVariant()
{
  if(auto* literal = dynamic_cast<Literal*>(this); literal)
    return literal->operator PyTypeVariant();
  else
    return this;
}

/** An event annotation of a frame. */
class Annotation
{
public:
  Annotation(const std::string& name, const std::string& description) :
    name(name), description(description)
  {}

  std::string name;
  std::string description;
};

class TypeStream : public Out
{
public:
  TypeStream(Value& value) :
    root(&value)
  {}

private:
  template<typename T>
  void out(const T& value)
  {
    Entry& entry = stack.top();
    if(!entry.value)
      return;
    (*entry.value) = new Literal(value);
  }

  void outBool(bool value) override { out(value); }

  void outChar(char value) override { out(static_cast<long>(value)); }

  void outSChar(signed char value) override { out(static_cast<long>(value)); }

  void outUChar(unsigned char value) override { out(static_cast<unsigned long>(value)); }

  void outShort(short value) override { out(static_cast<long>(value)); }

  void outUShort(unsigned short value) override { out(static_cast<unsigned long>(value)); }

  void outInt(int value) override { out(static_cast<long>(value)); }

  void outUInt(unsigned int value) override
  {
    Entry& entry = stack.top();
    if(!entry.value)
      return;
    if(entry.type == -1)
      (*entry.value) = new Array(value);
    else
      (*entry.value) = new Literal(static_cast<unsigned long>(value));
  }

  void outFloat(float value) override { out(static_cast<double>(value)); }

  void outDouble(double value) override { out(value); }

  void outString(const char* value) override { out(std::string(value)); }

  void outAngle(const Angle& value) override { out(static_cast<double>(value)); }

  void outEndL() override {}

  void write(const void*, std::size_t) override {}

  void select(const char* name, int type, const char* = nullptr) override
  {
    Streaming::trimName(name);
    Value** value = stack.empty() ? &root : stack.top().value;
    if(!value)
      stack.emplace(type, nullptr);
    else if(type >= 0)
    {
      // if the DebugDataStreamer didn't always write the size of the array to the stream, the following would be necessary
      /*
      if(!*value)
        *value = new Array;
      */
      auto* array = dynamic_cast<Array*>(*value);
      if(array)
        stack.emplace(type, &((*array)[type]));
      else
        stack.emplace(type, nullptr);
    }
    else
    {
      if(!*value)
        *value = new Record;
      auto* record = dynamic_cast<Record*>(*value);
      if(record)
        stack.emplace(type, &record->attributes[name]);
      else
        stack.emplace(type, nullptr);
    }
  }

  void deselect() override
  {
    stack.pop();
  }

  struct Entry final
  {
    Entry(int type, Value** value) :
      type(type), value(value)
    {}
    int type;
    Value** value;
  };

  Value* root;
  std::stack<Entry, std::vector<Entry>> stack;
};

class Log;

/** Represents a frame in a log. */
class Frame
{
public:
  Frame(Log& log);

  /** Returns all representation names of this Frame. */
  std::vector<std::string> getRepresentations() const
  {
    std::vector<std::string> result;
    result.reserve(representations.size());
    for(const auto& pair : representations)
      result.push_back(pair.first);
    return result;
  }

  /** Returns all annotations of this Frame. */
  std::vector<Annotation> getAnnotations() const
  {
    return annotations;
  }

  /** Returns the representation with the given name. */
  Record getitem(const std::string& representation);

  bool contains(const std::string& representation)
  {
    return representations.find(representation) != representations.end();
  }

  Frame& next();
  Frame& iter() { return *this; }

  bool readFrame();

  std::string thread;

private:
  Log& log;
  int lastMessage = -1;
  int frameNumber = -1;
  std::vector<Annotation> annotations;
  std::unordered_map<std::string, std::pair<const void*, std::size_t>> representations;
};

class Log : public MessageQueue
{
public:
  Log(const std::string& path, bool keepGoing = false) :
    typeInfo(false),
    keepGoing(keepGoing)
  {
    setSize(0xfffffffffull);
    InBinaryFile stream(File::isAbsolute(path.c_str()) ? path : "./" + path);
    if(!stream.exists())
      throw std::runtime_error("Could not open log.");

    LoggingTools::parseName(path.substr(path.find_last_of("/\\") + 1), &prefix, &headName, &bodyName, &scenario, &location, &identifier, &playerNumber, &suffix);

    LoggingTools::LogFileFormat magicByte;
    stream >> magicByte;
    if(magicByte == LoggingTools::logFileSettings)
    {
      LoggingTools::skipSettings(stream);
      stream >> magicByte;
    }
    if(magicByte != LoggingTools::logFileMessageIDs)
      throw std::runtime_error("Log does not contain message ID section.");
    readMessageIDMapping(stream);
    stream >> magicByte;
    if(magicByte != LoggingTools::logFileTypeInfo)
      throw std::runtime_error("Log does not contain type info section.");
    stream >> typeInfo;
    stream >> magicByte;

    auto messageIDEnum = typeInfo.enums.find("MessageID");
    if(messageIDEnum == typeInfo.enums.end())
      throw std::runtime_error("Log type info does not contain MessageID.");
    messageIDNames = &messageIDEnum->second;

    // TODO: do not read everything into memory at once
    switch(magicByte)
    {
      case LoggingTools::logFileUncompressed:
        append(stream, stream.getFile()->getSize() - stream.getFile()->getPosition());
        break;
      case LoggingTools::logFileCompressed:
        while(!stream.eof())
        {
          unsigned compressedSize;
          stream >> compressedSize;
          if(!compressedSize)
            throw std::runtime_error("Corrupt compressed log file.");
          std::vector<char> compressedBuffer(compressedSize);
          stream.read(compressedBuffer.data(), compressedSize);

          std::size_t uncompressedSize = 0;
          snappy_uncompressed_length(compressedBuffer.data(), compressedSize, &uncompressedSize);
          std::vector<char> uncompressedBuffer(uncompressedSize);
          if(snappy_uncompress(compressedBuffer.data(), compressedSize, uncompressedBuffer.data(), &uncompressedSize) != SNAPPY_OK)
            break;
          InBinaryMemory mem(uncompressedBuffer.data(), uncompressedSize);
          mem >> *this;
        }
        break;
      default:
        throw std::runtime_error("Unknown magic byte!");
    }

    // Calc numberOfFrames
    numberOfFrames = 0;
    for(int i = 0; i < getNumberOfMessages(); ++i)
    {
      queue.setSelectedMessageForReading(i);

      if(queue.getMessageID() == idFrameBegin)
        ++numberOfFrames;
    }
  }

  Frame iter()
  {
    return Frame(*this);
  }

  std::string prefix;
  std::string headName;
  std::string bodyName;
  std::string scenario;
  std::string location;
  std::string identifier;
  int playerNumber = -1;
  std::string suffix;
  int numberOfFrames = 0;

private:
  friend class Frame;
  TypeInfo typeInfo;
  bool keepGoing = false;
  const std::vector<std::string>* messageIDNames = nullptr;
};

Frame::Frame(Log& log) :
  log(log)
{}

bool Frame::readFrame()
{
  representations.clear();
  annotations.clear();

  if(lastMessage >= log.getNumberOfMessages())
    return false;

  log.queue.setSelectedMessageForReading(lastMessage);
  if(log.queue.getMessageID() != idFrameBegin)
    throw std::runtime_error("Frame does not begin with idFrameBegin.");
  thread = log.in.readThreadIdentifier();

  while(true)
  {
    ++lastMessage;
    if(lastMessage >= log.getNumberOfMessages())
      return false;
    log.queue.setSelectedMessageForReading(lastMessage);
    if(log.queue.getMessageID() == idFrameFinished)
      break;
    else if(log.queue.getMessageID() == idFrameBegin)
    {
      if(log.keepGoing)
      {
        py::module_::import("logging").attr("getLogger")("pybh").attr("warning")("Frame %s does not end with idFrameFinished, but was asked to keep going.", frameNumber);
        --lastMessage;
        return true;
      }
      throw std::runtime_error("Frame does not end with idFrameFinished.");
    }
    else if(log.queue.getMessageID() == idAnnotation)
    {
      unsigned unused;
      std::string name;
      log.in.text.reset();
      log.in.bin >> unused;
      if(!(unused & 0x80000000u))
        log.in.bin >> unused;
      log.in.text >> name;
      annotations.emplace_back(name, log.in.text.readAll());
    }
    else if(log.queue.getMessageID() == idStopwatch)
    {
      // TODO: implement
    }
    else
    {
      // ASSERT(log.queue.getMessageID(true) < log.messageIDNames.size());
      const std::string representation = (*log.messageIDNames)[log.queue.getMessageID(true)].substr(2);
      representations[representation] = {log.queue.getData(), log.queue.getMessageSize()};
    }
  }

  if(log.in.readThreadIdentifier() != thread)
    throw std::runtime_error("Frame does not end with matching idFrameFinished.");

  return true;
}

Frame& Frame::next()
{
  ++lastMessage;
  ++frameNumber;
  if(!readFrame())
    throw py::stop_iteration();
  return *this;
}

Record Frame::getitem(const std::string& representation)
{
  auto it = representations.find(representation);
  if(it == representations.end())
    throw py::key_error("Frame has no representation '" + representation + "'");

  Record value;
  InBinaryMemory in(it->second.first, it->second.second);
  TypeStream out(value);
  DebugDataStreamer streamer(log.typeInfo, in, representation, nullptr);
  out << streamer;
  if(!in.eof())
  {
    // TODO: only the parts after the current pointer
    std::vector<char> data(it->second.second);
    std::memcpy(data.data(), it->second.first, it->second.second);
    value.attributes["_data"] = new Literal(data);
  }
  return value;
}

/** Wrap LoggingTools::parseName. */
py::tuple parseLogName(const std::string& path)
{
  std::string prefix;
  std::string headName;
  std::string bodyName;
  std::string scenario;
  std::string location;
  std::string identifier;
  int playerNumber = -1;
  std::string suffix;

  LoggingTools::parseName(path.substr(path.find_last_of("/\\") + 1), &prefix, &headName, &bodyName, &scenario, &location, &identifier, &playerNumber, &suffix);
  return py::make_tuple(prefix, headName, bodyName, scenario, location, identifier, playerNumber, suffix);
}

SystemCall::Mode SystemCall::getMode()
{
  return logFileReplay;
}


// Required to extract version info.
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

struct ArrayAccessor
{
  PyTypeVariant operator()(const std::vector<Value*>::iterator& it)
  {
    return (*it)->toVariant();
  }
};

PYBIND11_MODULE(logs, m)
{
#ifdef VERSION_INFO
  m.attr("__version__") = TOSTRING(VERSION_INFO);
#else
  m.attr("__version__") = "dev";
#endif

  m.attr("__name__") = "pybh.logs";
  m.doc() = R"bhdoc(
B-Human Logs Wrapper

Python bindings to go through B-Human log files.

.. literalinclude:: ../../Tests/logs.py
   :lines: 4-

Walk through representations:

.. literalinclude:: ../../Tests/logscrawler.py
   :lines: 9-25
)bhdoc";

  FunctionList::execute();

  m.def("parse_log_name", &parseLogName, R"bhdoc(Parses a log file name into its components.

Args:
    logfileName: The file name to parse.

Returns:
    prefix, headName, bodyName, scenario, location, identifier, playerNumber, suffix of the log.
)bhdoc", py::arg("path"));

  py::class_<Log>(m, "Log")
    .def(py::init<const std::string&, bool>(), R"bhdoc(This class represents an log File.

The entire log is always loaded into the RAM.

Args:
    path: The file path.
    keep_going: Whether corrupt frames should be ignored as much as possible.
)bhdoc", py::arg("path"), py::arg("keep_going") = false)
    .def_readonly("prefix", &Log::prefix, "The prefix of the log.")
    .def_readonly("headName", &Log::headName, "The head name of the log.")
    .def_readonly("bodyName", &Log::bodyName, "The body name of the log.")
    .def_readonly("scenario", &Log::scenario, "The scenario of the log.")
    .def_readonly("location", &Log::location, "The location of the log.")
    .def_readonly("identifier", &Log::identifier, "The identifier of the log.")
    .def_readonly("playerNumber", &Log::playerNumber, "The player number of the log.")
    .def_readonly("suffix", &Log::suffix, "The suffix of the log.")
    .def("__len__", [](const Log& log) { return log.numberOfFrames; })
    // The log is alive as long as a reference to a frame exists.
    .def("__iter__", &Log::iter, py::keep_alive<0, 1>()); // loop
  py::class_<Frame>(m, "Frame", "Represents the data of a single frame in the log.")
    .def_readonly("thread", &Frame::thread, "The thread of this frame.")
    .def_property_readonly("representations", &Frame::getRepresentations, "A list of names of the representations this frame has.")
    .def_property_readonly("annotations", &Frame::getAnnotations, "A list of annotations this frame has.")
    .def("__next__", &Frame::next) // loop
    .def("__iter__", &Frame::iter) // loop
    .def("__contains__", &Frame::contains, py::arg("x")) // 'x' in frame
    .def("__getitem__", &Frame::getitem, py::arg("index")); // frame['x']
  py::class_<Value>(m, "Value", "A base class for all value types for usage in the same STL-containers.");
  py::class_<Literal, Value>(m, "Literal", R"bhdoc(Represents a literal.

Is mostly extracted automatically by the lib.
)bhdoc")
    .def_property_readonly("literal", &Literal::operator PyTypeVariant, "Get the real literal.");
  //py::bind_vector<Array>(m, "Array");  // Cannot be used because of __getitem__.
  py::class_<Array, Value>(m, "Array", "A :class:`list` of type :class:`.Value` .")
    .def("__len__", [](const Array& array) {
      return array.size();
    })
    .def("__getitem__", [](Array& array, std::size_t index) -> PyTypeVariant {
      return array.at(index)->toVariant();
    }, py::return_value_policy::reference_internal, py::arg("index"))
    .def("__iter__", [](Array& array) {
      return py::detail::make_iterator_impl<ArrayAccessor, py::return_value_policy::reference_internal, decltype(array.begin()), decltype(array.end()), PyTypeVariant>(array.begin(), array.end());
    }, py::keep_alive<0, 1>());
  py::class_<Record, Value>(m, "Record", "A collection of named attributes (i.e. a streamable class).")
    .def("__getattr__", &Record::getattr, py::return_value_policy::reference_internal, py::arg("attr")) // record.x
    .def("__iter__", [](Record& record) {
      const auto& keys = record.getKeys();
      return py::make_iterator(keys.begin(), keys.end());
    }, py::keep_alive<0, 1>());
  py::class_<Annotation>(m, "Annotation", "An event annotation of a frame.")
    .def_readonly("name", &Annotation::name, "A short name of the annotation creator.")
    .def_readonly("description", &Annotation::description, "A description of the event.");
}
