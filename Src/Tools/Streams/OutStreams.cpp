/**
 * @file OutStreams.cpp
 *
 * Implementation of out stream classes.
 *
 * @author Thomas Röfer
 * @author Martin Lötzsch
 */

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <regex>

#include "OutStreams.h"
#include "Platform/File.h"
#include "Platform/BHAssert.h"
#include "Tools/Motion/SensorData.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Math/Angle.h"
#include "Tools/Streams/TypeRegistry.h"

void OutBinary::writeString(const char* d, PhysicalOutStream& stream)
{
  size_t size = strlen(d);
  stream.writeToStream(&size, sizeof(unsigned));
  stream.writeToStream(d, size);
}

void OutBinary::writeAngle(const Angle& d, PhysicalOutStream& stream)
{
  writeFloat(d, stream);
}

void OutText::writeBool(bool value, PhysicalOutStream& stream)
{
  if(value)
    stream.writeToStream(" true", 5);
  else
    stream.writeToStream(" false", 6);
}

void OutText::writeString(const char* value, PhysicalOutStream& stream)
{
  stream.writeToStream(" ", 1);
  bool containsSpaces = !*value || *value == '"' || strcspn(value, " \n\r\t") < strlen(value);
  if(containsSpaces)
    stream.writeToStream("\"", 1);
  for(; *value; ++value)
    if(*value == '"' && containsSpaces)
      stream.writeToStream("\\\"", 2);
    else if(*value == '\n')
      stream.writeToStream("\\n", 2);
    else if(*value == '\r')
      stream.writeToStream("\\r", 2);
    else if(*value == '\t')
      stream.writeToStream("\\t", 2);
    else if(*value == '\\')
      stream.writeToStream("\\\\", 2);
    else
      stream.writeToStream(value, 1);
  if(containsSpaces)
    stream.writeToStream("\"", 1);
}

void OutText::writeData(const void* p, size_t size, PhysicalOutStream& stream)
{
  for(size_t i = 0; i < size; ++i)
    writeChar(*reinterpret_cast<const char*&>(p)++, stream);
}

void OutText::writeChar(char d, PhysicalOutStream& stream)
{
  sprintf(buf, " %d", int(d));
  stream.writeToStream(buf, strlen(buf));
}

void OutText::writeSChar(signed char d, PhysicalOutStream& stream)
{
  sprintf(buf, " %u", int(d));
  stream.writeToStream(buf, strlen(buf));
}

void OutText::writeUChar(unsigned char d, PhysicalOutStream& stream)
{
  sprintf(buf, " %u", int(d));
  stream.writeToStream(buf, strlen(buf));
}

void OutText::writeShort(short d, PhysicalOutStream& stream)
{
  sprintf(buf, " %d", int(d));
  stream.writeToStream(buf, strlen(buf));
}

void OutText::writeUShort(unsigned short d, PhysicalOutStream& stream)
{
  sprintf(buf, " %u", int(d));
  stream.writeToStream(buf, strlen(buf));
}

void OutText::writeInt(int d, PhysicalOutStream& stream)
{
  sprintf(buf, " %d", d);
  stream.writeToStream(buf, strlen(buf));
}

void OutText::writeUInt(unsigned int d, PhysicalOutStream& stream)
{
  sprintf(buf, " %u", d);
  stream.writeToStream(buf, strlen(buf));
}

void OutText::writeFloat(float d, PhysicalOutStream& stream)
{
  sprintf(buf, " %g", double(d));
  stream.writeToStream(buf, strlen(buf));
}

void OutText::writeDouble(double d, PhysicalOutStream& stream)
{
  sprintf(buf, " %g", d);
  stream.writeToStream(buf, strlen(buf));
}

void OutText::writeAngle(const Angle& d, PhysicalOutStream& stream)
{
  sprintf(buf, " %gdeg", d.toDegrees());
  stream.writeToStream(buf, strlen(buf));
}

void OutText::writeEndL(PhysicalOutStream& stream)
{
  sprintf(buf, "\n");
  stream.writeToStream(buf, strlen(buf));
}

void OutTextRaw::writeBool(bool value, PhysicalOutStream& stream)
{
  if(value)
    stream.writeToStream("true", 4);
  else
    stream.writeToStream("false", 5);
}

void OutTextRaw::writeString(const char* value, PhysicalOutStream& stream)
{
  stream.writeToStream(value, strlen(value));
}

void OutTextRaw::writeData(const void* p, size_t size, PhysicalOutStream& stream)
{
  for(size_t i = 0; i < size; ++i)
    writeChar(*reinterpret_cast<const char*&>(p)++, stream);
}

void OutTextRaw::writeChar(char d, PhysicalOutStream& stream)
{
  sprintf(buf, "%d", int(d));
  stream.writeToStream(buf, strlen(buf));
}

void OutTextRaw::writeSChar(signed char d, PhysicalOutStream& stream)
{
  sprintf(buf, "%u", int(d));
  stream.writeToStream(buf, strlen(buf));
}

void OutTextRaw::writeUChar(unsigned char d, PhysicalOutStream& stream)
{
  sprintf(buf, "%u", int(d));
  stream.writeToStream(buf, strlen(buf));
}

void OutTextRaw::writeShort(short d, PhysicalOutStream& stream)
{
  sprintf(buf, "%d", int(d));
  stream.writeToStream(buf, strlen(buf));
}

void OutTextRaw::writeUShort(unsigned short d, PhysicalOutStream& stream)
{
  sprintf(buf, "%u", int(d));
  stream.writeToStream(buf, strlen(buf));
}

void OutTextRaw::writeInt(int d, PhysicalOutStream& stream)
{
  sprintf(buf, "%d", d);
  stream.writeToStream(buf, strlen(buf));
}

void OutTextRaw::writeUInt(unsigned int d, PhysicalOutStream& stream)
{
  sprintf(buf, "%u", d);
  stream.writeToStream(buf, strlen(buf));
}

void OutTextRaw::writeFloat(float d, PhysicalOutStream& stream)
{
  sprintf(buf, "%g", double(d));
  stream.writeToStream(buf, strlen(buf));
}

void OutTextRaw::writeDouble(double d, PhysicalOutStream& stream)
{
  sprintf(buf, "%g", d);
  stream.writeToStream(buf, strlen(buf));
}

void OutTextRaw::writeAngle(const Angle& d, PhysicalOutStream& stream)
{
  if(d == SensorData::off)
    sprintf(buf, "%g", static_cast<float>(d));
  else
    sprintf(buf, "%gdeg", d.toDegrees());
  stream.writeToStream(buf, strlen(buf));
}

void OutTextRaw::writeEndL(PhysicalOutStream& stream)
{
  sprintf(buf, "\n");
  stream.writeToStream(buf, strlen(buf));
}

OutFile::~OutFile()
{
  if(stream != nullptr)
    delete stream;
}

OutFile& OutFile::operator=(OutFile&& other)
{
  if(stream)
    delete stream;
  stream = other.stream;
  other.stream = nullptr;
  return *this;
}

bool OutFile::exists() const
{
  return stream != nullptr && stream->exists();
}

void OutFile::open(const std::string& name)
{
  stream = new File(name, "wb", false);
}

void OutFile::open(const std::string& name, bool append)
{
  stream = append ? new File(name, "ab", false) : new File(name, "wb", false);
}

void OutFile::writeToStream(const void* p, size_t size)
{
  if(stream != nullptr)
    stream->write(p, size);
}

std::string OutFile::getFullName() const
{
  ASSERT(stream);
  return stream->getFullName();
}

OutMemory::OutMemory(OutMemory&& other)
: buffer(other.buffer),
  reserved(other.reserved),
  bytes(other.bytes),
  dynamic(other.dynamic)
{
  other.buffer = nullptr;
  other.reserved = 0;
  other.bytes = 0;
  other.dynamic = true;
}

OutMemory& OutMemory::operator=(OutMemory&& other)
{
  if(dynamic && buffer)
    std::free(buffer);
  buffer = other.buffer;
  reserved = other.reserved;
  bytes = other.bytes;
  dynamic = other.dynamic;
  other.buffer = nullptr;
  other.reserved = 0;
  other.bytes = 0;
  other.dynamic = true;
  return *this;
}

void OutMemory::writeToStream(const void* p, size_t size)
{
  if(bytes + size > reserved)
  {
    if(dynamic)
    {
      reserved = std::max(std::min(reserved, std::numeric_limits<size_t>::max() / 2) * 2, bytes + size);
      buffer = reinterpret_cast<char*>(std::realloc(buffer, reserved));
    }
    else
    {
      reserved = bytes;
      return;
    }
  }
  std::memcpy(buffer + bytes, p, size);
  bytes += size;
}

char* OutMemory::obtainData()
{
  char* data = buffer;
  buffer = nullptr;
  bytes = reserved = 0;
  dynamic = true;
  return data;
}

void OutMemoryForText::addTerminatingZero()
{
  if(!OutMemory::size() || OutMemory::data()[OutMemory::size() - 1])
  {
    char c = 0;
    writeToStream(&c, sizeof(c));
  }
}

OutMap::OutMap(Out& stream, Mode mode, size_t maxCollapsedLength) :
  stream(&stream), target(stream), mode(mode), maxCollapsedLength(maxCollapsedLength)
{}

void OutMap::writeLn()
{
  if(mode == singleLine)
    *stream << " ";
  else
    *stream << endl;
}

void OutMap::flush(bool singleLine)
{
  if(stream == &buffer)
  {
    char* data = buffer.obtainData();
    if(singleLine)
    {
      static std::regex inBetweenSpace("(.)\n *([^ }\\]])");
      static std::regex otherSpace("\n *");
      std::string temp = std::regex_replace(data, inBetweenSpace, "$1 $2");
      temp = std::regex_replace(temp, otherSpace, "");
      if(temp.size() <= maxCollapsedLength)
        target << temp.data();
      else
        target << data;
    }
    else
      target << data;
    std::free(data);
    stream = &target;
  }
}

void OutMap::outUChar(unsigned char value)
{
  Entry& e = stack.back();
  if(e.enumType)
  {
    const char* constant = TypeRegistry::getEnumName(e.enumType, value);
    *stream << (constant ? constant : "UNKNOWN");
  }
  else
    *stream << static_cast<unsigned>(value);
}

void OutMap::outUInt(unsigned int value)
{
  if(stack.back().type != -1)
    *stream << value;
}

void OutMap::outString(const char* value)
{
  char buf[2] = {0};
  bool containsSpecialChars = !*value || *value == '"' || strcspn(value, " \n\r\t=,;]}") < strlen(value);
  if(containsSpecialChars)
    *stream << "\"";
  for(; *value; ++value)
    if(*value == '"' && containsSpecialChars)
      *stream << "\\\"";
    else if(*value == '\n')
      *stream << "\\n";
    else if(*value == '\r')
      *stream << "\\r";
    else if(*value == '\t')
      *stream << "\\t";
    else if(*value == '\\')
      *stream << "\\\\";
    else
    {
      buf[0] = *value;
      *stream << buf;
    }
  if(containsSpecialChars)
    *stream << "\"";
}

void OutMap::select(const char* name, int type, const char* enumType)
{
  Streaming::trimName(name);
  if(!stack.empty())
  {
    ASSERT(name || type >= 0);
    Entry& e = stack.back();
    if(!e.hasSubEntries)
    {
      if(e.type == -1) // array
      {
        *stream << "[";
        if(mode == singleLineInnermost)
        {
          flush();
          stream = &buffer;
        }
        writeLn();
      }
      else // other attribute or array element
      {
        *stream << "{";
        if(mode == singleLineInnermost)
        {
          flush();
          stream = &buffer;
        }
        writeLn();
      }
      if(mode != singleLine)
        indentation += "  ";
      e.hasSubEntries = true;
    }
  }

  if(type < 0) // attribute
  {
    *stream << indentation;
    if(name)
      *stream << name << " = ";
  }
  else if(type == 0) // first array element
    *stream << indentation;
  else // further array elements
  {
    *stream << ",";
    writeLn();
    *stream << indentation;
  }

  stack.push_back(Entry(type, enumType));
}

void OutMap::deselect()
{
  Entry& e = stack.back();
  if(e.hasSubEntries)
  {
    if(mode != singleLine)
      indentation = indentation.substr(2);
    if(e.type == -1) // array
    {
      writeLn();
      *stream << indentation << "]";
    }
    else // other attribute or array element
      *stream << indentation << "}";
    if(mode == singleLineInnermost)
      flush(true);
  }
  else if(e.type == -1) // empty array
    *stream << "[]";

  if(e.type < 0) // attribute
  {
    *stream << ";";
    writeLn();
  }
  stack.pop_back();
}

void OutMap::write(const void* p, size_t size)
{
  FAIL("Unsupported operation.");
}

OutMapFile::OutMapFile(const std::string& name, bool singleLineInnermost, size_t maxCollapsedLength)
  : OutMap(stream, singleLineInnermost ? OutMap::singleLineInnermost : multiLine, maxCollapsedLength), stream(name) {}

OutMapMemory::OutMapMemory(bool singleLine, size_t capacity, char* buffer)
  : OutMap(stream, singleLine ? OutMap::singleLine : multiLine), stream(capacity, buffer) {}
