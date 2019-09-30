/**
 * @file MsgPack.cpp
 *
 * This file implements a method that parses a packet according to the MsgPack format.
 * Only the subset of the format is supported that is required for the communication
 * with the NAO.
 *
 * @author Thomas Röfer
 */

#include "MsgPack.h"
#include "Platform/BHAssert.h"
#include "Tools/Debugging/Debugging.h"
#include <cstring>

namespace MsgPack
{
  static bool parseMap(const unsigned char*& p, const unsigned char* pEnd,
                       const std::function<void(const std::string&, const unsigned char*)>& handleFloat,
                       const std::function<void(const std::string&, const unsigned char*)>& handleUChar,
                       const std::function<void(const std::string&, const unsigned char*, size_t size)>& handleString)
  {
    if((p < pEnd && (*p & 0xf0) == 0x80)
       || (p + 2 < pEnd && *p == 0xde))
    {
      int valuesToRead;
      if(*p == 0xde)
      {
        valuesToRead = p[1] << 8 | p[2];
        p += 3;
      }
      else
        valuesToRead = *p++ & 0x0f;

      std::string name;
      for(; valuesToRead > 0 && p < pEnd; --valuesToRead)
      {
        // read value name
        size_t charsToRead;
        if((*p & 0xe0) == 0xa0) // msgpack fixstr
          charsToRead = *p & 0x1f;
        else if(*p == 0xd9 && p + 1 < pEnd) // msgpack str 8
          charsToRead = *++p;
        else
          return false;;

        if(++p + charsToRead > pEnd)
          return false;

        name.resize(charsToRead);
        std::strncpy(&name[0], reinterpret_cast<const char*>(p), charsToRead);
        p += charsToRead;

        // read value
        if(!(*p & 0x80)) // msgpack positive fixint
          handleUChar(name, p++);
        else if((*p & 0xe0) == 0xa0 && p + (charsToRead = (*p & 0x1f)) < pEnd) // msgpack fixstr
        {
          handleString(name, ++p, charsToRead);
          p += charsToRead;
        }
        else if(*p == 0xca && p + 4 < pEnd) // msgpack float 32
        {
          handleFloat(name, ++p);
          p += 4;
        }
        else if((p < pEnd && (*p & 0xf0) == 0x90)
                || (p + 2 < pEnd && *p == 0xdc)) // msgpack fixarray or array 16
        {
          int valuesToRead;
          if(*p == 0xdc)
          {
            valuesToRead = p[1] << 8 | p[2];
            p += 3;
          }
          else
            valuesToRead = *p++ & 0x0f;
          int i = 0;
          for(; i < valuesToRead && p < pEnd; ++i)
            // read values
            if(!(*p & 0x80)) // msgpack positive fixint
              handleUChar(name + ":" + std::to_string(i), p++);
            else if((*p & 0xe0) == 0xa0 && p + (charsToRead = (*p & 0x1f)) < pEnd) // msgpack fixstr
            {
              handleString(name + ":" + std::to_string(i), ++p, charsToRead);
              p += charsToRead;
            }
            else if(*p == 0xca && p + 4 < pEnd) // msgpack float 32
            {
              handleFloat(name + ":" + std::to_string(i), ++p);
              p += 4;
            }
            else
              return false;
        }
        else if(((*p & 0xf0) != 0x80 && *p != 0xde) // msgpack fixmap or map 16
                || !parseMap(p, pEnd, handleFloat, handleUChar, handleString))
          return false;
      }
      if(valuesToRead > 0)
      {
        OUTPUT_WARNING(valuesToRead << " values were skipped");
        return false;
      }
      else
        return true;
    }
    else
      return false;
  }

  void parse(const unsigned char* packet, size_t size,
             const std::function<void(const std::string&, const unsigned char*)>& handleFloat,
             const std::function<void(const std::string&, const unsigned char*)>& handleUChar,
             const std::function<void(const std::string&, const unsigned char*, size_t size)>& handleString)
  {
    const unsigned char* p = packet;
    const unsigned char* pEnd = packet + size;
    if(!parseMap(p, pEnd, handleFloat, handleUChar, handleString))
      OUTPUT_WARNING("Could not interpret byte at offset " << static_cast<int>(p - packet));
  }

  void writeMapHeader(size_t numOfPairs, unsigned char*& p)
  {
    ASSERT(numOfPairs < 16);
    *p++ = 0x80 | static_cast<unsigned char>(numOfPairs); // msgpack map 16
  }

  void writeArrayHeader(size_t numOfValues, unsigned char*& p)
  {
    ASSERT(numOfValues < 65536);
    if(numOfValues < 16)
      *p++ = 0x90 | static_cast<unsigned char>(numOfValues); // msgpack fixarray
    else
    {
      *p++ = 0xdc; // msgpack array 16
      *p++ = static_cast<unsigned char>(numOfValues >> 8);
      *p++ = static_cast<unsigned char>(numOfValues);
    }
  }

  void write(const std::string& value, unsigned char*& p)
  {
    ASSERT(value.size() < 32);
    *p++ = static_cast<unsigned char>(0xa0 |value.size()); // msgpack fixstr
    std::memcpy(p, value.data(), value.size());
    p += value.size();
  }

  unsigned char* write(float value, unsigned char*& p)
  {
    *p++ = 0xca; // msgpack float 32
    writeFloat(value, p);
    p += 4;
    return p - 4;
  }
}
