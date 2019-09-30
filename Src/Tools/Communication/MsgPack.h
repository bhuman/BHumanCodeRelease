/**
 * @file MsgPack.h
 *
 * This file declares a method that parses a packet according to the MsgPack format
 * (https://msgpack.org). Only the subset of the format is supported that is required
 *  for the communication with the NAO.
 *
 * @author Thomas Röfer
 */

#pragma once

#include <cstddef>
#include <functional>
#include <string>

namespace MsgPack
{
  /**
   * Parse a packet according the MsgPack format. The packet is expected to contain
   * a map (format "map 16") of name and value pairs. For names, only the formats
   * "fixstr" and "str 8" are supported. For values, only the formats
   * "positive fixint", "fixstr", and "float 32" are supported.
   * @param packet The packet to parse.
   * @param size The length of the packet in bytes.
   * @param handleFloat This function is called for each float value parsed. The
   *                    first parameter is the name of the map entry, the second
   *                    points to the matching big endian float value in the
   *                    packet, i.e. the value still has to be converted to
   *                    little endian byte order before it can be interpreted as
   *                    a float.
   * @param handleUChar This function is called for each positive fixint value
   *                    parsed.
   * @param handleString This function is called for each string value parsed.
   */
  void parse(const unsigned char* packet, size_t size,
             const std::function<void(const std::string&, const unsigned char*)>& handleFloat,
             const std::function<void(const std::string&, const unsigned char*)>& handleUChar,
             const std::function<void(const std::string&, const unsigned char*, size_t size)>& handleString);

  /**
   * Write a map header to memory. Note that only the format "fixmap" is supported.
   * @param numOfPairs The number of key/value pairs that will follow.
   * @param p The address that is written to. The variable will point behind the
   *          data written afterwards.
   */
  void writeMapHeader(size_t numOfPairs, unsigned char*& p);

  /**
   * Write an array header to memory. Note that only the formats "fixarray" and
   * "array 16" are supported.
   * @param numOfValues The number of values that will follow.
   * @param p The address that is written to. The variable will point behind the
   *          data written afterwards.
   */
  void writeArrayHeader(size_t numOfValues, unsigned char*& p);

  /**
   * Write a string.
   * @param value The string with a maximum length of 31 bytes.
   * @param p The address that is written to. The variable will point behind the
   *          data written afterwards.
   */
  void write(const std::string& key, unsigned char*& p);

  /**
   * Write a float.
   * @param value The float value.
   * @param p The address that is written to. The variable will point behind the
   *          data written afterwards.
   * @return The address where the first byte of the value was written to. This
   *         address is a suitable input for the function writeFloat to overwrite
   *         the float again.
   */
  unsigned char* write(float value, unsigned char*& p);

  /**
   * Compute a float from a big endian float stored at an address in memory.
   * @param p The address of the big endian encoded float in memory.
   * @return The float value. If "p" was nullptr, 0 is returned.
   */
  inline float readFloat(const unsigned char* p)
  {
    if(p)
    {
      const unsigned buffer = p[0] << 24 | p[1] << 16 | p[2] << 8 | p[3];
      return reinterpret_cast<const float&>(buffer);
    }
    else
      return 0.f;
  }

  /**
   * Write a float to memory in big endian format.
   * @param value The float to write.
   * @param p The address the float is written to. Must point to at least
   *          4 bytes of writable memory.
   */
  inline void writeFloat(float value, unsigned char* p)
  {
    *p++ = static_cast<unsigned char>(reinterpret_cast<unsigned&>(value) >> 24);
    *p++ = static_cast<unsigned char>(reinterpret_cast<unsigned&>(value) >> 16);
    *p++ = static_cast<unsigned char>(reinterpret_cast<unsigned&>(value) >> 8);
    *p++ = static_cast<unsigned char>(reinterpret_cast<unsigned&>(value));
  }
}
