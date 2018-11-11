/**
 * The file declares a class that makes the debug data in a stream streamable
 * according to the Streamable interface.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Streams/Streamable.h"

struct TypeInfo;

/**
 * A class that makes the debug data in a stream streamable according to the
 * Streamable interface. There are two different constructors. One creates
 * an object for reading from a debug data stream, the other one for writing
 * to it. Since the object represents the stream that it encapsulates, it
 * must be used with matching streaming operators, i.e. >> if it encapsulates
 * a writing stream and << for a reading stream.
 */
class DebugDataStreamer : public Streamable
{
private:
  const TypeInfo& typeInfo; /**< The type information for the data streamed. */
  In* inData = nullptr; /**< The debug data stream to read from. 0 if we are writing. */
  Out* outData = nullptr; /**< The debug data stream to write to. 0 if we are reading. */
  std::string type; /**< The string representation of the type of the next data streamed. */
  const char* name; /**< The name of the next entry streamed. 0 if it does not have a name, because it is an array element. */
  int index = -2; /**< The index of the next element streamed or -2 if we are currently not streaming an array. */

  /**
   * The method streams the debug data according to its specification.
   * @param in The stream from which the object is read. Must be 0 if this object was constructed for a reading stream.
   * @param out The stream to which the object is written. Must be 0 if this object was constructed for a writing stream.
   */
  void serialize(In* in, Out* out) override;

  /**
   * The method streams an entry.
   * @param T The type of the entry to be streamed.
   * @param in The stream from which the object is read. Must be 0 if this object was constructed for a reading stream.
   * @param out The stream to which the object is written. Must be 0 if this object was constructed for a writing stream.
   * @param enumType The type as string if it is an enum. Otherwise nullptr.
   */
  template<typename T> void streamIt(In* in, Out* out, const char* enumType = nullptr);

public:
  /**
   * Constructs an object that reads from a debug data stream.
   * Note that this object can only be used together with the << operator.
   * @param typeInfo The type information for the data streamed.
   * @param stream The debug data stream to read from.
   * @param type The string representation of the type of the data streamed.
   * @param name The name of the data streamed. 0 if it does not have a name.
   */
  DebugDataStreamer(const TypeInfo& typeInfo, In& stream, const std::string& type, const char* name = nullptr);

  /**
   * Constructs an object that writes to a debug data stream.
   * Note that this object can only be used together with the >> operator.
   * @param typeInfo The type information for the data streamed.
   * @param stream The debug data stream to write to.
   * @param type The string representation of the type of the data streamed.
   * @param name The name of the data streamed. 0 if it does not have a name.
   */
  DebugDataStreamer(const TypeInfo& typeInfo, Out& stream, const std::string& type, const char* name = nullptr);
};

template<typename T> void DebugDataStreamer::streamIt(In* in, Out* out, const char* enumType)
{
  T t = T();
  if(in)
  {
    in->select(name, index, enumType);
    *in >> t;
    *outData << t;
    in->deselect();
  }
  else
  {
    out->select(name, index, enumType);
    *inData >> t;
    *out << t;
    out->deselect();
  }
}
