/**
 * The file declares a class that makes the debug data in a stream streamable
 * according to the Streamable interface.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Platform/Thread.h"
#include "Tools/Streams/Streamable.h"

class StreamHandler;

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
  StreamHandler& streamHandler; /**< The stream handler that provides the specification of the data streamed. */
  In* inData = nullptr; /**< The debug data stream to read from. 0 if we are writing. */
  Out* outData = nullptr; /**< The debug data stream to write to. 0 if we are reading. */
  std::string type; /**< The string representation of the type of the next data streamed. */
  const char* name; /**< The name of the next entry streamed. 0 if it does not have a name, because it is an array element. */
  int index = -2; /**< The index of the next element streamed or -2 if we are currently not streaming an array. */
  static thread_local const std::vector<const char*>* enumNames; /**< Helper to provide element names for the enum currently streamed. */

  /**
   * The method streams the debug data according to its specification.
   * @param in The stream from which the object is read. Must be 0 if this object was constructed for a reading stream.
   * @param out The stream to which the object is written. Must be 0 if this object was constructed for a writing stream.
   */
  virtual void serialize(In* in, Out* out);

  /**
   * Helper to provide element names for the enum currently streamed.
   * @param value The index of the enum element.
   * @return The name of the element or 0 if value is outside the enum's range.
   */
  static const char* getName(int value);

  /**
   * The method streams an entry.
   * @param T The type of the entry to be streamed.
   * @param in The stream from which the object is read. Must be 0 if this object was constructed for a reading stream.
   * @param out The stream to which the object is written. Must be 0 if this object was constructed for a writing stream.
   * @param enumToString A function that can provide element names if we are stream an enum value. Otherwise 0.
   */
  template<typename T> void streamIt(In* in, Out* out, const char* (*enumToString)(int) = 0);

public:
  /**
   * Constructs an object that reads from a debug data stream.
   * Note that this object can only be used together with the << operator.
   * @param streamHandler The stream handler that provides the specification of the data streamed.
   * @param stream The debug data stream to read from.
   * @param type The string representation of the type of the data streamed.
   * @param name The name of the data streamed. 0 if it does not have a name.
   */
  DebugDataStreamer(StreamHandler& streamHandler, In& stream, const std::string& type, const char* name = 0);

  /**
   * Constructs an object that writes to a debug data stream.
   * Note that this object can only be used together with the >> operator.
   * @param streamHandler The stream handler that provides the specification of the data streamed.
   * @param stream The debug data stream to write to.
   * @param type The string representation of the type of the data streamed.
   * @param name The name of the data streamed. 0 if it does not have a name.
   */
  DebugDataStreamer(StreamHandler& streamHandler, Out& stream, const std::string& type, const char* name = 0);
};

template<typename T> void DebugDataStreamer::streamIt(In* in, Out* out, const char* (*enumToString)(int))
{
  T t = T();
  if(in)
  {
    in->select(name, index, enumToString);
    *in >> t;
    *outData << t;
    in->deselect();
  }
  else
  {
    out->select(name, index, enumToString);
    *inData >> t;
    *out << t;
    out->deselect();
  }
}
