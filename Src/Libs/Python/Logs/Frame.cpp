/**
 * @file Frame.cpp
 *
 * This file implements the class representing a frame in a log.
 *
 * @author Arne Hasselbring
 * @author Jan Fiedler
 */

#include "Frame.h"
#include "Log.h"
#include "Debugging/DebugDataStreamer.h"
#include "Streaming/MessageIDs.h"
#include <pybind11/pybind11.h>
#include <cstring>
#include <stdexcept>

Frame::Frame(Log& log) :
  log(log),
  it(log.begin())
{}

bool Frame::readFrame()
{
  representations.clear();
  annotations.clear();

  if(it == log.end())
    return false;

  if(log.id(*it) != idFrameBegin)
    throw std::runtime_error("Frame does not begin with idFrameBegin.");

  (*it).bin() >> thread;

  for(++it; it != log.end(); ++it)
  {
    const MessageQueue::Message message = *it;
    const MessageID id = log.id(message);
    if(id == idFrameFinished)
    {
      std::string thread2;
      message.bin() >> thread2;
      ++it;
      if(thread != thread2)
        throw std::runtime_error("Frame does not end with matching idFrameFinished.");
      return true;
    }
    else if(id == idFrameBegin)
    {
      if(log.keepGoing)
      {
        pybind11::module_::import("logging").attr("getLogger")("pybh").attr("warning")("Frame %s does not end with idFrameFinished, but was asked to keep going.", frameNumber);
        return true;
      }
      throw std::runtime_error("Frame does not end with idFrameFinished.");
    }
    else if(id == idAnnotation)
    {
      auto stream = message.bin();
      unsigned unused;
      std::string name;
      stream >> unused;
      if(!(unused & 0x80000000u))
        stream >> unused;
      const size_t size = stream.getSize() - stream.getPosition();
      std::string text;
      text.resize(size);
      stream.read(text.data(), size);
      InTextMemory textStream(text.data(), size);
      textStream >> name;
      annotations.emplace_back(name, textStream.readAll());
    }
    else if(id == idStopwatch)
    {
      // TODO: implement
    }
    else
    {
      // ASSERT(message.id() < log.messageIDNames.size());
      const std::string representation = (*log.messageIDNames)[message.id()].substr(2);
      representations.emplace(representation, message);
    }
  }

  return false;
}

Frame& Frame::next()
{
  ++frameNumber;
  if(!readFrame())
    throw pybind11::stop_iteration();
  return *this;
}

Record Frame::getitem(const std::string& representation)
{
  auto it = representations.find(representation);
  if(it == representations.end())
    throw pybind11::key_error("Frame has no representation '" + representation + "'");

  Record value;
  auto in = it->second.bin();
  TypeStream out(value);
  DebugDataStreamer streamer(log.typeInfo, in, representation, nullptr);
  out << streamer;
  if(!in.eof())
  {
    // TODO: only the parts after the current pointer
    std::vector<char> data(it->second.size());
    it->second.bin().read(data.data(), it->second.size());
    value.attributes["_data"] = new Literal(data);
  }
  return value;
}
