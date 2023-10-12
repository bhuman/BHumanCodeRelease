/**
 * @file Frame.h
 *
 * This file declares the class representing a frame in a log.
 *
 * @author Arne Hasselbring
 * @author Jan Fiedler
 */

#pragma once

#include "Types.h"
#include "Streaming/MessageQueue.h"
#include <cstddef>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

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

  bool contains(const std::string& representation) const
  {
    return representations.find(representation) != representations.end();
  }

  Frame& next();
  Frame& iter() { return *this; }

  bool readFrame();

  std::string thread;

private:
  Log& log;
  MessageQueue::const_iterator it;
  int frameNumber = -1;
  std::vector<Annotation> annotations;
  std::unordered_map<std::string, MessageQueue::Message> representations;
};
