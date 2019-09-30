/**
 * @file ModulePacket.h
 * Declaration of a class representing a packet transmitted between different threads.
 * @author Thomas Röfer
 */

#pragma once

#include "ModuleGraphRunner.h"

/**
 * @struct ModulePacket
 * A class representing a packet transmitted between different threads.
 */
struct ModulePacket
{
  ModuleGraphRunner* moduleGraphRunner = nullptr; /**< A pointer to the module graph runner. It knows the actual data to be streamed. */
  size_t index = -1; /**< The index of the thread of the packet. */
};

/**
 * The operator will use the module manager to write the required blackboard entries
 * to the stream.
 * @param stream The stream that is written to.
 * @param modulePacket The packet associated to the module graph runner.
 * @return The stream.
 */
inline Out& operator<<(Out& stream, const ModulePacket& modulePacket)
{
  modulePacket.moduleGraphRunner->writePacket(stream, modulePacket.index);
  return stream;
}

/**
 * The operator will use the module manager to read the required blackboard entries
 * from the stream.
 * @param stream The stream that is read from.
 * @param modulePacket The packet associated to the module graph runner.
 * @return The stream.
 */
inline In& operator>>(In& stream, ModulePacket& modulePacket)
{
  modulePacket.moduleGraphRunner->readPacket(stream, modulePacket.index);
  return stream;
}
