/**
 * @file ModulePackage.h
 * Declaration of a class representing a package transmitted between different processes.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "ModuleManager.h"

/**
 * @struct ModulePackage
 * A class representing a package transmitted between different processes.
 */
struct ModulePackage
{
  ModuleManager* moduleManager = nullptr; /**< A pointer to the module manager. It knows the actual data to be streamed. */
  unsigned timeStamp = 0; /**< The time stamp of the package. */
};

/**
 * The operator will use the module manager to write the required blackboard entries
 * to the stream.
 * @param stream The stream that is written to.
 * @param modulePackage The package associated to the module manager.
 * @return The stream.
 */
inline Out& operator<<(Out& stream, const ModulePackage& modulePackage)
{
  stream << modulePackage.timeStamp;
  modulePackage.moduleManager->writePackage(stream);
  return stream;
}

/**
 * The operator will use the module manager to read the required blackboard entries
 * from the stream.
 * @param stream The stream that is read from.
 * @param modulePackage The package associated to the module manager.
 * @return The stream.
 */
inline In& operator>>(In& stream, ModulePackage& modulePackage)
{
  stream >> modulePackage.timeStamp;
  modulePackage.moduleManager->readPackage(stream);
  return stream;
}

class CognitionToMotion : public ModulePackage {};
class MotionToCognition : public ModulePackage {};
