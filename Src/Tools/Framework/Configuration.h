/**
 * @file Tools/Framework/Configuration.h
 *
 * This file declares a list of threads with their parameters and moduls.
 *
 * @author Jan Fiedler
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

/**
 * The class is used for reading configurations.
 * It contains all threads with their parameters and moduls.
 */
STREAMABLE(Configuration,
{
  STREAMABLE(RepresentationProvider,
  {
    RepresentationProvider() = default;
    RepresentationProvider(const std::string& representation, const std::string& provider) :
      representation(representation)COMMA provider(provider) {};

    /**
     * Comparison operator. Uses the representation name for comparison.
     * @param other The other RepresentationProvider this one is compared to.
     * @return Is this representations name lower than the other?
     */
    bool operator<(const RepresentationProvider& other) const
    {
      return representation < other.representation;
    },

    (std::string) representation,
    (std::string) provider,
  });

  STREAMABLE(Thread,
  {
    /**
     * Comparison operator. Uses the namoe for comparison with strings.
     * @param other The name this one is compared to.
     * @return Is this thread name like the string.
     */
    bool operator==(const std::string& other) const
    {
      return name == other;
    },

    (std::string) name,
    (int)(0) priority,
    (unsigned)(0) debugReceiverSize, /**< The maximum size of the queue in Bytes. */
    (unsigned)(0) debugSenderSize, /**< The maximum size of the queue in Bytes. */
    (unsigned)(0) debugSenderInfrastructureSize,
    (std::string) executionUnit,
    (std::vector<RepresentationProvider>) representationProviders,
  });

  /**
   * Avoid writing config.threads all the time.
   * @return The only member of configuration.
   */
  std::vector<Thread>& operator()() { return threads; };
  const std::vector<Thread>& operator()() const { return threads; },

  (std::vector<std::string>) defaultRepresentations,
  (std::vector<Thread>) threads, /**< Should be accessed via operator(). */
});
