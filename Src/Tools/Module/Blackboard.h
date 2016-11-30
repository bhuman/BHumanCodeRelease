/**
 * The file declares a class that represents the blackboard containing all
 * representations used in a process.
 * The file will be included by all modules and therefore avoids including
 * headers by itself.
 * @author Thomas Röfer
 */

#pragma once

class Streamable;

class Blackboard
{
private:
  /** A single entry of the blackboard. */
  struct Entry
  {
    Streamable* data = nullptr; /**< The representation. */
    int counter = 0; /**< How many modules requested its existance? */
  };

  class Entries; /**< Type of the map for all entries. */
  Entries& entries; /**< All entries of the blackboard. */
  int version = 0; /**< A version that is increased with each configuration change. */

  /**
   * Set the blackboard instance of a process.
   * Only Process::setGlobals calls this method.
   * @param instance The blackboard of this process.
   */
  static void setInstance(Blackboard& instance);
  friend class Process;

  /**
   * Retrieve the blackboard entry for the name of a representation.
   * @param representation The name of the representation.
   * @return The blackboard entry. If it does not exist, it will
   * be created, but not the representation.
   */
  Entry& get(const char* representation);
  const Entry& get(const char* representation) const;

public:
  /**
   * The default constructor creates the blackboard and sets it as
   * the instance of this process.
   */
  Blackboard();

  /**
   * The destructor frees the blackboard and resets the instance of
   * this process.
   */
  ~Blackboard();

  /**
   * Does a certain representation exist?
   * @param representation The name of the representation.
   * @return Does it exist in this blackboard?
   */
  bool exists(const char* representation) const;

  /**
   * Allocate a new blackboard entry for a representation of a
   * certain type and name. The representation is only created
   * if this is its first allocation.
   * @param T The type of the representation.
   * @param representation The name of the representation.
   * @return The representation.
   */
  template<typename T> T& alloc(const char* representation)
  {
    Entry& entry = get(representation);
    if(entry.counter++ == 0)
    {
      entry.data = new T;
      ++version;
    }
    return *dynamic_cast<T*>(entry.data);
  }

  /**
   * Free the blackboard entry for a representation of a certain
   * name. It is only removed if it was freed as often as it was
   * allocated.
   * @param representation The name of the representation.
   */
  void free(const char* representation);

  /**
   * Access a representation of a certain name. The representation
   * must already exist.
   * @param representation The name of the representation.
   * @return The instance of the representation in the blackboard.
   */
  Streamable& operator[](const char* representation);
  const Streamable& operator[](const char* representation) const;

  /**
   * Return the current version.
   * It can be used to determine whether the configuration of the
   * blackboard changed.
   * @return The current version. It starts with 0 and might
   *         be increased in larger steps.
   */
  int getVersion() const {return version;}

  /**
   * Access the blackboard of this process.
   * @return The instance that belongs to this process.
   */
  static Blackboard& getInstance();
};
