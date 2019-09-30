/**
 * The file declares a class that represents the blackboard containing all
 * representations used in a thread.
 * The file will be included by all modules and therefore avoids including
 * headers by itself.
 * @author Thomas Röfer
 */

#pragma once

#include <memory>
#include <functional>

class Streamable;

/**
 * Helper class to check whether a type has an accessible serialize method.
 */
struct HasSerialize
{
  template<typename T> static auto test(T* t) -> decltype(t->serialize(nullptr, nullptr), bool()) {return true;}
  static bool test(void*) {return false;}
};

class Blackboard
{
private:
  /** A single entry of the blackboard. */
  struct Entry
  {
    std::unique_ptr<Streamable> data; /**< The representation. */
    int counter = 0; /**< How many modules requested its existence? */
    std::function<void(Streamable*)> reset;
  };

  class Entries; /**< Type of the map for all entries. */
  std::unique_ptr<Entries> entries; /**< All entries of the blackboard. */
  int version = 0; /**< A version that is increased with each configuration change. */

  /**
   * Set the blackboard instance of a thread.
   * Only Thread::setGlobals calls this method.
   * @param instance The blackboard of this thread.
   */
  static void setInstance(Blackboard& instance);
  friend class ThreadFrame; /**< A thread is allowed to set the instance. */

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
   * the instance of this thread.
   */
  Blackboard();

  /**
   * The destructor frees the blackboard and resets the instance of
   * this thread.
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
      entry.data = std::make_unique<T>();
      if(HasSerialize::test(dynamic_cast<T*>(&*entry.data)))
        entry.reset = [](Streamable* data)
      {
        dynamic_cast<T*>(data)->~T();
        new(dynamic_cast<T*>(data)) T();
      };
      else
        entry.reset = [](Streamable* data) {};
      ++version;
    }
    return dynamic_cast<T&>(*entry.data);
  }

  /**
   * Free the blackboard entry for a representation of a certain
   * name. It is only removed if it was freed as often as it was
   * allocated.
   * @param representation The name of the representation.
   */
  void free(const char* representation);

  /**
   * Reset the blackboard entry for a representation of a certain
   * name to its default state.
   * @param representation The name of the representation.
   */
  void reset(const char* representation);

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
   * Access the blackboard of this thread.
   * @return The instance that belongs to this thread.
   */
  static Blackboard& getInstance();
};
