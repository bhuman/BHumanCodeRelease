
#pragma once

#include <vector>
#include <stack>
#include <typeinfo>
#include <unordered_map>
#include "Tools/Streams/OutStreams.h"

class StreamHandler;

In& operator>>(In& in, StreamHandler& streamHandler);
Out& operator<<(Out& out, const StreamHandler& streamHandler);

class ConsoleRoboCupCtrl;
class RobotConsole;
class DebugDataStreamer;
class Framework;
class Settings;

/**
* singleton stream handler class
*/
class StreamHandler
{
private:
  /**
   * Default constructor.
   * No other instance of this class is allowed except the one accessible via getStreamHandler
   * therefore the constructor is private.
   */
  StreamHandler();

  /**
   * Copy constructor.
   * Copying instances of this class is not allowed
   * therefore the copy constructor is private. */
  StreamHandler(const StreamHandler&) {}

  /*
  * only a process is allowed to create the instance.
  */
  friend class Process;

  struct RegisteringAttributes
  {
    short baseClass;
    bool registering;
    bool externalOperator;
  };

  typedef std::unordered_map<const char*, const char*> BasicTypeSpecification;
  BasicTypeSpecification basicTypeSpecification;

  typedef std::pair< std::string, const char*> TypeNamePair;
  typedef std::unordered_map<const char*, std::vector<TypeNamePair> > Specification;
  Specification specification;

  typedef std::unordered_map<const char*, std::vector<const char*> > EnumSpecification;
  EnumSpecification enumSpecification;

  typedef std::unordered_map<std::string, int> StringTable;
  StringTable stringTable;

  typedef std::pair<Specification::iterator, RegisteringAttributes> RegisteringEntry;
  typedef std::stack<RegisteringEntry> RegisteringEntryStack;
  RegisteringEntryStack registeringEntryStack;

  bool registering;
  bool registeringBase;

  const char* getString(const std::string& string);

public:
  void clear();
  void startRegistration(const char* name, bool registerWithExternalOperator);
  void registerBase() {registeringBase = true;}
  void finishRegistration();
  void registerWithSpecification(const char* name, const std::type_info& ti);
  void registerEnum(const std::type_info& ti, const char* (*fp)(int));
  OutBinarySize dummyStream;

private:
  friend In& operator>>(In&, StreamHandler&);
  friend Out& operator<<(Out&, const StreamHandler&);
  friend class ConsoleRoboCupCtrl; // constructs a StreamHandler used when outside process contexts.
  friend class RobotConsole; // constructs a StreamHandler storing the information received.
  friend class TeamComm3DCtrl; // constructs a StreamHandler used by all serialize methods.
  friend class Framework;
  friend class Settings; // construct a default StreamHandler when they are first loaded.
  friend class DebugDataStreamer; // needs access to internal data types.
};
