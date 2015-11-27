#include <cstring>

#ifndef WINDOWS
#include <cxxabi.h>
#endif

#include "Streamable.h"
#include "Tools/Streams/StreamHandler.h"
#include "Tools/Global.h"

#include <typeinfo>

void Streamable::streamOut(Out& out) const
{
  const_cast<Streamable*>(this)->serialize(nullptr, &out);
}

void Streamable::streamIn(In& in)
{
  serialize(&in, nullptr);
}

In& operator>>(In& in, Streamable& streamable)
{
  streamable.streamIn(in);
  return in;
}

Out& operator<<(Out& out, const Streamable& streamable)
{
  streamable.streamOut(out);
  return out;
}

namespace Streaming
{
  void finishRegistration()
  {
    Global::getStreamHandler().finishRegistration();
  }

  void startRegistration(const std::type_info& ti, bool registerWithExternalOperator)
  {
    Global::getStreamHandler().startRegistration(ti.name(), registerWithExternalOperator);
  }

  void registerBase()
  {
    Global::getStreamHandler().registerBase();
  }

  void registerWithSpecification(const char* name, const std::type_info& ti)
  {
    Global::getStreamHandler().registerWithSpecification(name, ti);
  }

  void registerEnum(const std::type_info& ti, const char * (*fp)(int))
  {
    Global::getStreamHandler().registerEnum(ti, fp);
  }

  std::string demangle(const char* name)
  {
#ifdef WINDOWS
    if(!strncmp(name, "struct ", 7))
      return name + 7;
    else if(!strncmp(name, "class ", 6))
      return name + 6;
    else
      return name;
#else
    char realName[1000]; // This should be big enough, so realloc is never called.
    int status;
    size_t length = sizeof(realName);
    abi::__cxa_demangle(name, realName, &length, &status);
    if(!status)
      return realName;
    else
      return "UNKNOWN";
#endif
  }

  Out& dummyStream()
  {
    return Global::getStreamHandler().dummyStream;
  }

  const char* skipDot(const char* name)
  {
    const char* dotPos = strchr(name, '.');
    return dotPos ? dotPos + 1 : name;
  }
}
