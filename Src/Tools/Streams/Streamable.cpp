#include "Streamable.h"
#include <cstring>

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
  const char* skipDot(const char* name)
  {
    const char* dotPos = strchr(name, '.');
    return dotPos ? dotPos + 1 : name;
  }

  In& streamStaticArray(In& in, unsigned char inArray[], size_t size, const char* enumType) {return streamBasicStaticArray(in, inArray, size, enumType);}
  Out& streamStaticArray(Out& out, unsigned char outArray[], size_t size, const char* enumType) {return streamBasicStaticArray(out, outArray, size, enumType);}
  In& streamStaticArray(In& in, signed char inArray[], size_t size, const char* enumType) {return streamBasicStaticArray(in, inArray, size, enumType);}
  Out& streamStaticArray(Out& out, signed char outArray[], size_t size, const char* enumType) {return streamBasicStaticArray(out, outArray, size, enumType);}
  In& streamStaticArray(In& in, char inArray[], size_t size, const char* enumType) {return streamBasicStaticArray(in, inArray, size, enumType);}
  Out& streamStaticArray(Out& out, char outArray[], size_t size, const char* enumType) {return streamBasicStaticArray(out, outArray, size, enumType);}
  In& streamStaticArray(In& in, unsigned short inArray[], size_t size, const char* enumType) {return streamBasicStaticArray(in, inArray, size, enumType);}
  Out& streamStaticArray(Out& out, unsigned short outArray[], size_t size, const char* enumType) {return streamBasicStaticArray(out, outArray, size, enumType);}
  In& streamStaticArray(In& in, short inArray[], size_t size, const char* enumType) {return streamBasicStaticArray(in, inArray, size, enumType);}
  Out& streamStaticArray(Out& out, short outArray[], size_t size, const char* enumType) {return streamBasicStaticArray(out, outArray, size, enumType);}
  In& streamStaticArray(In& in, unsigned int inArray[], size_t size, const char* enumType) {return streamBasicStaticArray(in, inArray, size, enumType);}
  Out& streamStaticArray(Out& out, unsigned int outArray[], size_t size, const char* enumType) {return streamBasicStaticArray(out, outArray, size, enumType);}
  In& streamStaticArray(In& in, int inArray[], size_t size, const char* enumType) {return streamBasicStaticArray(in, inArray, size, enumType);}
  Out& streamStaticArray(Out& out, int outArray[], size_t size, const char* enumType) {return streamBasicStaticArray(out, outArray, size, enumType);}
  In& streamStaticArray(In& in, float inArray[], size_t size, const char* enumType) {return streamBasicStaticArray(in, inArray, size, enumType);}
  Out& streamStaticArray(Out& out, float outArray[], size_t size, const char* enumType) {return streamBasicStaticArray(out, outArray, size, enumType);}
  In& streamStaticArray(In& in, double inArray[], size_t size, const char* enumType) {return streamBasicStaticArray(in, inArray, size, enumType);}
  Out& streamStaticArray(Out& out, double outArray[], size_t size, const char* enumType) {return streamBasicStaticArray(out, outArray, size, enumType);}
}
