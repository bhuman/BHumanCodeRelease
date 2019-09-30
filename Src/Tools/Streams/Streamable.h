/**
 * @file Tools/Streams/Streamable.h
 *
 * Base class for all types streamed through STREAM macros.
 *
 * @author Michael Spranger
 * @author Tobias Oberlies
 * @author Thomas Röfer
 */

#pragma once

#include <array>
#include <list>
#include <vector>
#include "InOut.h"
#include "TypeRegistry.h"
#include "Tools/AlignedMemory.h"

/** Register the class that is specified as parameter. */
#define REG_CLASS(...) \
  const char* _type = typeid(__VA_ARGS__).name(); \
  const __VA_ARGS__* _class = nullptr; \
  static_cast<void>(_class); \
  TypeRegistry::addClass(_type)

/**
 * Register the class with a base class.
 * @param class The class that is registered. It is not allowed to contain commas (use COMMA instead).
 * @param ... The base class. It can contain commas.
 */
#define REG_CLASS_WITH_BASE(class, ...) \
  const char* _type = typeid(class).name(); \
  const class* _class = nullptr; \
  static_cast<void>(_class); \
  TypeRegistry::addClass(_type, typeid(__VA_ARGS__).name())

/** Concatenate the two parameters. */
#define _REG_JOIN(a, b) _REG_JOIN_I(a, b)
#define _REG_JOIN_I(a, b) _REG_JOIN_II(a ## b)
#define _REG_JOIN_II(res) res

/** Determine the number of entries in a tuple. */
#define _REG_TUPLE_SIZE(...) _REG_TUPLE_SIZE_I((__VA_ARGS__, _REG_TUPLE_SIZE_III))
#define _REG_TUPLE_SIZE_I(params) _REG_TUPLE_SIZE_II params
#define _REG_TUPLE_SIZE_II(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, ...) a10
#define _REG_TUPLE_SIZE_III 10, 9, 8, 7, 6, 5, 4, 3, 2, 1

/**
 * Register an attribute.
 * Optionally, the attribute can be preceded by a type.
 */
#define REG(...) _REG_I(_REG_TUPLE_SIZE(__VA_ARGS__), __VA_ARGS__)
#define _REG_I(n, ...) _REG_II(n, (__VA_ARGS__))
#define _REG_II(n, args) _REG_II_##n args
#define _REG_II_1(attribute) TypeRegistry::addAttribute(_type, typeid(decltype(_class->attribute)).name(), #attribute)
#define _REG_II_2(a0, a1) _REG_III(a1, a0)
#define _REG_II_3(a0, a1, a2) _REG_III(a2, a0, a1)
#define _REG_II_4(a0, a1, a2, a3) _REG_III(a3, a0, a1, a2)
#define _REG_II_5(a0, a1, a2, a3, a4) _REG_III(a4, a0, a1, a2, a3)
#define _REG_II_6(a0, a1, a2, a3, a4, a5) _REG_III(a5, a0, a1, a2, a3, a4)
#define _REG_II_7(a0, a1, a2, a3, a4, a5, a6) _REG_III(a6, a0, a1, a2, a3, a4, a5)
#define _REG_II_8(a0, a1, a2, a3, a4, a5, a6, a7) _REG_III(a7, a0, a1, a2, a3, a4, a5, a6)
#define _REG_II_9(a0, a1, a2, a3, a4, a5, a6, a7, a8) _REG_III(a8, a0, a1, a2, a3, a4, a5, a6, a7)
#define _REG_II_10(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9) _REG_III(a9, a0, a1, a2, a3, a4, a5, a6, a7, a8)
#define _REG_III(attribute, ...) TypeRegistry::addAttribute(_type, typeid(__VA_ARGS__).name(), #attribute)

#define REG_ENUM(...) \
  const char* _type = typeid(__VA_ARGS__).name(); \
  TypeRegistry::addEnum(_type)

#define REG_CONSTANT(constant) TypeRegistry::addEnumConstant(_type, #constant)

#define STREAM_BASE(s) \
  this->s::serialize(in, out);

#define STREAM_BASE_EXT(stream, s, type) \
  type& _base(const_cast<type&>(static_cast<const type&>(s))); \
  Out* _out = dynamic_cast<Out*>(&stream); \
  In* _in = dynamic_cast<In*>(&stream); \
  if(_out) \
    *_out << _base; \
  else \
    *_in >> _base;

/**
 * Registration and streaming of a member.
 * @param The attribute to be registered and streamed.
 */
#define STREAM(s) Streaming::streamIt(in, out, #s, s);

/**
 * Registration and streaming of a member in an external streaming operator
 * (<< or >>).
 * @param stream Reference to the stream, that should be streamed to.
 * @param s The attribute to be registered and streamed.
 */
#define STREAM_EXT(stream, s) Streaming::streamIt(stream, #s, s);

/**
 * Base class for all classes using the STREAM macros for streaming instances.
 */
class Streamable : public AlignedMemory
{
protected:
  virtual void serialize(In*, Out*) = 0;

public:
  virtual ~Streamable() = default;
  void streamOut(Out&) const;
  void streamIn(In&);
};

In& operator>>(In& in, Streamable& streamable);

Out& operator<<(Out& out, const Streamable& streamable);

// Helpers

namespace Streaming
{
  template<typename T>
  In& streamComplexStaticArray(In& in, T inArray[], size_t size, const char* enumType)
  {
    int numberOfEntries = int(size / sizeof(T));
    for(int i = 0; i < numberOfEntries; ++i)
    {
      in.select(0, i, enumType);
      in >> inArray[i];
      in.deselect();
    }
    return in;
  }

  template<typename T>
  Out& streamComplexStaticArray(Out& out, T outArray[], size_t size, const char* enumType)
  {
    int numberOfEntries = int(size / sizeof(T));
    for(int i = 0; i < numberOfEntries; ++i)
    {
      out.select(0, i, enumType);
      out << outArray[i];
      out.deselect();
    }
    return out;
  }

  template<typename T>
  In& streamBasicStaticArray(In& in, T inArray[], size_t size, const char* enumType)
  {
    if(in.isBinary())
    {
      in.read(inArray, size);
      return in;
    }
    else
      return streamComplexStaticArray(in, inArray, size, enumType);
  }

  template<typename T>
  Out& streamBasicStaticArray(Out& out, T outArray[], size_t size, const char* enumType)
  {
    if(out.isBinary())
    {
      out.write(outArray, size);
      return out;
    }
    else
      return streamComplexStaticArray(out, outArray, size, enumType);
  }

  In& streamStaticArray(In& in, unsigned char inArray[], size_t size, const char* enumType);
  Out& streamStaticArray(Out& out, unsigned char outArray[], size_t size, const char* enumType);
  In& streamStaticArray(In& in, signed char inArray[], size_t size, const char* enumType);
  Out& streamStaticArray(Out& out, signed char outArray[], size_t size, const char* enumType);
  In& streamStaticArray(In& in, char inArray[], size_t size, const char* enumType);
  Out& streamStaticArray(Out& out, char outArray[], size_t size, const char* enumType);
  In& streamStaticArray(In& in, unsigned short inArray[], size_t size, const char* enumType);
  Out& streamStaticArray(Out& out, unsigned short outArray[], size_t size, const char* enumType);
  In& streamStaticArray(In& in, short inArray[], size_t size, const char* enumType);
  Out& streamStaticArray(Out& out, short outArray[], size_t size, const char* enumType);
  In& streamStaticArray(In& in, unsigned int inArray[], size_t size, const char* enumType);
  Out& streamStaticArray(Out& out, unsigned int outArray[], size_t size, const char* enumType);
  In& streamStaticArray(In& in, int inArray[], size_t size, const char* enumType);
  Out& streamStaticArray(Out& out, int outArray[], size_t size, const char* enumType);
  In& streamStaticArray(In& in, float inArray[], size_t size, const char* enumType);
  Out& streamStaticArray(Out& out, float outArray[], size_t size, const char* enumType);
  In& streamStaticArray(In& in, double inArray[], size_t size, const char* enumType);
  Out& streamStaticArray(Out& out, double outArray[], size_t size, const char* enumType) ;
  template<typename T>
  In& streamStaticArray(In& in, T inArray[], size_t size, const char* enumType) {return streamComplexStaticArray(in, inArray, size, enumType);}
  template<typename T>
  Out& streamStaticArray(Out& out, T outArray[], size_t size, const char* enumType) {return streamComplexStaticArray(out, outArray, size, enumType);}

  template<typename T, typename U> void cast(T& t, const U& u) {t = static_cast<T>(u);}

  const char* skipDot(const char* name);

  template<typename S> struct Streamer
  {
    static void stream(In* in, Out* out, const char* name, S& s)
    {
      const char* enumType = std::is_enum<S>::value ? typeid(S).name() : nullptr;
      if(in)
      {
        in->select(name, -2, enumType);
        *in >> s;
        in->deselect();
      }
      else
      {
        out->select(name, -2, enumType);
        *out << s;
        out->deselect();
      }
    }
  };

  template<typename E, size_t N> struct Streamer<E[N]>
  {
    using S = E[N];
    static void stream(In* in, Out* out, const char* name, S& s)
    {
      const char* enumType = std::is_enum<E>::value ? typeid(E).name() : nullptr;
      if(in)
      {
        in->select(name, -1);
        streamStaticArray(*in, s, sizeof(s), enumType);
        in->deselect();
      }
      else
      {
        out->select(name, -1);
        streamStaticArray(*out, s, sizeof(s), enumType);
        out->deselect();
      }
    }
  };

  // This one is used by MSC
  template<typename E, size_t N> struct Streamer<const E[N]>
  {
    using S = const E[N];
    static void stream(In* in, Out* out, const char* name, S& s)
    {
      const char* enumType = std::is_enum<E>::value ? typeid(E).name() : nullptr;
      out->select(name, -1);
      streamStaticArray(*out, reinterpret_cast<E*>(s), N * sizeof(E), enumType);
      out->deselect();
    }
  };

  // This one is used by clang
  template<typename E, size_t N> struct Streamer<E(*)[N]>
  {
    using S = E (*)[N];
    static void stream(In* in, Out* out, const char* name, S& s)
    {
      const char* enumType = std::is_enum<E>::value ? typeid(E).name() : nullptr;
      if(in)
      {
        in->select(name, -1);
        streamStaticArray(*in, reinterpret_cast<E*>(s), N * sizeof(E), enumType);
        in->deselect();
      }
      else
      {
        out->select(name, -1);
        streamStaticArray(*out, reinterpret_cast<E*>(s), N * sizeof(E), enumType);
        out->deselect();
      }
    }
  };

  template<typename E, typename A> struct Streamer<std::vector<E, A>>
  {
    using S = std::vector<E, A>;
    static void stream(In* in, Out* out, const char* name, S& s)
    {
      const char* enumType = std::is_enum<E>::value ? typeid(E).name() : nullptr;
      if(in)
      {
        in->select(name, -1);
        unsigned size;
        *in >> size;
        s.resize(size);
        if(!s.empty())
          streamStaticArray(*in, &s[0], s.size() * sizeof(E), enumType);
        in->deselect();
      }
      else
      {
        out->select(name, -1);
        *out << static_cast<unsigned>(s.size());
        if(!s.empty())
          streamStaticArray(*out, &s[0], s.size() * sizeof(E), enumType);
        out->deselect();
      }
    }
  };

  template<typename E, typename A> struct Streamer<std::list<E, A>>
  {
    using S = std::list<E, A>;
    static void stream(In* in, Out* out, const char* name, S& s)
    {
      const char* enumType = std::is_enum<E>::value ? typeid(E).name() : nullptr;
      if(in)
      {
        in->select(name, -1);
        unsigned size;
        *in >> size;
        s.resize(size);
        int i = 0;
        for(E& elem : s)
        {
          in->select(0, i, enumType);
          *in >> elem;
          in->deselect();
          i++;
        }
        in->deselect();
      }
      else
      {
        out->select(name, -1);
        *out << static_cast<unsigned>(s.size());
        int i = 0;
        for(const E& elem : s)
        {
          out->select(0, i, enumType);
          *out << elem;
          out->deselect();
          i++;
        }
        out->deselect();
      }
    }
  };

  template<typename E, size_t n> struct Streamer<std::array<E, n>>
  {
    using S = std::array<E, n>;
    static void stream(In* in, Out* out, const char* name, S& s)
    {
      const char* enumType = std::is_enum<E>::value ? typeid(E).name() : nullptr;
      if(in)
      {
        in->select(name, -1);
        streamStaticArray(*in, s.data(), n * sizeof(E), enumType);
        in->deselect();
      }
      else
      {
        out->select(name, -1);
        streamStaticArray(*out, s.data(), n * sizeof(E), enumType);
        out->deselect();
      }
    }
  };

  /**
   * The following three functions are helpers for streaming data. (in == nullptr) != (out == nullptr).
   * @param S The type of the variable to be streamed.
   * @param in The stream to read from.
   * @param out The stream to write to.
   * @param name The name of the variable to be streamed.
   * @param s The variable to be streamed.
   * This is the version for using inside of serialize methods.
   */
  template<typename S> void streamIt(In* in, Out* out, const char* name, S& s)
  {
    Streamer<S>::stream(in, out, name, s);
  }

  /** This is the version for using inside operator>>. */
  template<typename S> void streamIt(In& in, const char* name, S& s)
  {
    Streamer<S>::stream(&in, nullptr, skipDot(name), s);
  }

  /** This is the version for using inside operator<<. */
  template<typename S> void streamIt(Out& out, const char* name, const S& s)
  {
    Streamer<S>::stream(nullptr, &out, skipDot(name), const_cast<S&>(s));
  }

  /**
   * Together with decltype, the following template allows to use any type
   * for declarations, even array types such as int[4]. It also works with
   * template parameters without the use of typename.
   * decltype(Streaming::TypeWrapper<myType>::type) myVar;
   */
  template<typename T> struct TypeWrapper {static T type;};
}
