/**
 * @file InOut.h
 *
 * Definition of the abstract base classes In and Out for streams.
 * Include this header file for declaring streaming operators.
 *
 * @author Thomas Röfer
 * @author Martin Lötzsch
 */

#pragma once

#include <string>

class Angle;

namespace EnumHelpers
{
  template<typename T, bool isEnum> struct EnumOrClass;
}

/**
 * The class Out is the abstract base class for all classes
 * that implement writing into streams.
 */
class Out
{
protected:
  /**
   * Virtual redirection for operator<<(const bool& value).
   */
  virtual void outBool(bool) = 0;

  /**
   * Virtual redirection for operator<<(const char& value).
   */
  virtual void outChar(char) = 0;

  /**
   * Virtual redirection for operator<<(const signed char& value).
   */
  virtual void outSChar(signed char) = 0;

  /**
   * Virtual redirection for operator<<(const unsigned char& value).
   */
  virtual void outUChar(unsigned char) = 0;

  /**
   * Virtual redirection for operator<<(const short& value).
   */
  virtual void outShort(short) = 0;

  /**
   * Virtual redirection for operator<<(const unsigned short& value).
   */
  virtual void outUShort(unsigned short) = 0;

  /**
   * Virtual redirection for operator<<(const int& value).
   */
  virtual void outInt(int) = 0;

  /**
   * Virtual redirection for operator<<(const unsigned& value).
   */
  virtual void outUInt(unsigned int) = 0;

  /**
   * Virtual redirection for operator<<(const float& value).
   */
  virtual void outFloat(float) = 0;

  /**
   * Virtual redirection for operator<<(const double& value).
   */
  virtual void outDouble(double) = 0;

  /**
   * Virtual redirection for operator<<(const char* value).
   */
  virtual void outString(const char*) = 0;

  /**
   * Virtual redirection for operator<<(const Angle& value).
   */
  virtual void outAngle(const Angle&) = 0;

  /**
   * Virtual redirection for operator<<(Out& (*f)(Out&)) that writes
   * the symbol "endl";
   */
  virtual void outEndL() = 0;

public:
  /** Virtual destructor for derived classes. */
  virtual ~Out() = default;

  /**
   * The function writes a number of bytes into a stream.
   * @param p The address the data is located at.
   * @param size The number of bytes to be written.
   */
  virtual void write(const void* p, size_t size) = 0;

  /**
   * The function returns whether this is a binary stream.
   * @return Does it output data in binary format?
   */
  virtual bool isBinary() const {return false;}

  virtual void select(const char* name, int type, const char* enumType = nullptr) {}
  virtual void deselect() {}

  /**
   * Streaming operator for classes, arrays, and enums.
   * The sizeof expression is evaluated at compile time. It allows distinguishing
   * between a class or array (is not automatically converted to int) and
   * an enum (can be converted to int). The implementation assumes that
   * sizeof(char) != sizeof(short).
   * @param t The object, array, or enum to stream.
   * @return The stream.
   */
  template<typename T> Out& operator<<(const T& t)
  {
    return EnumHelpers::EnumOrClass<T, std::is_enum<T>::value>::write(*this, t);
  }

  friend Out& operator<<(Out& out, const bool value);
  friend Out& operator<<(Out& out, const char value);
  friend Out& operator<<(Out& out, const signed char value);
  friend Out& operator<<(Out& out, const unsigned char value);
  friend Out& operator<<(Out& out, const short value);
  friend Out& operator<<(Out& out, const unsigned short value);
  friend Out& operator<<(Out& out, const int value);
  friend Out& operator<<(Out& out, const unsigned int value);
  friend Out& operator<<(Out& out, const float value);
  friend Out& operator<<(Out& out, const double value);
  friend Out& operator<<(Out& out, const char* value);
  friend Out& operator<<(Out& out, const std::string& value);
  friend Out& operator<<(Out& out, const Angle& value);
  friend Out& operator<<(Out& out, Out& (*f)(Out&));
  friend Out& endl(Out& stream);
};

/**
 * Operator that writes a Boolean into a stream.
 * @param out The stream to which is written.
 * @param value The value that is written.
 * @return The stream.
 */
inline Out& operator<<(Out& out, const bool value) {out.outBool(value); return out;}

/**
 * Operator that writes a char into a stream.
 * @param out The stream to which is written.
 * @param value The value that is written.
 * @return The stream.
 */
inline Out& operator<<(Out& out, const char value) {out.outChar(value); return out;}

/**
 * Operator that writes an signed char into a stream.
 * @param out The stream to which is written.
 * @param value The value that is written.
 * @return The stream.
 */
inline Out& operator<<(Out& out, const signed char value) {out.outSChar(value); return out;}

/**
 * Operator that writes an unsigned char into a stream.
 * @param out The stream to which is written.
 * @param value The value that is written.
 * @return The stream.
 */
inline Out& operator<<(Out& out, const unsigned char value) {out.outUChar(value); return out;}

/**
 * Operator that writes a short int into a stream.
 * @param out The stream to which is written.
 * @param value The value that is written.
 * @return The stream.
 */
inline Out& operator<<(Out& out, const short value) {out.outShort(value); return out;}

/**
 * Operator that writes an unsigned short int into a stream.
 * @param out The stream to which is written.
 * @param value The value that is written.
 * @return The stream.
 */
inline Out& operator<<(Out& out, const unsigned short value) {out.outUShort(value); return out;}

/**
 * Operator that writes an int into a stream.
 * @param out The stream to which is written.
 * @param value The value that is written.
 * @return The stream.
 */
inline Out& operator<<(Out& out, const int value) {out.outInt(value); return out;}

/**
 * Operator that writes an unsigned int into a stream.
 * @param out The stream to which is written.
 * @param value The value that is written.
 * @return The stream.
 */
inline Out& operator<<(Out& out, const unsigned int value) {out.outUInt(value); ; return out;}

/**
 * Operator that writes a float into a stream.
 * @param out The stream to which is written.
 * @param value The value that is written.
 * @return The stream.
 */
inline Out& operator<<(Out& out, const float value) {out.outFloat(value); return out;}

/**
 * Operator that writes a double into a stream.
 * @param out The stream to which is written.
 * @param value The value that is written.
 * @return The stream.
 */
inline Out& operator<<(Out& out, const double value) {out.outDouble(value); return out;}

/**
 * Operator that writes a string into a stream.
 * @param out The stream to which is written.
 * @param value The value that is written.
 * @return The stream.
 */
inline Out& operator<<(Out& out, const char* value) {out.outString(value); return out;}

/**
 * Operator that writes a string into a stream.
 * @param out The stream to which is written.
 * @param value The value that is written.
 * @return The stream.
 */
inline Out& operator<<(Out& out, const std::string& value) {out.outString(value.c_str()); return out;}

/**
 * Operator that writes an Angle into a stream.
 * @param out The stream to which is written.
 * @param value The value that is written.
 * @return The stream.
 */
inline Out& operator<<(Out& out, const Angle& value) {out.outAngle(value); return out;}

/**
 * Operator that calls the function pointed to by f.
 * @param out The stream to which is written.
 * @param f A function that is normally endl.
 * @return The stream.
 */
inline Out& operator<<(Out& out, Out& (*f)(Out&)) {return f(out);}

/**
 * This function can be inserted into a stream to represent an end of line.
 * @param out The stream the endl-symbol is inserted into.
 * @return The stream.
 */
Out& endl(Out& out);

/**
 * The class In is the abstract base class for all classes
 * that implement reading from streams.
 */
class In
{
protected:
  /**
   * Virtual redirection for operator>>(bool& value).
   */
  virtual void inBool(bool&) = 0;

  /**
   * Virtual redirection for operator>>(char& value).
   */
  virtual void inChar(char&) = 0;

  /**
   * Virtual redirection for operator>>(signed char& value).
   */
  virtual void inSChar(signed char&) = 0;

  /**
   * Virtual redirection for operator>>(unsigned char& value).
   */
  virtual void inUChar(unsigned char&) = 0;

  /**
   * Virtual redirection for operator>>(short& value).
   */
  virtual void inShort(short&) = 0;

  /**
   * Virtual redirection for operator>>(unsigned short& value).
   */
  virtual void inUShort(unsigned short&) = 0;

  /**
   * Virtual redirection for operator>>(int& value).
   */
  virtual void inInt(int&) = 0;

  /**
   * Virtual redirection for operator>>(unsigned int& value).
   */
  virtual void inUInt(unsigned int&) = 0;

  /**
   * Virtual redirection for operator>>(float& value).
   */
  virtual void inFloat(float&) = 0;

  /**
   * Virtual redirection for operator>>(double& value).
   */
  virtual void inDouble(double&) = 0;

  /**
   * Virtual redirection for operator>>(std::string& value).
   */
  virtual void inString(std::string&) = 0;

  /**
   * Virtual redirection for operator>>(Angle& value).
   */
  virtual void inAngle(Angle&) = 0;

  /**
   * Virtual redirection for operator>>(In& (*f)(In&)) that reads
   * the symbol "endl";
   */
  virtual void inEndL() = 0;

public:
  /** Virtual destructor for derived classes. */
  virtual ~In() = default;

  /**
   * The function reads a number of bytes from a stream.
   * @param p The address the data is written to. Note that p
   *          must point to a memory area that is at least
   *          "size" bytes large.
   * @param size The number of bytes to be read.
   */
  virtual void read(void* p, size_t size) = 0;

  /**
   * The function skips a number of bytes in a stream.
   * @param size The number of bytes to be skipped.
   */
  virtual void skip(size_t size) = 0;

  /**
   * Determines whether the end of file has been reached.
   */
  virtual bool eof() const = 0;

  /**
   * The function returns whether this is a binary stream.
   * @return Does it output data in binary format?
   */
  virtual bool isBinary() const { return false; }

  /**
   * Select an entry for reading.
   * Will only be implemented by class InMapFile.
   * @param name The name of the entry if type == -2, otherwise 0.
   * @param type The type of the entry.
   *             -2: value or record,
   *             -1: array or list.
   *             >= 0: array/list element index.
   * @param enumType The type as string if it is an enum. Otherwise nullptr.
   */
  virtual void select(const char* name, int type, const char* enumType = nullptr) {}

  /**
   * Deselects a field for reading.
   * Will only be implemented by class InMapFile.
   */
  virtual void deselect() {}

  /**
   * Streaming operator for classes, arrays, and enums.
   * The sizeof expression is evaluated at compile time. It allows distinguishing
   * between a class or array (is not automatically converted to int) and
   * an enum (can be converted to int). The implementation assumes that
   * sizeof(char) != sizeof(short).
   * @param t The object, array, or enum to stream.
   * @return The stream.
   */
  template<typename T> In& operator>>(T& t)
  {
    return EnumHelpers::EnumOrClass<T, std::is_enum<T>::value>::read(*this, t);
  }

  friend In& operator>>(In& in, bool& value);
  friend In& operator>>(In& in, char& value);
  friend In& operator>>(In& in, signed char& value);
  friend In& operator>>(In& in, unsigned char& value);
  friend In& operator>>(In& in, short& value);
  friend In& operator>>(In& in, unsigned short& value);
  friend In& operator>>(In& in, int& value);
  friend In& operator>>(In& in, unsigned int& value);
  friend In& operator>>(In& in, float& value);
  friend In& operator>>(In& in, double& value);
  friend In& operator>>(In& in, std::string& value);
  friend In& operator>>(In& in, Angle& value);
  friend In& operator>>(In& in, In& (*f)(In&));
  friend In& endl(In& stream);
};

/**
 * Operator that reads a Boolean from a stream.
 * @param in The stream from which is read.
 * @param value The value that is read.
 * @return The stream.
 */
inline In& operator>>(In& in, bool& value) {in.inBool(value); return in;}

/**
 * Operator that reads a char from a stream.
 * @param in The stream from which is read.
 * @param value The value that is read.
 * @return The stream.
 */
inline In& operator>>(In& in, char& value) {in.inChar(value); return in;}

/**
 * Operator that reads a signed char from a stream.
 * @param in The stream from which is read.
 * @param value The value that is read.
 * @return The stream.
 */
inline In& operator>>(In& in, signed char& value) {in.inSChar(value); return in;}

/**
 * Operator that reads an unsigned char from a stream.
 * @param in The stream from which is read.
 * @param value The value that is read.
 * @return The stream.
 */
inline In& operator>>(In& in, unsigned char& value) {in.inUChar(value); return in;}

/**
 * Operator that reads a short int from a stream.
 * @param in The stream from which is read.
 * @param value The value that is read.
 * @return The stream.
 */
inline In& operator>>(In& in, short& value) {in.inShort(value); return in;}

/**
 * Operator that reads an unsigned short int from a stream.
 * @param in The stream from which is read.
 * @param value The value that is read.
 * @return The stream.
 */
inline In& operator>>(In& in, unsigned short& value) {in.inUShort(value); return in;}

/**
 * Operator that reads an int from a stream.
 * @param in The stream from which is read.
 * @param value The value that is read.
 * @return The stream.
 */
inline In& operator>>(In& in, int& value) {in.inInt(value); return in;}

/**
 * Operator that reads an unsigned int from a stream.
 * @param in The stream from which is read.
 * @param value The value that is read.
 * @return The stream.
 */
inline In& operator>>(In& in, unsigned int& value) {in.inUInt(value); return in;}

/**
 * Operator that reads a float from a stream.
 * @param in The stream from which is read.
 * @param value The value that is read.
 * @return The stream.
 */
inline In& operator>>(In& in, float& value) {in.inFloat(value); return in;}

/**
 * Operator that reads a double from a stream.
 * @param in The stream from which is read.
 * @param value The value that is read.
 * @return The stream.
 */
inline In& operator>>(In& in, double& value) {in.inDouble(value); return in;}

/**
 * Operator that reads a string from a stream.
 * @param in The stream from which is read.
 * @param value The value that is read.
 * @return The stream.
 */
inline In& operator>>(In& in, std::string& value) {in.inString(value); return in;}

/**
 * Operator that reads an Angle from a stream.
 * @param in The stream from which is read.
 * @param value The value that is read.
 * @return The stream.
 */
inline In& operator>>(In& in, Angle& value) {in.inAngle(value); return in;}

/**
 * Operator that reads the endl-symbol from a stream.
 * @param in The stream from which is read.
 * @param f A function that is normally endl.
 * @return The stream.
 */
inline In& operator>>(In& in, In& (*f)(In&)) {return f(in);}

/**
 * This function can be read from a stream to represent an end of line.
 * @param in The stream the endl-symbol is read from.
 * @return The stream.
 */
In& endl(In& in);

namespace EnumHelpers
{
  /**
   * Two classes that simply hide the template versions of the streaming
   * operators by versions that will never be called. They aren't even
   * implemented.
   */
  class Out2 : public ::Out {template<typename T> Out& operator<<(const int&);};
  class In2 : public ::In {template<typename T> In& operator>>(int&);};

  /**
   * @class EnumOrClass
   * A template class to stream in a different way depending on whether
   * it is instantiated for an enum or a class. This is the version for
   * classes. Here the normal streaming operators are called. To avoid
   * an endless recursion with the template streaming operators below,
   * those operators are defined in the classes In und Out and they
   * are hidden in the derivations In2 and Out2. Since hiding
   * seems to be implemented differently in different compilers,
   * all other streaming operators are defined outside of classes.
   * @param T The type of the value to stream.
   * @param isEnum Is T an enum type? For this version, this is always false.
   */
  template<typename T, bool isEnum> struct EnumOrClass
  {
    // An error here usually means that you try to stream data that is not streamable
    static Out& write(Out& out, const T& t) {return (Out2&)out << t;}
    static In& read(In& in, T& t) {return (In2&)in >> t;}
  };

  /**
   * @class EnumOrClass
   * A template class to stream in a different way depending on whether
   * it is instantiated for an enum or a class. This is the version for
   * streaming enums (as integers).
   * @param T The type of the value to stream.
   */
  template<typename T> struct EnumOrClass<T, true>
  {
    static Out& write(Out& out, const T& t)
    {
      return sizeof(t) == 1 ? out << static_cast<unsigned char>(t) : out << static_cast<int>(t);
    }

    static In& read(In& in, T& t)
    {
      if(sizeof(t) == 1)
      {
        unsigned char c = static_cast<unsigned char>(t); // keep old value in case streaming does nothing
        in >> c;
        t = static_cast<T>(c);
      }
      else
      {
        int i = static_cast<int>(t); // keep old value in case streaming does nothing
        in >> i;
        t = static_cast<T>(i);
      }
      return in;
    }
  };
}

namespace Streaming
{
  void trimName(const char*& name);
}
