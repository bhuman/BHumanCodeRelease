/**
 * @file OutStreams.h
 *
 * Declaration of out stream classes for different media and formats.
 *
 * @author Thomas Röfer
 * @author Martin Lötzsch
 */

#pragma once

#include <vector>
#include "InOut.h"

class File;

/**
 * @class PhysicalOutStream
 *
 * The base class for physical out streams. Derivatives of PhysicalOutStream only handle the
 * writing of data to a medium, not of formatting data.
 */
class PhysicalOutStream
{
public:
  /**
   * The function writes a number of bytes into a physical stream.
   * @param p The address the data is located at.
   * @param size The number of bytes to be written.
   */
  virtual void writeToStream(const void* p, size_t size) = 0;
};

/**
 * @class StreamWriter
 *
 * Generic class for formatting data to be used in streams.
 * The physical writing is then done by OutStream derivatives.
 */
class StreamWriter
{
protected:
  /**
   * Writes a bool to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeBool(bool d, PhysicalOutStream& stream) = 0;

  /**
   * Writes a character to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeChar(char d, PhysicalOutStream& stream) = 0;

  /**
   * Writes a signed character to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeSChar(signed char d, PhysicalOutStream& stream) = 0;

  /**
   * Writes a unsigned character to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeUChar(unsigned char d, PhysicalOutStream& stream) = 0;

  /**
   * Writes a short to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeShort(short d, PhysicalOutStream& stream) = 0;

  /**
   * Writes a unsigned short to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeUShort(unsigned short d, PhysicalOutStream& stream) = 0;

  /**
   * Writes a int to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeInt(int d, PhysicalOutStream& stream) = 0;

  /**
   * Writes a unsigned int to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeUInt(unsigned int d, PhysicalOutStream& stream) = 0;

  /**
   * Writes a float to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeFloat(float d, PhysicalOutStream& stream) = 0;

  /**
   * Writes a double to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeDouble(double d, PhysicalOutStream& stream) = 0;

  /**
   * Writes a string to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeString(const char* d, PhysicalOutStream& stream) = 0;

  /**
   * Writes an Angle to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeAngle(const Angle& d, PhysicalOutStream& stream) = 0;

  /**
   * Writes a 'end of line' to a stream.
   * @param stream the stream to write on.
   */
  virtual void writeEndL(PhysicalOutStream& stream) = 0;

  /**
   * The function writes a number of bytes into the stream.
   * @param p The address the data is located at.
   * @param size The number of bytes to be written.
   * @param stream the stream to write on.
   */
  virtual void writeData(const void* p, size_t size, PhysicalOutStream& stream) = 0;
};

/**
 * @class OutStream
 *
 * Generic class for classes that do both formatting and physical writing of data to streams.
 */
template <class S, class W> class OutStream : public S, public W, public Out
{
public:
  /**
   * The function writes a number of bytes into a stream.
   * @param p The address the data is located at.
   * @param size The number of bytes to be written.
   */
  virtual void write(const void* p, size_t size)
  {
    W::writeData(p, size, *this);
  }

protected:
  /**
   * Virtual redirection for operator<<(const bool& value).
   */
  virtual void outBool(bool d)
  {
    W::writeBool(d, *this);
  }

  /**
   * Virtual redirection for operator<<(const char& value).
   */
  virtual void outChar(char d)
  {
    W::writeChar(d, *this);
  }

  /**
   * Virtual redirection for operator<<(const signed char& value).
   */
  virtual void outSChar(signed char d)
  {
    W::writeSChar(d, *this);
  }

  /**
   * Virtual redirection for operator<<(const unsigned char& value).
   */
  virtual void outUChar(unsigned char d)
  {
    W::writeUChar(d, *this);
  }

  /**
   * Virtual redirection for operator<<(const short& value).
   */
  virtual void outShort(short d)
  {
    W::writeShort(d, *this);
  }

  /**
   * Virtual redirection for operator<<(const unsigned short& value).
   */
  virtual void outUShort(unsigned short d)
  {
    W::writeUShort(d, *this);
  }

  /**
   * Virtual redirection for operator<<(const int& value).
   */
  virtual void outInt(int d)
  {
    W::writeInt(d, *this);
  }

  /**
   * Virtual redirection for operator<<(const unsigned& value).
   */
  virtual void outUInt(unsigned int d)
  {
    W::writeUInt(d, *this);
  }

  /**
   * Virtual redirection for operator<<(const float& value).
   */
  virtual void outFloat(float d)
  {
    W::writeFloat(d, *this);
  }

  /**
   * Virtual redirection for operator<<(const double& value).
   */
  virtual void outDouble(double d)
  {
    W::writeDouble(d, *this);
  }

  /**
   * Virtual redirection for operator<<(const char* value).
   */
  virtual void outString(const char* d)
  {
    W::writeString(d, *this);
  }

  /**
   * Virtual redirection for operator<<(const Angle& value).
   */
  virtual void outAngle(const Angle& d)
  {
    W::writeAngle(d, *this);
  }

  /**
   * Virtual redirection for operator<<(Out& (*f)(Out&)) that writes
   * the symbol "endl";
   */
  virtual void outEndL()
  {
    W::writeEndL(*this);
  }
};

/**
 * @class OutBinary
 *
 * Formats data binary to be used in streams.
 * The physical writing is then done by OutStream derivatives.
 */
class OutBinary : public StreamWriter
{
protected:
  /**
   * Writes a bool to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeBool(bool d, PhysicalOutStream& stream)
  {
    char c = static_cast<char>(d);
    stream.writeToStream(&c, sizeof(c));
  }

  /**
   * Writes a character to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeChar(char d, PhysicalOutStream& stream)
  {
    stream.writeToStream(&d, sizeof(d));
  }

  /**
   * Writes a signed character to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeSChar(signed char d, PhysicalOutStream& stream)
  {
    stream.writeToStream(&d, sizeof(d));
  }

  /**
   * Writes a unsigned character to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeUChar(unsigned char d, PhysicalOutStream& stream)
  {
    stream.writeToStream(&d, sizeof(d));
  }

  /**
   * Writes a short to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeShort(short d, PhysicalOutStream& stream)
  {
    stream.writeToStream(&d, sizeof(d));
  }

  /**
   * Writes a unsigned short to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeUShort(unsigned short d, PhysicalOutStream& stream)
  {
    stream.writeToStream(&d, sizeof(d));
  }

  /**
   * Writes a int to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeInt(int d, PhysicalOutStream& stream)
  {
    stream.writeToStream(&d, sizeof(d));
  }

  /**
   * Writes a unsigned int to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeUInt(unsigned int d, PhysicalOutStream& stream)
  {
    stream.writeToStream(&d, sizeof(d));
  }

  /**
   * Writes a float to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeFloat(float d, PhysicalOutStream& stream)
  {
    stream.writeToStream(&d, sizeof(d));
  }

  /**
   * Writes a double to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeDouble(double d, PhysicalOutStream& stream)
  {
    stream.writeToStream(&d, sizeof(d));
  }

  /**
   * Writes a string to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeString(const char* d, PhysicalOutStream& stream);

  /**
   * Writes an Angle to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeAngle(const Angle& d, PhysicalOutStream& stream);

  /**
   * Writes a 'end of line' to a stream.
   * @param stream the stream to write on.
   */
  virtual void writeEndL(PhysicalOutStream& stream) {}

  /**
   * The function writes a number of bytes into the stream.
   * @param p The address the data is located at.
   * @param size The number of bytes to be written.
   * @param stream the stream to write on.
   */
  virtual void writeData(const void* p, size_t size, PhysicalOutStream& stream)
  {
    stream.writeToStream(p, size);
  }
};

/**
 * @class OutText
 *
 * Formats data as text to be used in streams.
 * The physical writing is then done by PhysicalOutStream derivatives.
 */
class OutText : public StreamWriter
{
private:
  /** A buffer for formatting the numeric data to a text format. */
  char buf[50];
protected:
  /**
   * Writes a bool to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeBool(bool d, PhysicalOutStream& stream);

  /**
   * Writes a character to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeChar(char d, PhysicalOutStream& stream);

  /**
   * Writes a signed character to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeSChar(signed char d, PhysicalOutStream& stream);

  /**
   * Writes a unsigned character to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeUChar(unsigned char d, PhysicalOutStream& stream);

  /**
   * Writes a short to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeShort(short d, PhysicalOutStream& stream);

  /**
   * Writes a unsigned short to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeUShort(unsigned short d, PhysicalOutStream& stream);

  /**
   * Writes a int to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeInt(int d, PhysicalOutStream& stream);

  /**
   * Writes a unsigned int to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeUInt(unsigned int d, PhysicalOutStream& stream);

  /**
   * Writes a float to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeFloat(float d, PhysicalOutStream& stream);

  /**
   * Writes a double to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeDouble(double d, PhysicalOutStream& stream);

  /**
   * Writes a string to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeString(const char* d, PhysicalOutStream& stream);

  /**
   * Writes an Angle to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeAngle(const Angle& d, PhysicalOutStream& stream);

  /**
   * Writes a 'end of line' to a stream.
   * @param stream the stream to write on.
   */
  virtual void writeEndL(PhysicalOutStream& stream);

  /**
   * The function writes a number of bytes into the stream.
   * @param p The address the data is located at.
   * @param size The number of bytes to be written.
   * @param stream the stream to write on.
   */
  virtual void writeData(const void* p, size_t size, PhysicalOutStream& stream);
};

/**
 * @class OutTextRaw
 *
 * Formats data as raw text to be used in streams.
 * The physical writing is then done by PhysicalOutStream derivatives.
 * Different from OutText, OutTextRaw does not escape spaces
 * and other special characters and no spaces are inserted before numbers.
 * (The result of the OutTextRaw StreamWriter is the same as the result of "std::cout")
 */
class OutTextRaw : public StreamWriter
{
private:
  /** A buffer for formatting the numeric data to a text format. */
  char buf[50];
protected:
  /**
   * Writes a bool to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeBool(bool d, PhysicalOutStream& stream);

  /**
   * Writes a character to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeChar(char d, PhysicalOutStream& stream);

  /**
   * Writes a signed character to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeSChar(signed char d, PhysicalOutStream& stream);

  /**
   * Writes a unsigned character to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeUChar(unsigned char d, PhysicalOutStream& stream);

  /**
   * Writes a short to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeShort(short d, PhysicalOutStream& stream);

  /**
   * Writes a unsigned short to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeUShort(unsigned short d, PhysicalOutStream& stream);

  /**
   * Writes a int to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeInt(int d, PhysicalOutStream& stream);

  /**
   * Writes a unsigned int to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeUInt(unsigned int d, PhysicalOutStream& stream);

  /**
   * Writes a float to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeFloat(float d, PhysicalOutStream& stream);

  /**
   * Writes a double to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeDouble(double d, PhysicalOutStream& stream);

  /**
   * Writes a string to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeString(const char* d, PhysicalOutStream& stream);

  /**
   * Writes an Angle to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeAngle(const Angle& d, PhysicalOutStream& stream);

  /**
   * Writes a 'end of line' to a stream.
   * @param stream the stream to write on.
   */
  virtual void writeEndL(PhysicalOutStream& stream);

  /**
   * The function writes a number of bytes into the stream.
   * @param p The address the data is located at.
   * @param size The number of bytes to be written.
   * @param stream the stream to write on.
   */
  virtual void writeData(const void* p, size_t size, PhysicalOutStream& stream);
};

/**
 * @class OutFile
 *
 * An PhysicalOutStream that writes the data to a file.
 */
class OutFile : public PhysicalOutStream
{
private:
  File* stream = nullptr; /**< Object representing the file. */

public:
  /** Destructor */
  ~OutFile();

  /**
   * The function states whether the file actually exists.
   * @return Does the file exist?
   */
  virtual bool exists() const;

  /**
   * The function returns the full path of the file.
   * @return The full path name actually used or the file searched for
   *         if it was not found. If the file was opened, the path can
   *         still be relative to the current directory if the B-Human
   *         directory was specified this way.
   */
  std::string getFullName() const;

protected:
  /**
   * Opens the stream.
   * @param name The name of the file to open. It will be interpreted
   *             as relative to the configuration directory. If the file
   *             does not exist, it will be created. If it already
   *             exists, its previous contents will be discared.
   */
  void open(const std::string& name);

  /**
   * Opens the stream.
   * @param name The name of the file to open. It will be interpreted
   *             as relative to the configuration directory. If the file
   *             does not exist, it will be created. If it already
   *             exists, its previous contents will be preserved,
   *             if append = true.
   * @param append Determines, if the file content is preserved or discared.
   */
  void open(const std::string& name, bool append);

  /**
   * The function writes a number of bytes into the file.
   * @param p The address the data is located at.
   * @param size The number of bytes to be written.
   */
  virtual void writeToStream(const void* p, size_t size);
};

/**
 * @class OutMemory
 *
 * A  PhysicalOutStream that writes the data to a memory block.
 */
class OutMemory : public PhysicalOutStream
{
private:
  char* memory = nullptr; /**< Points to the next byte to write at. */
  size_t length = 0; /**< The number of stored bytes */
  void* start = nullptr; /**< Points to the first byte */

public:
  /**
   * Returns the number of written bytes
   */
  size_t getLength() const { return length; }

  /**
   * Returns the address of the first byte
   */
  void* getMemory() { return start; }

protected:
  /**
   * opens the stream.
   * @param mem The address of the memory block into which is written.
   */
  void open(void* mem)
  {
    memory = reinterpret_cast<char*>(mem);
    start = mem;
    length = 0;
  }

  /**
   * The function writes a number of bytes into memory.
   * @param p The address the data is located at.
   * @param size The number of bytes to be written.
   */
  virtual void writeToStream(const void* p, size_t size);
};

/**
 * @class OutSize
 *
 * A PhysicalOutStream that doesn't write any data. Instead it works as a
 * counter that determines how many bytes would be streamed into memory.
 * It can be used to determine the size of the memory block that is
 * required as the parameter for an instance of class OutMemoryStream.
 */
class OutSize : public PhysicalOutStream
{
private:
  size_t size = 0; /**< Accumulator of the required number of bytes. */

public:
  /**
   * The function resets the counter to zero.
   */
  void reset() {size = 0;}

  /**
   * The function returns the number of bytes required to store the
   * data written so far.
   * @return The size of the memory block required for the data written.
   */
  size_t getSize() const {return size;}

protected:
  /**
   * The function counts the number of bytes that should be written.
   * @param p The address the data is located at.
   * @param s The number of bytes to be written.
   */
  virtual void writeToStream(const void* p, size_t s) {size += s;}
};

/**
 * @class OutBinaryFile
 *
 * A binary stream into a file.
 */
class OutBinaryFile : public OutStream<OutFile, OutBinary>
{
public:
  /**
   * Constructor.
   * @param name The name of the file to open. It will be interpreted
   *             as relative to the configuration directory. If the file
   *             does not exist, it will be created. If it already
   *             exists, its previous contents will be discared.
   */
  OutBinaryFile(const std::string& name)
  {
    open(name);
  }

  /**
   * Constructor.
   * @param name The name of the file to open. It will be interpreted
   *             as relative to the configuration directory. If the file
   *             does not exist, it will be created. If it already
   *             exists, its previous contents will be preserved,
   *             if append = true.
   * @param append Determines, if the file content is preserved or discared.
   */
  OutBinaryFile(const std::string& name, bool append)
  {
    open(name, append);
  }

  /**
   * The function returns whether this is a binary stream.
   * @return Does it output data in binary format?
   */
  virtual bool isBinary() const {return true;}
};

/**
 * @class OutBinaryMemory
 *
 * A binary stream into a memory region.
 */
class OutBinaryMemory : public OutStream<OutMemory, OutBinary>
{
public:
  /**
   * Constructor.
   * @param mem The address of the memory block into which is written.
   */
  OutBinaryMemory(void* mem)
  {
    open(mem);
  }

  /**
   * The function returns whether this is a binary stream.
   * @return Does it output data in binary format?
   */
  virtual bool isBinary() const {return true;}
};

/**
 * @class OutBinarySize
 *
 * A binary stream size counter
 */
class OutBinarySize : public OutStream<OutSize, OutBinary>
{
public:
  /**
   * The function returns whether this is a binary stream.
   * @return Does it output data in binary format?
   */
  virtual bool isBinary() const {return true;}
};

/**
 * @class OutTextFile
 *
 * A text stream into a file.
 */
class OutTextFile : public OutStream<OutFile, OutText>
{
public:
  /**
   * Constructor.
   * @param name The name of the file to open. It will be interpreted
   *             as relative to the configuration directory. If the file
   *             does not exist, it will be created. If it already
   *             exists, its previous contents will be discared.
   */
  OutTextFile(const std::string& name)
  {
    open(name);
  }

  /**
   * Constructor.
   * @param name The name of the file to open. It will be interpreted
   *             as relative to the configuration directory. If the file
   *             does not exist, it will be created. If it already
   *             exists, its previous contents will be preserved,
   *             if append = true.
   * @param append Determines, if the file content is preserved or discared.
   */
  OutTextFile(const std::string& name, bool append)
  {
    open(name, append);
  }
};

/**
 * @class OutTextRawFile
 *
 * A text stream into a file.
 */
class OutTextRawFile : public OutStream<OutFile, OutTextRaw>
{
public:
  /**
   * Constructor.
   * @param name The name of the file to open. It will be interpreted
   *             as relative to the configuration directory. If the file
   *             does not exist, it will be created. If it already
   *             exists, its previous contents will be discared.
   */
  OutTextRawFile(const std::string& name)
  {
    open(name);
  }

  /**
   * Constructor.
   * @param name The name of the file to open. It will be interpreted
   *             as relative to the configuration directory. If the file
   *             does not exist, it will be created. If it already
   *             exists, its previous contents will be preserved,
   *             if append = true.
   * @param append Determines, if the file content is preserved or discared.
   */
  OutTextRawFile(const std::string& name, bool append)
  {
    open(name, append);
  }
};

/**
 * @class OutTextMemory
 *
 * A text stream into a memory region.
 */
class OutTextMemory : public OutStream<OutMemory, OutText>
{
public:
  /**
   * Constructor.
   * @param mem The address of the memory block into which is written.
   */
  OutTextMemory(void* mem)
  {
    open(mem);
  }
};

/**
 * @class OutTextRawMemory
 *
 * A text stream into a memory region.
 */
class OutTextRawMemory : public OutStream<OutMemory, OutTextRaw>
{
public:
  /**
   * Constructor.
   * @param mem The address of the memory block into which is written.
   */
  OutTextRawMemory(void* mem)
  {
    open(mem);
  }
};

/**
 * @class OutTextSize
 *
 * A Text stream size counter
 */
class OutTextSize : public OutStream<OutSize, OutText> {};

/**
 * @class OutTextRawSize
 *
 * A Text stream size counter
 */
class OutTextRawSize : public OutStream<OutSize, OutTextRaw> {};

/**
 * @class OutMap
 *
 * A stream that writes data in config map format.
 */
class OutMap : public Out
{
  /**
   * An entry representing the current state in the writing process.
   */
  class Entry
  {
  public:
    int type; /**< The type of the entry. -2: value or record, -1: array, >= 0: array element index. */
    const char* (*enumToString)(int); /**< A function that translates an enum to a string. */
    bool hasSubEntries; /**< Has the current entry sub entries? */

    Entry(int type, const char* (*enumToString)(int)) :
      type(type), enumToString(enumToString), hasSubEntries(false)
    {}
  };
  Out& stream; /**< The map that is written to. */
  bool singleLine; /**< Output to a single line. */
  std::vector<Entry> stack; /**< The hierarchy of values that are written. */
  std::string indentation; /**< The indentation as sequence of spaces. */

  /** Writes a line break or simply a space if singleLine is active. */
  void writeLn();

protected:
  /**
   * Writes to a stream in config map format.
   * @param stream The stream that is written to.
   * @param singleLine Output to a single line.
   */
  OutMap(Out& stream, bool singleLine);

  /** Helper to write a value. */
  template<class T> void out(const T& value) {stream << value;}

  virtual void outBool(bool value) {out(value);}
  virtual void outChar(char value) {out(static_cast<int>(value));}
  virtual void outSChar(signed char value) {out(static_cast<int>(value));}
  virtual void outUChar(unsigned char value);
  virtual void outShort(short value) {out(value);}
  virtual void outUShort(unsigned short value) {out(value);}
  virtual void outInt(int value) {out(value);}
  virtual void outUInt(unsigned int value);
  virtual void outFloat(float value) {out(value);}
  virtual void outDouble(double value) {out(value);}
  virtual void outString(const char* value);
  virtual void outAngle(const Angle& value) {out(value);};
  virtual void outEndL() {}

public:
  /**
   * Select an entry for writing.
   * @param name The name of the entry if type == -2, otherwise 0.
   * @param type The type of the entry.
   *             -2: value or record,
   *             -1: array,
   *             >= 0: array element index.
   * @param enumToString A function that translates an enum to a string.
   */
  virtual void select(const char* name, int type, const char* (*enumToString)(int));

  /** Deselects a field for writing. */
  virtual void deselect();

  /** Writing raw data is not supported. Do not call. */
  virtual void write(const void* p, size_t size);
};

/**
 * @class OutMapFile
 *
 * A stream that writes data to a text file in config map format.
 */
class OutMapFile : public OutMap
{
private:
  OutTextRawFile stream; /**< The text stream to write to. */

public:
  /**
   * Constructor.
   * @param name The name of the config file to write to.
   * @param singleLine Output to a single line.
   */
  OutMapFile(const std::string& name, bool singleLine = false);

  /**
   * Checks whether was successfully opened.
   * @return Was it possible to open that file for writing?
   */
  bool exists() const {return stream.exists();}
};

/**
 * @class OutMapMemory
 *
 * A stream that writes data to a memory block config map format.
 */
class OutMapMemory : public OutMap
{
private:
  OutTextRawMemory stream; /**< The memory stream to write to. */

public:
  /**
   * Constructor.
   * @param memory The block of memory that will be written to.
   * @param singleLine Output to a single line.
   */
  OutMapMemory(void* memory, bool singleLine = false);

  /**
   * Returns the number of written bytes
   */
  size_t getLength() const {return stream.getLength();}
};

/**
 * @class OutMapSize
 *
 * A stream that determines the number of bytes required to write data in config map format.
 */
class OutMapSize : public OutMap
{
private:
  OutTextRawSize stream; /**< The stream that determines the size required. */

public:
  /**
   * Constructor.
   * @param singleLine Assume that output is to a single line.
   */
  OutMapSize(bool singleLine = false);

  /**
   * The function returns the number of bytes required to store the
   * data written so far.
   * @return The size of the memory block required for the data written.
   */
  size_t getSize() const {return stream.getSize();}
};
