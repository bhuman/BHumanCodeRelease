/**
 * @file InStreams.h
 *
 * Declaration of in stream classes for different media and formats.
 *
 * @author Thomas Röfer
 * @author Martin Lötzsch
 */

#pragma once

#include "SimpleMap.h"

class File;

/**
 * @class PhysicalInStream
 *
 * The base class for physical in streams. Derivates of PhysicalInStream only handle the
 * reading of data from a medium, not of formatting data.
 */
class PhysicalInStream
{
public:
  /**
   * The function reads a number of bytes from a stream.
   * @param p The address the data is written to. Note that p
   *          must point to a memory area that is at least
   *          "size" bytes large.
   * @param size The number of bytes to be read.
   */
  virtual void readFromStream(void* p, size_t size) = 0;

  /**
   * The function skips a number of bytes in a stream.
   * @param size The number of bytes to be read.
   */
  virtual void skipInStream(size_t size);

  /**
   * The function states whether this stream actually exists.
   * This function is relevant if the stream represents a file.
   * @return Does the stream exist?
   */
  virtual bool exists() const {return true;}

  /**
   * The function states whether the end of the stream has been reached.
   * @return End of stream reached?
   */
  virtual bool getEof() const = 0;
};

/**
 * @class StreamReader
 *
 * Generic class for formatted reading of data to be used in streams.
 * The physical reading is then done by PhysicalOutStream derivates.
 */
class StreamReader
{
protected:
  /**
   * reads a bool from a stream
   * @param d the data to read from the stream
   * @param stream the stream to read from
   */
  virtual void readBool(bool& d, PhysicalInStream& stream) = 0;

  /**
   * reads a character from a stream
   * @param d the data to read from the stream
   * @param stream the stream to read from
   */
  virtual void readChar(char& d, PhysicalInStream& stream) = 0;

  /**
   * reads a signed character from a stream
   * @param d the data to read from the stream
   * @param stream the stream to read from
   */
  virtual void readSChar(signed char& d, PhysicalInStream& stream) = 0;

  /**
   * reads an unsigned character from a stream
   * @param d the data to read from the stream
   * @param stream the stream to read from
   */
  virtual void readUChar(unsigned char& d, PhysicalInStream& stream) = 0;

  /**
   * reads a short from a stream
   * @param d the data to read from the stream
   * @param stream the stream to read from
   */
  virtual void readShort(short& d, PhysicalInStream& stream) = 0;

  /**
   * reads an unsigned short from a stream
   * @param d the data to read from the stream
   * @param stream the stream to read from
   */
  virtual void readUShort(unsigned short& d, PhysicalInStream& stream) = 0;

  /**
   * reads an int from a stream
   * @param d the data to read from the stream
   * @param stream the stream to read from
   */
  virtual void readInt(int& d, PhysicalInStream& stream) = 0;

  /**
   * reads an unsigned int from a stream
   * @param d the data to read from the stream
   * @param stream the stream to read from
   */
  virtual void readUInt(unsigned int& d, PhysicalInStream& stream) = 0;

  /**
   * reads a float from a stream
   * @param d the data to read from the stream
   * @param stream the stream to read from
   */
  virtual void readFloat(float& d, PhysicalInStream& stream) = 0;

  /**
   * reads a double from a stream
   * @param d the data to read from the stream
   * @param stream the stream to read from
   */
  virtual void readDouble(double& d, PhysicalInStream& stream) = 0;

  /**
   * reads a string from a stream
   * @param d the data to read from the stream
   * @param stream the stream to read from
   */
  virtual void readString(std::string& d, PhysicalInStream& stream) = 0;

  /**
   * reads an Angle from a stream
   * @param d the data to read from the stream
   * @param stream the stream to read from
   */
  virtual void readAngle(Angle& d, PhysicalInStream& stream) = 0;

  /**
   * reads the 'end of line' from a stream
   * @param stream the stream to read from
   */
  virtual void readEndl(PhysicalInStream& stream) = 0;

  /**
   * The function reads a number of bytes from the file.
   * @param p The address the data is written to. Note that p
   *          must point to a memory area that is at least
   *          "size" bytes large.
   * @param size The number of bytes to be read.
   * @param stream The stream to read from.
   */
  virtual void readData(void* p, size_t size, PhysicalInStream& stream) = 0;

  /**
   * The function skips a number of bytes in the file.
   * @param size The number of bytes to be skipped.
   * @param stream The stream to read from.
   */
  virtual void skipData(size_t size, PhysicalInStream& stream);

  /**
   * The function states whether the end of the stream has been reached.
   * @param stream The stream to be tested
   * @return End of stream reached?
   */
  virtual bool isEof(const PhysicalInStream& stream) const = 0;
};

/**
 * @class InStream
 *
 * Generic class for classes that do both formatted and physical reading of data from streams.
 */
template<typename S, typename R> class InStream : public S, public R, public In
{
public:
  /**
   * The function reads a number of bytes from a stream.
   * @param p The address the data is written to. Note that p
   *          must point to a memory area that is at least
   *          "size" bytes large.
   * @param size The number of bytes to be read.
   */
  void read(void* p, size_t size) override
  {
    R::readData(p, size, *this);
  }

  /**
   * The function skips a number of bytes in the stream.
   * @param size The number of bytes to be skipped.
   */
  void skip(size_t size) override
  {
    R::skipData(size, *this);
  }

  /**
   * Determines whether the end of file has been reached.
   */
  bool eof() const override { return R::isEof(*this); }

protected:
  /**
   * Virtual redirection for operator>>(bool& value).
   */
  void inBool(bool& d) override
  {
    R::readBool(d, *this);
  }

  /**
   * Virtual redirection for operator>>(char& value).
   */
  void inChar(char& d) override
  {
    R::readChar(d, *this);
  }

  /**
   * Virtual redirection for operator>>(signed char& value).
   */
  void inSChar(signed char& d) override
  {
    R::readSChar(d, *this);
  }

  /**
   * Virtual redirection for operator>>(unsigned char& value).
   */
  void inUChar(unsigned char& d) override
  {
    R::readUChar(d, *this);
  }

  /**
   * Virtual redirection for operator>>(short& value).
   */
  void inShort(short& d) override
  {
    R::readShort(d, *this);
  }

  /**
   * Virtual redirection for operator>>(unsigned short& value).
   */
  void inUShort(unsigned short& d) override
  {
    R::readUShort(d, *this);
  }

  /**
   * Virtual redirection for operator>>(int& value).
   */
  void inInt(int& d) override
  {
    R::readInt(d, *this);
  }

  /**
   * Virtual redirection for operator>>(unsigned int& value).
   */
  void inUInt(unsigned int& d) override
  {
    R::readUInt(d, *this);
  }

  /**
   * Virtual redirection for operator>>(float& value).
   */
  void inFloat(float& d) override
  {
    R::readFloat(d, *this);
  }

  /**
   * Virtual redirection for operator>>(double& value).
   */
  void inDouble(double& d) override
  {
    R::readDouble(d, *this);
  }

  /**
   * Virtual redirection for operator>>(std::string& value).
   */
  void inString(std::string& d) override
  {
    R::readString(d, *this);
  }

  /**
   * Virtual redirection for operator>>(Angle& value).
   */
  void inAngle(Angle& d) override
  {
    R::readAngle(d, *this);
  }

  /**
   * Virtual redirection for operator>>(In& (*f)(In&)) that reads
   * the symbol "endl";
   */
  void inEndL() override
  {
    R::readEndl(*this);
  }
};

/**
 * @class InText
 *
 * Formatted reading of text data to be used in streams.
 * The physical reading is done by PhysicalInStream derivates.
 */
class InText : public StreamReader
{
protected:
  /** The last character read. */
  char theChar = ' ',
       theNextChar = ' ';
private:
  std::string buf; /**< A buffer to convert read strings. */
  bool eof = false,  /**< Stores whether the end of file was reached during the last call to nextChar. */
       nextEof = false;

public:
  InText() {buf.reserve(200);};

  /**
   * Resets theChar to be able to use the same instance of InText or InConfig
   * more than once.
   */
  void reset()
  {
    theChar = theNextChar = ' ';
    eof = nextEof = false;
  }

protected:
  /**
   * The function initializes the end-of-file flag.
   * It has to be called only once after the stream was initialized.
   * @param stream The stream.
   */
  virtual void initEof(PhysicalInStream& stream)
  {
    eof = nextEof = stream.getEof();
    if(stream.exists())
      nextChar(stream);
  }

  /**
   * The function returns whether the end of stream has been reached.
   * If this function returns false, "theChar" is valid, otherwise it is not.
   * @param stream The stream.
   * @return End of stream reached?
   */
  bool isEof(const PhysicalInStream& stream) const override { return eof; }

  /**
   * reads a bool from a stream
   * @param d the data to read from the stream
   * @param stream the stream to read from
   */
  void readBool(bool& d, PhysicalInStream& stream) override;

  /**
   * reads a character from a stream
   * @param d the data to read from the stream
   * @param stream the stream to read from
   */
  void readChar(char& d, PhysicalInStream& stream) override;

  /**
   * reads a signed character from a stream
   * @param d the data to read from the stream
   * @param stream the stream to read from
   */
  void readSChar(signed char& d, PhysicalInStream& stream) override;

  /**
   * reads an unsigned character from a stream
   * @param d the data to read from the stream
   * @param stream the stream to read from
   */
  void readUChar(unsigned char& d, PhysicalInStream& stream) override;

  /**
   * reads a short from a stream
   * @param d the data to read from the stream
   * @param stream the stream to read from
   */
  void readShort(short& d, PhysicalInStream& stream) override;

  /**
   * reads an unsigned short from a stream
   * @param d the data to read from the stream
   * @param stream the stream to read from
   */
  void readUShort(unsigned short& d, PhysicalInStream& stream) override;

  /**
   * reads an int from a stream
   * @param d the data to read from the stream
   * @param stream the stream to read from
   */
  void readInt(int& d, PhysicalInStream& stream) override;

  /**
   * reads an unsigned int from a stream
   * @param d the data to read from the stream
   * @param stream the stream to read from
   */
  void readUInt(unsigned int& d, PhysicalInStream& stream) override;

  /**
   * reads a float from a stream
   * @param d the data to read from the stream
   * @param stream the stream to read from
   */
  void readFloat(float& d, PhysicalInStream& stream) override;

  /**
   * reads a double from a stream
   * @param d the data to read from the stream
   * @param stream the stream to read from
   */
  void readDouble(double& d, PhysicalInStream& stream) override;

  /**
   * The function reads a string from a stream.
   * It skips all whitespace characters, and then reads
   * a sequence of non-whitespace characters to a buffer, until it
   * again recognizes a whitespace.
   * @param d The value that is read.
   * @param stream the stream to read from
   */
  void readString(std::string& d, PhysicalInStream& stream) override;

  /**
   * reads a float from a stream
   * @param d the data to read from the stream
   * @param stream the stream to read from
   */
  void readAngle(Angle& d, PhysicalInStream& stream) override;

  /**
   * reads the 'end of line' from a stream
   * @param stream the stream to read from
   */
  void readEndl(PhysicalInStream& stream) override {}

  /**
   * The function determines whether the current character is a whitespace.
   */
  virtual bool isWhitespace();

  /**
   * The function skips the whitespace.
   */
  virtual void skipWhitespace(PhysicalInStream& stream);

  /**
   * The function reads the next character from the stream.
   */
  virtual void nextChar(PhysicalInStream& stream)
  {
    if(!eof)
    {
      eof = nextEof;
      theChar = theNextChar;
      if(stream.getEof())
      {
        nextEof = true;
        theNextChar = ' ';
      }
      else
        stream.readFromStream(&theNextChar, 1);
    }
  }

  /**
   * The function reads a number of bytes from the file.
   * @param p The address the data is written to. Note that p
   *          must point to a memory area that is at least
   *          "size" bytes large.
   * @param size The number of bytes to be read.
   * @param stream The stream to read from.
   */
  void readData(void* p, size_t size, PhysicalInStream& stream) override;

private:
  /**
   * Tries to read the given string from the stream.
   * @param str The string which is expected.
   * @param stream The stream to read from.
   * @return true if the string could be read from the stream.
   */
  bool expectString(const std::string& str, PhysicalInStream& stream);
};

/**
 * The class InConfig reads text data from config (file) streams
 * that contain comments and sections.
 * The following comment styles are supported:
 * / * comment * / (ignore the space between "*" and "/")
 * // comment till endl
 * # comment till endl
 * Note that "/" is not allowed elsewhere in the stream.
 */
class InConfig : public InText
{
private:
  bool readSection = false;  ///< Are we reading a section?

protected:
  /**
   * Creates the reader.
   * @param sectionName If given the section is searched
   * @param stream The medium that should be read from.
   */
  void create(const std::string& sectionName, PhysicalInStream& stream);

  /**
   * The function determines whether the current character is a whitespace.
   * In this context, the start of
   */
  bool isWhitespace() override;

  /**
   * The function skips the whitespace.
   */
  void skipWhitespace(PhysicalInStream& stream) override;

  /**
   * The function reads the next character from the stream.
   */
  void nextChar(PhysicalInStream& stream) override;

private:
  /**
   * The functions skip all characters to the end of the line.
   */
  void skipLine(PhysicalInStream& stream);

  /**
   * The functions skip all characters to the end of the comment.
   */
  void skipComment(PhysicalInStream& stream);
};

/**
 * @class InBinary
 *
 * Formatted reading of binary data to be used in streams.
 * The physical reading is done by PhysicalInStream derivates.
 */
class InBinary : public StreamReader
{
protected:
  /**
   * The function returns whether the end of stream has been reached.
   * @return End of stream reached?
   */
  bool isEof(const PhysicalInStream& stream) const override { return stream.getEof(); }

  /**
   * The function reads a bool from the stream.
   * @param d The value that is read.
   * @param stream A stream to read from.
   */
  void readBool(bool& d, PhysicalInStream& stream) override
  {
    char c;
    stream.readFromStream(&c, sizeof(d));
    d = c != 0;
  }

  /**
   * The function reads a char from the stream.
   * @param d The value that is read.
   * @param stream A stream to read from.
   */
  void readChar(char& d, PhysicalInStream& stream) override
  {
    stream.readFromStream(&d, sizeof(d));
  }

  /**
   * The function reads an signed char from the stream.
   * @param d The value that is read.
   * @param stream A stream to read from.
   */
  void readSChar(signed char& d, PhysicalInStream& stream) override
  {
    stream.readFromStream(&d, sizeof(d));
  }

  /**
   * The function reads an unsigned char from the stream.
   * @param d The value that is read.
   * @param stream A stream to read from.
   */
  void readUChar(unsigned char& d, PhysicalInStream& stream) override
  {
    stream.readFromStream(&d, sizeof(d));
  }

  /**
   * The function reads a short int from the stream.
   * @param d The value that is read.
   * @param stream A stream to read from.
   */
  void readShort(short& d, PhysicalInStream& stream) override
  {
    stream.readFromStream(&d, sizeof(d));
  }

  /**
   * The function reads an unsigned short int from the stream.
   * @param d The value that is read.
   * @param stream A stream to read from.
   */
  void readUShort(unsigned short& d, PhysicalInStream& stream) override
  {
    stream.readFromStream(&d, sizeof(d));
  }

  /**
   * The function reads an int from the stream.
   * @param d The value that is read.
   * @param stream A stream to read from.
   */
  void readInt(int& d, PhysicalInStream& stream) override
  {
    stream.readFromStream(&d, sizeof(d));
  }

  /**
   * The function reads an unsigned int from the stream.
   * @param d The value that is read.
   * @param stream A stream to read from.
   */
  void readUInt(unsigned int& d, PhysicalInStream& stream) override
  {
    stream.readFromStream(&d, sizeof(d));
  }

  /**
   * The function reads a float from the stream.
   * @param d The value that is read.
   * @param stream A stream to read from.
   */
  void readFloat(float& d, PhysicalInStream& stream) override
  {
    stream.readFromStream(&d, sizeof(d));
  }

  /**
   * The function reads a double from the stream.
   * @param d The value that is read.
   * @param stream A stream to read from.
   */
  void readDouble(double& d, PhysicalInStream& stream) override
  {
    stream.readFromStream(&d, sizeof(d));
  }

  /**
   * The function reads a string from the stream.
   * @param d The value that is read.
   * @param stream A stream to read from.
   */
  void readString(std::string& d, PhysicalInStream& stream) override
  {
    size_t size = 0;
    stream.readFromStream(&size, sizeof(unsigned));
    d.resize(size);
    if(size)
      stream.readFromStream(&d[0], size);
  }

  /**
   * The function reads an Angle from the stream.
   * @param d The value that is read.
   * @param stream A stream to read from.
   */
  void readAngle(Angle& d, PhysicalInStream& stream) override;

  /**
   * The function is intended to read an endl-symbol from the stream.
   * In fact, the function does nothing.
   * @param stream A stream to read from.
   */
  void readEndl(PhysicalInStream& stream) override {}

  /**
   * The function reads a number of bytes from a stream.
   * @param p The address the data is written to. Note that p
   *          must point to a memory area that is at least
   *          "size" bytes large.
   * @param size The number of bytes to be read.
   * @param stream A stream to read from.
   */
  void readData(void* p, size_t size, PhysicalInStream& stream) override
  {
    stream.readFromStream(p, size);
  }

  /**
   * The function skips a number of bytes in the file.
   * @param size The number of bytes to be skipped.
   * @param stream The stream to read from.
   */
  void skipData(size_t size, PhysicalInStream& stream) override
  {
    stream.skipInStream(size);
  }
};

/**
 * @class InFile
 *
 * An PhysicalInStream that reads the data from a file.
 */
class InFile : public PhysicalInStream
{
private:
  File* stream = nullptr; /**< Object representing the file. */

public:
  InFile() = default;

  /** No copy constructor. */
  InFile(const InFile&) = delete;

  /**
   * Move copy constructor.
   * @param other The stream the contents of which are moved to this one.
   */
  InFile(InFile&& other) noexcept : stream(other.stream) {other.stream = nullptr;}

  /** Destructor. */
  ~InFile();

  /** No assignment operator. */
  InFile& operator=(const InFile&) = delete;

  /**
   * Move assignment operator.
   * @param other The stream the contents of which are moved to this one.
   * @return This stream.
   */
  InFile& operator=(InFile&& other);

  /**
   * The function states whether the file actually exists.
   * @return Does the file exist?
   */
  virtual bool exists() const;

  /**
   * The function states whether the end of the file has been reached.
   * @return End of file reached?
   */
  virtual bool getEof() const;

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
   * opens the file.
   * @param name The name of the file to open. It will be interpreted
   *             as relative to the configuration directory.
   */
  void open(const std::string& name);

  /**
   * The function reads a number of bytes from the file.
   * @param p The address the data is written to. Note that p
   *          must point to a memory area that is at least
   *          "size" bytes large.
   * @param size The number of bytes to be read.
   */
  virtual void readFromStream(void* p, size_t size);
};

/**
 * @class InMemory
 *
 * An PhysicalInStream that reads the data from a memory region.
 */
class InMemory : public PhysicalInStream
{
private:
  const char* memory = nullptr, /**< Points to the next byte to read from memory. */
            * end = nullptr; /**< Points to the end of the memory block. */

public:
  /**
   * The function states whether the stream actually exists.
   * @return Does the stream exist? This is always true for memory streams.
   */
  bool exists() const override {return (memory != nullptr);}

  /**
   * The function states whether the end of the file has been reached.
   * It will only work if the correct size of the memory block was
   * specified during the construction of the stream.
   * @return End of file reached?
   */
  bool getEof() const override
  {
    return memory != nullptr && memory >= end;
  }

protected:
  /**
   * Opens the stream.
   * @param mem The address of the memory block from which is read.
   * @param size The size of the memory block. It is only used to
   *             implement the function eof(). If the size is not
   *             specified, eof() will always return true, but reading
   *             from the stream is still possible.
   */
  void open(const void* mem, size_t size = 0)
  {
    if(memory == nullptr)
    {
      memory = reinterpret_cast<const char*>(mem);
      end = memory + size;
    }
  }

  /**
   * The function reads a number of bytes from memory.
   * @param p The address the data is written to. Note that p
   *          must point to a memory area that is at least
   *          "size" bytes large.
   * @param size The number of bytes to be read.
   */
  void readFromStream(void* p, size_t size) override;

  /**
   * The function skips a number of bytes.
   * @param size The number of bytes to be skipped.
   */
  void skipInStream(size_t size) override {memory += size;}
};

/**
 * @class InBinaryFile
 *
 * A binary stream from a file.
 */
class InBinaryFile : public InStream<InFile, InBinary>
{
public:
  /**
   * Constructor.
   * @param name The name of the file to open. It will be interpreted
   *             as relative to the configuration directory.
   */
  InBinaryFile(const std::string& name) {open(name);}

  /**
   * The function returns whether this is a binary stream.
   * @return Does it output data in binary format?
   */
  bool isBinary() const override {return true;}
};

/**
 * @class InBinaryMemory
 *
 * A Binary Stream from a memory region.
 */
class InBinaryMemory : public InStream<InMemory, InBinary>
{
public:
  /**
   * Constructor.
   * @param mem The address of the memory block from which is read.
   * @param size The size of the memory block. It is only used to
   *             implement the function eof(). If the size is not
   *             specified, eof() will always return true, but reading
   *             from the stream is still possible.
   */
  InBinaryMemory(const void* mem, size_t size = 0) {open(mem, size);}

  /**
   * The function returns whether this is a binary stream.
   * @return Does it output data in binary format?
   */
  bool isBinary() const override {return true;}
};

/**
 * @class InTextFile
 *
 * A binary stream from a file.
 */
class InTextFile : public InStream<InFile, InText>
{
public:
  /**
   * Constructor.
   * @param name The name of the file to open. It will be interpreted
   *             as relative to the configuration directory.
   */
  InTextFile(const std::string& name)
  {
    open(name);
    initEof(*this);
  }
};

/**
 * @class InTextMemory
 *
 * A Binary Stream from a memory region.
 */
class InTextMemory : public InStream<InMemory, InText>
{
public:
  /**
   * Constructor.
   * @param mem The address of the memory block from which is read.
   * @param size The size of the memory block. It is only used to
   *             implement the function eof().
   */
  InTextMemory(const void* mem, size_t size)
  {
    open(mem, size);
    initEof(*this);
  }
};

/**
 * @class InConfigFile
 *
 * A config-file-style-formatted text stream from a file.
 */
class InConfigFile : public InStream<InFile, InConfig>
{
public:
  /**
   * Constructor.
   * @param name The name of the file to open. It will be interpreted
   *             as relative to the configuration directory. Note that
   *             the file is treated as binary file, in order
   *             to gain the same results on all supported platforms.
   * @param sectionName If given the section is searched
   */
  InConfigFile(const std::string& name, const std::string& sectionName = "")
  {
    open(name);
    initEof(*this);
    create(sectionName, *this);
  }
};

/**
 * @class InConfigMemory
 *
 * A config-file-style-formatted text stream from a memory region.
 */
class InConfigMemory : public InStream<InMemory, InConfig>
{
public:
  /**
   * Constructor.
   * @param mem The address of the memory block from which is read.
   * @param size The size of the memory block. It is only used to
   *             implement the function eof(). If the size is not
   *             specified, eof() will always return true, but reading
   *             from the stream is still possible.
   * @param sectionName If given the section is searched
   */
  InConfigMemory(const void* mem, size_t size = 0, const std::string& sectionName = "")
  {
    open(mem, size);
    initEof(*this);
    create(sectionName, *this);
  }
};

/**
 * @class InMap
 *
 * A stream that reads data from a text in config map format.
 */
class InMap : public In
{
private:
  /**
   * An entry representing a position in the ConfigMap.
   */
  class Entry
  {
  public:
    const char* key; /**< The name of the current key (used by printError()). */
    const SimpleMap::Value* value; /**< The current value in the map. */
    int type; /**< The type of the entry. -2: value or record, -1: array , >= 0: array element index. */
    const char* enumType; /**< The type of the elements as string if it is an enum. Otherwise nullptr. */

    Entry(const char* key, const SimpleMap::Value* value, int type, const char* enumType) :
      key(key), value(value), type(type), enumType(enumType)
    {}
  };

  SimpleMap* map = nullptr; /**< The configuration map that was read. */
  std::string name; /**< The name of the opened file. */
  std::vector<Entry> stack; /**< The hierarchy of values to read. */
  bool showErrors; /**< Show error messages if specification does not match. */

  /**
   * The method OUTPUTs an error message.
   * @param msg The error message.
   */
  void printError(const std::string& msg);

  /**
   * The method reads an entry from the config map.
   * The entry has been selected by select() before.
   * @param value The value that is read.
   */
  template<typename T> void in(T& value)
  {
    Entry& e = stack.back();
    if(e.value)
    {
      const SimpleMap::Literal* literal = dynamic_cast<const SimpleMap::Literal*>(e.value);
      if(literal)
      {
        In& stream = *literal;
        stream >> value;
        if(!stream.eof())
          printError("wrong format");
      }
      else
        printError("literal expected");
    }
  }

protected:
  /**
   * Constructor.
   * @param showErrors Show error messages if specification does not match.
   */
  InMap(bool showErrors) : showErrors(showErrors) {}

  ~InMap()
  {
    if(map != nullptr)
      delete map;
  }

  /** No assignment operator. */
  InMap& operator=(const InMap&) = delete;

  /**
   * Parse the stream.
   * @param stream The stream to read from.
   * @param name The name of the map if it is a file.
   */
  void parse(In& stream, const std::string& name = "");

  /**
   * Virtual redirection for operator>>(bool& value).
   */
  void inBool(bool& value) override {in(value);}

  /**
   * Virtual redirection for operator>>(char& value).
   */
  void inChar(char& value) override;

  /**
   * Virtual redirection for operator>>(unsigned char& value).
   */
  void inSChar(signed char& value) override;

  /**
   * Virtual redirection for operator>>(unsigned char& value).
   */
  void inUChar(unsigned char& value) override;

  /**
   * Virtual redirection for operator>>(short& value).
   */
  void inShort(short& value) override {in(value);}

  /**
   * Virtual redirection for operator>>(unsigned short& value).
   */
  void inUShort(unsigned short& value) override {in(value);}

  /**
   * Virtual redirection for operator>>(int& value).
   */
  void inInt(int& value) override;

  /**
   * Virtual redirection for operator>>(unsigned int& value).
   */
  void inUInt(unsigned int& value) override;

  /**
   * Virtual redirection for operator>>(float& value).
   */
  void inFloat(float& value) override {in(value);}

  /**
   * Virtual redirection for operator>>(double& value).
   */
  void inDouble(double& value) override {in(value);}

  /**
   * Virtual redirection for operator>>(std::string& value).
   */
  void inString(std::string& value) override {in(value);}

  /**
   * Virtual redirection for operator>>(Angle& value).
   */
  void inAngle(Angle& value) override {in(value);}

  /**
   * Virtual redirection for operator>>(In& (*f)(In&)) that reads
   * the symbol "endl";
   */
  void inEndL() override {}

public:
  /**
   * The function reads a number of bytes from a stream.
   * Not allowed for this stream!
   * @param p The address the data is written to. Note that p
   *          must point to a memory area that is at least
   *          "size" bytes large.
   * @param size The number of bytes to be read.
   */
  void read(void* p, size_t size) override;

  /**
   * The function skips a number of bytes in a stream.
   * Not allowed for this stream!
   * @param size The number of bytes to be skipped.
   */
  void skip(size_t size) override;

  /**
   * Select an entry for reading.
   * @param name The name of the entry if type == -2, otherwise 0.
   * @param type The type of the entry.
   *             -2: value or record,
   *             -1: array,
   *             >= 0: array element index.
   * @param enumType The element type as string if it is an enum. Otherwise nullptr.
   */
  void select(const char* name, int type, const char* enumType) override;

  /**
   * Deselects a field for reading.
   */
  void deselect() override;

  /**
   * Determines whether the end of file has been reached.
   * This is only the case if the file does not exist or
   * reading failed.
   */
  bool eof() const override {return (const SimpleMap::Value*) *map == 0;}

  friend class DebugDataStreamer; // needs access to printError to report suppressible error message
};

/**
 * @class InMapFile
 *
 * A stream that reads data from a text file in config map format.
 */
class InMapFile : public InMap
{
private:
  InBinaryFile stream;

public:
  /**
   * Constructor.
   * @param name The name of the config file to read.
   * @param showErrors Show error messages if specification does not match.
   */
  InMapFile(const std::string& name, bool showErrors = true);

  /**
   * The function states whether this stream actually exists.
   * @return Does the stream exist?
   */
  bool exists() const {return stream.exists();}
};

/**
 * @class InMapMemory
 *
 * A stream that reads data from memory in config map format.
 */
class InMapMemory : public InMap
{
private:
  InBinaryMemory stream;

public:
  /**
   * Constructor.
   * @param memory The block of memory to read from.
   * @param size The size of the memory block to read from.
   * @param showErrors Show error messages if specification does not match.
   */
  InMapMemory(const void* memory, size_t size, bool showErrors = true);
};
