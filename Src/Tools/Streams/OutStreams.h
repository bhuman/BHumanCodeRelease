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
   * Writes an unsigned character to a stream.
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
   * Writes an unsigned short to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeUShort(unsigned short d, PhysicalOutStream& stream) = 0;

  /**
   * Writes an int to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  virtual void writeInt(int d, PhysicalOutStream& stream) = 0;

  /**
   * Writes an unsigned int to a stream.
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
template<typename S, typename W> class OutStream : public S, public W, public Out
{
public:
  /**
   * The function writes a number of bytes into a stream.
   * @param p The address the data is located at.
   * @param size The number of bytes to be written.
   */
  void write(const void* p, size_t size) override
  {
    W::writeData(p, size, *this);
  }

protected:
  /**
   * Virtual redirection for operator<<(const bool& value).
   */
  void outBool(bool d) override
  {
    W::writeBool(d, *this);
  }

  /**
   * Virtual redirection for operator<<(const char& value).
   */
  void outChar(char d) override
  {
    W::writeChar(d, *this);
  }

  /**
   * Virtual redirection for operator<<(const signed char& value).
   */
  void outSChar(signed char d) override
  {
    W::writeSChar(d, *this);
  }

  /**
   * Virtual redirection for operator<<(const unsigned char& value).
   */
  void outUChar(unsigned char d) override
  {
    W::writeUChar(d, *this);
  }

  /**
   * Virtual redirection for operator<<(const short& value).
   */
  void outShort(short d) override
  {
    W::writeShort(d, *this);
  }

  /**
   * Virtual redirection for operator<<(const unsigned short& value).
   */
  void outUShort(unsigned short d) override
  {
    W::writeUShort(d, *this);
  }

  /**
   * Virtual redirection for operator<<(const int& value).
   */
  void outInt(int d) override
  {
    W::writeInt(d, *this);
  }

  /**
   * Virtual redirection for operator<<(const unsigned& value).
   */
  void outUInt(unsigned int d) override
  {
    W::writeUInt(d, *this);
  }

  /**
   * Virtual redirection for operator<<(const float& value).
   */
  void outFloat(float d) override
  {
    W::writeFloat(d, *this);
  }

  /**
   * Virtual redirection for operator<<(const double& value).
   */
  void outDouble(double d) override
  {
    W::writeDouble(d, *this);
  }

  /**
   * Virtual redirection for operator<<(const char* value).
   */
  void outString(const char* d) override
  {
    W::writeString(d, *this);
  }

  /**
   * Virtual redirection for operator<<(const Angle& value).
   */
  void outAngle(const Angle& d) override
  {
    W::writeAngle(d, *this);
  }

  /**
   * Virtual redirection for operator<<(Out& (*f)(Out&)) that writes
   * the symbol "endl";
   */
  void outEndL() override
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
  void writeBool(bool d, PhysicalOutStream& stream) override
  {
    char c = static_cast<char>(d);
    stream.writeToStream(&c, sizeof(c));
  }

  /**
   * Writes a character to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  void writeChar(char d, PhysicalOutStream& stream) override
  {
    stream.writeToStream(&d, sizeof(d));
  }

  /**
   * Writes a signed character to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  void writeSChar(signed char d, PhysicalOutStream& stream) override
  {
    stream.writeToStream(&d, sizeof(d));
  }

  /**
   * Writes an unsigned character to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  void writeUChar(unsigned char d, PhysicalOutStream& stream) override
  {
    stream.writeToStream(&d, sizeof(d));
  }

  /**
   * Writes a short to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  void writeShort(short d, PhysicalOutStream& stream) override
  {
    stream.writeToStream(&d, sizeof(d));
  }

  /**
   * Writes an unsigned short to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  void writeUShort(unsigned short d, PhysicalOutStream& stream) override
  {
    stream.writeToStream(&d, sizeof(d));
  }

  /**
   * Writes an int to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  void writeInt(int d, PhysicalOutStream& stream) override
  {
    stream.writeToStream(&d, sizeof(d));
  }

  /**
   * Writes an unsigned int to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  void writeUInt(unsigned int d, PhysicalOutStream& stream) override
  {
    stream.writeToStream(&d, sizeof(d));
  }

  /**
   * Writes a float to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  void writeFloat(float d, PhysicalOutStream& stream) override
  {
    stream.writeToStream(&d, sizeof(d));
  }

  /**
   * Writes a double to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  void writeDouble(double d, PhysicalOutStream& stream) override
  {
    stream.writeToStream(&d, sizeof(d));
  }

  /**
   * Writes a string to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  void writeString(const char* d, PhysicalOutStream& stream) override;

  /**
   * Writes an Angle to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  void writeAngle(const Angle& d, PhysicalOutStream& stream) override;

  /**
   * Writes a 'end of line' to a stream.
   * @param stream the stream to write on.
   */
  void writeEndL(PhysicalOutStream& stream) override {}

  /**
   * The function writes a number of bytes into the stream.
   * @param p The address the data is located at.
   * @param size The number of bytes to be written.
   * @param stream the stream to write on.
   */
  void writeData(const void* p, size_t size, PhysicalOutStream& stream) override
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
  void writeBool(bool d, PhysicalOutStream& stream) override;

  /**
   * Writes a character to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  void writeChar(char d, PhysicalOutStream& stream) override;

  /**
   * Writes a signed character to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  void writeSChar(signed char d, PhysicalOutStream& stream) override;

  /**
   * Writes an unsigned character to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  void writeUChar(unsigned char d, PhysicalOutStream& stream) override;

  /**
   * Writes a short to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  void writeShort(short d, PhysicalOutStream& stream) override;

  /**
   * Writes an unsigned short to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  void writeUShort(unsigned short d, PhysicalOutStream& stream) override;

  /**
   * Writes an int to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  void writeInt(int d, PhysicalOutStream& stream) override;

  /**
   * Writes an unsigned int to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  void writeUInt(unsigned int d, PhysicalOutStream& stream) override;

  /**
   * Writes a float to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  void writeFloat(float d, PhysicalOutStream& stream) override;

  /**
   * Writes a double to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  void writeDouble(double d, PhysicalOutStream& stream) override;

  /**
   * Writes a string to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  void writeString(const char* d, PhysicalOutStream& stream) override;

  /**
   * Writes an Angle to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  void writeAngle(const Angle& d, PhysicalOutStream& stream) override;

  /**
   * Writes a 'end of line' to a stream.
   * @param stream the stream to write on.
   */
  void writeEndL(PhysicalOutStream& stream) override;

  /**
   * The function writes a number of bytes into the stream.
   * @param p The address the data is located at.
   * @param size The number of bytes to be written.
   * @param stream the stream to write on.
   */
  void writeData(const void* p, size_t size, PhysicalOutStream& stream) override;
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
  void writeBool(bool d, PhysicalOutStream& stream) override;

  /**
   * Writes a character to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  void writeChar(char d, PhysicalOutStream& stream) override;

  /**
   * Writes a signed character to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  void writeSChar(signed char d, PhysicalOutStream& stream) override;

  /**
   * Writes an unsigned character to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  void writeUChar(unsigned char d, PhysicalOutStream& stream) override;

  /**
   * Writes a short to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  void writeShort(short d, PhysicalOutStream& stream) override;

  /**
   * Writes an unsigned short to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  void writeUShort(unsigned short d, PhysicalOutStream& stream) override;

  /**
   * Writes an int to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  void writeInt(int d, PhysicalOutStream& stream) override;

  /**
   * Writes an unsigned int to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  void writeUInt(unsigned int d, PhysicalOutStream& stream) override;

  /**
   * Writes a float to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  void writeFloat(float d, PhysicalOutStream& stream) override;

  /**
   * Writes a double to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  void writeDouble(double d, PhysicalOutStream& stream) override;

  /**
   * Writes a string to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  void writeString(const char* d, PhysicalOutStream& stream) override;

  /**
   * Writes an Angle to a stream.
   * @param d the data to write.
   * @param stream the stream to write on.
   */
  void writeAngle(const Angle& d, PhysicalOutStream& stream) override;

  /**
   * Writes a 'end of line' to a stream.
   * @param stream the stream to write on.
   */
  void writeEndL(PhysicalOutStream& stream) override;

  /**
   * The function writes a number of bytes into the stream.
   * @param p The address the data is located at.
   * @param size The number of bytes to be written.
   * @param stream the stream to write on.
   */
  void writeData(const void* p, size_t size, PhysicalOutStream& stream) override;
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
  OutFile() = default;

  /** No copy constructor. */
  OutFile(const OutFile&) = delete;

  /**
   * Move copy constructor.
   * @param other The stream the contents of which are moved to this one.
   */
  OutFile(OutFile&& other) noexcept : stream(other.stream) {other.stream = nullptr;}

  /** Destructor. */
  ~OutFile();

  /** No assignment operator. */
  OutFile& operator=(const OutFile&) = delete;

  /**
   * Move assignment operator.
   * @param other The stream the contents of which are moved to this one.
   * @return This stream.
   */
  OutFile& operator=(OutFile&& other);

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
   *             exists, its previous contents will be discarded.
   */
  void open(const std::string& name);

  /**
   * Opens the stream.
   * @param name The name of the file to open. It will be interpreted
   *             as relative to the configuration directory. If the file
   *             does not exist, it will be created. If it already
   *             exists, its previous contents will be preserved,
   *             if append = true.
   * @param append Determines, if the file content is preserved or discarded.
   */
  void open(const std::string& name, bool append);

  /**
   * The function writes a number of bytes into the file.
   * @param p The address the data is located at.
   * @param size The number of bytes to be written.
   */
  void writeToStream(const void* p, size_t size) override;
};

/**
 * @class OutMemory
 *
 * A  PhysicalOutStream that writes the data to a memory block.
 */
class OutMemory : public PhysicalOutStream
{
private:
  char* buffer = nullptr; /**< The buffer to which it is streamed. */
  size_t reserved = 0; /**< The current size of the buffer. */
  size_t bytes = 0; /**< The number of bytes already written. */
  bool dynamic = false; /**< Should the buffer grow dynamically? */

public:
  OutMemory() = default;

  /** No copy constructor. */
  OutMemory(const OutMemory&) = delete;

  /**
   * Move copy constructor.
   * @param other The stream the contents of which are moved to this one.
   */
  OutMemory(OutMemory&& other);

  /**
   * The destructor frees the memory buffer.
   */
  ~OutMemory() { if(dynamic && buffer) std::free(buffer); }

  /** No assignment operator. */
  OutMemory& operator=(const OutMemory&) = delete;

  /**
   * Move assignment operator.
   * @param other The stream the contents of which are moved to this one.
   * @return This stream.
   */
  OutMemory& operator=(OutMemory&& other);

  /**
   * Returns the number of written bytes.
   */
  size_t size() const { return bytes; }

  /**
   * Returns the address of the first byte.
   */
  const char* data() const { return buffer; }

  /**
   * Obtain ownership of the memory. The caller must free the memory.
   * This stream looses access to the memory.
   */
  char* obtainData();

protected:
  /**
   * Opens the stream.
   * @param capacity The number of bytes that are initially reserved in memory.
   *                 If the stream grows beyond this size, reallocations of
   *                 memory will take place.
   * @param buffer The buffer to which will be written. If nullptr is passed,
   *               a buffer with the defined capacity will be allocated instead.
   *               It will automatically be freed in the end if it is not
   *               obtained before destruction of this stream. If a buffer is
   *               passed, it must be at least of the size of the
   *               given capacity. It will not grow. The caller must free it
   *               after the destruction of this stream.
   */
  void open(size_t capacity, char* buffer)
  {
    reserved = capacity;
    dynamic = !buffer;
    this->buffer = dynamic ? reinterpret_cast<char*>(std::malloc(capacity)) : buffer;
  }

  /**
   * The function writes a number of bytes into memory.
   * @param p The address the data is located at.
   * @param size The number of bytes to be written.
   */
  void writeToStream(const void* p, size_t size) override;
};

/**
 * Special memory stream that terminates the data in memory with a zero byte.
 */
class OutMemoryForText : public OutMemory
{
  void addTerminatingZero();

public:
  /**
   * Returns the number of bytes in the memory block.
   * If necessary, the function also adds a zero byte to the end of the
   * memory block. Nothing should be written the the steam after this call
   * anymore.
   * @return The number of bytes in the memory block, not counting a
   *         terminating zero byte.
   */
  size_t size() const
  {
    const_cast<OutMemoryForText*>(this)->addTerminatingZero();
    return OutMemory::size() - 1;
  }

  /**
   * Returns the address of the first byte of the memory block.
   * If necessary, the function also adds a zero byte to the end of the
   * memory block. Nothing should be written the the steam after this call
   * anymore.
   * @return The address of the memory block.
   */
  const char* data() const
  {
    const_cast<OutMemoryForText*>(this)->addTerminatingZero();
    return OutMemory::data();
  }

  /**
   * Obtain ownership of the memory. The caller must free the memory.
   * This stream looses access to the memory.
   * If necessary, the function also adds a zero byte to the end of the
   * memory block.
   * @return The memory block the ownership of which is now with the caller.
   */
  char* obtainData()
  {
    addTerminatingZero();
    return OutMemory::obtainData();
  }
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
  bool isBinary() const override {return true;}
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
   * Opens the stream.
   * @param capacity The number of bytes that are initially reserved in memory.
   *                 If the stream grows beyond this size, reallocations of
   *                 memory will take place.
   * @param buffer The buffer to which will be written. If nullptr is passed,
   *               a buffer with the defined capacity will be allocated instead.
   *               It will automatically be freed in the end if it is not
   *               obtained before destruction of this stream. If a buffer is
   *               passed, it must be at least of the size of the
   *               given capacity. It will not grow. The caller must free it
   *               after the destruction of this stream.
   */
  OutBinaryMemory(size_t capacity = 1024, char* buffer = nullptr)
  {
    open(capacity, buffer);
  }

  /**
   * The function returns whether this is a binary stream.
   * @return Does it output data in binary format?
   */
  bool isBinary() const override {return true;}
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
class OutTextMemory : public OutStream<OutMemoryForText, OutText>
{
public:
  /**
   * Opens the stream.
   * @param capacity The number of bytes that are initially reserved in memory.
   *                 If the stream grows beyond this size, reallocations of
   *                 memory will take place.
   * @param buffer The buffer to which will be written. If nullptr is passed,
   *               a buffer with the defined capacity will be allocated instead.
   *               It will automatically be freed in the end if it is not
   *               obtained before destruction of this stream. If a buffer is
   *               passed, it must be at least of the size of the
   *               given capacity. It will not grow. The caller must free it
   *               after the destruction of this stream.
   */
  OutTextMemory(size_t capacity = 1024, char* buffer = nullptr)
  {
    open(capacity, buffer);
  }
};

/**
 * @class OutTextRawMemory
 *
 * A text stream into a memory region.
 */
class OutTextRawMemory : public OutStream<OutMemoryForText, OutTextRaw>
{
public:
  /**
   * Opens the stream.
   * @param capacity The number of bytes that are initially reserved in memory.
   *                 If the stream grows beyond this size, reallocations of
   *                 memory will take place.
   * @param buffer The buffer to which will be written. If nullptr is passed,
   *               a buffer with the defined capacity will be allocated instead.
   *               It will automatically be freed in the end if it is not
   *               obtained before destruction of this stream. If a buffer is
   *               passed, it must be at least of the size of the
   *               given capacity. It will not grow. The caller must free it
   *               after the destruction of this stream.
   */
  OutTextRawMemory(size_t capacity = 1024, char* buffer = nullptr)
  {
    open(capacity, buffer);
  }
};

/**
 * @class OutMap
 *
 * A stream that writes data in config map format.
 */
class OutMap : public Out
{
protected:
  enum Mode {singleLine, multiLine, singleLineInnermost};

private:
  /**
   * An entry representing the current state in the writing process.
   */
  class Entry
  {
  public:
    int type; /**< The type of the entry. -2: value or record, -1: array, >= 0: array element index. */
    const char* enumType; /**< The type of the elements as string if it is an enum. Otherwise nullptr. */
    bool hasSubEntries; /**< Has the current entry sub entries? */

    Entry(int type, const char* enumType) :
      type(type), enumType(enumType), hasSubEntries(false)
    {}
  };
  Out* stream; /**< The map that is currently written to. */
  Out& target; /** The actual target stream. */
  OutTextRawMemory buffer; /**< A buffer that will be used sometimes. */
  Mode mode; /**< How to structure the output into lines? */
  size_t maxCollapsedLength; /**< The maximum length of text collapsed to a single line. */
  std::vector<Entry> stack; /**< The hierarchy of values that are written. */
  std::string indentation; /**< The indentation as sequence of spaces. */

  /** Writes a line break or simply a space if singleLine is active. */
  void writeLn();

  /** Write buffered data to the actual target stream. */
  void flush(bool singleLine = false);

protected:
  /**
   * Writes to a stream in config map format.
   * @param stream The stream that is written to.
   * @param mode How to structure the output into lines?
   * @param maxCollapsedLength The maximum length of text on the
   *        innermost level collapsed to a single line.
   */
  OutMap(Out& stream, Mode mode, size_t maxCollapsedLength = 80);

  /** No assignment operator. */
  OutMap& operator=(const OutMap&) = delete;

  /** Helper to write a value. */
  template<typename T> void out(const T& value) {*stream << value;}

  void outBool(bool value) override {out(value);}
  void outChar(char value) override {out(static_cast<int>(value));}
  void outSChar(signed char value) override {out(static_cast<int>(value));}
  void outUChar(unsigned char value) override;
  void outShort(short value) override {out(value);}
  void outUShort(unsigned short value) override {out(value);}
  void outInt(int value) override {out(value);}
  void outUInt(unsigned int value) override;
  void outFloat(float value) override {out(value);}
  void outDouble(double value) override {out(value);}
  void outString(const char* value) override;
  void outAngle(const Angle& value) override {out(value);};
  void outEndL() override {}

public:
  /**
   * Select an entry for writing.
   * @param name The name of the entry if type == -2, otherwise 0.
   * @param type The type of the entry.
   *             -2: value or record,
   *             -1: array,
   *             >= 0: array element index.
   * @param enumType The type of the elements as string if it is an enum. Otherwise nullptr.
   */
  void select(const char* name, int type, const char* enumType) override;

  /** Deselects a field for writing. */
  void deselect() override;

  /** Writing raw data is not supported. Do not call. */
  void write(const void* p, size_t size) override;
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
   * @param singleLineInnermost Output the innermost level to a single line.
   * @param maxCollapsedLength The maximum length of text on the innermost
   *                           level collapsed to a single line.
   */
  OutMapFile(const std::string& name, bool singleLineInnermost = false, size_t maxCollapsedLength = 80);

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
   * @param singleLine Output to a single line.
   * @param capacity The number of bytes that are initially reserved in memory.
   *                 If the stream grows beyond this size, reallocations of
   *                 memory will take place.
   * @param buffer The buffer to which will be written. If nullptr is passed,
   *               a buffer with the defined capacity will be allocated instead.
   *               It will automatically be freed in the end if it is not
   *               obtained before destruction of this stream. If a buffer is
   *               passed, it must be at least of the size of the
   *               given capacity. It will not grow. The caller must free it
   *               after the destruction of this stream.
   */
  OutMapMemory(bool singleLine = false, size_t capacity = 1024, char* buffer = nullptr);

  /**
   * Returns the number of written bytes
   */
  size_t size() const {return stream.size();}

  /**
   * Returns the memory.
   */
  const char* data() {return stream.data();}

  /**
   * Obtain ownership of the memory. The caller must free the memory.
   * This stream looses access to the memory.
   * If necessary, the function also adds a zero byte to the end of the
   * memory block.
   * @return The memory block the ownership of which is now with the caller.
   */
  const char* obtainData() {return stream.obtainData();}
};
