/**
 * @file CompressedTeamCommunicationStreams.h
 *
 * This file declares streams for compressed team communication.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Tools/Math/Constants.h"
#include "Tools/Streams/InOut.h"
#include <cstdint>
#include <functional>
#include <memory>
#include <stack>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace CompressedTeamCommunication
{
  /**
   * Transforms a fully qualified C++ type name into a Java type name (basically replaced colons by underscores).
   * @param name The C++ type name.
   * @return The Java type name.
   */
  std::string transformTypeName(const std::string& name);

  struct Type
  {
    /** Virtual destructor for polymorphism. */
    virtual ~Type() = default;

    /**
     * Checks whether the type is compatible with the C++ type.
     * @param type The string representation of a C++ type from the \c TypeInfo.
     * @return Whether the type is compatible with the C++ type.
     */
    virtual bool check(const std::string& type) const = 0;

    /**
     * Writes a Java type for this type to a stream.
     * @param stream The stream to which the type should be written.
     * @param cxxType The string representation of a C++ type from the \c TypeInfo.
     * @param boxed Whether the type must be a boxed type (i.e. no basic type).
     */
    virtual void javaType(Out& stream, const std::string& cxxType, bool boxed = false) const = 0;

    /**
     * Appends an initialization statement to a stream (including the initial equals character, but excluding the final semicolon).
     * @param stream The stream to which the initialization statement should be written.
     * @param cxxType The string representation of a C++ type from the \c TypeInfo.
     */
    virtual void javaInitialize(Out& stream, const std::string& cxxType) const;

    /**
     * Writes a list of Java statements to read an object to a stream.
     * @param stream The stream to which the Java code should be written.
     * @param cxxType The string representation of a C++ type from the \c TypeInfo.
     * @param identifier The Java identifier of the object to be read.
     * @param indentation The indentation per line for this code fragment.
     */
    virtual void javaRead(Out& stream, const std::string& cxxType, const std::string& identifier, const std::string& indentation) const = 0;
  };

  inline void Type::javaInitialize(Out&, const std::string&) const {}

  struct RecordType : Type
  {
    bool check(const std::string& type) const override;
    void javaType(Out& stream, const std::string& cxxType, bool boxed) const override;
    void javaInitialize(Out& stream, const std::string& cxxType) const override;
    void javaRead(Out& stream, const std::string&, const std::string& identifier, const std::string& indentation) const override;

    std::string name; /**< The name of the record type. */
    std::unordered_map<std::string, const Type*> members; /**< The members of the record and their types. */
  };

  struct ArrayType : Type
  {
    bool check(const std::string& type) const override;
    void javaType(Out& stream, const std::string& cxxType, bool boxed) const override;
    void javaInitialize(Out& stream, const std::string& cxxType) const override;
    void javaRead(Out& stream, const std::string& cxxType, const std::string& identifier, const std::string& indentation) const override;

    const Type* element = nullptr; /**< The type of the array elements. */
    unsigned int lowerCount = 0; /**< The lowest possible number of elements in the array. */
    unsigned int upperCount = 0; /**< The highest possible number of elements in the array. */
    unsigned int bits = 0; /**< The number of bits needed to encode the number of elements. */
  };

  struct BooleanType : Type
  {
    bool check(const std::string& type) const override;
    void javaType(Out& stream, const std::string& cxxType, bool boxed) const override;
    void javaRead(Out& stream, const std::string& cxxType, const std::string& identifier, const std::string& indentation) const override;
  };

  struct IntegerType : Type
  {
    bool check(const std::string& type) const override;
    void javaType(Out& stream, const std::string& cxxType, bool boxed) const override;
    void javaRead(Out& stream, const std::string& cxxType, const std::string& identifier, const std::string& indentation) const override;

    std::int64_t min = 0; /**< The minimum value the integer can have. */
    std::int64_t max = 0; /**< The maximum value the integer can have. */
    unsigned int bits = 0; /**< The number of bits needed to encode the integer (or 0 if it should be encoded in its native size). */
  };

  struct AngleType : Type
  {
    bool check(const std::string& type) const override;
    void javaType(Out& stream, const std::string& cxxType, bool boxed) const override;
    void javaRead(Out& stream, const std::string& cxxType, const std::string& identifier, const std::string& indentation) const override;

    unsigned int bits = 0; /**< The number of bits used to encode the angle (or 0 if it should be encoded in its native size). */
  };

  struct FloatType : Type
  {
    FloatType() = default;
    FloatType(const AngleType& type) :
      min(-Constants::pi), max(Constants::pi), bits(type.bits)
    {}

    bool check(const std::string& type) const override;
    void javaType(Out& stream, const std::string& cxxType, bool boxed) const override;
    void javaRead(Out& stream, const std::string& cxxType, const std::string& identifier, const std::string& indentation) const override;

    double min = 0; /**< The minimum value the float can have. */
    double max = 0; /**< The maximum value the float can have. */
    unsigned int bits = 0; /**< The number of bits used to encode the float (or 0 if it should be encoded in its native size). */
  };

  struct TimestampType : Type
  {
    enum Reference
    {
      absolute, /**< The timestamp is absolute. */
      relativePast, /**< The timestamp is encoded as being relative to the timestamp of the message and in the past. */
      relativeFuture /**< The timestamp is encoded as being relative to the timestamp of the message and in the future. */
    };

    bool check(const std::string& type) const override;
    void javaType(Out& stream, const std::string& cxxType, bool boxed) const override;
    void javaRead(Out& stream, const std::string& cxxType, const std::string& identifier, const std::string& indentation) const override;

    Reference reference = absolute; /**< The reference system in which the timestamp is encoded. */
    unsigned int bits = 0; /**< The number of bits used to encode the timestamp. */
    unsigned int shift = 0; /**< The amount by which the timestamp is shifted during encoding/decoding. */
    bool noclip = false; /**< Whether the timestamp should not be clipped while reading but instead set to its maximum/minimum value. */
  };

  struct EnumType : Type
  {
    bool check(const std::string& type) const override;
    void javaType(Out& stream, const std::string& cxxType, bool boxed) const override;
    void javaRead(Out& stream, const std::string& cxxType, const std::string& identifier, const std::string& indentation) const override;

    std::string name; /**< The name of the C++ enum type. */
    unsigned int bits = 0; /**< The number of bits needed to encode the enum (or 0 if it should be encoded in its native size). */
  };

  struct VectorType : Type
  {
    bool check(const std::string& type) const override;
    void javaType(Out& stream, const std::string& cxxType, bool boxed) const override;
    void javaInitialize(Out& stream, const std::string& cxxType) const override;
    void javaRead(Out& stream, const std::string& cxxType, const std::string& identifier, const std::string& indentation) const override;

    const Type* elementType = nullptr; /**< The type of the vector elements. */
    unsigned int elements = 0; /**< The number of elements (rows/columns) of the vector. */
  };

  struct MatrixType : Type
  {
    bool check(const std::string& type) const override;
    void javaType(Out& stream, const std::string& cxxType, bool boxed) const override;
    void javaInitialize(Out& stream, const std::string& cxxType) const override;
    void javaRead(Out& stream, const std::string& cxxType, const std::string& identifier, const std::string& indentation) const override;

    const Type* elementType = nullptr; /**< The type of the matrix elements. */
    unsigned int rows = 0; /**< The number of rows of the matrix. */
    unsigned int columns = 0; /**< The number of columns of the matrix. */
    bool symmetric = false; /**< Whether the matrix is known to be symmetric. */
  };

  class TypeRegistry
  {
  public:
    /** Compiles the type hierarchy. */
    void compile();

    /**
     * Returns a handle to a user-defined type.
     * @param name The name which to resolve.
     * @return The type with the given name.
     */
    const Type* getTypeByName(const std::string& name) const;

    /**
     * Writes a Java class for the TCM plugin to a file.
     * @param file The path to the file where the plugin class should be generated.
     * @param type The type which should be the root of the class.
     */
    void generateTCMPluginClass(const std::string& file, const RecordType* type);

    /**
     * Adds types to the registry. The registry must then be compiled in order to get newly added types.
     * @param source The description of the types.
     */
    void addTypes(const std::string& source);

  private:
    using Parameter = std::pair<std::string, std::string>;

    struct TypeSpecification
    {
      std::string name; /**< The name of the type. */
      std::vector<TypeSpecification> typeParameters; /**< Type parameters to the type. */
      std::vector<Parameter> parameters; /**< Value parameters to the type. */
      unsigned int lowerCount = 0; /**< The minimum number of elements if this is an array. */
      unsigned int upperCount = 0; /**< The maximum number of elements (or 0 if this is not an array). */
    };

    struct MemberDeclaration
    {
      std::string name; /**< The name of the member. */
      TypeSpecification type; /**< Its type. */
    };

    struct TypeDeclaration
    {
      std::string name; /**< The name of the type. */
      std::vector<MemberDeclaration> members; /**< Its members. */
    };

    /**
     * Checks whether code for a type has already been generated and if not, does so.
     * @param out The stream to which potentially generated types are written.
     * @param type The type to ensure existence of.
     */
    void ensureTypeExists(Out& out, const Type* type);

    /**
     * Generates a Java enum.
     * @param out The stream to which the enum is written.
     * @param type The enum for which code should be generated.
     */
    void generateEnum(Out& out, const EnumType* type);

    /**
     * Generates a Java class from a record.
     * @param out The stream to which the class is written.
     * @param type The record for which code should be generated.
     */
    void generateRecord(Out& out, const RecordType* type);

    /**
     * Generates the member declarations for a record type.
     * @param out The stream to which the declarations are written.
     * @param type The record of which the members are declared.
     * @param indentation A string that is inserted in the beginning of each line.
     */
    void generateRecordMemberDeclarations(Out& out, const RecordType* type, const std::string& indentation) const;

    /**
     * Generates code to read the members of a record type.
     * @param out The stream to which the code is written.
     * @param type The record of which the code for reading its members is generated.
     * @param indentation A string that is inserted in the beginning of each line.
     */
    void generateRecordMemberReading(Out& out, const RecordType* type, const std::string& indentation) const;

    /** Parses a type declaration. */
    TypeDeclaration parseTypeDeclaration();

    /** Parses a member declaration. */
    MemberDeclaration parseMemberDeclaration();

    /** Parses a type specification. */
    TypeSpecification parseTypeSpecification();

    /** Parses a parameter. */
    Parameter parseParameter();

    /** Reads an identifier. */
    std::string readIdentifier();

    /** Reads a number. */
    unsigned int readNumber();

    /** Reads a value. */
    std::string readValue();

    /** Reads a token. */
    void readToken();

    /** Asserts that the current token has a certain value and then reads up to the next one. */
    void expectToken(char c);

    /** Reads until \c pointer points to a non-whitespace character. */
    void skipWhitespace();

    char token = '\0'; /**< The last read token. */
    const char* pointer = nullptr; /**< The pointer to the current position in the source code. */

    std::vector<TypeDeclaration> typeDeclarations; /**< The available type declarations. */
    std::vector<std::unique_ptr<Type>> typeContainer; /**< All types that were created during compilation of the type registry. */
    std::unordered_map<std::string, const Type*> typesByName; /**< Map from type names to type handles for user-defined types. */
    std::unordered_set<const Type*> tcmPluginGeneratedTypes; /**< The set of types for which code has already been generated (during TCM plugin generation). */
  };

  class Base
  {
  protected:
    struct Entry final
    {
      Entry(const Type* dataType, int type, const char* enumType) :
          dataType(dataType), type(type), enumType(enumType)
      {}

      const Type* dataType; /**< The compressed data type or \c nullptr if the element should not be communicated. */
      int type; /**< The type of the entry as in the \c type parameter of \c select. */
      const char* enumType; /**< The name of the enum type or \c nullptr. */
    };

    class EntryStack : public std::stack<Entry, std::vector<Entry>>
    {
    public:
      EntryStack() { c.reserve(20); }
    };

    struct MatrixStreaming final
    {
      unsigned int level = 0; /**< The nest level within the matrix streaming (0 means no matrix is currently being streamed). */
      unsigned int majorN; /**< The number of elements along the major axis (i.e. rows if row major and columns if column major). */
      unsigned int minorN; /**< The number of elements along the minor axis (i.e. columns if row major and rows if column major). */
      bool symmetric; /**< Whether the matrix currently being streamed is symmetric. */
      int firstIndex; /**< The index of the first selected slice (row or column), in order to avoid duplicate streaming in symmetric matrices. */
      std::vector<std::size_t> symmetricTriangleOffsets; /**< A buffer for the offsets where matrix elements of a triangle of the matrix begin. */
      std::size_t savedOffset; /**< The offset which has been replaced by an element of \c symmetricTriangleOffsets and should be restored after deselection. */

      /**
       * Calculates the index in \c symmetricTriangleOffsets from two matrix indices.
       * @param minIndex The lower index.
       * @param maxIndex The upper index (must be < \c N and > \c minIndex).
       * @param N The number of rows/columns in the matrix.
       * @return An index in [0, N * (N - 1) / 2].
       */
      static std::size_t triangleIndex(int minIndex, int maxIndex, unsigned int N)
      {
        // sum i=1 to minIndex (N - i) + (maxIndex - minIndex - 1)
        // minIndex * N - sum i=1 to minIndex i + (maxIndex - minIndex - 1)
        // minIndex * (N - 1) - sum i=1 to minIndex i + maxIndex - 1
        // minIndex * (N - 1) - (minIndex * (minIndex + 1)) / 2 + maxIndex - 1
        return minIndex * (N - 1) - (minIndex * (minIndex + 1)) / 2 + maxIndex - 1;
      }
    };

    Base(const Type* rootType, bool in, bool check = false) :
      check(check), in(in), rootType(rootType)
    {}

    /**
     * Select an entry for streaming.
     * @param name The name of the entry if type == -2, otherwise 0.
     * @param type The type of the entry.
     *             -2: value or record,
     *             -1: array or list.
     *             >= 0: array/list element index.
     * @param enumType The type as string if it is an enum. Otherwise nullptr.
     */
    void select(const char* name, int type, const char* enumType);

    /** Deselects an entry for streaming. */
    void deselect();

    EntryStack stack; /**< The type stack that is manipulated by \c select and \c deselect. */
    MatrixStreaming matrixStreaming; /**< Information about the matrix currently being streamed (if so), in order to avoid wild accesses across the stack. */
    const bool check; /**< Whether advanced checks should be done (only in Debug and Develop) that are too expensive to be done always. */
    std::size_t containerOffset = 0; /**< The current bit-offset in the container. */

  private:
    const bool in; /**< Whether the derived stream is a reading stream. */
    const Type* rootType; /**< The type of the object that is streamed into or out of this stream. */
  };
}

class CompressedTeamCommunicationIn : public In, public CompressedTeamCommunication::Base
{
public:
  /**
   * Constructor.
   * @param container The container from which the data should be read.
   * @param baseTimestamp The timestamp relative to which other timestamps are.
   * @param rootType The type of the object that will be streamed out of this stream.
   * @param toLocalTimestamp This function converts a read timtestamp to a local timestamp.
   */
  CompressedTeamCommunicationIn(const std::vector<std::uint8_t>& container, unsigned baseTimestamp, const CompressedTeamCommunication::Type* rootType,
                                const std::function<unsigned(unsigned)>& toLocalTimestamp = std::function<unsigned(unsigned)>());

private:
  const std::vector<std::uint8_t>& container; /**< The container from which to read. */
  unsigned baseTimestamp; /**< The timestamp relative to which other timestamps are. */
  std::function<unsigned(unsigned)> toLocalTimestamp; /**< This function converts a read timestamp to a local timestamp.  */

  /**
   * Reads data bitwise from the container.
   * @param data The destination of the data.
   * @param bits The number of bits to read.
   */
  void readBits(void* data, std::size_t bits);

  /**
   * Reads an integer from the container.
   * @tparam Integer The integer type.
   * @param value The destination value.
   * @param type The compression settings of the integer.
   */
  template<typename Integer>
  void readInteger(Integer& value, const CompressedTeamCommunication::IntegerType& type);

  /**
   * Reads a float from the container.
   * @tparam Float The floating point type.
   * @param value The destination value.
   * @param type The compression settings of the float.
   */
  template<typename Float>
  void readFloat(Float& value, const CompressedTeamCommunication::FloatType& type);

  void inBool(bool&) override;
  void inChar(char&) override;
  void inSChar(signed char&) override;
  void inUChar(unsigned char&) override;
  void inShort(short&) override;
  void inUShort(unsigned short&) override;
  void inInt(int&) override;
  void inUInt(unsigned int&) override;
  void inFloat(float&) override;
  void inDouble(double&) override;
  void inString(std::string&) override;
  void inAngle(Angle&) override;
  void inEndL() override {}

  void read(void* p, std::size_t size) override;
  void skip(std::size_t size) override;
  bool eof() const override;

  void select(const char* name, int type, const char* enumType = nullptr) override;
  void deselect() override;
};

class CompressedTeamCommunicationOut : public Out, public CompressedTeamCommunication::Base
{
public:
  /**
   * Constructor.
   * @param container The container to which the data should be written.
   * @param baseTimestamp The timestamp relative to which other timestamps are.
   * @param rootType The type of the object that will be streamed into this stream.
   * @param check Whether advanced checks should be done (only in Debug and Develop) that are too expensive to be done always.
   */
  CompressedTeamCommunicationOut(std::vector<std::uint8_t>& container, unsigned baseTimestamp, const CompressedTeamCommunication::Type* rootType, bool check = false);

private:
  std::vector<std::uint8_t>& container; /**< The container to which to write. */
  unsigned baseTimestamp; /**< The timestamp relative to which other timestamps are. */

  /**
   * Writes data bitwise to the container.
   * @param data The source of the data.
   * @param bits The number of bits to write.
   */
  void writeBits(const void* data, std::size_t bits);

  /**
   * Writes an integer to the container.
   * @tparam Integer The integer type.
   * @param value The value.
   * @param type The compression settings of the integer.
   */
  template<typename Integer>
  void writeInteger(Integer value, const CompressedTeamCommunication::IntegerType& type);

  /**
   * Writes a float to the container.
   * @tparam Float The floating point type.
   * @param value The value.
   * @param type The compression settings of the float.
   */
  template<typename Float>
  void writeFloat(Float value, const CompressedTeamCommunication::FloatType& type);

  void outBool(bool value) override;
  void outChar(char value) override;
  void outSChar(signed char value) override;
  void outUChar(unsigned char value) override;
  void outShort(short value) override;
  void outUShort(unsigned short value) override;
  void outInt(int value) override;
  void outUInt(unsigned int value) override;
  void outFloat(float value) override;
  void outDouble(double value) override;
  void outString(const char* value) override;
  void outAngle(const Angle& value) override;
  void outEndL() override {}

  void write(const void* p, std::size_t size) override;

  void select(const char* name, int type, const char* enumType = nullptr) override;
  void deselect() override;
};
