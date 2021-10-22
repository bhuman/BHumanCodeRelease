/**
 * @file CompressedTeamCommunicationStreams.cpp
 *
 * This file implements streams for compressed team communication.
 *
 * @author Arne Hasselbring
 */

#include "CompressedTeamCommunicationStreams.h"
#include "Platform/BHAssert.h"
#include "Tools/Communication/BHumanTeamMessageParts/BHumanStandardMessage.h"
#include "Tools/Math/Angle.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/OutStreams.h"
#include "Tools/Streams/TypeInfo.h"
#include "Tools/Streams/TypeRegistry.h"
#include <algorithm>
#include <cstring>
#include <functional>
#include <memory>

std::string CompressedTeamCommunication::transformTypeName(const std::string& name)
{
  std::string copyOfName = name;
  for(char& c : copyOfName)
    if(c == ':')
      c = '_';
  return copyOfName;
}

bool CompressedTeamCommunication::RecordType::check(const std::string& type) const
{
  return type == name;
}

void CompressedTeamCommunication::RecordType::javaType(Out& stream, const std::string&, bool) const
{
  stream << name;
}

void CompressedTeamCommunication::RecordType::javaInitialize(Out& stream, const std::string&) const
{
  stream << " = new " << name << "()";
}

void CompressedTeamCommunication::RecordType::javaRead(Out& stream, const std::string&, const std::string& identifier, const std::string& indentation) const
{
  stream << indentation << identifier + ".read(bitStream, __timestampBase);" << endl;
}

bool CompressedTeamCommunication::ArrayType::check(const std::string& type) const
{
  if(type.empty())
    return false;
  if(type.back() == '*')
    return element->check(type.substr(0, type.size() - 1));
  else if(type.back() == ']')
  {
    if(lowerCount != upperCount)
      return false;
    const std::size_t bracket = type.find_last_of('[');
    if(static_cast<unsigned int>(std::stoul(type.substr(bracket + 1, type.size() - bracket - 2))) != upperCount)
      return false;
    return element->check(type.substr(0, bracket));
  }
  return false;
}

void CompressedTeamCommunication::ArrayType::javaType(Out& stream, const std::string& cxxType, bool) const
{
  if(bits)
    stream << "List<";
  element->javaType(stream, cxxType.substr(0, cxxType.back() == '*' ? (cxxType.size() - 1) : cxxType.find_last_of('[')), bits != 0);
  if(bits)
    stream << ">";
  else
    stream << "[]";
}

void CompressedTeamCommunication::ArrayType::javaInitialize(Out& stream, const std::string& cxxType) const
{
  if(bits)
    return;
  stream << " = new ";
  element->javaType(stream, cxxType.substr(0, cxxType.back() == '*' ? (cxxType.size() - 1) : cxxType.find_last_of('[')));
  stream << "[" << upperCount << "]";
}

void CompressedTeamCommunication::ArrayType::javaRead(Out& stream, const std::string& cxxType, const std::string& identifier, const std::string& indentation) const
{
  const std::string elementCXXType = cxxType.substr(0, cxxType.back() == '*' ? (cxxType.size() - 1) : cxxType.find_last_of('['));
  if(bits)
  {
    stream << indentation << "final int _" << identifier << "Size = (int) (bitStream.readBits(" << bits << ")";
    if(lowerCount)
      stream << " + " << lowerCount;
    stream << ");" << endl;
    stream << indentation << identifier << " = new ArrayList<>(_" << identifier << "Size);" << endl;
  }
  else
    stream << indentation << "final int _" << identifier << "Size = " << upperCount << ";" << endl;
  stream << indentation << "for (int i = 0; i < _" << identifier << "Size; ++i) {" << endl;
  stream << indentation << "    ";
  element->javaType(stream, elementCXXType);
  stream << " _" << identifier;
  element->javaInitialize(stream, elementCXXType);
  stream << ";" << endl;
  element->javaRead(stream, elementCXXType, "_" + identifier, indentation + "    ");
  if(bits)
    stream << indentation << "    " << identifier << ".add(_" << identifier << ");" << endl;
  else
    stream << indentation << "    " << identifier << "[i] = " << "_" << identifier << ";" << endl;
  stream << indentation << "}" << endl;
}

bool CompressedTeamCommunication::BooleanType::check(const std::string& type) const
{
  return type == "bool";
}

void CompressedTeamCommunication::BooleanType::javaType(Out& stream, const std::string&, bool boxed) const
{
  stream << (boxed ? "Boolean" : "boolean");
}

void CompressedTeamCommunication::BooleanType::javaRead(Out& stream, const std::string&, const std::string& identifier, const std::string& indentation) const
{
  stream << indentation << identifier << " = bitStream.readBoolean();" << endl;
}

bool CompressedTeamCommunication::IntegerType::check(const std::string& type) const
{
  return type == "char" || type == "signed char" || type == "unsigned char" || type == "short" || type == "unsigned short" || type == "int" || type == "unsigned int";
}

void CompressedTeamCommunication::IntegerType::javaType(Out& stream, const std::string& cxxType, bool boxed) const
{
  if(cxxType == "signed char")
    stream << (boxed ? "Byte" : "byte");
  else if(cxxType == "char" || cxxType == "unsigned char" || cxxType == "short")
    stream << (boxed ? "Short" : "short");
  else if(cxxType == "unsigned short" || cxxType == "int")
    stream << (boxed ? "Integer" : "int");
  else
    stream << (boxed ? "Long" : "long");
}

void CompressedTeamCommunication::IntegerType::javaRead(Out& stream, const std::string& cxxType, const std::string& identifier, const std::string& indentation) const
{
  stream << indentation << identifier << " = bitStream.read";
  if(cxxType == "char")
    stream << "Char";
  else if(cxxType == "signed char")
    stream << "SignedChar";
  else if(cxxType == "unsigned char")
    stream << "UnsignedChar";
  else if(cxxType == "short")
    stream << "Short";
  else if(cxxType == "unsigned short")
    stream << "UnsignedShort";
  else if(cxxType == "int")
    stream << "Int";
  else
    stream << "UnsignedInt";
  stream << "(" << std::to_string(min) << ", " << std::to_string(max) << ", " << bits << ");" << endl;
}

bool CompressedTeamCommunication::AngleType::check(const std::string& type) const
{
  return type == "Angle";
}

void CompressedTeamCommunication::AngleType::javaType(Out& stream, const std::string&, bool) const
{
  stream << "Angle";
}

void CompressedTeamCommunication::AngleType::javaRead(Out& stream, const std::string&, const std::string& identifier, const std::string& indentation) const
{
  stream << indentation << identifier << " = bitStream.readAngle(" << bits << ");" << endl;
}

bool CompressedTeamCommunication::FloatType::check(const std::string& type) const
{
  return type == "float" || type == "double";
}

void CompressedTeamCommunication::FloatType::javaType(Out& stream, const std::string& cxxType, bool boxed) const
{
  stream << (boxed ? static_cast<char>(cxxType[0] + 'A' - 'a') + cxxType.substr(1) : cxxType);
}

void CompressedTeamCommunication::FloatType::javaRead(Out& stream, const std::string& cxxType, const std::string& identifier, const std::string& indentation) const
{
  stream << indentation << identifier << " = bitStream.read";
  stream << (static_cast<char>(cxxType[0] + 'A' - 'a') + cxxType.substr(1));
  stream << "(" << min << ", " << max << ", " << bits << ");" << endl;
}

bool CompressedTeamCommunication::TimestampType::check(const std::string& type) const
{
  return type == "unsigned int";
}

void CompressedTeamCommunication::TimestampType::javaType(Out& stream, const std::string&, bool) const
{
  stream << "Timestamp";
}

void CompressedTeamCommunication::TimestampType::javaRead(Out& stream, const std::string&, const std::string& identifier, const std::string& indentation) const
{
  stream << indentation << identifier << " = bitStream.readTimestamp(__timestampBase, " << bits << ", " << shift << ", " <<
            (reference == relativePast ? -1 : (reference == relativeFuture ? 1 : 0)) << ", " << (noclip ? "true" : "false") << ");" << endl;
}

bool CompressedTeamCommunication::EnumType::check(const std::string& type) const
{
  return type == name;
}

void CompressedTeamCommunication::EnumType::javaType(Out& stream, const std::string&, bool) const
{
  stream << transformTypeName(name);
}

void CompressedTeamCommunication::EnumType::javaRead(Out& stream, const std::string&, const std::string& identifier, const std::string& indentation) const
{
  const std::string javaName = transformTypeName(name);
  stream << indentation << identifier << " = " << javaName << ".values()[Math.min((int) bitStream.readBits(" << (bits ? bits : static_cast<unsigned int>(sizeof(unsigned char) * 8)) << "), " << javaName << ".values().length - 1)];" << endl;
}

bool CompressedTeamCommunication::VectorType::check(const std::string& type) const
{
  // type should something like Eigen::Matrix<type,elements,1,0,elements,1> or Eigen::Matrix<type,1,elements,0,1,elements>.
  if(type.compare(0, 14, "Eigen::Matrix<") != 0)
    return false;
  // Yes, this breaks if the vector contains a template.
  const std::size_t comma = type.find_first_of(',', 14);
  const std::size_t secondComma = type.find_first_of(',', comma + 1);
  const unsigned int rows = static_cast<unsigned int>(std::stoul(type.substr(comma + 1, secondComma - comma - 1)));
  const std::size_t thirdComma = type.find_first_of(',', secondComma + 1);
  const unsigned int columns = static_cast<unsigned int>(std::stoul(type.substr(secondComma + 1, thirdComma - secondComma - 1)));
  if(!((rows == elements && columns == 1) || (rows == 1 && columns == elements)))
    return false;
  return elementType->check(type.substr(14, comma - 14));
}

void CompressedTeamCommunication::VectorType::javaType(Out& stream, const std::string& cxxType, bool) const
{
  const std::string elementCXXType = cxxType.substr(14, cxxType.find_first_of(',', 14) - 14);
  stream << "Eigen.Vector";
  if(elements == 2 || elements == 3)
    stream << elements;
  if(elements == 2 && elementCXXType == "float")
    stream << "f";
  else if(elements == 2 && elementCXXType == "short")
    stream << "s";
  else if(elements == 2 && elementCXXType == "int")
    stream << "i";
  else
  {
    stream << "<";
    elementType->javaType(stream, elementCXXType);
    stream << ">";
  }
}

void CompressedTeamCommunication::VectorType::javaInitialize(Out& stream, const std::string& cxxType) const
{
  stream << " = new ";
  javaType(stream, cxxType, false);
  stream << "()";
}

void CompressedTeamCommunication::VectorType::javaRead(Out& stream, const std::string& cxxType, const std::string& identifier, const std::string& indentation) const
{
  const std::string elementCXXType = cxxType.substr(14, cxxType.find_first_of(',', 14) - 14);
  if(elements == 2 || elements == 3)
  {
    elementType->javaRead(stream, elementCXXType, identifier + ".x", indentation);
    elementType->javaRead(stream, elementCXXType, identifier + ".y", indentation);
    if(elements == 3)
      elementType->javaRead(stream, elementCXXType, identifier + ".z", indentation);
  }
  else
  {
    stream << indentation << identifier << ".elems = new ";
    elementType->javaType(stream, elementCXXType, true);
    stream << "[" << elements << "];" << endl;
    stream << indentation << "for (int i = 0; i < " << elements << "; ++i) {" << endl;
    stream << indentation << "    ";
    elementType->javaType(stream, elementCXXType);
    stream << " _" << identifier;
    elementType->javaInitialize(stream, elementCXXType);
    stream << ";" << endl;
    elementType->javaRead(stream, elementCXXType, "_" + identifier, indentation + "    ");
    stream << indentation << "    " << identifier << ".elems[i] = _" << identifier << ";" << endl;
    stream << indentation << "}" << endl;
  }
}

bool CompressedTeamCommunication::MatrixType::check(const std::string& type) const
{
  // type should something like Eigen::Matrix<type,rows,columns,options,rows,columns>.
  if(type.compare(0, 14, "Eigen::Matrix<") != 0)
    return false;
  // Yes, this breaks if the matrix contains a template.
  const std::size_t comma = type.find_first_of(',', 14);
  const std::size_t secondComma = type.find_first_of(',', comma + 1);
  if(static_cast<unsigned int>(std::stoul(type.substr(comma + 1, secondComma - comma - 1))) != rows)
    return false;
  const std::size_t thirdComma = type.find_first_of(',', secondComma + 1);
  if(static_cast<unsigned int>(std::stoul(type.substr(secondComma + 1, thirdComma - secondComma - 1))) != columns)
    return false;
  return elementType->check(type.substr(14, comma - 14));
}

void CompressedTeamCommunication::MatrixType::javaType(Out& stream, const std::string& cxxType, bool) const
{
  const std::string elementCXXType = cxxType.substr(14, cxxType.find_first_of(',', 14) - 14);
  const int options = std::stoi(cxxType.substr(cxxType.find_first_of(',', cxxType.find_first_of(',', 14 + elementCXXType.length() + 1) + 1) + 1));
  stream << "Eigen.";
  if(options & Eigen::RowMajor)
    stream << "Row";
  else
    stream << "Column";
  stream << "Matrix<";
  elementType->javaType(stream, elementCXXType, true);
  stream << ">";
}

void CompressedTeamCommunication::MatrixType::javaInitialize(Out& stream, const std::string& cxxType) const
{
  stream << " = new ";
  javaType(stream, cxxType, false);
  stream << "()";
}

void CompressedTeamCommunication::MatrixType::javaRead(Out& stream, const std::string& cxxType, const std::string& identifier, const std::string& indentation) const
{
  const std::string elementCXXType = cxxType.substr(14, cxxType.find_first_of(',', 14) - 14);
  const int options = std::stoi(cxxType.substr(cxxType.find_first_of(',', cxxType.find_first_of(',', 14 + elementCXXType.length() + 1) + 1) + 1));
  const unsigned int major = (options & Eigen::RowMajor) ? rows : columns;
  const unsigned int minor = (options & Eigen::RowMajor) ? columns : rows;
  const std::string majorMember = (options & Eigen::RowMajor) ? "rows" : "cols";
  stream << indentation << identifier << "." << majorMember << " = new Eigen.Vector[" << major << "];" << endl;
  stream << indentation << "for (int i = 0; i < " << major << "; ++i) {" << endl;
  stream << indentation << "    " << identifier << "." << majorMember << "[i] = new Eigen.Vector<>();" << endl;
  stream << indentation << "    " << identifier << "." << majorMember << "[i].elems = new ";
  elementType->javaType(stream, elementCXXType, true);
  stream << "[" << minor << "];" << endl;
  if(symmetric)
  {
    stream << indentation << "    for (int j = 0; j < i; ++j) {" << endl;
    stream << indentation << "        " << identifier << "." << majorMember << "[i].elems[j] = " << identifier << "." << majorMember << "[j].elems[i];" << endl;
    stream << indentation << "    }" << endl;
  }
  stream << indentation << "    for (int j = " << (symmetric ? "i" : "0") << "; j < " << minor << "; ++j) {" << endl;
  stream << indentation << "        ";
  elementType->javaType(stream, elementCXXType);
  stream << " _" << identifier;
  elementType->javaInitialize(stream, elementCXXType);
  stream << ";" << endl;
  elementType->javaRead(stream, elementCXXType, "_" + identifier, indentation + "        ");
  stream << indentation << "        " << identifier << "." << majorMember << "[i].elems[j] = _" << identifier << ";" << endl;
  stream << indentation << "    }" << endl;
  stream << indentation << "}" << endl;
}

const CompressedTeamCommunication::Type* CompressedTeamCommunication::TypeRegistry::getTypeByName(const std::string& name) const
{
  const auto it = typesByName.find(name);
  if(it == typesByName.end())
    return nullptr;
  return it->second;
}

void CompressedTeamCommunication::TypeRegistry::compile()
{
  typeContainer.clear();
  typesByName.clear();

  TypeInfo::initCurrent();

  std::function<const Type*(const TypeSpecification&)> resolveType = [&, this](const TypeSpecification& typeSpecification) -> const Type*
  {
    auto getNumberOfBits = [](std::size_t n) -> unsigned int
    {
      n -= 1;
      unsigned int result = 0;
      while(n)
      {
        ++result;
        n >>= 1;
      }
      return result;
    };

    const Type* resultType = nullptr;
    std::size_t end;
    if(typeSpecification.name == "Boolean")
    {
      ASSERT(typeSpecification.typeParameters.empty());
      ASSERT(typeSpecification.parameters.empty());
      typeContainer.push_back(std::make_unique<BooleanType>());
      resultType = typeContainer.back().get();
    }
    else if(typeSpecification.name == "Integer")
    {
      ASSERT(typeSpecification.typeParameters.empty());
      auto type = std::make_unique<IntegerType>();
      for(const Parameter& p : typeSpecification.parameters)
      {
        if(p.first == "min")
        {
          type->min = std::stoll(p.second, &end);
          ASSERT(end == p.second.length());
        }
        else if(p.first == "max")
        {
          type->max = std::stoll(p.second, &end);
          ASSERT(end == p.second.length());
        }
        else
          FAIL("Unknown Integer parameter \"" << p.first << "\".");
      }
      ASSERT(type->min <= type->max);
      if(type->min || type->max)
        type->bits = getNumberOfBits(type->max - type->min + 1);
      typeContainer.push_back(std::move(type));
      resultType = typeContainer.back().get();
    }
    else if(typeSpecification.name == "Angle")
    {
      ASSERT(typeSpecification.typeParameters.empty());
      auto type = std::make_unique<AngleType>();
      for(const Parameter& p : typeSpecification.parameters)
      {
        if(p.first == "bits")
        {
          type->bits = static_cast<unsigned int>(std::stoul(p.second, &end));
          ASSERT(end == p.second.length());
          ASSERT(type->bits > 0);
          ASSERT(type->bits < 32);
        }
        else
          FAIL("Unknown Angle parameter \"" << p.first << "\".");
      }
      typeContainer.push_back(std::move(type));
      resultType = typeContainer.back().get();
    }
    else if(typeSpecification.name == "Float")
    {
      ASSERT(typeSpecification.typeParameters.empty());
      auto type = std::make_unique<FloatType>();
      for(const Parameter& p : typeSpecification.parameters)
      {
        if(p.first == "min")
        {
          type->min = std::stod(p.second, &end);
          ASSERT(end == p.second.length());
        }
        else if(p.first == "max")
        {
          type->max = std::stod(p.second, &end);
          ASSERT(end == p.second.length());
        }
        else if(p.first == "bits")
        {
          type->bits = static_cast<unsigned int>(std::stoul(p.second, &end));
          ASSERT(end == p.second.length());
          ASSERT(type->bits > 0);
          ASSERT(type->bits < 32);
        }
        else
          FAIL("Unknown Float parameter \"" << p.first << "\".");
      }
      ASSERT(!type->bits || (type->max > type->min));
      typeContainer.push_back(std::move(type));
      resultType = typeContainer.back().get();
    }
    else if(typeSpecification.name == "Timestamp")
    {
      ASSERT(typeSpecification.typeParameters.empty());
      auto type = std::make_unique<TimestampType>();
      for(const Parameter& p : typeSpecification.parameters)
      {
        if(p.first == "reference")
        {
          if(p.second == "absolute")
            type->reference = TimestampType::absolute;
          else if(p.second == "relativePast")
            type->reference = TimestampType::relativePast;
          else if(p.second == "relativeFuture")
            type->reference = TimestampType::relativeFuture;
          else
            FAIL("Unknown Timestamp reference \"" << p.second << "\".");
        }
        else if(p.first == "bits")
        {
          type->bits = static_cast<unsigned int>(std::stoul(p.second, &end));
          ASSERT(end == p.second.length());
          ASSERT(type->bits > 0);
          ASSERT(type->bits < 32);
        }
        else if(p.first == "shift")
        {
          type->shift = static_cast<unsigned int>(std::stoul(p.second, &end));
          ASSERT(end == p.second.length());
          ASSERT(type->shift > 0);
          ASSERT(type->shift < 32);
        }
        else if(p.first == "noclip")
        {
          ASSERT(p.second.empty());
          type->noclip = true;
        }
        else
          FAIL("Unknown Timestamp parameter \"" << p.first << "\".");
      }
      typeContainer.push_back(std::move(type));
      resultType = typeContainer.back().get();
    }
    else if(typeSpecification.name == "Enum")
    {
      ASSERT(typeSpecification.typeParameters.empty());
      auto type = std::make_unique<EnumType>();
      bool uncompressed = false;
      for(const Parameter& p : typeSpecification.parameters)
      {
        if(p.first == "type")
        {
          const auto it = TypeInfo::current->enums.find(p.second);
          ASSERT(it != TypeInfo::current->enums.end());
          type->name = p.second;
          type->bits = getNumberOfBits(std::max<std::size_t>(1, it->second.size()));
        }
        else if(p.first == "uncompressed")
        {
          ASSERT(p.second.empty());
          uncompressed = true;
        }
        else
          FAIL("Unknown Enum parameter \"" << p.first << "\".");
      }
      ASSERT(!type->name.empty());
      if(uncompressed)
        type->bits = 0;
      typeContainer.push_back(std::move(type));
      resultType = typeContainer.back().get();
    }
    else if(typeSpecification.name == "Vector")
    {
      ASSERT(typeSpecification.typeParameters.size() == 1);
      auto type = std::make_unique<VectorType>();
      type->elementType = resolveType(typeSpecification.typeParameters[0]);
      for(const Parameter& p : typeSpecification.parameters)
      {
        if(p.first == "n")
        {
          type->elements = static_cast<unsigned int>(std::stoul(p.second, &end));
          ASSERT(end == p.second.length());
        }
        else
          FAIL("Unknown Vector parameter \"" << p.first << "\".");
      }
      ASSERT(type->elements);
      typeContainer.push_back(std::move(type));
      resultType = typeContainer.back().get();
    }
    else if(typeSpecification.name == "Matrix")
    {
      ASSERT(typeSpecification.typeParameters.size() == 1);
      auto type = std::make_unique<MatrixType>();
      type->elementType = resolveType(typeSpecification.typeParameters[0]);
      for(const Parameter& p : typeSpecification.parameters)
      {
        if(p.first == "m")
        {
          type->rows = static_cast<unsigned int>(std::stoul(p.second, &end));
          ASSERT(end == p.second.length());
        }
        else if(p.first == "n")
        {
          type->columns = static_cast<unsigned int>(std::stoul(p.second, &end));
          ASSERT(end == p.second.length());
        }
        else if(p.first == "symmetric")
        {
          ASSERT(p.second.empty());
          type->symmetric = true;
        }
        else
          FAIL("Unknown Matrix parameter \"" << p.first << "\".");
      }
      ASSERT(type->rows);
      ASSERT(type->columns);
      ASSERT(!type->symmetric || (type->rows == type->columns));
      typeContainer.push_back(std::move(type));
      resultType = typeContainer.back().get();
    }
    else
    {
      const auto it = typesByName.find(typeSpecification.name);
      if(it == typesByName.end())
        FAIL("Undeclared type \"" << typeSpecification.name << "\".");
      ASSERT(typeSpecification.typeParameters.empty());
      ASSERT(typeSpecification.parameters.empty());
      resultType = it->second;
    }

    // Handle arrays.
    if(typeSpecification.upperCount)
    {
      auto arrayType = std::make_unique<ArrayType>();
      arrayType->element = resultType;
      arrayType->lowerCount = typeSpecification.lowerCount;
      arrayType->upperCount = typeSpecification.upperCount;
      arrayType->bits = getNumberOfBits(arrayType->upperCount - arrayType->lowerCount + 1);
      typeContainer.push_back(std::move(arrayType));
      resultType = typeContainer.back().get();
    }

    return resultType;
  };

  for(const auto& typeDeclaration : typeDeclarations)
  {
    auto recordType = std::make_unique<RecordType>();
    const auto classFromTypeInfo = TypeInfo::current->classes.find(typeDeclaration.name);
    if(classFromTypeInfo == TypeInfo::current->classes.end())
      FAIL("The record type " << typeDeclaration.name << " has no C++ streamable counterpart.");
    recordType->name = typeDeclaration.name;
    for(const auto& memberDeclaration : typeDeclaration.members)
    {
      const auto memberFromTypeInfo = std::find_if(classFromTypeInfo->second.begin(), classFromTypeInfo->second.end(),
                                                   [&memberDeclaration](const TypeInfo::Attribute& attr){ return attr.name == memberDeclaration.name; });
      if(memberFromTypeInfo == classFromTypeInfo->second.end())
        FAIL("The member " << memberDeclaration.name << " in the record " << typeDeclaration.name << " has no C++ counterpart.");
      const auto* memberType = resolveType(memberDeclaration.type);
      if(!memberType->check(memberFromTypeInfo->type))
        FAIL("The member " << memberDeclaration.name << " in the record " << typeDeclaration.name << " has the type " << memberDeclaration.type.name <<
             " which is incompatible with the C++ type " << memberFromTypeInfo->type << ".");
      recordType->members[memberDeclaration.name] = memberType;
    }
    typeContainer.push_back(std::move(recordType));
    typesByName[typeDeclaration.name] = typeContainer.back().get();
  }
}

void CompressedTeamCommunication::TypeRegistry::generateTCMPluginClass(const std::string& file, const RecordType* type)
{
  static_assert(BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION == 13, "This method is not adjusted for the current message version");

  tcmPluginGeneratedTypes.clear();

  OutTextRawFile out(file);
  out << "package bhuman.message;" << endl;
  out << endl;
  out << "import bhuman.message.data.Angle;" << endl;
  out << "import bhuman.message.data.BitStream;" << endl;
  out << "import bhuman.message.data.ComplexStreamReader;" << endl;
  out << "import bhuman.message.data.Eigen;" << endl;
  out << "import bhuman.message.data.Timestamp;" << endl;
  out << "import java.nio.ByteBuffer;" << endl;
  out << "import java.util.ArrayList;" << endl;
  out << "import java.util.LinkedList;" << endl;
  out << "import java.util.List;" << endl;
  out << "import util.Unsigned;" << endl;
  out << endl;
  out << "/**" << endl;
  out << " * This class was generated automatically. DO NOT EDIT!" << endl;
  out << " */" << endl;
  out << "public class BHumanStandardMessage implements ComplexStreamReader<BHumanStandardMessage> {" << endl;
  out << "    public static final String BHUMAN_STANDARD_MESSAGE_STRUCT_HEADER = \"" BHUMAN_STANDARD_MESSAGE_STRUCT_HEADER "\";" << endl;
  out << "    public static final short BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION = " << BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION << ";" << endl;
  out << endl;
  out << "    public static class BNTPMessage {" << endl;
  out << "        public long requestOrigination;" << endl;
  out << "        public long requestReceipt;" << endl;
  out << "        public short receiver;" << endl;
  out << "    }" << endl;
  out << endl;
  for(const auto& pair : type->members)
    ensureTypeExists(out, pair.second);
  out << "    public short magicNumber;" << endl;
  out << "    public long timestamp;" << endl;
  out << "    public boolean requestsNTPMessage;" << endl;
  out << "    public List<BNTPMessage> ntpMessages;" << endl;
  out << endl;
  generateRecordMemberDeclarations(out, type, "    ");
  out << endl;
  out << "    @Override" << endl;
  out << "    public int getStreamedSize(final ByteBuffer stream) {" << endl;
  out << "        int size = 1 + 4 + 2;" << endl;
  out << "        if (stream.remaining() < size) {" << endl;
  out << "            return size;" << endl;
  out << "        }" << endl;
  out << "        final int container = Unsigned.toUnsigned(stream.getShort(stream.position() + size - 2));" << endl;
  out << "        int ntpReceivers = container & 0x3F;" << endl;
  out << "        int ntpCount = 0;" << endl;
  out << "        while (ntpReceivers != 0) {" << endl;
  out << "            if ((ntpReceivers & 1) == 1) {" << endl;
  out << "                ++ntpCount;" << endl;
  out << "            }" << endl;
  out << "            ntpReceivers >>= 1;" << endl;
  out << "        }" << endl;
  out << "        final int compressedSize = container >> 7;" << endl;
  out << "        return size + ntpCount * 5 + compressedSize;" << endl;
  out << "    }" << endl;
  out << endl;
  out << "    @Override" << endl;
  out << "    public BHumanStandardMessage read(final ByteBuffer stream) {" << endl;
  out << "        magicNumber = Unsigned.toUnsigned(stream.get());" << endl;
  out << "        timestamp = Unsigned.toUnsigned(stream.getInt());" << endl;
  out << "        final int ntpAndSizeContainer = Unsigned.toUnsigned(stream.getShort());" << endl;
  out << "        requestsNTPMessage = (ntpAndSizeContainer & (1 << 6)) != 0;" << endl;
  out << "        ntpMessages = new LinkedList<>();" << endl;
  out << "        long runner = 1 << 6;" << endl;
  out << "        for (short i = 1; runner != 0; ++i) {" << endl;
  out << "            if ((ntpAndSizeContainer & (runner >>= 1)) != 0) {" << endl;
  out << "                final BNTPMessage message = new BNTPMessage();" << endl;
  out << "                message.receiver = i;" << endl;
  out << "                ntpMessages.add(message);" << endl;
  out << "                final long timeStruct32 = Unsigned.toUnsigned(stream.getInt());" << endl;
  out << "                final long timeStruct8 = (long) Unsigned.toUnsigned(stream.get());" << endl;
  out << "                message.requestOrigination = timeStruct32 & 0xFFFFFFF;" << endl;
  out << "                message.requestReceipt = timestamp - ((timeStruct32 >> 20) & 0xF00) | timeStruct8;" << endl;
  out << "            }" << endl;
  out << "        }" << endl;
  out << "        final int positionAfterCompressed = stream.position() + (ntpAndSizeContainer >> 7);" << endl;
  out << "        final BitStream bitStream = new BitStream(stream);" << endl;
  out << "        final long __timestampBase = timestamp;" << endl;
  generateRecordMemberReading(out, type, "        ");
  out << "        assert stream.position() == positionAfterCompressed;" << endl;
  out << "        return this;" << endl;
  out << "    }" << endl;
  out << "}" << endl;
}

void CompressedTeamCommunication::TypeRegistry::ensureTypeExists(Out& out, const CompressedTeamCommunication::Type* type)
{
  if(tcmPluginGeneratedTypes.count(type))
    return;
  if(const auto* recordType = dynamic_cast<const CompressedTeamCommunication::RecordType*>(type); recordType)
  {
    for(const auto& pair : recordType->members)
      ensureTypeExists(out, pair.second);
    generateRecord(out, recordType);
  }
  else if(const auto* arrayType = dynamic_cast<const CompressedTeamCommunication::ArrayType*>(type); arrayType)
    ensureTypeExists(out, arrayType->element);
  else if(const auto* enumType = dynamic_cast<const CompressedTeamCommunication::EnumType*>(type); enumType)
    generateEnum(out, enumType);
  else if(const auto* vectorType = dynamic_cast<const CompressedTeamCommunication::VectorType*>(type); vectorType)
    ensureTypeExists(out, vectorType->elementType);
  else if(const auto* matrixType = dynamic_cast<const CompressedTeamCommunication::MatrixType*>(type); matrixType)
    ensureTypeExists(out, matrixType->elementType);
}

void CompressedTeamCommunication::TypeRegistry::generateEnum(Out& out, const CompressedTeamCommunication::EnumType* type)
{
  out << "    public enum ";
  type->javaType(out, type->name, false);
  out << " {" << endl;
  const std::vector<std::string>& constants = TypeInfo::current->enums.find(type->name)->second;
  for(const auto& constant : constants)
    out << "        " << constant << "," << endl;
  out << "        UNKNOWN" << endl;
  out << "    }" << endl;
  out << endl;
  tcmPluginGeneratedTypes.insert(type);
}

void CompressedTeamCommunication::TypeRegistry::generateRecord(Out& out, const CompressedTeamCommunication::RecordType* type)
{
  out << "    public static class ";
  type->javaType(out, type->name, false);
  out << " {" << endl;
  generateRecordMemberDeclarations(out, type, "        ");
  out << endl;
  // The assignment of cols/rows of a matrix generates an "unchecked" warning.
  if(std::any_of(type->members.begin(), type->members.end(), [](const std::pair<std::string, const Type*>& member){ return dynamic_cast<const MatrixType*>(member.second) != nullptr; }))
    out << "        @SuppressWarnings(\"unchecked\")" << endl;
  out << "        public void read(final BitStream bitStream, final long __timestampBase) {" << endl;
  generateRecordMemberReading(out, type, "            ");
  out << "        }" << endl;
  out << "    }" << endl;
  out << endl;
  tcmPluginGeneratedTypes.insert(type);
}

void CompressedTeamCommunication::TypeRegistry::generateRecordMemberDeclarations(Out& out, const CompressedTeamCommunication::RecordType* type, const std::string& indentation) const
{
  const auto classFromTypeInfo = TypeInfo::current->classes.find(type->name);
  ASSERT(classFromTypeInfo != TypeInfo::current->classes.end());

  for(const auto& memberFromTypeInfo : classFromTypeInfo->second)
  {
    const auto memberType = type->members.find(memberFromTypeInfo.name);
    if(memberType == type->members.end())
      continue;
    out << indentation << "public ";
    memberType->second->javaType(out, memberFromTypeInfo.type);
    out << " " << memberFromTypeInfo.name;
    memberType->second->javaInitialize(out, memberFromTypeInfo.type);
    out << ";" << endl;
  }
}

void CompressedTeamCommunication::TypeRegistry::generateRecordMemberReading(Out& out, const CompressedTeamCommunication::RecordType* type, const std::string& indentation) const
{
  const auto classFromTypeInfo = TypeInfo::current->classes.find(type->name);
  ASSERT(classFromTypeInfo != TypeInfo::current->classes.end());

  for(const auto& memberFromTypeInfo : classFromTypeInfo->second)
  {
    const auto memberType = type->members.find(memberFromTypeInfo.name);
    if(memberType == type->members.end())
      continue;
    memberType->second->javaRead(out, memberFromTypeInfo.type, memberFromTypeInfo.name, indentation);
  }
}

/*
 * The grammar of type declarations is as follows:
 *
 * typeDeclarations ::= { typeDeclaration }
 * typeDeclaration ::= identifier '{' memberDeclaration { memberDeclaration } '}'
 * memberDeclaration ::= identifier ':' typeSpecification
 * typeSpecification ::= identifier [ '<' typeSpecification  { ',' typeSpecification } '>' ] [ '(' parameter { ',' parameter } ')' ] [ '[' [ [ number ] ':' ] number ']' ]
 * parameter ::= identifier [ '=' value ]
 *
 * identifier ::= 'A-Za-z_' { 'A-Za-z0-9_' }
 * number ::= '0-9' { '0-9' }
 * value ::= 'A-Za-z0-9_:.-' { 'A-Za-z0-9_:.-' }
 */

void CompressedTeamCommunication::TypeRegistry::addTypes(const std::string& source)
{
  pointer = source.c_str();
  skipWhitespace();
  token = *pointer;
  do
    typeDeclarations.push_back(parseTypeDeclaration());
  while(token != '\0');
}

CompressedTeamCommunication::TypeRegistry::TypeDeclaration CompressedTeamCommunication::TypeRegistry::parseTypeDeclaration()
{
  TypeDeclaration result;
  result.name = readIdentifier();
  expectToken('{');
  do
    result.members.push_back(parseMemberDeclaration());
  while(token != '}' && token != '\0');
  expectToken('}');
  return result;
}

CompressedTeamCommunication::TypeRegistry::MemberDeclaration CompressedTeamCommunication::TypeRegistry::parseMemberDeclaration()
{
  MemberDeclaration result;
  result.name = readIdentifier();
  expectToken(':');
  result.type = parseTypeSpecification();
  return result;
}

CompressedTeamCommunication::TypeRegistry::TypeSpecification CompressedTeamCommunication::TypeRegistry::parseTypeSpecification()
{
  TypeSpecification result;
  result.name = readIdentifier();
  if(token == '<')
  {
    readToken();
    result.typeParameters.push_back(parseTypeSpecification());
    while(token == ',')
    {
      readToken();
      result.typeParameters.push_back(parseTypeSpecification());
    }
    expectToken('>');
  }
  if(token == '(')
  {
    readToken();
    result.parameters.push_back(parseParameter());
    while(token == ',')
    {
      readToken();
      result.parameters.push_back(parseParameter());
    }
    expectToken(')');
  }
  if(token == '[')
  {
    readToken();
    if(token == ':')
    {
      readToken();
      result.upperCount = readNumber();
      if(!result.upperCount)
        FAIL("Expected upperCount > 0.");
    }
    else
    {
      result.lowerCount = readNumber();
      if(token == ':')
      {
        readToken();
        result.upperCount = readNumber();
        if(result.upperCount <= result.lowerCount)
          FAIL("Expected upperCount > lowerCount.");
      }
      else
        result.upperCount = result.lowerCount;
    }
    expectToken(']');
  }
  return result;
}

CompressedTeamCommunication::TypeRegistry::Parameter CompressedTeamCommunication::TypeRegistry::parseParameter()
{
  const std::string key = readIdentifier();
  if(token == '=')
  {
    readToken();
    return {key, readValue()};
  }
  return {key, std::string()};
}

std::string CompressedTeamCommunication::TypeRegistry::readIdentifier()
{
  if((*pointer < 'A' || *pointer > 'Z') && (*pointer < 'a' || *pointer > 'z') && *pointer != '_')
    FAIL("Expected identifier.");
  std::string result;
  result.reserve(20);
  while((*pointer >= 'A' && *pointer <= 'Z') || (*pointer >= 'a' && *pointer <= 'z') || (*pointer >= '0' && *pointer <= '9') || *pointer == '_')
    result += *(pointer++);
  skipWhitespace();
  token = *pointer;
  return result;
}

unsigned int CompressedTeamCommunication::TypeRegistry::readNumber()
{
  if(*pointer < '0' || *pointer > '9')
    FAIL("Expected number.");
  unsigned int result = 0;
  while(*pointer >= '0' && *pointer <= '9')
    result = result * 10 + *(pointer++) - '0';
  skipWhitespace();
  token = *pointer;
  return result;
}

std::string CompressedTeamCommunication::TypeRegistry::readValue()
{
  if((*pointer < 'A' || *pointer > 'Z') && (*pointer < 'a' || *pointer > 'z') && (*pointer < '0' && *pointer > '9') &&
     *pointer != '_' && *pointer != ':' && *pointer != '.' && *pointer != '-')
    FAIL("Expected value.");
  std::string result;
  result.reserve(20);
  while((*pointer >= 'A' && *pointer <= 'Z') || (*pointer >= 'a' && *pointer <= 'z') || (*pointer >= '0' && *pointer <= '9') ||
        *pointer == '_' || *pointer == ':' || *pointer == ':' || *pointer == '.' || *pointer == '-')
    result += *(pointer++);
  skipWhitespace();
  token = *pointer;
  return result;
}

void CompressedTeamCommunication::TypeRegistry::readToken()
{
  ++pointer;
  skipWhitespace();
  token = *pointer;
}

void CompressedTeamCommunication::TypeRegistry::expectToken(char c)
{
  if(*pointer != c)
    FAIL("Expected token '" << c << "' but got '" << *pointer << "'.");
  readToken();
}

void CompressedTeamCommunication::TypeRegistry::skipWhitespace()
{
  while(true)
  {
    while(*pointer == ' ' || *pointer == '\t' || *pointer == '\r' || *pointer == '\n')
      ++pointer;

    // Handle comments.
    if(*pointer == '/' && *(pointer + 1) == '/')
    {
      ++pointer;
      ++pointer;
      while(*pointer != '\0' && *pointer != '\n')
        ++pointer;
    }
    else if(*pointer == '/' && *(pointer + 1) == '*')
    {
      ++pointer;
      ++pointer;
      while(*pointer != '\0' && !(*pointer == '*' && *(pointer + 1) == '/'))
        ++pointer;
    }
    else
      break;
  }
}

void CompressedTeamCommunication::Base::select(const char* name, int type, const char* enumType)
{
  ASSERT(name || type >= 0);

  Streaming::trimName(name);

  const Type* dataType = stack.empty() ? rootType : stack.top().dataType;
  if(!dataType)
    stack.emplace(nullptr, type, enumType);
  else if(type >= 0)
  {
    if(const auto* array = dynamic_cast<const ArrayType*>(dataType); array)
    {
      if(static_cast<unsigned int>(type) < array->upperCount)
        stack.emplace(array->element, type, enumType);
      else
        stack.emplace(nullptr, type, enumType);
    }
    else if(const auto* vector = dynamic_cast<const VectorType*>(dataType); vector)
    {
      if(static_cast<unsigned int>(type) < vector->elements)
        stack.emplace(vector->elementType, type, enumType);
      else
        stack.emplace(nullptr, type, enumType);
    }
    else if(const auto* matrix = dynamic_cast<const MatrixType*>(dataType); matrix)
    {
      ASSERT(matrixStreaming.level == 1 || matrixStreaming.level == 3);
      if(matrixStreaming.level == 1)
      {
        if(static_cast<unsigned int>(type) < matrixStreaming.majorN)
        {
          matrixStreaming.firstIndex = type;
          stack.emplace(matrix, type, enumType);
        }
        else
          stack.emplace(nullptr, type, enumType);
      }
      else
      {
        if(static_cast<unsigned int>(type) < matrixStreaming.minorN)
        {
          if(matrixStreaming.symmetric && type < matrixStreaming.firstIndex)
          {
            if(in)
            {
              matrixStreaming.savedOffset = containerOffset;
              containerOffset = matrixStreaming.symmetricTriangleOffsets[MatrixStreaming::triangleIndex(type, matrixStreaming.firstIndex, matrixStreaming.majorN)];
              stack.emplace(matrix->elementType, type, enumType);
            }
            else
              stack.emplace(nullptr, type, enumType);
          }
          else
          {
            if(in && matrixStreaming.symmetric && type > matrixStreaming.firstIndex)
              matrixStreaming.symmetricTriangleOffsets[MatrixStreaming::triangleIndex(matrixStreaming.firstIndex, type, matrixStreaming.majorN)] = containerOffset;
            stack.emplace(matrix->elementType, type, enumType);
          }
        }
        else
          stack.emplace(nullptr, type, enumType);
      }
      ++matrixStreaming.level;
    }
    else
      FAIL("Select of array element failed because the parent is not an array.");
  }
  else if(const auto* record = dynamic_cast<const RecordType*>(dataType); record)
  {
    if(const auto memberIt = record->members.find(name); memberIt != record->members.end())
    {
      stack.emplace(memberIt->second, type, enumType);
    }
    else
      stack.emplace(nullptr, type, enumType);
  }
  else if(const auto* vector = dynamic_cast<const VectorType*>(dataType); vector)
  {
    if(vector->elements == 2 || vector->elements == 3)
    {
      if((name[0] == 'x' || name[0] == 'y' || (vector->elements == 3 && name[0] == 'z')) && name[1] == '\0')
        stack.emplace(vector->elementType, type, enumType);
      else
        stack.emplace(nullptr, type, enumType);
    }
    else if(!std::strcmp(name, "elems"))
      stack.emplace(vector, type, enumType);
    else
      stack.emplace(nullptr, type, enumType);
  }
  else if(const auto* matrix = dynamic_cast<const MatrixType*>(dataType); matrix)
  {
    ASSERT(matrixStreaming.level == 0 || matrixStreaming.level == 2);
    if(matrixStreaming.level)
    {
      if(!std::strcmp(name, "elems"))
        stack.emplace(matrix, type, enumType);
      else
        stack.emplace(nullptr, type, enumType);
    }
    else
    {
      if(!std::strcmp(name, "rows") || !std::strcmp(name, "cols"))
      {
        const bool rowMajor = name[0] == 'r';
        matrixStreaming.majorN = rowMajor ? matrix->rows : matrix->columns;
        matrixStreaming.minorN = rowMajor ? matrix->columns : matrix->rows;
        matrixStreaming.symmetric = matrix->symmetric;
        if(matrixStreaming.symmetric && in)
          matrixStreaming.symmetricTriangleOffsets.resize(matrix->rows * (matrix->rows - 1) / 2);
        stack.emplace(matrix, type, enumType);
      }
      else
        stack.emplace(nullptr, type, enumType);
    }
    ++matrixStreaming.level;
  }
  else
    FAIL("Select failed due to invalid type.");
}

void CompressedTeamCommunication::Base::deselect()
{
  ASSERT(!stack.empty());
  const int type = stack.top().type;
  stack.pop();
  if(matrixStreaming.level && !stack.empty() && dynamic_cast<const MatrixType*>(stack.top().dataType))
  {
    if(matrixStreaming.level == 4 && in && matrixStreaming.symmetric && type < matrixStreaming.firstIndex && static_cast<unsigned int>(type) < matrixStreaming.majorN)
      containerOffset = matrixStreaming.savedOffset;
    --matrixStreaming.level;
  }
}

#ifdef NDEBUG
#define TYPE_CAST static_cast
#else
#define TYPE_CAST dynamic_cast
#endif

void CompressedTeamCommunicationIn::readBits(void* data, std::size_t bits)
{
  std::uint8_t* cdata = reinterpret_cast<std::uint8_t*>(data);
  for(std::size_t i = 0; i < bits; ++i, ++containerOffset)
  {
    if(container[containerOffset / 8] & (1 << (containerOffset % 8)))
      cdata[i / 8] |= 1 << (i % 8);
    else
      cdata[i / 8] &= ~(1 << (i % 8));
  }
}

template<typename Integer>
void CompressedTeamCommunicationIn::readInteger(Integer& value, const CompressedTeamCommunication::IntegerType& type)
{
  if(type.bits)
  {
    std::uint64_t x = 0;
    readBits(&x, type.bits);
    value = static_cast<Integer>(x + type.min);
  }
  else
    readBits(&value, sizeof(value) * 8);
}

template<typename Float>
void CompressedTeamCommunicationIn::readFloat(Float& value, const CompressedTeamCommunication::FloatType& type)
{
  if(type.bits)
  {
    std::int64_t integerValue = 0;
    readBits(&integerValue, type.bits);
    value = static_cast<Float>(integerValue / static_cast<double>((1LL << type.bits) - 1) * (type.max - type.min) + type.min);
  }
  else
    readBits(&value, sizeof(value) * 8);
}

void CompressedTeamCommunicationIn::inBool(bool& value)
{
  ASSERT(!stack.empty());
  if(!stack.top().dataType)
    return;
  ASSERT(TYPE_CAST<const CompressedTeamCommunication::BooleanType*>(stack.top().dataType));
  unsigned char byte = 0;
  readBits(&byte, 1);
  value = byte != 0;
}

void CompressedTeamCommunicationIn::inChar(char& value)
{
  ASSERT(!stack.empty());
  Entry& entry = stack.top();
  if(!entry.dataType)
    return;
  const auto* type = TYPE_CAST<const CompressedTeamCommunication::IntegerType*>(entry.dataType);
  ASSERT(type);
  readInteger<char>(value, *type);
}

void CompressedTeamCommunicationIn::inSChar(signed char& value)
{
  ASSERT(!stack.empty());
  Entry& entry = stack.top();
  if(!entry.dataType)
    return;
  const auto* type = TYPE_CAST<const CompressedTeamCommunication::IntegerType*>(entry.dataType);
  ASSERT(type);
  readInteger<signed char>(value, *type);
}

void CompressedTeamCommunicationIn::inUChar(unsigned char& value)
{
  ASSERT(!stack.empty());
  Entry& entry = stack.top();
  if(!entry.dataType)
    return;
  if(entry.enumType)
  {
    const auto* type = TYPE_CAST<const CompressedTeamCommunication::EnumType*>(entry.dataType);
    ASSERT(type);
    ASSERT(!check || type->name == TypeRegistry::demangle(entry.enumType));
    value = 0;
    readBits(&value, type->bits ? type->bits : sizeof(unsigned char) * 8);
  }
  else
  {
    const auto* type = TYPE_CAST<const CompressedTeamCommunication::IntegerType*>(entry.dataType);
    ASSERT(type);
    readInteger<unsigned char>(value, *type);
  }
}

void CompressedTeamCommunicationIn::inShort(short& value)
{
  ASSERT(!stack.empty());
  Entry& entry = stack.top();
  if(!entry.dataType)
    return;
  const auto* type = TYPE_CAST<const CompressedTeamCommunication::IntegerType*>(entry.dataType);
  ASSERT(type);
  readInteger<short>(value, *type);
}

void CompressedTeamCommunicationIn::inUShort(unsigned short& value)
{
  ASSERT(!stack.empty());
  Entry& entry = stack.top();
  if(!entry.dataType)
    return;
  const auto* type = TYPE_CAST<const CompressedTeamCommunication::IntegerType*>(entry.dataType);
  ASSERT(type);
  readInteger<unsigned short>(value, *type);
}

void CompressedTeamCommunicationIn::inInt(int& value)
{
  ASSERT(!stack.empty());
  Entry& entry = stack.top();
  if(!entry.dataType)
    return;
  const auto* type = TYPE_CAST<const CompressedTeamCommunication::IntegerType*>(entry.dataType);
  ASSERT(type);
  readInteger<int>(value, *type);
}

void CompressedTeamCommunicationIn::inUInt(unsigned int& value)
{
  ASSERT(!stack.empty());
  Entry& entry = stack.top();
  if(!entry.dataType)
    return;
  if(entry.type == -1)
  {
    const auto* type = TYPE_CAST<const CompressedTeamCommunication::ArrayType*>(entry.dataType);
    ASSERT(type);
    value = 0;
    readBits(&value, type->bits);
    value += type->lowerCount;
  }
  else if(const auto* type = dynamic_cast<const CompressedTeamCommunication::TimestampType*>(entry.dataType); type)
  {
    if(type->reference != CompressedTeamCommunication::TimestampType::absolute)
    {
      ASSERT(type->reference == CompressedTeamCommunication::TimestampType::relativePast || type->reference == CompressedTeamCommunication::TimestampType::relativeFuture);
      unsigned int diff = 0;
      readBits(&diff, type->bits);
      if(diff == (1U << type->bits) - 1 && type->noclip)
        value = type->reference == CompressedTeamCommunication::TimestampType::relativePast ? 0 : std::numeric_limits<unsigned int>::max();
      else
      {
        diff <<= type->shift;
        value = type->reference == CompressedTeamCommunication::TimestampType::relativePast ? baseTimestamp - diff : baseTimestamp + diff;
      }
    }
    else
      readBits(&value, sizeof(value) * 8);
    if(toLocalTimestamp)
      value = toLocalTimestamp(value);
  }
  else
  {
    const auto* integerType = TYPE_CAST<const CompressedTeamCommunication::IntegerType*>(entry.dataType);
    ASSERT(integerType);
    readInteger<unsigned int>(value, *integerType);
  }
}

void CompressedTeamCommunicationIn::inFloat(float& value)
{
  ASSERT(!stack.empty());
  Entry& entry = stack.top();
  if(!entry.dataType)
    return;
  const auto* type = TYPE_CAST<const CompressedTeamCommunication::FloatType*>(entry.dataType);
  ASSERT(type);
  readFloat<float>(value, *type);
}

void CompressedTeamCommunicationIn::inDouble(double& value)
{
  ASSERT(!stack.empty());
  Entry& entry = stack.top();
  if(!entry.dataType)
    return;
  const auto* type = TYPE_CAST<const CompressedTeamCommunication::FloatType*>(entry.dataType);
  ASSERT(type);
  readFloat<double>(value, *type);
}

void CompressedTeamCommunicationIn::inString(std::string&)
{
  FAIL("Unsupported operation.");
}

void CompressedTeamCommunicationIn::inAngle(Angle& value)
{
  ASSERT(!stack.empty());
  Entry& entry = stack.top();
  if(!entry.dataType)
    return;
  const auto* type = TYPE_CAST<const CompressedTeamCommunication::AngleType*>(entry.dataType);
  ASSERT(type);
  readFloat<float>(value, *type);
}

void CompressedTeamCommunicationIn::read(void*, std::size_t)
{
  FAIL("Unsupported operation.");
}

void CompressedTeamCommunicationIn::skip(std::size_t)
{
  FAIL("Unsupported operation.");
}

bool CompressedTeamCommunicationIn::eof() const
{
  return containerOffset == container.size() * 8;
}

void CompressedTeamCommunicationIn::select(const char* name, int type, const char* enumType)
{
  Base::select(name, type, enumType);
}

void CompressedTeamCommunicationIn::deselect()
{
  Base::deselect();
}

CompressedTeamCommunicationIn::CompressedTeamCommunicationIn(const std::vector<std::uint8_t>& container, unsigned baseTimestamp, const CompressedTeamCommunication::Type* rootType,
                                                             const std::function<unsigned(unsigned)>& toLocalTimestamp) :
  Base(rootType, true),
  container(container),
  baseTimestamp(baseTimestamp),
  toLocalTimestamp(toLocalTimestamp)
{}

void CompressedTeamCommunicationOut::writeBits(const void* data, std::size_t bits)
{
  const std::uint8_t* cdata = reinterpret_cast<const std::uint8_t*>(data);
  container.resize((containerOffset + bits + 7) / 8, 0);
  for(std::size_t i = 0; i < bits; ++i, ++containerOffset)
    container[containerOffset / 8] |= ((cdata[i / 8] >> (i % 8)) & 1) << (containerOffset % 8);
}

template<typename Integer>
void CompressedTeamCommunicationOut::writeInteger(Integer value, const CompressedTeamCommunication::IntegerType& type)
{
  if(type.bits)
  {
    const std::uint64_t x = static_cast<std::uint64_t>(std::max(type.min, std::min(static_cast<std::int64_t>(value), type.max)) - type.min);
    writeBits(&x, type.bits);
  }
  else
    writeBits(&value, sizeof(value) * 8);
}

template<typename Float>
void CompressedTeamCommunicationOut::writeFloat(Float value, const CompressedTeamCommunication::FloatType& type)
{
  if(type.bits)
  {
    const double scaledValue = (std::max(type.min, std::min(static_cast<double>(value), type.max)) - type.min) / (type.max - type.min) * ((1LL << type.bits) - 1);
    const std::int64_t integerValue = static_cast<std::int64_t>(scaledValue + 0.5f);
    writeBits(&integerValue, type.bits);
  }
  else
    writeBits(&value, sizeof(value) * 8);
}

void CompressedTeamCommunicationOut::outBool(bool value)
{
  ASSERT(!stack.empty());
  Entry& entry = stack.top();
  if(!entry.dataType)
    return;
  ASSERT(TYPE_CAST<const CompressedTeamCommunication::BooleanType*>(entry.dataType));
  unsigned char byte = value ? 1 : 0;
  writeBits(&byte, 1);
}

void CompressedTeamCommunicationOut::outChar(char value)
{
  ASSERT(!stack.empty());
  Entry& entry = stack.top();
  if(!entry.dataType)
    return;
  const auto* type = TYPE_CAST<const CompressedTeamCommunication::IntegerType*>(entry.dataType);
  ASSERT(type);
  writeInteger<char>(value, *type);
}

void CompressedTeamCommunicationOut::outSChar(signed char value)
{
  ASSERT(!stack.empty());
  Entry& entry = stack.top();
  if(!entry.dataType)
    return;
  const auto* type = TYPE_CAST<const CompressedTeamCommunication::IntegerType*>(entry.dataType);
  ASSERT(type);
  writeInteger<signed char>(value, *type);
}

void CompressedTeamCommunicationOut::outUChar(unsigned char value)
{
  ASSERT(!stack.empty());
  Entry& entry = stack.top();
  if(!entry.dataType)
    return;
  if(entry.enumType)
  {
    const auto* type = TYPE_CAST<const CompressedTeamCommunication::EnumType*>(entry.dataType);
    ASSERT(type);
    ASSERT(!check || type->name == TypeRegistry::demangle(entry.enumType));
    writeBits(&value, type->bits ? type->bits : sizeof(unsigned char) * 8);
  }
  else
  {
    const auto* type = TYPE_CAST<const CompressedTeamCommunication::IntegerType*>(entry.dataType);
    ASSERT(type);
    writeInteger<unsigned char>(value, *type);
  }
}

void CompressedTeamCommunicationOut::outShort(short value)
{
  ASSERT(!stack.empty());
  Entry& entry = stack.top();
  if(!entry.dataType)
    return;
  const auto* type = TYPE_CAST<const CompressedTeamCommunication::IntegerType*>(entry.dataType);
  ASSERT(type);
  writeInteger<short>(value, *type);
}

void CompressedTeamCommunicationOut::outUShort(unsigned short value)
{
  ASSERT(!stack.empty());
  Entry& entry = stack.top();
  if(!entry.dataType)
    return;
  const auto* type = TYPE_CAST<const CompressedTeamCommunication::IntegerType*>(entry.dataType);
  ASSERT(type);
  writeInteger<unsigned short>(value, *type);
}

void CompressedTeamCommunicationOut::outInt(int value)
{
  ASSERT(!stack.empty());
  Entry& entry = stack.top();
  if(!entry.dataType)
    return;
  const auto* type = TYPE_CAST<const CompressedTeamCommunication::IntegerType*>(entry.dataType);
  ASSERT(type);
  writeInteger<int>(value, *type);
}

void CompressedTeamCommunicationOut::outUInt(unsigned int value)
{
  ASSERT(!stack.empty());
  Entry& entry = stack.top();
  if(!entry.dataType)
    return;
  if(entry.type == -1)
  {
    const auto* type = TYPE_CAST<const CompressedTeamCommunication::ArrayType*>(entry.dataType);
    ASSERT(type);
    ASSERT(value >= type->lowerCount);
    const unsigned int count = std::min(value, type->upperCount) - static_cast<unsigned int>(type->lowerCount);
    writeBits(&count, type->bits);
  }
  else if(const auto* type = dynamic_cast<const CompressedTeamCommunication::TimestampType*>(entry.dataType); type)
  {
    if(type->reference != CompressedTeamCommunication::TimestampType::absolute)
    {
      ASSERT(type->reference == CompressedTeamCommunication::TimestampType::relativePast || type->reference == CompressedTeamCommunication::TimestampType::relativeFuture);
      const unsigned int diff = std::min((type->reference == CompressedTeamCommunication::TimestampType::relativePast ?
                                          (baseTimestamp - std::min(baseTimestamp, value)) : (std::max(baseTimestamp, value) - baseTimestamp)) >> type->shift,
                                         (1U << type->bits) - 1);
      writeBits(&diff, type->bits);
    }
    else
      writeBits(&value, sizeof(value) * 8);
  }
  else
  {
    const auto* integerType = TYPE_CAST<const CompressedTeamCommunication::IntegerType*>(entry.dataType);
    ASSERT(integerType);
    writeInteger<unsigned int>(value, *integerType);
  }
}

void CompressedTeamCommunicationOut::outFloat(float value)
{
  ASSERT(!stack.empty());
  Entry& entry = stack.top();
  if(!entry.dataType)
    return;
  const auto* type = TYPE_CAST<const CompressedTeamCommunication::FloatType*>(entry.dataType);
  ASSERT(type);
  writeFloat<float>(value, *type);
}

void CompressedTeamCommunicationOut::outDouble(double value)
{
  ASSERT(!stack.empty());
  Entry& entry = stack.top();
  if(!entry.dataType)
    return;
  const auto* type = TYPE_CAST<const CompressedTeamCommunication::FloatType*>(entry.dataType);
  ASSERT(type);
  writeFloat<double>(value, *type);
}

void CompressedTeamCommunicationOut::outString(const char*)
{
  FAIL("Unsupported operation.");
}

void CompressedTeamCommunicationOut::outAngle(const Angle& value)
{
  ASSERT(!stack.empty());
  Entry& entry = stack.top();
  if(!entry.dataType)
    return;
  const auto* type = TYPE_CAST<const CompressedTeamCommunication::AngleType*>(entry.dataType);
  ASSERT(type);
  writeFloat<float>(value, *type);
}

void CompressedTeamCommunicationOut::write(const void*, std::size_t)
{
  FAIL("Unsupported operation.");
}

void CompressedTeamCommunicationOut::select(const char* name, int type, const char* enumType)
{
  Base::select(name, type, enumType);
}

void CompressedTeamCommunicationOut::deselect()
{
  Base::deselect();
}

CompressedTeamCommunicationOut::CompressedTeamCommunicationOut(std::vector<std::uint8_t>& container, unsigned baseTimestamp, const CompressedTeamCommunication::Type* rootType, bool check) :
  Base(rootType, false, check),
  container(container),
  baseTimestamp(baseTimestamp)
{
  container.clear();
}
