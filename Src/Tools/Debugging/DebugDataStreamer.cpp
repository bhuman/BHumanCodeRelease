/**
 * The file implements a class that makes the debug data in a stream streamable
 * according to the Streamable interface.
 *
 * @author Thomas Röfer
 */

#include "DebugDataStreamer.h"
#include "Platform/BHAssert.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Math/Angle.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Streams/TypeInfo.h"
#include <sstream>

DebugDataStreamer::DebugDataStreamer(const TypeInfo& typeInfo, In& stream, const std::string& type, const char* name) :
  typeInfo(typeInfo), inData(&stream), type(type), name(name)
{}

DebugDataStreamer::DebugDataStreamer(const TypeInfo& typeInfo, Out& stream, const std::string& type, const char* name) :
  typeInfo(typeInfo), outData(&stream), type(type), name(name)
{}

void DebugDataStreamer::serialize(In* in, Out* out)
{
  ASSERT((inData && out) || (outData && in));

  if(type[type.size() - 1] == ']' || type[type.size() - 1] == '*')
  {
    unsigned size = 0;
    std::string elementType;
    bool staticSize = type[type.size() - 1] == ']';
    if(staticSize)
    {
      size_t endOfType = type.find_last_of('[');
      size = atoi(&type[endOfType + 1]);
      elementType = type.substr(0, endOfType);
    }
    else
    {
      elementType = type.substr(0, type.size() - 1);
      if(inData)
        *inData >> size;
    }

    if(in)
    {
      in->select(name, -1);
      unsigned dynamicSize;
      *in >> dynamicSize;
      if(!staticSize)
      {
        size = dynamicSize;
        *outData << size;
      }
      else if(size != dynamicSize)
      {
        std::ostringstream stream;
        stream << "array has " << dynamicSize << " elements instead of " << size;
        InMap* inMap = dynamic_cast<InMap*>(in);
        if(inMap)
          inMap->printError(stream.str());
        else
        {
          OUTPUT_ERROR(stream.str());
          size = dynamicSize;
        }
      }
    }
    else
    {
      out->select(name, -1);
      *out << size;
    }

    for(unsigned i = 0; i < size; ++i)
    {
      // set attributes for recursive call to this method
      type = elementType;
      name = 0;
      index = static_cast<int>(i);
      if(in)
        *in >> *this;
      else
        *out << *this;
    }

    if(in)
      in->deselect();
    else
      out->deselect();
  }
  else if(typeInfo.primitives.find(type) != typeInfo.primitives.end())
  {
    if(type == "char")
      streamIt<char>(in, out);
    else if(type == "signed char")
      streamIt<signed char>(in, out);
    else if(type == "unsigned char")
      streamIt<unsigned char>(in, out);
    else if(type == "short")
      streamIt<short>(in, out);
    else if(type == "unsigned short")
      streamIt<unsigned short>(in, out);
    else if(type == "int")
      streamIt<int>(in, out);
    else if(type == "unsigned" || type == "unsigned int")
      streamIt<unsigned>(in, out);
    else if(type == "float")
      streamIt<float>(in, out);
    else if(type == "double")
      streamIt<double>(in, out);
    else if(type == "bool")
      streamIt<bool>(in, out);
    else if(type == "Angle")
      streamIt<Angle>(in, out);
    else if(type == "std::string")
      streamIt<std::string>(in, out);
    else
      FAIL(type << " is not a streamable type!");
  }
  else if(typeInfo.enums.find(type) != typeInfo.enums.end())
  {
    const std::vector<std::string>* enumType[2] = {nullptr, &typeInfo.enums.find(type)->second};
    streamIt<unsigned char>(in, out, reinterpret_cast<const char*>(enumType));
  }
  else if(typeInfo.classes.find(type) != typeInfo.classes.end())
  {
    bool select = name != 0 || index >= 0;
    if(select)
    {
      if(in)
        in->select(name, index);
      else
        out->select(name, index);
    }
    for(const TypeInfo::Attribute& attribute : typeInfo.classes.find(type)->second)
    {
      // set attributes for recursive call to this method
      type = attribute.type;
      name = attribute.name.c_str();
      index = -2;
      if(in)
        *in >> *this;
      else
        *out << *this;
    }
    if(select)
    {
      if(in)
        in->deselect();
      else
        out->deselect();
    }
  }
  else
    FAIL("Specification for " << type << " not found");
}
