/**
 * The file implements a class that makes the debug data in a stream streamable
 * according to the Streamable interface.
 *
 * @author Thomas Röfer
 */

#include "DebugDataStreamer.h"
#include "Platform/BHAssert.h"
#include "Tools/Streams/StreamHandler.h"
#include <cstdlib>
#include <cstring>

PROCESS_WIDE_STORAGE(const std::vector<const char*>) DebugDataStreamer::enumNames = 0;

DebugDataStreamer::DebugDataStreamer(StreamHandler& streamHandler, In& stream, const std::string& type, const char* name)
: streamHandler(streamHandler),
  inData(&stream),
  outData(0),
  type(type),
  name(name),
  index(-2) {}

DebugDataStreamer::DebugDataStreamer(StreamHandler& streamHandler, Out& stream, const std::string& type, const char* name)
: streamHandler(streamHandler),
  inData(0),
  outData(&stream),
  type(type),
  name(name),
  index(-2) {}

void DebugDataStreamer::serialize(In* in, Out* out)
{
  ASSERT((inData && out) || (outData && in));

  if(type[type.size() - 1] == ']' || type[type.size() - 1] == '*')
  {
    unsigned size;
    std::string elementType;
    bool staticSize = type[type.size() - 1] == ']';
    if(staticSize)
    {
      unsigned i = type.size();
      while(type[i - 1] != '[')
        --i;
      unsigned j = type[i - 2] == ' ' ? i - 2 : i - 1;
      size = atoi(&type[i]);
      elementType = std::string(type).substr(0, j);
    }
    else
    {
      elementType = std::string(type).substr(0, type.size() - (type.size() > 1 && type[type.size() - 2] == ' ' ? 2 : 1));
      if(inData)
        *inData >> size;
    }

    if(in)
    {
      in->select(name, -1);
      *in >> size;
      if(!staticSize)
        *outData << size;
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
      index = (int) i;
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
  else
  {
    std::string typeName = type;
    if(typeName.size() > 6 && typeName.substr(typeName.size() - 6) == " const")
      typeName = typeName.substr(0, typeName.size() - 6);
    const char* t = streamHandler.getString(typeName.c_str());
    StreamHandler::Specification::const_iterator i = streamHandler.specification.find(t);
    StreamHandler::BasicTypeSpecification::const_iterator b = streamHandler.basicTypeSpecification.find(t);
    StreamHandler::EnumSpecification::const_iterator e = streamHandler.enumSpecification.find(t);
    if(i != streamHandler.specification.end())
    {
      bool select = name != 0 || index >= 0;
      if(select)
      {
        if(in)
          in->select(name, index);
        else
          out->select(name, index);
      }
      for(std::vector<StreamHandler::TypeNamePair>::const_iterator j = i->second.begin(); j != i->second.end(); ++j)
      {
        // set attributes for recursive call to this method
        type = j->second;
        name = j->first.c_str();
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
    else if(b != streamHandler.basicTypeSpecification.end())
    {
      if(!strcmp("char", t))
        streamIt<char>(in, out);
      else if(!strcmp("unsigned char", t))
        streamIt<unsigned char>(in, out);
      else if(!strcmp("short", t))
        streamIt<short>(in, out);
      else if(!strcmp("unsigned short", t))
        streamIt<unsigned short>(in, out);
      else if(!strcmp("int", t))
        streamIt<int>(in, out);
      else if(!strcmp("unsigned", t) || !strcmp("unsigned int", t))
        streamIt<unsigned>(in, out);
      else if(!strcmp("float", t))
        streamIt<float>(in, out);
      else if(!strcmp("double", t))
        streamIt<double>(in, out);
      else if(!strcmp("bool", t))
        streamIt<bool>(in, out);
      else if(std::string(t).find("string") != std::string::npos)
        streamIt<std::string>(in, out);
      else
        ASSERT(false);
    }
    else if(e != streamHandler.enumSpecification.end())
    {
      enumNames = &e->second;
      streamIt<unsigned char>(in, out, &DebugDataStreamer::getName);
    }
    else
      ASSERT(false); // specification missing
  }
}

const char* DebugDataStreamer::getName(int value)
{
  if(value >= 0 && value < (int) enumNames->size())
    return (*enumNames)[value];
  else
    return 0;
}
