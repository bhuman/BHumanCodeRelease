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
#include "Tools/Streams/StreamHandler.h"
#include <cstdlib>
#include <cstring>
#include <iostream>

thread_local const std::vector<const char*>* DebugDataStreamer::enumNames = nullptr;

DebugDataStreamer::DebugDataStreamer(StreamHandler& streamHandler, In& stream, const std::string& type, const char* name) :
  streamHandler(streamHandler), inData(&stream), type(type), name(name)
{}

DebugDataStreamer::DebugDataStreamer(StreamHandler& streamHandler, Out& stream, const std::string& type, const char* name) :
  streamHandler(streamHandler), outData(&stream), type(type), name(name)
{}

void DebugDataStreamer::serialize(In* in, Out* out)
{
  ASSERT((inData && out) || (outData && in));

  if(type.size() > 8 && type.substr(type.size() - 8) == " __ptr64")
    type = type.substr(0, type.size() - 8);

  if(type[type.size() - 1] == ']' || type[type.size() - 1] == '*')
  {
    unsigned size;
    std::string elementType;
    bool staticSize = type[type.size() - 1] == ']';
    if(staticSize)
    {
      size_t i = type.size();
      while(type[i - 1] != '[')
        --i;
      size_t j = type[i - 2] == ' ' ? i - 2 : i - 1;
      size = atoi(&type[i]);
      if(type[j - 1] == ')')
      {
        j -= type[j - 2] == '4' ? 8 : 0; // " __ptr64"
        j -= type[j - 4] == ' ' ? 4 : 3;
      }
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
      unsigned dynamicSize;
      *in >> dynamicSize;
      if(!staticSize)
      {
        size = dynamicSize;
        *outData << size;
      }
      else if(size != dynamicSize)
      {
        char buf[100];
        sprintf(buf, "array has %d elements instead of %d", dynamicSize, size);
        InMap* inMap = dynamic_cast<InMap*>(in);
        if(inMap)
          inMap->printError(buf);
        else
        {
          OUTPUT_ERROR(buf);
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
      else if(!strcmp("signed char", t))
        streamIt<signed char>(in, out);
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
      else if(std::string(t).find("Angle") != std::string::npos)
        streamIt<Angle>(in, out);
      else
        ASSERT(false);
    }
    else if(e != streamHandler.enumSpecification.end())
    {
      enumNames = &e->second;
      streamIt<unsigned char>(in, out, &DebugDataStreamer::getName);
    }
    else
    {
      std::cerr << "Specification for " << t << " not found" << std::endl;
      ASSERT(false); // specification missing
    }
  }
}

const char* DebugDataStreamer::getName(int value)
{
  if(value >= 0 && value < static_cast<int>(enumNames->size()))
    return (*enumNames)[value];
  else
    return nullptr;
}
